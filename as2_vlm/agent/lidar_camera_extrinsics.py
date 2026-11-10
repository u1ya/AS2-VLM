"""Offline candidate camera-from-LiDAR calibration using manually identified boards.

An operator must identify the same physical board plane in both sensors. Unknown
units, unstable intrinsics, bad timing, parallel poses and inconsistent holdouts
refuse output. This tool has no robot transport and never installs calibration.
"""
import argparse
import json
from pathlib import Path
import re

import numpy as np

from .camera_lidar import CameraLidarCalibration


def fit_plane(points):
    points = np.asarray(points, dtype=float)
    if (points.ndim != 2 or points.shape[1] != 3 or not 30 <= len(points) <= 20000 or
            not np.isfinite(points).all()):
        raise ValueError("Board region needs 30-20000 finite XYZ points")
    rng = np.random.default_rng(0)
    best = None
    best_cost = float("inf")
    best_plane = None
    for _ in range(128):
        a,b,c = points[rng.choice(len(points),3,replace=False)]
        normal = np.cross(b-a,c-a)
        length = np.linalg.norm(normal)
        if length < 1e-9:
            continue
        normal /= length
        distance = (points-a)@normal
        mask = np.abs(distance) <= .008
        cost = float(np.minimum(distance**2,.008**2).sum())
        if cost < best_cost:
            best = mask
            best_cost = cost
            best_plane = (normal,-float(a@normal))
    if best is None or best.sum() < 30 or best.mean() < .8:
        raise ValueError("Selected region is not a well-supported single plane")
    # Robust residual trimming prevents a few distant points close to the plane
    # from dominating the normal estimate through their spatial leverage.
    residual = points@best_plane[0]+best_plane[1]
    scale = 1.4826*np.median(np.abs(residual[best]-np.median(residual[best])))
    best &= np.abs(residual) <= max(.001,3*scale)
    if best.sum() < 30 or best.mean() < .8:
        raise ValueError("Board plane loses support after robust residual trimming")
    selected = points[best]
    center = selected.mean(axis=0)
    _,singular,vt = np.linalg.svd(selected-center,full_matrices=False)
    if singular[1]/np.sqrt(len(selected)) < .025:
        raise ValueError("Board points do not span a sufficiently large 2D surface")
    normal = vt[-1]
    if np.dot(normal,center) > 0:
        normal = -normal
    offset = -float(np.dot(normal,center))
    rms = float(np.sqrt(np.mean(((selected-center)@normal)**2)))
    if rms > .008 or offset < .05:
        raise ValueError("Plane noise or distance is outside supported bounds")
    return normal,offset,{"points":len(points),"inliers":int(best.sum()),"plane_rms_m":rms}


def camera_plane(corners, intrinsics, square_m):
    import cv2
    points = np.asarray(corners,dtype=np.float64)
    if points.shape != (54,1,2) or not np.isfinite(points).all():
        raise ValueError("Expected 54 measured camera corners")
    obj = np.zeros((54,3),np.float64)
    obj[:,:2] = np.mgrid[0:9,0:6].T.reshape(-1,2)*square_m
    K = np.asarray(intrinsics["K"],dtype=float)
    distortion = np.asarray(intrinsics["distortion"],dtype=float)
    ok,rvec,tvec = cv2.solvePnP(obj,points,K,distortion)
    if not ok:
        raise ValueError("Board pose could not be estimated")
    rotation,_ = cv2.Rodrigues(rvec)
    transformed = obj@rotation.T+tvec.reshape(3)
    if np.any(transformed[:,2] <= .05):
        raise ValueError("Board is behind or too close to camera")
    predicted,_ = cv2.projectPoints(obj,rvec,tvec,K,distortion)
    rms = float(np.sqrt(np.mean(np.sum((predicted-points)**2,axis=2))))
    if not np.isfinite(rms) or rms > 2:
        raise ValueError("Board camera pose reprojection error exceeds 2 pixels")
    normal = rotation[:,2]
    if np.dot(normal,tvec.reshape(3)) > 0:
        normal = -normal
    return normal,-float(np.dot(normal,tvec.reshape(3))),rms


def solve_planes(pairs):
    """Fit n_camera=R*n_lidar, d_camera=d_lidar-n_camera*t.

    Last two independently acquired board poses are reserved for validation.
    Plane normals must face each sensor; both must observe the same board side.
    """
    if len(pairs) < 8:
        raise ValueError("At least eight distinct paired board poses are required")
    camera = np.asarray([p["camera_normal"] for p in pairs],dtype=float)
    lidar = np.asarray([p["lidar_normal"] for p in pairs],dtype=float)
    dc = np.asarray([p["camera_offset_m"] for p in pairs],dtype=float)
    dl = np.asarray([p["lidar_offset_m"] for p in pairs],dtype=float)
    if (camera.shape != (len(pairs),3) or lidar.shape != camera.shape or
            not all(np.isfinite(v).all() for v in (camera,lidar,dc,dl)) or
            not np.allclose(np.linalg.norm(camera,axis=1),1,atol=1e-6) or
            not np.allclose(np.linalg.norm(lidar,axis=1),1,atol=1e-6)):
        raise ValueError("Invalid finite normalized plane correspondences")
    train = slice(None,-2)
    for normals in (camera[train],lidar[train]):
        singular = np.linalg.svd(normals,compute_uv=False)
        if singular[-1] < 1e-6 or singular[0]/singular[-1] > 30:
            raise ValueError("Board orientations are parallel or too poorly conditioned")
    u,_,vt = np.linalg.svd(lidar[train].T@camera[train])
    correction = np.eye(3)
    correction[-1,-1] = np.linalg.det(vt.T@u.T)
    rotation = vt.T@correction@u.T
    translation,_,_,_ = np.linalg.lstsq(camera[train],dl[train]-dc[train],rcond=None)
    predicted_normals = lidar@rotation.T
    angles = np.degrees(np.arccos(np.clip(np.sum(predicted_normals*camera,axis=1),-1,1)))
    offsets = dl-camera@translation-dc
    report = {"training_views":len(pairs)-2,"holdout_views":2,
              "normal_error_deg":angles.tolist(),"plane_offset_error_m":offsets.tolist(),
              "max_normal_error_deg":float(angles.max()),"max_offset_error_m":float(np.abs(offsets).max()),
              "calibration_verified":False,"navigation_ready":False}
    # Engineering rejection thresholds, not guarantees of physical accuracy.
    if angles.max() > 2 or np.abs(offsets).max() > .03 or np.linalg.norm(translation) > 2:
        raise ValueError("Plane correspondences or held-out board poses are inconsistent")
    transform = np.eye(4)
    transform[:3,:3] = rotation
    transform[:3,3] = translation
    return transform,report


def solve_session(directory, regions):
    root = Path(directory).resolve()
    if (not isinstance(regions,dict) or type(regions.get("schema")) is not int or regions.get("schema") != 1 or
            regions.get("lidar_length_unit") != "m" or
            not isinstance(regions.get("unit_evidence"),str) or not regions["unit_evidence"].strip()):
        raise ValueError("Independently checked LiDAR metre units and evidence required")
    intrinsics = json.loads((root/"camera-intrinsics.json").read_text(encoding="utf-8"))
    stability = np.asarray(intrinsics.get("focal_stability_relative_range",[]),dtype=float)
    if (intrinsics.get("quality_checks_passed") is not True or stability.shape != (2,) or
            not np.isfinite(stability).all() or np.any(stability < 0) or np.max(stability) > .05):
        raise ValueError("Camera intrinsics must pass the current coverage and stability checks first")
    # Lens intrinsics do not depend on the physical square size of a later
    # extrinsic-calibration target. Use the measured size of these paired views.
    square_m = regions.get("square_m")
    if type(square_m) not in (int,float) or not np.isfinite(square_m) or not .005 <= square_m <= .1:
        raise ValueError("Measured checkerboard square size required")
    selected = regions.get("views")
    if not isinstance(selected,list) or not 8 <= len(selected) <= 100:
        raise ValueError("Select eight or more manually identified paired board views")
    pairs, diagnostics, identities = [],[],set()
    frame = source = None
    for region in selected:
        name = region.get("view")
        if not isinstance(name,str) or not re.fullmatch(r"view-\d{3}",name) or name in identities:
            raise ValueError("Invalid or duplicate view identifier")
        identities.add(name)
        if region.get("same_board_confirmed") is not True:
            raise ValueError("Same physical board identification must be checked for every view")
        metadata = json.loads((root/(name+".json")).read_text(encoding="utf-8"))
        pair = metadata.get("lidar_pair")
        if (not isinstance(pair,dict) or pair.get("stationary") is not True or
                pair.get("frame_id") != metadata.get("frame_id") or
                not 0 <= pair.get("camera_age_s",-1) <= .5 or
                not 0 <= pair.get("lidar_age_s",-1) <= .5 or
                not np.isfinite(pair.get("receipt_skew_s",np.nan)) or abs(pair["receipt_skew_s"]) > .15):
            raise ValueError(name+": no valid acquisition-time camera/LiDAR pair")
        if tuple(metadata["image_size"]) != tuple(intrinsics["image_size"]) or metadata.get("square_m") != square_m:
            raise ValueError("Image dimensions or measured board size do not match calibration")
        if not pair.get("hub_instance") or not pair.get("lidar_frame"):
            raise ValueError("Sensor source identity missing")
        if source is not None and (source != pair["hub_instance"] or frame != pair["lidar_frame"]):
            raise ValueError("Dataset mixes sensor sources or coordinate frames")
        source,frame = pair["hub_instance"],pair["lidar_frame"]
        with np.load(root/(name+".cloud.npz"),allow_pickle=False) as archive:
            xyz = np.asarray(archive["xyz"])
        if xyz.ndim != 2 or xyz.shape[1] != 3 or len(xyz) > 1_000_000:
            raise ValueError("Invalid saved XYZ cloud")
        bounds = CameraLidarCalibration._array(region.get("bounds_xyz_m"),(6,))
        if np.any(bounds[:3] >= bounds[3:]):
            raise ValueError("Board region must have increasing XYZ bounds")
        mask = np.isfinite(xyz).all(axis=1) & np.all(xyz >= bounds[:3],axis=1) & np.all(xyz <= bounds[3:],axis=1)
        nl,dl,plane_stats = fit_plane(xyz[mask])
        nc,dc,reprojection = camera_plane(metadata["corners"],intrinsics,square_m)
        pairs.append({"camera_normal":nc,"camera_offset_m":dc,"lidar_normal":nl,"lidar_offset_m":dl})
        diagnostics.append({"view":name,"cloud_sequence":pair["cloud_sequence"],
                            **plane_stats,"camera_pose_rms_px":reprojection})
    transform,report = solve_planes(pairs)
    candidate = {"schema":1,"lidar_frame":frame,"length_unit":"m",
                 "image_size":intrinsics["image_size"],"K":intrinsics["K"],"distortion":intrinsics["distortion"],
                 "T_camera_from_lidar":transform.tolist(),
                 "evidence":str(root/"extrinsics-candidate-report.json")}
    CameraLidarCalibration(candidate)  # Same contract used by the observation pipeline.
    report.update(views=diagnostics,hub_instance=source,unit_evidence=regions["unit_evidence"],
                  candidate_only=True,hardware_synchronized=False)
    return candidate,report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("session")
    parser.add_argument("--regions",required=True,help="Manually identified board regions and metre evidence")
    args = parser.parse_args(argv)
    root = Path(args.session).resolve()
    regions_path = Path(args.regions)
    if regions_path.stat().st_size > 262144:
        raise ValueError("Oversized board region configuration")
    candidate,report = solve_session(root,json.loads(regions_path.read_text(encoding="utf-8")))
    (root/"extrinsics-candidate-report.json").write_text(json.dumps(report,indent=2,allow_nan=False),encoding="utf-8")
    path = root/"camera-lidar.candidate.json"
    path.write_text(json.dumps(candidate,indent=2,allow_nan=False),encoding="utf-8")
    print(path)
    print("Candidate only. Independently validate projected LiDAR points; navigation remains disabled.")


if __name__ == "__main__":
    main()
