"""Camera/LiDAR association for observation, never an actuator or clear-path signal.

Transforms must be supplied from calibration. There is no identity/FOV fallback.
Receipt-time pairing is explicitly distinct from hardware synchronization.
"""
from dataclasses import dataclass
import json
from pathlib import Path

import numpy as np


@dataclass
class ObservationBundle:
    xyz: np.ndarray
    frame_id: int
    cloud_sequence: int
    lidar_frame: str
    receipt_skew_s: float
    camera_age_s: float
    lidar_age_s: float
    stationary: bool
    hub_instance: str | None = None

    def status(self):
        return {"frame_id": self.frame_id, "cloud_sequence": self.cloud_sequence,
                "lidar_frame": self.lidar_frame, "receipt_skew_s": self.receipt_skew_s,
                "camera_age_s": self.camera_age_s, "lidar_age_s": self.lidar_age_s,
                "stationary": self.stationary, "points": len(self.xyz),
                "hub_instance": self.hub_instance,
                "hardware_synchronized": False, "time_basis": "host_receipt"}


def capture_receipt_bundle(hub, snapshot, frame_id, elapsed=0):
    """One sequence-checked receipt pair, shared by recognition and calibration."""
    import urllib.error
    from .lidar_geometry import decode_xyz
    camera, lidar, state = (snapshot.get(key, {}) for key in ("camera", "lidar", "state"))
    if (camera.get("sequence") != frame_id or camera.get("age_s", -1) < 0 or
            lidar.get("age_s", -1) < 0 or state.get("error") != "0" or
            lidar.get("error") != 0 or not 0 <= state.get("age_s", -1)+elapsed <= .3):
        return None
    camera_age, lidar_age = camera["age_s"]+elapsed, lidar["age_s"]+elapsed
    if not 0 <= camera_age <= .5 or not 0 <= lidar_age <= .5 or abs(camera_age-lidar_age) > .15:
        return None
    try:
        raw = hub.request("/cloud?sequence="+str(lidar["cloud_sequence"]), binary=True)
    except urllib.error.HTTPError as exc:
        if exc.code == 409:
            return None
        raise
    xyz = decode_xyz(raw, lidar)
    if xyz is None:
        return None
    velocity = np.asarray(state.get("velocity", []), dtype=float)
    stationary = velocity.shape == (3,) and np.isfinite(velocity).all() and np.linalg.norm(velocity) <= .03
    return ObservationBundle(xyz, frame_id, lidar["cloud_sequence"], lidar["frame"],
        camera_age-lidar_age, camera_age, lidar_age, bool(stationary), snapshot.get("instance"))


class CameraLidarCalibration:
    def __init__(self, profile):
        required = {"schema", "lidar_frame", "length_unit", "image_size", "K",
                    "distortion", "T_camera_from_lidar", "evidence"}
        if (not isinstance(profile, dict) or set(profile) != required or
                type(profile["schema"]) is not int or profile["schema"] != 1):
            raise ValueError("Unsupported camera/LiDAR calibration schema")
        if profile["length_unit"] != "m":
            raise ValueError("Measured metre units required")
        if not isinstance(profile["lidar_frame"], str) or not profile["lidar_frame"].strip():
            raise ValueError("Explicit LiDAR frame required")
        if not isinstance(profile["evidence"], str) or not profile["evidence"].strip():
            raise ValueError("Calibration evidence reference required")
        self.frame = profile["lidar_frame"]
        self.evidence = profile["evidence"]
        size = profile["image_size"]
        if not isinstance(size, list) or len(size) != 2 or any(type(v) is not int or not 0 < v <= 8192 for v in size):
            raise ValueError("Calibration image_size must be [width, height]")
        self.width, self.height = size
        self.K = self._array(profile["K"], (3, 3))
        self.distortion = self._array(profile["distortion"], (5,))
        self.transform = self._array(profile["T_camera_from_lidar"], (4, 4))
        if (self.K[0, 0] <= 0 or self.K[1, 1] <= 0 or
                not np.allclose(self.K[2], [0, 0, 1], atol=1e-8, rtol=0) or
                not np.isclose(self.K[1, 0], 0, atol=1e-8)):
            raise ValueError("Invalid pinhole camera matrix")
        rotation = self.transform[:3, :3]
        if (not np.allclose(self.transform[3], [0, 0, 0, 1], atol=1e-8, rtol=0) or
                not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-5, rtol=0) or
                not np.isclose(np.linalg.det(rotation), 1, atol=1e-5, rtol=0)):
            raise ValueError("Transform must contain a proper rotation and homogeneous last row")

    @staticmethod
    def _array(value, shape):
        try:
            raw = np.asarray(value, dtype=object)
            if raw.shape != shape or any(type(v) not in (int, float) for v in raw.flat):
                raise ValueError("Calibration coefficients must be numeric with the declared shape")
            array = np.asarray(value, dtype=float)
            if not np.isfinite(array).all():
                raise ValueError("Calibration coefficients must be finite")
            return array
        except (TypeError, ValueError) as exc:
            raise ValueError("Invalid calibration matrix: " + str(exc)) from exc

    @classmethod
    def load(cls, path):
        path = Path(path)
        if path.stat().st_size > 65536:
            raise ValueError("Oversized calibration profile")
        return cls(json.loads(path.read_text(encoding="utf-8")))

    def project(self, xyz, image_size):
        if tuple(image_size) != (self.width, self.height):
            raise ValueError("Image resolution differs from calibration; do not rescale intrinsics implicitly")
        xyz = np.asarray(xyz)
        if xyz.ndim != 2 or xyz.shape[1] != 3 or len(xyz) > 1_000_000:
            raise ValueError("Invalid XYZ cloud")
        camera = xyz @ self.transform[:3, :3].T + self.transform[:3, 3]
        valid = np.isfinite(camera).all(axis=1) & (camera[:, 2] > .05)
        indices = np.flatnonzero(valid)
        camera = camera[valid]
        x, y = camera[:, 0] / camera[:, 2], camera[:, 1] / camera[:, 2]
        k1, k2, p1, p2, k3 = self.distortion
        r2 = x*x + y*y
        radial = 1 + k1*r2 + k2*r2*r2 + k3*r2*r2*r2
        xd = x*radial + 2*p1*x*y + p2*(r2 + 2*x*x)
        yd = y*radial + p1*(r2 + 2*y*y) + 2*p2*x*y
        u = self.K[0, 0]*xd + self.K[0, 1]*yd + self.K[0, 2]
        v = self.K[1, 1]*yd + self.K[1, 2]
        visible = np.isfinite(u) & np.isfinite(v) & (u >= 0) & (v >= 0) & (u < self.width) & (v < self.height)
        return np.column_stack((u[visible]/self.width, v[visible]/self.height)), camera[visible, 2], indices[visible]


def associate(detections, bundle, calibration, image_size):
    """Return candidate LiDAR locations; an image box is not a verified 3D identity."""
    base = {"navigation_ready": False, "association_verified": False,
            "hardware_synchronized": False, "objects": []}
    if calibration is None:
        return {**base, "status": "calibration_required"}
    if bundle is None:
        return {**base, "status": "paired_cloud_unavailable"}
    base["pair"] = bundle.status()
    if (not bundle.stationary or not np.isfinite(bundle.receipt_skew_s) or
            abs(bundle.receipt_skew_s) > .15 or
            not 0 <= bundle.camera_age_s <= .5 or not 0 <= bundle.lidar_age_s <= .5):
        return {**base, "status": "pair_rejected_motion_or_timing"}
    if bundle.lidar_frame != calibration.frame:
        return {**base, "status": "calibration_frame_mismatch"}
    uv, depth, indices = calibration.project(bundle.xyz, image_size)
    objects = []
    for detection in detections:
        box = detection.bbox
        # Erode the box to reduce incidental background at its edges.
        dx, dy = .15*(box[2]-box[0]), .15*(box[3]-box[1])
        inside = ((uv[:, 0] >= box[0]+dx) & (uv[:, 0] <= box[2]-dx) &
                  (uv[:, 1] >= box[1]+dy) & (uv[:, 1] <= box[3]-dy))
        chosen = np.flatnonzero(inside)
        entry = {"label": detection.label, "bbox": list(box), "support_points": len(chosen),
                 "position_lidar_m": None, "camera_depth_m": None, "status": "insufficient_points"}
        if len(chosen) >= 8:
            order = chosen[np.argsort(depth[chosen])]
            clusters = np.split(order, np.flatnonzero(np.diff(depth[order]) > .25)+1)
            clusters = sorted((c for c in clusters if len(c) >= 8), key=len, reverse=True)
            if not clusters:
                entry["status"] = "insufficient_depth_support"
            elif len(clusters) > 1 and len(clusters[1]) >= .5*len(clusters[0]):
                entry["status"] = "ambiguous_depth"
            else:
                cluster = clusters[0]
                spread = float(np.ptp(depth[cluster]))
                entry["depth_spread_m"] = spread
                if spread > .8:
                    entry["status"] = "diffuse_depth"
                else:
                    entry.update(status="candidate", support_points=len(cluster),
                                 position_lidar_m=np.median(bundle.xyz[indices[cluster]], axis=0).tolist(),
                                 camera_depth_m=float(np.median(depth[cluster])))
        objects.append(entry)
    return {**base, "status": "candidate_associations", "objects": objects,
            "frame": calibration.frame, "evidence": calibration.evidence,
            "time_basis": "host_receipt_stationary_only"}
