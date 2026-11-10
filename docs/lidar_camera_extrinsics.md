# Camera/LiDAR plane calibration candidate

This offline tool continues the on-site camera calibration. It estimates a
camera-from-LiDAR rigid transform from multiple observations of the **same physical
board plane**, with explicit rejection checks. It does not identify the board
automatically, install a transform, change the body obstacle profile or command
the robot. Numeric checks do not certify a physical calibration.

## Current real data

On 2026-11-02 the owner collected 42 camera views with measured 22 mm squares.
The current camera fit passed coverage and four-fold focal stability checks:
training RMS 0.736 px, held-out errors 0.591/1.526 px, focal relative ranges
3.62%/3.41%. Twenty-two views include acquisition-time LiDAR clouds. The original
17 camera-only views remain useful for intrinsics and are not given invented
LiDAR pairs. Source: the private session
`data/camera-calibration/session-20261102-080555-2df7da`.

The initial exploratory plane extraction on three actual clouds found unlabelled
surfaces but did not establish which was the displayed laptop screen/board. These
proposals stay private in `lidar-plane-proposals.json`; they are not calibration
correspondences. LiDAR metre scale still needs independent evidence. There is no
real camera/LiDAR transform result yet.

## Inputs

Use an intrinsics file produced by the current camera calibration tool and at
least eight distinct paired board views, with different board orientations. The
last two views are withheld from fitting the transform. Each view must contain:

- Original 54 corner observations at the calibrated image resolution and the same
  measured square size.
- `lidar_pair` metadata: matching image ID, stationary observation, source/frame
  identity, fresh data and receipt skew no greater than 0.15 seconds.
- The exact acquisition-time `view-NNN.cloud.npz`, loaded without pickle.
- An operator-identified bounded XYZ region containing the same board surface.

Start a private region file from `configs/lidar-camera-regions.example.json`.
Set independently checked `lidar_length_unit` to `m`, write the measurement
reference in `unit_evidence`, and list the actual selected views:

```json
{
  "view": "view-021",
  "same_board_confirmed": true,
  "bounds_xyz_m": ["measured xmin", "measured ymin", "measured zmin",
                   "measured xmax", "measured ymax", "measured zmax"]
}
```

The quoted bounds above are explanatory placeholders, not runnable values. Do not
mark a proposed plane as confirmed simply because it is planar. Check its location,
appearance and change with the target across views. Both sensors must observe the
same side of the board. A screen can be optically clear to the camera while
providing insufficient or misleading LiDAR returns; use an appropriate larger
rigid matte target if the board surface cannot be identified reliably.

## Solver and rejection rules

The camera board pose comes from [OpenCV solvePnP](https://docs.opencv.org/4.12.0/d5/d1f/calib3d_solvePnP.html),
which returns object-to-camera rotation/translation. The code constructs the plane
in camera optical coordinates and checks its corner reprojection error. For the
LiDAR region a deterministic truncated-residual plane fit and robust trimming
reject non-planar, sparse or nearly collinear samples. This implements a simple
plane-constraint solver, not a reproduction of the optimization method in
[Verma et al.](https://arxiv.org/abs/1904.12433).

With unit normals facing their respective sensors and plane convention
`n.dot(p) + d = 0`, solve:

```text
n_camera = R_camera_from_lidar @ n_lidar
n_camera.dot(t_camera_from_lidar) = d_lidar - d_camera
```

Rotation uses a proper-rotation SVD solution; translation uses least squares.
Parallel/poorly conditioned board orientations are refused. Every training and
held-out plane must agree within 2 degrees and 3 cm. These are engineering
rejection thresholds, not a measured accuracy specification. Individual board
camera poses must fit within 2 px. The plane region needs at least 30 finite
points, 80% plane support and a non-degenerate 2D spatial span.

Run in the optional calibration environment:

```powershell
.runtime/calibration-python/Scripts/python.exe -m as2_vlm.agent.lidar_camera_extrinsics `
  data/camera-calibration/SESSION --regions data/camera-calibration/SESSION/board-regions.json
```

On success the tool writes only `camera-lidar.candidate.json` and
`extrinsics-candidate-report.json` in that private session. The candidate satisfies
the projection profile format; the report keeps `calibration_verified=false`,
`hardware_synchronized=false`, and `navigation_ready=false`.

Before applying it, independently inspect projected LiDAR returns against image
features over additional poses and measured distances. This camera transform does
not determine body axes, floor height, robot footprint, stopping distance or global
localization. None of the physical web motion requirements are bypassed.

Verification: synthetic tests recover a known rigid transform and its direction,
reject wrong held-out matches and parallel poses, and exercise saved camera/cloud
files through the complete candidate solver. They do not establish successful
calibration of the real robot.
