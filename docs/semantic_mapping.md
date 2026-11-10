# Live person/bin recognition and camera/LiDAR association

Current hardware check: 2026-11-02. This extends the existing shared read-only
acquisition; it does not implement persistent SLAM or certify collision avoidance.

## Running services

The paired launcher starts simulation on 8770, the physical panel on 8771, and
the shared sensor hub on 8780. `./minimap.ps1` additionally starts the live
person/trash-bin view on http://127.0.0.1:8772/ using the installed local model.
`--all-objects` on `python -m as2_vlm.agent.minimap` restores general detection.
Recognition and LiDAR geometry run independently. Image observations carry their
original image, frame ID and age; they are not boxes over the latest camera frame.

The live local Qwen model returned person and trash-bin detections in approximately
3–7 seconds in the initial checks. A manually inspected image contained a partial
person and office bins, but the boxes were imprecise. This is not a detection
benchmark, tracking result or timely human collision detector.

## Camera/cloud pairing

The hub's read-only `/cloud?sequence=N` returns the bounded raw cloud with its
eight-byte sequence header. It returns HTTP 409 if a scan was replaced or is stale;
callers must not combine newer bytes with older PointCloud2 metadata. No arbitrary
filesystem path is accepted. Camera `/frame?sequence=N` keeps the same contract.

Cloud HTTP responses reuse the owner's validated immutable byte cache instead of
opening the WSL writer's live file again. Health now includes acquisition process
state and reader errors; native stderr persists in `logs/sensor-bridge.stderr.log`.
A native collector termination was observed and the process was restored during
development; its original cause was not captured, so long-run stability remains
unverified rather than being claimed fixed.

Before image inference, the mini-map waits up to 1.5 seconds for a fresh image and
captures a matching cloud. A pair requires valid state and LiDAR health, image and
cloud receive ages at most 0.5 seconds, state age at most 0.3 seconds, and receive
skew at most 0.15 seconds. The camera remains a 1 Hz sampled source. These are host
receipt times, **not exposure/scan synchronization**. Reported stationarity alone
does not establish physical synchronization or absence of motion within a scan.

Save one private sample without moving the robot:

```powershell
python -m as2_vlm.agent.capture_camera_lidar
```

It creates `data/camera-lidar/pair-.../{camera.jpg,cloud.npz,pair.json}`. Load the NPZ
with `allow_pickle=False`. The 2026-11-02 sample `pair-20261102-075417-06bab491`
contains an original 1920×1080 image and 88,412 points with 59.9 ms receive skew.
These private files are ignored by Git and are not uploaded.

## Camera/LiDAR calibration

`configs/camera-lidar.example.json` deliberately contains unknown values. Supply a
measured profile with `--calibration PATH`; incomplete/invalid profiles fail rather
than becoming an identity transform. Convention:

```text
p_camera_optical = T_camera_from_lidar @ [p_lidar_m, 1]
camera optical axes: x right, y down, z forward
K applies to the original image_size, not the 768-pixel model input
distortion = [k1, k2, p1, p2, k3] (Brown radial/tangential model)
```

The profile needs its sensor frame, metre units and a measurement evidence
reference. Rotation must be proper SO(3); image resolution must match exactly.
For strongly fisheye images a different calibrated projection model is needed;
do not force fisheye parameters into this pinhole model.

Projection rejects points behind the camera/outside the image. An eroded object
box selects supporting points; competing depth surfaces or diffuse depth leave
position unknown. A supported cluster produces a *candidate* point in the original
LiDAR frame, not a verified semantic identity or global-map coordinate. Association
requires reported stationarity and never enables navigation. Raw-Z geometry still
does not establish a gravity-aligned map. No factory calibration was found in the
local SDK; the installed videohub API exposes GetImageSample, not intrinsics.

## Independent obstacle interlock

Physical web execution now requires the shared LiDAR movement gate before starting
and before each native step. The native process receives STOP and remaining steps
are skipped when the Python watchdog detects an obstacle, missing observation,
stale input or monitor failure. The Python loop checks at approximately 0.15-second
intervals; native UI heartbeat/EOF handling remains the independent process-loss
fallback. The VLM is not queried on this path. Clearing the obstacle does not resume
the cancelled plan. The Stop button remains available when motion is refused.

`robot.lidar_body_calibration_path` must identify a measured profile based on
`configs/lidar-body.example.json`. Required convention is body x forward, y left,
z up; measured floor Z relative to the body; measured body half dimensions; proper
T_body_from_lidar; metre units and provenance. Missing calibration **refuses all
physical web movement**, including the legacy metre button. Planning/preview and
sensor viewing still work. Do not insert identity/guessed calibration to unlock it.

The experimental gate checks forward/reverse corridors and a rotation envelope,
keeps points at 5 cm–1.5 m above the supplied floor, excludes the supplied body
footprint and requires farther returns in coarse bearing sectors. Unknown sectors
and sparse/empty/floor-only scans do not count as clear space. Old cloud sequence
repeats cannot refresh the 0.3-second freshness watchdog. Body posture changes can
invalidate a fixed floor height; calibration and this envelope must be checked on
this installation. Coarse coverage is not a proof of traversability, and this
implementation does not protect against every small cable, glass surface, dropoff,
sensor blind spot or middleware/actuation delay.

No physical stopping-distance or obstacle-approach test has been performed for this
interlock. It is not a replacement for the independent operator remote stop. The
current real configuration is uncalibrated, so physical web movement is blocked.
No robot motion or network StopMove command was sent in these implementation checks.

## Remaining verification

The owner chose on-site calibration rather than factory files. The prepared
camera collection page and printable target are documented in
[on-site calibration](on_site_calibration.md). No real target views have yet been
collected, so no camera or body transform is claimed as calibrated.

1. Obtain or measure camera intrinsics/distortion, camera↔LiDAR and body↔LiDAR
   transforms, coordinate scale and timing; visually validate projected points
   on distinct targets and independently measure distances.
2. Validate person/bin precision and absence cases across multiple real frames;
   use a faster detector/tracker before real-time human interaction.
3. Collect an authorized moving dataset, implement and validate registration,
   persistent mapping and localization; current rolling density is not SLAM.
4. Validate obstacle envelopes, visibility, stopping distance and fault behavior
   with the operator's independent stop before any physical navigation trial.
