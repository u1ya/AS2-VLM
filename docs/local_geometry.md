# Real Local LiDAR Geometry

Implemented and read-only tested on AS2 EDU on 2026-10-25.
This is a real-data **stationary return-density projection**, not SLAM, a semantic
map, a free-space map, or a navigation-ready occupancy grid.

## Run and Inspect

From Ubuntu in the project root, rebuild `as2_sensor_stream`, or run the existing
`bash deploy/build_state_probe.sh` to configure and build all helpers. Windows
Python needs the `agent` extra, now including NumPy:

```powershell
python -m pip install -e '.[agent]'
.\sensor-hub.ps1  # keep running in one terminal
# In another terminal:
.\agent.ps1
```

On this machine the dependencies are already installed. The live read-only panel
is http://127.0.0.1:8770/. Refresh a previously open page after updating code.
The simulated apartment at port 8771 is not a map of the real room.

The live panel now shows **Local LiDAR map** instead of fake/empty semantic geometry.
The XY projection is in the published `hesai_lidar` coordinate frame, not the robot
body frame or camera view. Sensor origin is marked with a cross, not a robot pose.
No inferred room labels, map-based movement or camera registration are introduced.

`Z min` and `Z max` select the displayed height slice in raw sensor coordinates.
Defaults: -0.1 to 1.8. This is a visualization filter, not a calibrated ground or
obstacle classifier. Axes, extrinsics and metric scale still require verification;
the UI explicitly labels sensor units as unverified. Default XY range is +/-16
with 0.1-unit cells (320 x 320). Dark cells mean unknown/no returns, NOT clear floor.

## Data Path

1. C++ subscribes to `rt/utlidar/cloud_livox_mid360` and keeps the latest cloud.
2. At about 4-5 updates/sec, the worker atomically replaces a private binary file
   alongside the private JPEG in `.runtime/agent-sensors/`. A little-endian uint64
   sequence header binds the file to the JSON field-layout metadata; mismatches
   are skipped rather than interpreted with the wrong layout.
3. Python decodes XYZ using NumPy structured dtypes and declared field offsets,
   row padding and byte order. Float32 and float64 are supported. Invalid layouts,
   oversized frames and nonfinite returns are rejected/filtered.
4. NumPy histogram2d projects the selected height slice. A bounded rolling window
   contains at most 30 processed scans and at most 5 seconds of history.
5. Pillow renders a PNG and the live canvas draws it with sensor-frame axes.

The cloud remains around 10 Hz at the source; processing the latest cloud at a
lower rate is intentional. The pipeline does not queue old clouds indefinitely.
Raw binary files are bounded to 16 MiB; the current cloud is approximately 2.1 MiB.
Only bounded latest sensor data are exchanged on disk, not a continuous recording.

## Accumulation and Failure Handling

The pose is used only to check stability, never to invent a LiDAR-to-world transform.
Accumulation resets on >0.02 positional change or >0.03 rad orientation change
relative to its anchor, a source-frame/sequence discontinuity, or a data gap >1 s.
Reported body speed norm >0.03 uses one current scan only; it does not merge a
moving sensor's scans without registration. These numerical thresholds are
diagnostic settings, not motion safety limits or validated odometry accuracy.

Cloud age must be <=0.5 s, odometry age <=0.3 s, and reported state/lidar errors zero
before ingestion. The view is marked stale after 1 second without a fresh accepted
cloud; the rolling evidence expires after 5 seconds. Saving a stale view is refused.
Dynamic objects can still leave short-lived traces within the stationary window.

The default live adapter stays read-only and `get_scan()` remains unavailable:
this uncalibrated projection is NOT passed to the obstacle controller or planner.
VLM inference remains disabled in the live sensor profile. The camera still works.

## Save a Snapshot

Use **Save Snapshot** in the live panel. The server writes only to ignored
`data/geometry/`, with a unique base name and these side-by-side files:

- `.png`: density image, top-down +Y upward; no claim of known free space.
- `.npz`: NumPy `return_counts[x_index, y_index]`, load with `allow_pickle=False`.
- `.json`: frame, bounds, slice, resolution, receive freshness, window size and
  explicit `slam=false`, `navigation_ready=false`, `units_verified=false` flags.

Private initial smoke snapshot:
`local-lidar-20261025-175018-00f623fa`, 1505 return cells over 3 processed scans,
163378 in-range returns. A later HTTP snapshot had 1780 return cells over
19 scans. Values fluctuate with the room and the rolling window.
The UI Save Snapshot button was exercised successfully as well.
These real indoor observations are not committed or published.
Verification: all 116 Python tests and 7 C++ tests passed. Tests include packed
mixed-width XYZ, both byte orders, row padding, sequence mismatch, movement/frame
resets, stale-data expiry, filter changes and HTTP/private-file export.

Read-only GET endpoints: `/api/geometry` and `/api/map` (field `geometry`).
Same-origin POST endpoints: `/api/geometry/filter` with numeric `z_min`, `z_max`
and `/api/geometry/export` with `{}`. There is no client-controlled export path.
These endpoints do not invoke any robot motion or sensor configuration API.

## Next Milestone

Read-only recording and the first offline timing audit are now available:
[SLAM sensor preflight](slam_preflight.md). The measured IMU acceleration is
dominated by +Y, so this raw-Z slice is not a calibrated ground-height slice.

For a persistent metric map while the robot moves, first identify the actual
LiDAR and verify its frame axes, scale, IMU timing and transforms. Then integrate
an existing LiDAR/IMU SLAM implementation, record/replay synchronized private data,
and validate registration and loop closure before using the map for navigation.
This histogram layer is deliberately not a home-grown substitute for SLAM.
Camera-to-LiDAR calibration and validated semantic detections come afterward.
