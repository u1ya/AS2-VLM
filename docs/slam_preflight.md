# Read-Only SLAM Sensor Preflight

Current acquisition now uses one shared hub for panels and raw recording. The
load issue investigated below improved with shared acquisition: two panels plus
recording delivered source-time 10 Hz clouds (max interval 100.25 ms). See
[Shared sensors](shared_sensors.md) for the current launch/recording workflow.
The older standalone and panel-isolation commands below are diagnostic history,
not the default workflow when the shared hub is running.

This stage records existing DDS messages and audits them offline. It does not
start a SLAM engine, estimate calibration, publish robot commands or enable navigation.
The live XY density view remains an uncalibrated sensor-frame projection.

## Verified Capture: 2026-10-25

An eight-second stationary capture on `eth0` produced 73 clouds, 1042 IMU samples
and 148 sport-state samples, using 159,582,786 raw cloud bytes. The recorder reported
no cloud callback queue drops, I/O errors or pending clouds at shutdown. This does
not prove absence of DDS/network loss.

- LiDAR headers: approximately 10 Hz; frame `hesai_lidar`.
- Cloud fields: XYZ float32, ring uint16 and timestamp float64.
- Observed ring identifiers: 0 through 127. The user's specification is
  "Industrial-grade 64~128-line LiDAR"; neither establishes the exact model.
- Per-point time spans approximately 99.778 ms if interpreted as seconds;
  point minima match cloud headers in this capture. Units remain unverified.
- IMU delivered source-time average: 142.21 Hz; median interval 5.099 ms;
  maximum interval 55.001 ms. The cause of gaps is not yet identified.
- LiDAR/IMU source intervals overlap. Median nearest-header difference is
  0.223 ms, but this is not proof of hardware synchronization.
- Assuming point times are source seconds, 72 of 73 scans have IMU samples
  bracketing their full interval. Discard the unbracketed scan for estimation.
- No usable unit orientation quaternion was recorded. The inspected sample has
  an all-zero quaternion. An estimator must support raw six-axis IMU input.
- Mean acceleration is approximately [-0.806, 9.521, -1.207] in raw units,
  dominated by +Y. Do not assume +Z gravity or identity sensor extrinsics.
- Sport-state source stamps are zero; host receipt times cannot replace
  hardware synchronization. Reported odometry is consistent with stationary capture.

The static acceleration observation alone cannot establish the complete rotation
or translation between sensors. It does not calibrate LiDAR axes, camera axes,
units, gravity sign conventions or the body frame. Existing raw-Z view filtering
must not be interpreted as calibrated height above ground.

## Repeat Without Motion

Build in Ubuntu/WSL using the existing SDK installation:

```bash
cd "/mnt/c/Users/Ulugb/Desktop/SLAI VLM"
cmake -S robot/bridge_cpp -B build/as2-probe -DCMAKE_PREFIX_PATH="$PWD/.runtime/unitree-sdk-install"
cmake --build build/as2-probe --target as2_record_sensors --parallel 2
./build/as2-probe/as2_record_sensors eth0 recordings/slam-preflight-NEW 8 256
```

Use the currently verified interface, and a new output directory. The duration
is limited to 3-30 seconds, with a 512 MiB raw-cloud cap and a free-space check.
The helper subscribes only to cloud, IMU and sport-state topics. No camera request
or sensor/robot control API is used.

The optional fourth argument selects the IMU SDK callback queue depth (1-4096,
default 256). Cloud and odometry callback queues use 16 and 64 entries. These
are application-side SDK queues, not sensor settings or DDS reliability QoS.
Host monotonic timestamps measure callback execution after that queue, not packet
arrival; queued callbacks can add latency. Preserve source stamps for estimation.

From the project directory in the configured Python environment:

```powershell
python -m as2_vlm.agent.sensor_audit recordings/slam-preflight-NEW --replay-check
```

The audit writes `audit.json` alongside the private recording, including SHA-256
hashes and timing/coverage warnings. Replay-check means every cloud is decoded
again offline, not that SLAM has replayed successfully. Exit code zero means the
audit completed; inspect `slam_ready`, blockers and warnings separately.

Recordings and raw indoor geometry stay in ignored `recordings/`; do not commit
or upload them by default. JSONL retains source stamps and host monotonic receipt
times. Each cloud binary has an eight-byte little-endian sequence prefix followed
by PointCloud2 payload; the JSONL records its exact layout and matching sequence.

## Gate Before Actual SLAM

1. Obtain the exact sensor model, firmware/driver configuration and calibrated
   LiDAR-to-IMU transform from the hardware documentation or supplier. Do not
   select a Livox decoder solely from the DDS topic name.
2. Investigate IMU gaps using another stationary recording and driver/transport
   diagnostics. Cloud queue counters do not measure IMU network loss.
3. Verify point-time units, stamp semantics, accelerometer/gyro units and axes.
   Exclude scans without IMU bracketing and define an estimator-specific gap policy.
4. Adapt the verified stream to a proven LiDAR-inertial estimator, then evaluate
   it on a separately authorized moving-sensor recording, entirely offline first.
5. Only after trajectory/geometry validation, add persistent mapping and camera
   extrinsics, then project visual detections into that map for semantic labels.

Motion remains disabled throughout this preflight. No SLAM or semantic-map
readiness is inferred from an apparently plausible static point-cloud image.

Verification: 123 Python tests and nine C++ CTest cases passed. Python tests
require permission for temporary files and local test HTTP servers on this host;
robot command clients are mocked in those tests. All 73 captured clouds also
passed the offline decode check.

## Queue Investigation: 2026-10-25

The pinned SDK's `DdsReaderListener` in `dds_entity.hpp` uses a bounded callback
queue and evicts the earliest message when full. The initial recorder used a
depth of one for each subscription. Sequential stationary eight-second captures
gave the following results (not a controlled timing benchmark):

| Capture | IMU queue | IMU samples | Source-time average Hz | Maximum IMU interval ms | Intervals over 20 ms | Clouds | Maximum cloud interval ms |
| --- | --- | --- | --- | --- | --- | --- | --- |
| `slam-imu-q1-20261025-02` | 1 | 1088 | 147.19 | 49.900 | 47 | 69 | 299.980 |
| `slam-imu-q256-20261025-02` | 256 | 1478 | 199.73 | 10.021 | 0 | 65 | 300.107 |
| `slam-buffered-20261025-03` | 256 | 1465 | 200.00 | 5.150 | 0 | 66 | 400.000 |

Only IMU depth changed between the first two captures. The third also used
cloud depth 16 and odometry depth 64. All ran alongside the existing read-only
dashboard. No robot or sensor settings were changed. Each capture had one scan
without full IMU bracketing under the candidate seconds interpretation. All
200 clouds across these captures passed offline decoding; raw data stays private.

This strongly supports callback buffering as a contributor to IMU loss in this
setup. It does not prove the physical sensor is lossless or establish hardware
synchronization. Cloud intervals remain irregular despite the larger queue;
their cause is unresolved (publisher, transport, middleware and host load must
be distinguished). No cloud application queue drops were reported, but that
counter does not observe SDK queue eviction or packets lost before the callback.

The audit now reports source-period p95, intervals over 20 ms, and a warning
when the maximum cloud interval exceeds 1.5 times its median. The 20 ms count is
a diagnostic threshold, not an estimator acceptance rule; ordinary 10 Hz clouds
also exceed 20 ms.

No calibrated extrinsics were found in the local SDK source. The public Unitree
support page could not be retrieved by the documentation tool during this check.
The remaining supplier request is precise: actual LiDAR model, units/axes,
point-timestamp semantics, and the calibrated LiDAR-to-IMU rotation and translation,
including transform direction. Do not substitute identity extrinsics or infer
them from the generic 64-128-line specification.

## Panel Load Isolation: 2026-10-25

Both 8770 and 8771 were actually running live `cpp_readonly` adapters, despite
8771 having originally been launched with `--sim`. Runtime mode switching had
created two active `as2_sensor_stream` processes. Check `/api/state` rather than
inferring the current mode from the original launch command or port number.

The diagnostic temporarily switched both idle read-only panels to simulation
using `/api/mode`, verified that no `as2_sensor_stream` process remained, recorded
eight seconds, then restored both panels to live read-only mode in a `finally`
block. It did not change robot or sensor settings. A second eight-second capture
was taken after restoration with the same recorder binary and queue depths.

| Panels | Clouds | Source-time cloud Hz | Cloud interval p95 ms | Maximum cloud interval ms | IMU Hz | Maximum IMU interval ms |
| --- | --- | --- | --- | --- | --- | --- |
| Both sensor subscriptions off | 74 | 10.000 | 100.172 | 100.228 | 200.00 | 5.185 |
| Both restored/live | 64 | 8.750 | 280.031 | 400.204 | 199.73 | 9.829 |

Private datasets: `recordings/slam-panels-off-20261025-04` and
`recordings/slam-panels-on-20261025-04`. All 138 clouds replay-decoded successfully.
The off recording includes `load-experiment.json` with original panel modes,
capture exit status and an empty restoration-error list. Each capture still has
one scan without complete IMU bracketing, so the startup exclusion remains.

This is evidence that concurrent dashboard sensor activity contributes to the
observed cloud gaps in this setup. It does not isolate camera traffic, additional
DDS readers, network resources, WSL scheduling, disk I/O or geometry computation.
It is not evidence that the LiDAR hardware itself is faulty, nor a proof of
lossless operation beyond this short test. Current panels remain restored, so
their production load issue has not yet been fixed.

### Repeat the Isolation Test

From PowerShell in the workspace (with the already built recorder):

```powershell
./measure-sensor-load.ps1 -RecordingName slam-panels-off-NEW -Seconds 8
python -m as2_vlm.agent.sensor_audit recordings/slam-panels-off-NEW --replay-check
```

The script checks both panels before making changes. Only idle simulations or
verified unarmed `as2_cpp_readonly` live adapters are accepted; native motion
transports are rejected. Existing recording directories are rejected. Original
live modes are restored even if capture fails; restoration errors are reported
and recorded, not silently treated as success. After an external forced process
termination, verify panel modes manually because `finally` may not execute.

The script's syntax and existing-directory rejection were checked, and its normal
capture/restoration path was exercised on the real read-only streams. Existing
123 Python and nine C++ results remain applicable; no Python/C++ source changed
in this load-isolation step.

The next architectural change should avoid independent full-rate sensor readers
for every panel and recorder: a shared acquisition process can feed bounded
recording queues and downsampled UI views. Validate that change with the same
load comparison before enabling any SLAM consumer. Calibration remains a
separate prerequisite.
