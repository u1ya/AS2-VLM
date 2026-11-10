# Shared Read-Only Sensor Acquisition

The real panels now use one loopback sensor service instead of each launching a
DDS process. The service owns one `as2_sensor_stream` C++ process. That process
subscribes once to cloud, IMU, sport state and LiDAR health, and requests camera
JPEGs about once per second. It has no sport client, motion publisher or arming API.

```text
AS2 DDS / videohub
        |
one as2_sensor_stream
        +-- bounded raw recording queues --> private recordings/
        |
        +-- latest snapshots (~5 Hz) --> Python hub :8780
                                          +-- geometry computed once
                                          +-- panel :8770
                                          +-- panel :8771
```

Panels exchange metadata, rendered geometry and sequence-bound JPEGs over numeric
loopback HTTP. They do not receive full-rate raw clouds or run their own geometry
histograms. Geometry filtering is shared: changing its Z slice affects all panels.
Snapshot export also runs in the hub and writes to its fixed `data/geometry/` path.

## Start After a Restart

Build the current C++ stream in Ubuntu/WSL using the existing SDK install:

```bash
cd "/mnt/c/Users/Ulugb/Desktop/SLAI VLM"
cmake -S robot/bridge_cpp -B build/as2-probe -DCMAKE_PREFIX_PATH="$PWD/.runtime/unitree-sdk-install"
cmake --build build/as2-probe --target as2_sensor_stream --parallel 2
```

Use the project's Python environment with the agent dependencies installed.
In PowerShell, keep the hub running in its own terminal:

```powershell
./sensor-hub.ps1
```

Then start each desired panel in a separate terminal:

```powershell
./agent.ps1 -Port 8770
```

```powershell
./agent.ps1 -Port 8771
```

For a simulator instead, use `./agent.ps1 -Sim -Port 8771`; simulation alone needs
no hub. Runtime mode can change, so inspect `/api/state`, not only launch arguments.
Current real panels both run read-only and use the same hub instance identifier.

`robot.sensor_hub_url` defaults to `http://127.0.0.1:8780`. Panels verify the hub's
interface, DDS domain, WSL distribution and binary path before attaching. They do
not start fallback DDS readers if it is missing. A profile mismatch fails startup.
An empty URL explicitly opts into the legacy dedicated bridge for diagnostics;
do not use that mode concurrently with the hub during load-sensitive captures.
The direct owner is an internal component of the hub, not an actuation transport.

## Record Without Disconnecting Panels

```powershell
./record-shared-sensors.ps1 -Name slam-session-NEW -Seconds 8
```

Use a new alphanumeric/dash/underscore name. The script asks the already running
hub to record, waits for completion, then runs the offline audit and replay decode.
It does not start another DDS subscriber. The HTTP acknowledgement only confirms
submission: inspect `recording.finished`, `recording.ok`, errors and the private
manifest. Exit zero from the audit is not SLAM readiness.

Clouds, raw IMU and odometry are recorded from the acquisition callbacks, not from
the 5 Hz UI snapshots. A separate C++ writer consumes bounded queues (16 pending
clouds, at most 10,000 pending rows per IMU/odometry queue; one bounded batch may
also be in flight). The duration is 3-30 seconds, raw cloud cap 512 MiB, with a
600 MiB free-space preflight. Overload drops and I/O errors are recorded in the
manifest; neither the SDK queues nor network transport are certified lossless.
The shared format is compatible with `sensor_audit.py` and the existing offline
PointCloud2 decoder. Names resolve only within ignored `recordings/`.

Startup/end scans may lack full IMU bracketing. Exclude them using the audit; no
estimator should invent missing samples. Source stamps and callback receipt times
are retained separately. Units, clock alignment and extrinsics remain unverified.

Legacy `as2_record_sensors` remains for isolated diagnostic captures. Do not run it
alongside the hub when measuring single-acquisition performance. The old
`measure-sensor-load.ps1` now rejects shared panels: switching panel modes no longer
stops acquisition, so that former isolation method is no longer applicable.

## Lifecycle and Access

- Closing or switching one panel does not stop the hub or another panel.
- Stop the hub separately with Ctrl+C in its terminal. It closes its C++ owner
  with a local stdin quit message, not a robot stop/motion command.
- After hub loss, data expires rather than remaining connected forever. After a
  new hub instance starts, reconnect the panels to verify its profile again.
- The hub binds only to 127.0.0.1. POST requires JSON and matching Origin when
  present; Host checks reject rebinding to another hostname. No CORS is enabled.
  This is a trusted-local-machine interface, not an authenticated remote service.
- Only bounded recording, shared geometry filtering and fixed-path export are
  writable. There are no Move, StopMove, StandDown, Damp or arm endpoints.

Binding the hub port happens before starting DDS, preventing a second default-port
hub from launching a reader. Do not deliberately run independent hubs on different
ports against the same robot unless performing an explicit load diagnostic.

## Real Read-Only Verification: 2026-10-25

Both live panels attached to the same instance, with `motion_enabled=false` and
`armed=false`. WSL showed exactly one `as2_sensor_stream` process during recording.
Switching 8771 to simulation and back left the hub instance and 8770 connection
intact. Both panels were restored to shared real read-only mode afterward.

Capture `slam-shared-20261025-05` requested eight host-clock seconds and saved 87
clouds, 1740 IMU and 174 odometry rows. The source-stamp intervals yielded:

| Metric | Shared acquisition with two panels |
| --- | --- |
| Cloud source-time rate | 9.9998 Hz |
| Cloud interval p95 / maximum | 100.160 / 100.250 ms |
| IMU source-time rate | 199.997 Hz |
| IMU maximum interval | 5.145 ms |
| Reported application cloud / row drops | 0 / 0 |
| I/O errors / cap reached | No / No |
| Scans lacking full IMU bracketing | 2 |

All 87 clouds replay-decoded. The longer source interval than requested host
duration does not establish synchronized clocks; callback queue backlog and timing
semantics still require verification. Do not infer rates from counts/duration
without distinguishing these clocks. A second three-second end-to-end launcher
test (`slam-shared-cli-20261025-06`) saved and replay-decoded 28 clouds, with cloud
source rate 10.0004 Hz and maximum interval 100.196 ms. These short stationary
tests support improved acquisition under panel load, not validated moving SLAM.

Python regression coverage includes multiple panel clients, profile mismatch,
source staleness, hub restart, no DDS fallback, command rejection, unsafe recording
names, competing recordings and fixed-path export. The C++ shared writer has an
autonomous recording/completion test without DDS or any robot connection.

Calibration, semantic projection, SLAM and physical navigation remain disabled.
Final verification: 133 Python tests and ten C++ CTest cases passed. Both live
panels returned the same camera JPEG and geometry, with no sensor-client errors,
`motion_enabled=false`, `armed=false` and `slam=false`. Shell launchers passed
PowerShell syntax checks; the shared recording launcher completed on real data.
See [SLAM preflight](slam_preflight.md) for remaining gates.
