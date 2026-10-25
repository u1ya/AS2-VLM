# AS2 agent stack: semantic map, goal planner, recovery loop, dashboard

`as2_vlm/agent/` adds goal-directed autonomy on top of the dry-run MVP. Everything runs today against
`MockAs2Adapter`; verified real sensor access uses the read-only C++ transport described below. The existing
`as2_vlm` CLI, parser and demo are unchanged.

## Verified AS2 sensor setup (2026-10-25)

The current checkout merges SLAI-VLM/main at 629a9e6 with the local AS2 hardware
work. `robot.transport=cpp_readonly` is the real-mode default and has no movement
API. It starts `as2_sensor_stream` through WSL on Windows (directly on Linux).
The stream subscribes to sportmodestate and LiDAR cloud/state, and requests JPEGs
from videohub. Closing the adapter sends only a process-quit message over stdin;
it does not send StopMove, StandDown or Damp to the robot.

PowerShell from the project root:

```powershell
.\agent.ps1          # real sensors, commands disabled, http://127.0.0.1:8770
.\agent.ps1 -Sim     # simulated goals, synthetic sensors, no robot
```

Stop the first server before using the same port, or pass `-Port 8771`.
Install `python -m pip install -e '.[agent]'` if pydantic/Pillow are missing.
Rebuild the C++ targets with `bash deploy/build_state_probe.sh` in Ubuntu.
Check `ip route get 192.168.123.161`; the verified robot interface is currently eth0.

The versioned `configs/agent.as2-sensors.json` selects the actual setup:
1920x1080 JPEG, `videohub`, cloud `rt/utlidar/cloud_livox_mid360`,
frame `hesai_lidar`, about 10 Hz, with `robot.motion_enabled=false`.
The camera worker requests about one frame per second; it is not a 30 FPS viewer.
Real XYZ data now also drive a bounded stationary-window XY density layer in
the live map panel, with a height filter and PNG/JSON/NPZ snapshot export. See
[Local LiDAR Geometry](local_geometry.md). This does not supply navigation scans,
SLAM, or semantic labels.
The adapter discards stale images after 3 seconds and marks odometry/LiDAR stale
after 1 second. `/api/lidar` exposes metadata, not an invented calibrated 2D scan.
UI telemetry includes point count, cloud rate and sensor frame. Raw scans are not
registered for navigation until sensor-to-body calibration is verified.

Perception is disabled in this sensor-only profile, so no model server or fake
simulated detections are needed. Optional Ollama uses the installed qwen2.5vl:3b,
not 7b. With live sensors, uncalibrated frames cannot create metric map nodes,
even when a VLM suggests depth. Sensor connection is not autonomous navigation.

Verification: 93 Python tests and 7 C++ tests passed, including sensor expiry, command rejection
and uncalibrated-map guards. A live adapter smoke test received the camera,
odometry and LiDAR (~10 Hz, ~82k points), rejected Move/arm, then closed cleanly.
The dashboard was also inspected with the real camera and live sensor indicators.

The remaining SDK transport is experimental: select `robot.transport=sdk` only
for separately reviewed development. It requires unitree_sdk2py in Linux; it is
not the tested Windows/WSL path and was not armed or executed during integration.
The older sections below describe that upstream architecture, not certification
of real robot navigation or the use of Damp as an emergency stop.

## Run it today (no robot, no model needed)

### Relative motion commands

The rule planner also accepts explicit primitives without a mapped target or VLM:

```text
go forward 1 meter
go backward 0.5 meters
rotate by 360 degrees
rotate 360 degrees clockwise
turn right
turn left and go forward 0.5 meters
Повернись направо и пройди вперед на метр
stop
```

Walking uses a signed `walk_relative(distance_m)` step, maximum 2 m per step.
Turning uses `rotate_relative(angle_rad)`, maximum 360 degrees per step; positive
is left/counterclockwise. Without a direction, a positive angle means left.
A bare turn left/right means 90 degrees. Units are required for numeric commands.
These exact commands are parsed deterministically even when an LLM planner is selected.

Relative origins are captured when each step starts, so a turn followed by a walk
uses the new heading. Straight walking does not detour or invoke autonomous
recovery: obstacles, stale telemetry, jumps, heading/lateral drift, timeout or
no progress stop and fail the step. Remaining steps after a failed primitive are
skipped. Full turns accumulate signed yaw increments across the +/-pi boundary,
instead of treating the same final heading as already completed. Command `stop`
pauses an active task even while the engine is busy.

The numerical completion tolerance is 2 cm / 0.02 rad (smaller for tiny requests),
not a guarantee of physical positioning accuracy. These primitives are verified
in the kinematic simulator, not on the real robot. Real execution still requires
an explicitly armed motion adapter and a fresh calibrated body-frame scan; the
default live C++ sensor transport has no motion API and continues to reject commands.

Use `./agent.ps1 -Sim -Port 8771` for a separate simulation while read-only live
sensors remain on 8770. A primitive can legitimately stop if furniture is ahead;
the default apartment's starting heading points toward the sofa. Turn toward
free space before asking it to walk straight. A full rotation and the sequence
`turn left and go forward 1 meter` were exercised through the mock HTTP API.
After this update all 102 Python tests passed; the browser also displayed
SUCCEEDED after an actual simulated full turn. No physical movement was requested.

```bash
python3 -m venv .venv && .venv/bin/pip install -e '.[agent]'
.venv/bin/python -m as2_vlm.agent --sim --fresh-map
```

Open http://127.0.0.1:8770. Try the chips, e.g. *"Check if the back door is locked and make sure the stove
is off"*, then use **Fault injector → Close the next door on the route** while the robot walks, or
**Close utility room door (only route)** before sending a back-door goal to see the robot ask for help.
Use **Human helps** to open the door.

Options: `--speedup 5` (faster sim), `--planner ollama` (LLM planner), `--perception ollama` (real VLM on the
synthetic camera; set `vlm.perception_interval_s` to ~8 s), `--port`, `--verbose`.

Tests: `.venv/bin/python -m unittest discover -s tests` (the agent tests skip without pydantic).

## Architecture

```text
 dashboard.html ──SSE/HTTP── app.py (AgentRuntime)
                                │
             ┌──────────────────┼──────────────────────────┐
     task_planner.py      execution_engine.py         semantic_map.py
     rules | LLM          asyncio closed loop          SQLite + graph
     Pydantic plan  ───►  monitor → StopMove →  ◄───►  query_map / fuse
                          VLM diagnose → recover       update_node_state
                                │        ▲
                        navigation.py    │ vlm_backends.py (Simulated | Ollama | OpenAI-compatible)
                        A* + follower    │
                                ▼        │
                     robot_adapter.py: BaseRobotAdapter (clamp, arm, latch, watchdog)
                        ├─ MockAs2Adapter ── sim_world.py (truth, LiDAR, synthetic RGB-D)
                        └─ RealUnitreeAs2Adapter ── unitree_sdk2py SportClient + rt/lf/sportmodestate
```

Separation of concerns: models output validated JSON only (plans, detections, verdicts, diagnoses).
Velocities come from the deterministic follower in `navigation.py` and are clamped in the adapter. The
stack only calls high-level sport-mode functions (`Move`, `StopMove`, `Euler`, `StandUp`/`RecoveryStand`,
`StandDown`, `Damp`), never `LowCmd`.

| Module | Responsibility |
|---|---|
| `config.py` | `SIMULATION_MODE`, limits, endpoints; JSON file → env → CLI. Secrets only via env vars. |
| `robot_adapter.py` | HAL. Base class enforces clamping, `motion_enabled`, operator arm, emergency latch. |
| `sim_world.py` | Hidden truth: 5-room apartment, doors, objects, boxes; ray-cast LiDAR and camera. |
| `semantic_map.py` | Nodes `{id,label,category,coords,confidence,last_seen,state,...}`, 2D→3D projection, fusion, `query_map`, `update_node_state`, `get_all_nodes`, edges. |
| `vlm_backends.py` | `detect_objects`, `verify_visual_state`, `diagnose_blockage` for each backend. |
| `task_planner.py` | Tools `navigate_to`, `adjust_posture`, `verify_visual_state`, `speak`, `stand_down`, `return_home`; rule and LLM planners; map grounding. |
| `navigation.py` | Belief occupancy grid (floor plan + believed closed doors/obstacles), A*, follower, LiDAR corridor. |
| `execution_engine.py` | Plan execution, monitoring, recovery, overrides, perception loop, reasoning log. |
| `app.py`, `static/dashboard.html` | Stdlib HTTP + SSE dashboard, loopback by default, same-origin POST check. |

### Recovery loop (Module 3)

Monitoring signals while walking: robot contact, LiDAR corridor along the direction of travel, stall
(commanded but not moving), no progress, timeout, and map changes that newly block the route. On a
trigger the engine sends `StopMove()`, asks the VLM *why* (`door_closed | temporary_obstacle |
permanent_obstacle | person_in_way | unknown`), writes the result into the scene graph, then:

0. no visible cause or a known wall: back off 0.3 m and re-plan;
1. re-plan an alternate route if the updated belief allows one;
2. temporary obstacle: wait `obstacle_wait_s` for it to clear;
3. otherwise speak (*"Excuse me, the utility room door is closed. Could you open it for me?"*), wait up to
   `human_wait_timeout_s` with reminders, re-verify with LiDAR aimed at the blocker plus the VLM, thank the
   person and continue. On timeout the step fails and dependent verifications are reported as not verified.

Verification is closed-loop as well: if the VLM is uncertain, the engine changes the body pitch (`Euler`) and
looks again.

### Overrides

`PAUSE / STOP MOVE` → `StopMove()` and the task pauses (Space bar). `ABORT & RETURN` cancels and drives to the
charging dock. `EMERGENCY SIT DOWN + DAMP` → `StopMove()` → `StandDown()` → wait `damp_delay_s` → `Damp()`
and latches; new goals are refused until **Stand up** (`RecoveryStand`). `Damp` is never sent to a standing
robot first.

## Tomorrow: switching to the real AS2 EDU

For this robot, prefer the verified sensor-only procedure above. The following
native SDK workflow is unverified on hardware and requires explicit `transport=sdk`.

1. On a Linux machine wired to the robot: install CycloneDDS 0.10.2 and `unitree_sdk2_python` from source
   (it ships `unitree_sdk2py.as2.sport.sport_client.SportClient`), then `pip install -e '.[agent]'`.
2. `cp configs/agent.example.json configs/agent.local.json` and set `robot.network_interface`.
   Keep `motion_enabled: false` for the first run (read-only).
3. Build the TTS helper (optional): `cmake -S robot/bridge_cpp -B build/as2-probe -DCMAKE_PREFIX_PATH=/opt/unitree_robotics && cmake --build build/as2-probe`.
4. Plug onboard code into `RealUnitreeAs2Adapter` (search for `TODO: Plug in existing VLM/comms hook`):
   `register_frame_provider` (RGB JPEG), `register_depth_provider` (depth at normalized u, v),
   `register_scan_provider` (LiDAR angles/ranges, body frame), `register_battery_provider`, `register_tts`.
   Autonomous walking is refused while no scan provider is registered
   (`robot.allow_walking_without_scan` overrides this; not recommended).
5. VLM: local Ollama (`vlm.perception_backend: ollama`) or your endpoint (`openai_compatible`, HTTPS, key in
   the env var named by `vlm.api_key_env`). A different protocol only needs a `chat_json()` implementation.
6. Start `python -m as2_vlm.agent --real`, confirm telemetry on the dashboard, test **STOP**, then set
   `motion_enabled: true`, restart, press **Arm motion** and try a short "Go to x, y" with the remote in hand.

Must be verified on hardware (not provable from the public SDK): `SportModeState` velocity frame and
`mode` values on AS2, Euler pitch sign, TTS speaker ids, camera intrinsics/extrinsics (`camera.*`), real
stopping distance (`motion.stop_distance_m`), and whether `Move` needs a specific gait first.

## Dashboard API

`GET /api/state`, `/api/map`, `/api/camera.jpg`, `/api/events` (SSE: `state`, `log`), `/api/config`;
`POST /api/command {text, source, planner}`, `/api/override {action}`, `/api/anomaly {kind, params}`
(simulation only), `/api/mode {simulation}`, `/api/node_state {id, state}`, `/api/frame` (JPEG, simulation
webcam source).

## Known limitations

The simulator is kinematic and the synthetic camera draws labels, so the simulated VLM is an oracle over
visible objects; it validates the pipeline, not model quality. With Qwen2.5-VL 7B on the synthetic frames,
verification and diagnosis are reliable but bounding boxes are coarse. The rule planner understands English
goals built from check/verify/make sure/go/return home/say clauses; free-form goals need the LLM planner.
There is no global SLAM: without `robot.static_map_path` the real robot plans in open space and relies on
LiDAR stops plus recovery.
