# AS2 agent stack: semantic map, goal planner, recovery loop, dashboard

`as2_vlm/agent/` adds goal-directed autonomy on top of the dry-run MVP. Everything runs today against
`MockAs2Adapter`; switching to the real AS2 EDU is a config flag (`simulation_mode: false`). The existing
`as2_vlm` CLI, parser and demo are unchanged.

## Run it today (no robot, no model needed)

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
