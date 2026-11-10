# Shared Language Control: Simulation and Real AS2

2026-11-02: physical web execution additionally requires the calibrated shared
LiDAR obstacle interlock. Missing/stale/unobserved clearance refuses motion and
an obstacle during a step cancels the remaining plan. Current hardware has no
measured body/LiDAR profile, so physical web movement is blocked. Language preview,
sensor viewing and the standalone Stop request remain available. See
[current mapping/interlock status](semantic_mapping.md); historical results below
do not verify this new interlock on moving hardware.

## Run

From PowerShell in the project root:

```powershell
./start-services.ps1
```

The launcher keeps separate loopback services:

| Service | URL | Role |
| --- | --- | --- |
| Simulation | http://127.0.0.1:8770/ | Mock robot, shared relative language contract |
| Real AS2 | http://127.0.0.1:8771/ | Preview and execute bounded physical skill plans |
| Sensor acquisition | http://127.0.0.1:8780/health | One read-only DDS/camera owner |
| Local model | http://127.0.0.1:11434/ | Existing Qwen2.5-VL 3B via Ollama |

`./start-services.ps1 -SimulationOnly` starts only model and simulation. Existing
services with a conflicting profile are not killed or silently switched. Logs are
under ignored `logs/`. No startup request moves the robot or sends StopMove.

The current binaries must be built in WSL before real execution:

```bash
cmake -S robot/bridge_cpp -B build/as2-probe -DCMAKE_PREFIX_PATH="$PWD/.runtime/unitree-sdk-install"
cmake --build build/as2-probe --target as2_sensor_stream as2_relative_move as2_stop_only as2_walk_meter --parallel 2
```

The launcher uses existing Ollama/model files, not a new download. Legacy
`./agent.ps1 -Sim` still supports model-free simulation. In the paired simulation,
select Rule planner to use its original scene-graph/rules workflows; Ollama uses
the shared relative-language contract enabled by `--relative-language`.

## Supported Language and Actions

Relative movement, turns, reversal and sequences are no longer restricted to a
one-metre phrase. Example free-text instructions:

```text
Повернись на 45 градусов направо и пройди 1 метр
Отступи на полметра, затем повернись налево
Пройди полтора метра и развернись на 180 градусов
Please back up half a metre, then make a quarter turn to your left.
Стоп
```

Both modes use `PhysicalPlanner` and the same validated `PhysicalPlan`.
`language_intents.py` defines a shared bilingual vocabulary. Complete recognized
clauses bypass inference. Other paraphrases go to Qwen, whose semantic actions
are `walk_forward`, `walk_backward`, `turn_left`, `turn_right`, `look_around`,
`turn_around`, `stop`. Raw walking amounts are metres; raw turning amounts are
**degrees**. Code supplies signs and converts degrees to executor radians.

Fixed meanings: `оглянись вокруг` / `look around` = 360 degrees on the spot;
`развернись` / `turn around` = 180 degrees; left/right without an angle = 90
degrees. Full and half turns default to left; an explicit direction takes priority.
A 360-degree request remains a full accumulated rotation and becomes four native
90-degree steps. It is never normalized to zero.

Negations, quotations, conditions, common missing quantities and conflicting
directions return `clarify` without actions. Matching is complete-clause based,
not fuzzy keyword extraction. Numeric limits are checked without truncation.
Mixed known/unknown commands are checked against recognized meanings, counts and
order. Additional checks reject full/half/quarter-turn and explicit direction
changes even in unfamiliar clauses. Such checks can reject a model answer, but
do not themselves authorize unfamiliar commands. Multi-action clauses mixed with
known clauses may be conservatively rejected when action alignment is uncertain.
This does not guarantee flawless interpretation of arbitrary free text.

Language evaluation (read-only, no robot adapter or movement):

```powershell
python -m as2_vlm.agent.evaluate_language --output reports/language_rules.json
python -m as2_vlm.agent.evaluate_language --model-only --cases as2_vlm/agent/language_cases/language_paraphrases.json --output reports/language_model.json
python -m as2_vlm.agent.evaluate_language --cases as2_vlm/agent/language_cases/language_holdout.json --output reports/language_holdout.json
```

`language_commands.json` contains 36 standard/negative/ambiguous cases tested
without inference. `language_paraphrases.json` overlaps prompt examples and is a
development smoke test, not evidence of generalization. `language_holdout.json`
contains initially unseen paraphrases; semantic guards were subsequently expanded
using failures from this set, so it must now also be treated as a development set.
Reports compare actual status, action order, sign and magnitude, not just JSON
validity. `--model-only` bypasses direct command recognition while keeping guards.

Verification on 2026-11-01: 192 Python tests passed; 36/36 standard command cases
and 10/10 expanded development cases passed. The forced-model development smoke
test passed 11/12 cases: Qwen still misread `look around clockwise`, and the
independent direction check rejected its answer. Normal hybrid processing handles
that phrase directly and passes. These small development sets do not establish
general language accuracy. Reports are under `reports/language_*.json`.
No physical robot actions were sent, and running control services were not restarted.
Existing services must be restarted to load these Python changes.

An actual local-model test of the first example produced exactly
`rotate_relative(-pi/4)` followed by `walk_relative(1)`. The English example
produced `walk_relative(-0.5)` then `rotate_relative(pi/2)`. A full simulation
run of right 45 degrees followed by forward 0.2 m completed both steps.

Named-room/object navigation, visual inspection, climbing and following people
are not implemented on this physical path. A synthetic simulation map is not
evidence of a real kitchen or door. Such requests require real localization,
semantic perception and calibrated obstacle handling; the model is instructed
to decline or clarify, and invalid numeric plans are rejected without execution.

## Plan and Execute

On 8771, Send first calls `/api/manual/plan`. It never starts an actuator. The
dialog shows the actual generated actions and values. Execute submits only a
server-issued plan ID plus readiness confirmations, not arbitrary client steps.
Plans expire after 60 seconds and cannot be reused or replaced with injected
velocities. Stop invalidates pending plans and stale in-flight inference results.

The executor starts native `as2_relative_move` steps in order, stops and verifies
stationarity between steps, and skips all remaining steps on the first failure.
A native step is at most 1 m or 90 degrees; larger validated requests are split
without changing their total amount or direction. Plan budgets remain at most
four logical/eight native steps, 2 m total translation, 360 degrees total rotation.
Speeds are capped at 0.15 m/s and 0.25 rad/s, with 12 s per movement and a 120 s
plan deadline. These are bounded experimental movements, not precision guarantees.

The browser lease, C++ heartbeat/EOF cancellation, measured-state/progress guards
and independent operator stop remain in place. A literal stop bypasses the model.
If the model/server is unavailable or a plan is invalid, movement is not started.
The legacy manual one-metre button remains as a separate diagnostic operation.

## Robot Connectivity and Verification

On 2026-10-31 both services were started. The real service had **no fresh robot
telemetry**, so physical execution was unavailable. The sensor hub now supplies
an explicitly unavailable envelope when acquisition is absent, allowing the
panel to open disconnected rather than crashing; it does not invent live poses.
Reconnect/check Ethernet and restart sensor acquisition as needed before a real
run. Opening the page or generating a plan does not prove the robot is connected.

No physical movements or network StopMove requests were sent during this work.
New physical reverse/turn execution has policy/mock tests, not an end-to-end
hardware certification. Inspect the first physical runs with the independent
remote stop and clear space. Browser visual QA could not complete because the
in-app webview failed to attach; HTTP/service and model behavior were checked.

Final regression run: 175 Python tests and 14 C++ tests passed. Earlier full
runs exposed a Windows rejected-request socket reset (fixed by bounded body
draining while retaining 403) and an intermittent accelerated-simulation timeout;
the final full run passed without changing real motion limits.
