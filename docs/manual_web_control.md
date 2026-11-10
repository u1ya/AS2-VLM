# Bounded Manual Web Control

Current language mode supersedes the old one-metre-only text path below: it now
supports validated sequences of signed translations and turns through Qwen.
See [relative_language_control.md](relative_language_control.md) for the paired
8770 simulation / 8771 real services, startup, limits and current verification.

The live panel can opt into the previously tested physical forward-meter helper
without exposing the autonomous planner or changing the read-only sensor transport.
This is a narrow manual command channel, not general teleoperation or VLM navigation.

## Start

Keep `./sensor-hub.ps1` running, then launch one panel:

```powershell
./agent.ps1 -Port 8771 -ManualControl
```

Open http://127.0.0.1:8771/ in Live AS2 EDU mode. The **Manual AS2** toolbar has
two per-run confirmations: independent remote stop ready, and clear level path
with cable slack. After both are checked, the forward button becomes available.
It asks for an additional physical-motion confirmation before starting. Loading
or refreshing the panel does not move the robot or send StopMove.

Only one manual-control panel can reserve the OS-locked `.runtime/manual-control.lock`.
The default panel and sensor hub do not gain motion access. Switching modes is
rejected during an active manual command. Entering simulation closes this channel;
returning live recreates it subject to the same exclusive lock.

## Available Commands

### Text Input

The main command field is enabled on the opted-in live manual panel. Send or
Enter routes text to the same bounded channel, not the VLM/task engine. Examples:

```text
Go forward 1 meter
go forward one metre
Пройди 1 метр вперёд
пройди вперед на один метр
стоп
останови команду
stop moving
```

Only positive forward one-metre commands and stop are currently executable.
Unsupported distances, reverse, rotations, navigation targets, negation and
compound commands are rejected without starting a subprocess. The existing
strict primitive parser is reused after narrow normalization of distance-first
Russian wording. There is no LLM interpretation or fallback for live text.

`POST /api/manual/parse` accepts `{text: ...}` and only previews the action.
`POST /api/manual/text` parses again server-side, then either calls the existing
guarded forward start or requests stop. Movement still requires the two readiness
acknowledgements and browser confirmation. Stop requires neither acknowledgement
nor fresh sensor telemetry and does not wait for the current movement to finish.
The separate Stop button remains available. All existing speed/time/deviation/
freshness/heartbeat guards are retained; prior success is not a reason to remove
failure handling. The browser keeps its existing session heartbeat for text runs.

Text/HTTP behavior was tested with fake processes, and the live preview endpoint
was checked for RU/EN commands without executing them. The panel was restarted on
8771 with `--manual-control`; 159 Python tests passed. No new physical command or
StopMove was sent while deploying text support.

- Forward up to 1 m by odometry: `as2_walk_meter --execute-ui`. Fixed speed cap
  0.15 m/s, fixed movement time limit 12 s, existing freshness/posture/progress/
  deviation guards retained. The run may stop before reaching the target.
- Stop: interrupts the active UI run through its stdin, allowing the C++ StopGuard
  to call StopMove. When no UI run is active, starts `as2_stop_only --stop`, a
  best-effort network request with a cold-start delay. It is not an independent
  emergency stop and does not guarantee physical stopping.

There is no arbitrary velocity/distance/duration argument, turning, stand-up,
StandDown, Damp or return-home command in this channel. `/api/command` and the
upstream `/api/override` remain blocked on real read-only sensors. `motion_enabled`
and `armed` in sensor telemetry remain false because the planner is not armed;
`manual_control` separately describes the narrowly enabled physical test channel.

## Watchdogs and Failure Handling

Each forward run has a random lease token, not published in status. The visible
tab sends heartbeat requests every 200 ms. The server expires a lease after 1 s,
requests stop, and does not let an expired session resume. Hiding/unloading the
tab also requests stop; this browser callback is best effort.

The C++ helper separately requests heartbeat only after SDK startup and its initial
stop check. No first heartbeat means no forward Move. During the loop, stdin EOF,
STOP, invalid/oversized input, nonmonotonic time, or a gap over 0.8 s cancels the
walk and invokes the existing StopGuard. A late heartbeat cannot revive the run.
This watchdog and the 12-second bound are software protections, not hardware
certification or a substitute for the operator's independent remote stop.

Space (outside text fields) or Escape during a manual run requests network stop.
Use the physical remote immediately if actual movement is unexpected or stopping
is not confirmed. Do not rely on browser closure, Ethernet removal, Python/WSL
health or API acknowledgement as a guaranteed emergency stop. No calibrated
obstacle avoidance is available, so the path must be clear before each run.

## Local Access

This opt-in channel requires `ui_host=127.0.0.1` and `cpp_readonly` sensors. Manual
POSTs require exact numeric-loopback Host and matching Origin plus JSON. Unknown
parameters, simultaneous runs and stale sensor startup are rejected. Local trusted
software can deliberately call this interface; it is not authenticated remote
robot control and must not be exposed through port forwarding or a reverse proxy.

## Verification

152 Python tests and 11 C++ CTest cases passed, including mocked manual processes,
HTTP access checks, duplicate runs, per-run acknowledgements, browser lease expiry,
single-panel reservation and pure C++ heartbeat parsing/expiry/latching tests.
The UI was inspected with confirmations clear, forward disabled and stop available.
No physical command or network StopMove was triggered while testing/deploying the UI.
The end-to-end physical UI watchdog remains to be verified with an independent
remote stop, clear test area and explicit operator authorization.

The earlier CLI controlled run recorded 181 clouds/3698 IMU samples in
`recordings/slam-controlled-meter-lf-20261025-190008`. It stopped on `no_progress`
at 0.96827 m final odometry progress, StopMove=0, stationary telemetry reported.
That is not independent proof of an exact one-metre physical distance or an
end-to-end test of the newly added web channel.
