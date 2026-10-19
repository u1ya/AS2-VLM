# Resume the AS2 VLM Project

Session handoff: 2026-10-20. This file is the current operational summary;
older progress entries describe historical states, not current capabilities.
Final verification: 45 Python tests and 2 C++ policy tests passed. Python tests
required execution outside the restricted sandbox for temporary files and loopback sockets.

## What Works

- Russian one/two-action planning, validation and a browser simulator.
- Optional local Ollama Qwen2.5-VL 3B planning and experimental image grounding.
- AS2 GetState and sportmodestate telemetry through Windows + WSL2 mirrored networking.
- Separate, manually confirmed C++ stop, short forward pulse and odometry-distance tests.
- The browser and language planner do NOT control physical motion.

No camera/LiDAR integration, SLAM, obstacle-aware physical navigation or production
safety controller is implemented. Firmware version and onboard login remain unknown.

## Saved Setup

| Setting | Last working value |
| --- | --- |
| Robot | Unitree AS2 EDU |
| Robot Ethernet address | 192.168.123.161 |
| Laptop Ethernet address | 192.168.123.100/24 (255.255.255.0) |
| Ethernet gateway / DNS | None on this isolated link |
| Windows adapter | Ethernet, Realtek Gaming GbE Family Controller |
| Linux environment | WSL2 Ubuntu-22.04, mirrored networking |
| Last WSL interface | eth1; verify after each restart |
| SDK2 revision | d1abf001220d3e5f59e00a0c4aaa595e00089a4a |
| Local Windows workspace | C:\Users\Ulugb\Desktop\SLAI VLM |
| Local WSL workspace | /mnt/c/Users/Ulugb/Desktop/SLAI VLM |
| Model | qwen2.5vl:3b |
| Portable model runtime | .runtime/ollama-v0.40.2/ollama.exe |
| Model storage | .models/ollama |
| Demo / model endpoints | 127.0.0.1:8765 / 127.0.0.1:11434 |

These settings are a reference, not an automatically applied robot configuration.
The example Python config deliberately remains mock with motion disabled.
Never reuse these IP addresses without checking for conflicts on a different network.

The Windows user file `%USERPROFILE%\.wslconfig` contains:

```ini
[wsl2]
networkingMode=mirrored
```

If this setting is changed, save work in all WSL sessions before `wsl --shutdown`.
No WSL shutdown or network reconfiguration is needed just to reopen this project.

## Start Again on This Computer

1. Connect Ethernet, power up the robot using its official procedure, and keep
   the physical controller available. Do not launch a movement test yet.
2. Open PowerShell in the project folder. Inspect the two project firewall rules
   in an **administrator** PowerShell:

```powershell
cd "C:\Users\Ulugb\Desktop\SLAI VLM"
.\deploy\configure_dds_firewall.ps1 -Action Status
```

If absent, review the script and run it with `-Action Enable`. It permits inbound
UDP only from 192.168.123.161, on the host Ethernet interface and in WSL Hyper-V.
DDS negotiates dynamic ports, so 7400-7500 alone was insufficient. Do not disable
global firewalls. The script intentionally rejects a different adapter model.

3. Open Ubuntu and check the route and interface; substitute the actual interface
   below if it is no longer eth1:

```bash
cd "/mnt/c/Users/Ulugb/Desktop/SLAI VLM"
ip -br addr
ip route get 192.168.123.161
ping -c 3 192.168.123.161
timeout 12 ./build/as2-probe/as2_state_probe eth1 --json
./build/as2-probe/as2_walk_meter eth1 --observe
```

The last command only reads odometry. Ping alone does not establish DDS readiness.
GetState should return sdk_return_code=0. The last observed state was
NORMAL_WALK / process_state=0, but it must be checked again, not assumed.
This firmware publishes a zero source timestamp; the reader uses local DDS
arrival freshness and delivery sequence, with additional source-time checks when available.

4. Start the demo from ordinary PowerShell:

```powershell
.\demo.ps1 -RobotState -RobotInterface eth1 -WslDistro Ubuntu-22.04
```

Open http://127.0.0.1:8765/. Omit `-RobotState` for simulation without hardware.
The state refresh is read-only; all demo movement and stop buttons are simulation-only.

5. Optional VLM: in another PowerShell terminal start the existing local runtime:

```powershell
.\deploy\start_model_server.ps1 -OllamaPath "$PWD\.runtime\ollama-v0.40.2\ollama.exe"
```

Then use `./run.ps1 --healthcheck` in a separate terminal. Model inference is not
required for the deterministic simulator or the manual SDK tests.

## Rebuild or Restore on a New Computer

Clone https://github.com/u1ya/AS2-VLM.git and install Python 3.10+ and WSL2 Ubuntu
22.04. The paths above must be adapted to the new checkout. In Ubuntu, from the
project root:

```bash
sudo apt-get update
sudo apt-get install -y build-essential cmake libeigen3-dev ca-certificates git
mkdir -p .runtime
git clone https://github.com/unitreerobotics/unitree_sdk2.git .runtime/unitree_sdk2
git -C .runtime/unitree_sdk2 checkout d1abf001220d3e5f59e00a0c4aaa595e00089a4a
bash deploy/build_state_probe.sh
ctest --test-dir build/as2-probe --output-on-failure
python3 -m unittest discover -s tests -v
```

These build/test commands do not issue robot commands. The SDK clone and pinned
revision must be available from upstream. See README for optional Pillow installation.
To restore Ollama, run `deploy/bootstrap_ollama.ps1` on Windows, start the model
server, then run `deploy/start_model_server.ps1 -OllamaPath <path> -PullModel`
in another terminal. Weights require a fresh download unless backed up separately.

## Physical Results and Current Limits

The short pulse test received Move=0 and StopMove=0; distance was not measured.
Early distance tests stopped on vertical-speed checks: measured 0.187889 m/s
against 0.15, then 0.276612 m/s against 0.25. The operator observed walking onset
followed by immediate stopping. The threshold was changed at the operator's request.

Latest distance attempt, with abs(vz) limit 0.5 m/s:

```text
Result=no_progress
odometry_progress_m=0.969629
StopMove=0
interrupted=0
Final odometry progress_m=0.969413
telemetry_stationary=1
exit=1
```

Approximately 97 cm by projected odometry, NOT a verified full meter or an external
distance measurement. Command speed tapered to 0.04 m/s, after which progress
ceased; the cause of that stall is unverified. No repeat was sent automatically.

Current policy: target success >=0.98 m, speed 0.04-0.15 m/s, 12 s timeout,
freshness <=0.3 s, progress >=2 cm per 2 s, inter-sample planar jump <=7 cm,
yaw deviation <=0.25 rad, lateral deviation <=12 cm, backward progress >=-4 cm,
height change <=6 cm, planar measured speed <=0.35 m/s, abs(vz)<=0.5 m/s.
Posture, mode, errors and stationary-start checks also remain enabled.
The 0.5 m/s vertical limit is **not a validated safety limit**.

Before another physical attempt, read [Physical Test Safety](physical_test.md),
recheck telemetry, clear the floor and cable, verify the independent controller
stop, and obtain fresh operator confirmation. No physical test auto-starts.
Do not silently relax guards or widen the success tolerance to report success.
Next engineering task: characterize low-speed walking and odometry near the target,
then adjust the taper only with evidence and focused tests.

Unplugging Ethernet is not a verified stop. StopMove is network-dependent. A second
stop-only process cannot latch-cancel a loop sending Move. Use the independent
controller; Ctrl+C in the movement process is only best-effort cancellation.

## What GitHub Does and Does Not Preserve

Git preserves code, tested policy values, scripts, this handoff and summarized
results. It does not preserve the Windows registry/network settings, firewall
state, WSL filesystem, downloaded SDK/runtime, model weights, build artifacts or
raw local logs. `.runtime/`, `.models/`, `build/`, `logs/`, recordings and local
credentials stay ignored and remain on this computer. Back them up privately for
offline restoration; a fresh clone alone does not include them.

## End a Session

Verify the robot is physically stationary with the operator. Stop the demo and
model server with Ctrl+C in their terminals (or stop only the verified project
server processes). This is not a robot stop. Disconnect/power down the robot using
the official procedure. After leaving the trusted robot network, remove only the
project firewall exceptions in administrator PowerShell:

```powershell
.\deploy\configure_dds_firewall.ps1 -Action Disable
```

At handoff, both project firewall rules were inspected and enabled. They were
left unchanged to preserve the working setup; the command above is an operator
cleanup step, not an action already performed. The project demo was stopped for
session closure. No new motion or robot stop command was issued during handoff.

Keep the source folder and ignored data for the next session. Do not uninstall WSL
or remove Ubuntu; those steps are unnecessary.
