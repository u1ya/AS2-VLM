"""Central configuration for the AS2 agent stack.

Switch from the mock robot to the real AS2 EDU by any one of:
  * SIMULATION_MODE = False below,
  * "simulation_mode": false in configs/agent.local.json (git-ignored),
  * AS2_SIMULATION_MODE=0 in the environment,
  * `python -m as2_vlm.agent --real`.

Secrets (VLM API keys) are read from environment variables only; config files name the
variable, never the value.
"""
import json
import math
import os
import re
from dataclasses import asdict, dataclass, field, fields, is_dataclass
from pathlib import Path

SIMULATION_MODE = True

PROJECT_ROOT = Path(__file__).resolve().parents[2]
LOCAL_CONFIG = PROJECT_ROOT / "configs" / "agent.local.json"
PERCEPTION_BACKENDS = {"auto", "disabled", "simulated", "ollama", "openai_compatible"}
PLANNER_BACKENDS = {"rules", "ollama", "openai_compatible"}


@dataclass
class MotionConfig:
    max_vx: float = 0.5             # m/s forward; hard cap 1.0 enforced by validate()
    max_vy: float = 0.3             # m/s lateral
    max_vyaw: float = 0.8           # rad/s
    max_roll: float = 0.3           # rad, Euler() body posture
    max_pitch: float = 0.35         # rad; negative pitch = nose up (REP-103, verify on hardware)
    max_body_yaw: float = 0.4       # rad
    robot_radius_m: float = 0.30    # collision radius used by the planner
    robot_half_width_m: float = 0.25
    stop_distance_m: float = 0.45   # free space required in front of the body while walking
    goal_tolerance_m: float = 0.20
    yaw_tolerance_rad: float = 0.10
    command_timeout_s: float = 0.4  # adapters stop if no Move() arrives within this window
    control_hz: float = 10.0
    stall_time_s: float = 2.0       # commanded forward motion but measured speed ~0
    progress_window_s: float = 8.0  # no progress towards the waypoint within this window
    min_progress_m: float = 0.15


@dataclass
class CameraConfig:
    source: str = "synthetic"       # synthetic | webcam (browser upload, mock only) | opencv:<index|url>
    width: int = 640
    height: int = 480
    hfov_deg: float = 90.0
    offset_xyz: list = field(default_factory=lambda: [0.30, 0.0, 0.12])  # camera from body centre, m
    jpeg_quality: int = 80


@dataclass
class VLMConfig:
    perception_backend: str = "auto"   # auto = simulated in mock mode, ollama on hardware
    planner_backend: str = "rules"     # default planner; the UI can pick per command
    endpoint: str = "http://127.0.0.1:11434"
    model: str = "qwen2.5vl:3b"
    api_key_env: str = "AS2_VLM_API_KEY"
    timeout_s: float = 120.0
    perception_interval_s: float = 1.0
    min_detection_confidence: float = 0.35


@dataclass
class RobotConfig:
    transport: str = "cpp_readonly"   # verified Windows/WSL sensors; sdk is experimental actuation
    sensor_binary: str = "build/as2-probe/as2_sensor_stream"
    sensor_hub_url: str = "http://127.0.0.1:8780"
    lidar_body_calibration_path: str = ""  # measured transform/dimensions; missing => manual motion refused
    wsl_distro: str = "Ubuntu-22.04"
    network_interface: str = "eth0"
    dds_domain: int = 0
    motion_enabled: bool = False       # hardware only: False keeps the real adapter read-only
    state_topic: str = "rt/lf/sportmodestate"
    tts_binary: str = ""               # path to robot/bridge_cpp as2_tts build
    tts_speaker_id: int = 1
    damp_delay_s: float = 2.5          # StandDown() -> Damp() delay during emergency sit
    sdk_timeout_s: float = 3.0
    static_map_path: str = ""          # optional JSON floor plan (same format as SimWorld.static_map)
    allow_walking_without_scan: bool = False


@dataclass
class AgentConfig:
    relative_language_planner: bool = False
    manual_control: bool = False
    simulation_mode: bool = SIMULATION_MODE
    sim_speedup: float = 1.0
    map_db_path: str = "data/semantic_map.sqlite"
    sim_map_db_path: str = "data/semantic_map_sim.sqlite"
    ui_host: str = "127.0.0.1"
    ui_port: int = 8770
    human_wait_timeout_s: float = 90.0
    human_reminder_s: float = 30.0
    obstacle_wait_s: float = 6.0
    max_recoveries_per_step: int = 4
    home_pose: list = field(default_factory=lambda: [0.8, 0.8, 0.0])
    motion: MotionConfig = field(default_factory=MotionConfig)
    camera: CameraConfig = field(default_factory=CameraConfig)
    vlm: VLMConfig = field(default_factory=VLMConfig)
    robot: RobotConfig = field(default_factory=RobotConfig)

    def to_dict(self):
        return asdict(self)

    def resolve_path(self, value):
        if not value or value == ":memory:":
            return ":memory:"
        path = Path(value)
        return str(path if path.is_absolute() else PROJECT_ROOT / path)


ENVIRONMENT = {
    "AS2_SIMULATION_MODE": ("simulation_mode", bool),
    "AS2_SIM_SPEEDUP": ("sim_speedup", float),
    "AS2_UI_PORT": ("ui_port", int),
    "AS2_MAP_DB": ("map_db_path", str),
    "AS2_NETWORK_INTERFACE": ("robot.network_interface", str),
    "AS2_MOTION_ENABLED": ("robot.motion_enabled", bool),
    "AS2_TTS_BINARY": ("robot.tts_binary", str),
    "AS2_CAMERA_SOURCE": ("camera.source", str),
    "AS2_PERCEPTION_BACKEND": ("vlm.perception_backend", str),
    "AS2_PLANNER_BACKEND": ("vlm.planner_backend", str),
    "AS2_VLM_ENDPOINT": ("vlm.endpoint", str),
    "AS2_VLM_MODEL": ("vlm.model", str),
}


def _parse_bool(text):
    lowered = str(text).strip().lower()
    if lowered in {"1", "true", "yes", "on"}:
        return True
    if lowered in {"0", "false", "no", "off"}:
        return False
    raise ValueError(f"Expected a boolean, got {text!r}")


def _merge(target, values, prefix=""):
    if not isinstance(values, dict):
        raise ValueError(f"Expected an object for {prefix or 'config'}")
    known = {f.name for f in fields(target)}
    for key, value in values.items():
        if key not in known:
            raise ValueError(f"Unknown config key: {prefix}{key}")
        current = getattr(target, key)
        if is_dataclass(current):
            _merge(current, value, f"{prefix}{key}.")
        else:
            setattr(target, key, value)


def _set_dotted(config, dotted, value):
    *parents, name = dotted.split(".")
    target = config
    for parent in parents:
        target = getattr(target, parent)
    setattr(target, name, value)


def validate(config):
    if type(config.relative_language_planner) is not bool:
        raise ValueError("relative_language_planner must be boolean")
    if type(config.manual_control) is not bool:
        raise ValueError("manual_control must be boolean")
    if config.manual_control and (config.ui_host != "127.0.0.1" or config.robot.transport != "cpp_readonly"):
        raise ValueError("Manual control requires numeric loopback and cpp_readonly sensors")
    hub = config.robot.sensor_hub_url
    if not isinstance(hub, str) or (hub and not re.fullmatch(r"http://127\.0\.0\.1:[0-9]{1,5}", hub)):
        raise ValueError("sensor_hub_url must be empty or a numeric loopback HTTP endpoint")
    if hub and not 1 <= int(hub.rsplit(":", 1)[1]) <= 65535:
        raise ValueError("sensor_hub_url port out of range")
    if config.robot.transport not in {"cpp_readonly", "sdk"}:
        raise ValueError("robot.transport must be cpp_readonly or sdk")
    if config.robot.transport == "cpp_readonly" and config.robot.motion_enabled:
        raise ValueError("cpp_readonly transport cannot enable motion")
    m = config.motion
    for name in ("max_vx", "max_vy", "max_vyaw", "max_roll", "max_pitch", "max_body_yaw",
                 "robot_radius_m", "stop_distance_m", "goal_tolerance_m", "command_timeout_s", "control_hz"):
        value = getattr(m, name)
        if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
            raise ValueError(f"motion.{name} must be a positive number")
    if m.max_vx > 1.0 or m.max_vy > 0.6 or m.max_vyaw > 1.5:
        raise ValueError("Velocity limits exceed the agent's hard caps (1.0 m/s, 0.6 m/s, 1.5 rad/s)")
    if m.max_pitch > 0.6 or m.max_roll > 0.6 or m.max_body_yaw > 0.6:
        raise ValueError("Posture limits exceed the 0.6 rad hard cap")
    if config.vlm.perception_backend not in PERCEPTION_BACKENDS:
        raise ValueError(f"vlm.perception_backend must be one of {sorted(PERCEPTION_BACKENDS)}")
    if config.vlm.planner_backend not in PLANNER_BACKENDS:
        raise ValueError(f"vlm.planner_backend must be one of {sorted(PLANNER_BACKENDS)}")
    if not 0.05 <= float(config.sim_speedup) <= 100:
        raise ValueError("sim_speedup must be within [0.05, 100]")
    if len(config.home_pose) != 3 or len(config.camera.offset_xyz) != 3:
        raise ValueError("home_pose and camera.offset_xyz need three numbers")
    if not 1 <= int(config.ui_port) <= 65535:
        raise ValueError("ui_port out of range")
    return config


def load_config(path=None, env=None, overrides=None):
    """Defaults <- JSON file <- environment <- explicit overrides (CLI/tests)."""
    config = AgentConfig()
    path = LOCAL_CONFIG if path is None else (Path(path) if path else None)
    if path is not None and path.is_file():
        _merge(config, json.loads(path.read_text(encoding="utf-8")))
    env = os.environ if env is None else env
    for name, (dotted, kind) in ENVIRONMENT.items():
        if name in env:
            raw = env[name]
            _set_dotted(config, dotted, _parse_bool(raw) if kind is bool else kind(raw))
    if overrides:
        _merge(config, overrides)
    return validate(config)
