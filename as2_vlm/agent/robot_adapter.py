"""Hardware abstraction layer for the Unitree AS2 EDU.

Everything above this module (planner, engine, UI) talks to BaseRobotAdapter only.
Safety rules enforced here for every implementation:
  * only high-level sport-mode calls (Move/StopMove/StandUp/StandDown/Euler/Damp), never LowCmd;
  * velocities and posture angles are clamped to config limits;
  * motion requires motion_enabled AND an operator arm AND no latched emergency;
  * if Move() commands stop arriving the robot is stopped (command watchdog).
"""
import logging
import math
import subprocess
import threading
import time
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Callable, Optional

from .clock import Clock
from .models import Pose, Telemetry
from .sim_world import SimWorld, wrap

log = logging.getLogger(__name__)


class AdapterError(RuntimeError):
    pass


@dataclass
class CameraFrame:
    frame_id: int
    timestamp: float
    width: int
    height: int
    hfov_rad: float
    pose: Pose                      # body pose when the frame was captured
    body_pitch: float               # commanded body pitch (REP-103) at capture
    camera_offset: list
    jpeg: Optional[bytes] = None
    depth_fn: Optional[Callable[[float, float], Optional[float]]] = None  # normalized u, v -> metres
    source: str = "unknown"
    truth: list = field(default_factory=list)   # simulator only: ground-truth visible objects
    meta: dict = field(default_factory=dict)

    def depth_at(self, u, v):
        return self.depth_fn(u, v) if self.depth_fn else None


@dataclass
class LaserScan:
    angles: list                    # body frame, radians, 0 = forward, positive = left
    ranges: list                    # metres
    timestamp: float
    max_range: float = 8.0


def _clamp(value, limit):
    if not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError("Non-finite command")
    return max(-limit, min(limit, float(value)))


class BaseRobotAdapter(ABC):
    kind = "base"

    def __init__(self, config, clock=None):
        self.config = config
        self.limits = config.motion
        self.clock = clock or Clock(1.0)
        self.motion_enabled = False
        self.armed = False
        self.emergency_latched = False
        self.last_refusal = None
        self.on_speech: Optional[Callable[[str], None]] = None
        self._euler = [0.0, 0.0, 0.0]

    # ---- lifecycle ----------------------------------------------------------------
    @abstractmethod
    def start(self): ...

    @abstractmethod
    def close(self): ...

    # ---- sensing ------------------------------------------------------------------
    @abstractmethod
    def get_telemetry(self) -> Telemetry: ...

    @abstractmethod
    def get_frame(self) -> Optional[CameraFrame]: ...

    @abstractmethod
    def get_scan(self) -> Optional[LaserScan]: ...

    # ---- high-level actuation (implementations) -------------------------------------
    @abstractmethod
    def _move_impl(self, vx, vy, vyaw) -> bool: ...

    @abstractmethod
    def _stop_impl(self) -> None: ...

    @abstractmethod
    def _euler_impl(self, roll, pitch, yaw) -> bool: ...

    @abstractmethod
    def _sport_call(self, name) -> bool:
        """StandUp | StandDown | Damp | RecoveryStand | BalanceStand."""

    @abstractmethod
    def _speak_impl(self, text) -> bool: ...

    # ---- guarded public API ---------------------------------------------------------
    def motion_allowed(self):
        if self.emergency_latched:
            return False, "Emergency latched: press Stand Up to recover"
        if not self.motion_enabled:
            return False, "Motion disabled in config (robot.motion_enabled=false): read-only mode"
        if not self.armed:
            return False, "Motion not armed by the operator"
        return True, None

    def move(self, vx, vy, vyaw):
        vx, vy, vyaw = (_clamp(vx, self.limits.max_vx), _clamp(vy, self.limits.max_vy),
                        _clamp(vyaw, self.limits.max_vyaw))
        allowed, reason = self.motion_allowed()
        if not allowed:
            self.last_refusal = reason
            return False
        return self._move_impl(vx, vy, vyaw)

    def stop_move(self):
        """Always permitted, even when disarmed or latched."""
        self._stop_impl()

    def euler(self, roll, pitch, yaw):
        roll, pitch, yaw = (_clamp(roll, self.limits.max_roll), _clamp(pitch, self.limits.max_pitch),
                            _clamp(yaw, self.limits.max_body_yaw))
        if self.emergency_latched:
            return False
        if self._euler_impl(roll, pitch, yaw):
            self._euler = [roll, pitch, yaw]
            return True
        return False

    def stand_up(self):
        if not self.motion_enabled:
            self.last_refusal = "Motion disabled in config"
            return False
        self.emergency_latched = False
        return self._sport_call("RecoveryStand")

    def stand_down(self):
        self.stop_move()
        self._euler = [0.0, 0.0, 0.0]
        return self._sport_call("StandDown")

    def speak(self, text):
        text = str(text).strip()[:300]
        if not text:
            return False
        if self.on_speech:
            self.on_speech(text)
        return self._speak_impl(text)

    def emergency_sit(self):
        """StopMove -> StandDown -> (delay) -> Damp. Damp is never sent to a standing robot first."""
        self.emergency_latched = True
        self.stop_move()
        self._euler = [0.0, 0.0, 0.0]
        self._sport_call("StandDown")
        self.clock.sleep_sync(self.config.robot.damp_delay_s)
        return self._sport_call("Damp")

    def set_armed(self, armed):
        self.armed = bool(armed)
        if not self.armed:
            self.stop_move()
        return self.armed

    # ---- optional capabilities -------------------------------------------------------
    def static_map(self) -> Optional[dict]:
        return None

    def seed_nodes(self) -> list:
        return []

    def inject_anomaly(self, kind, **params) -> str:
        raise AdapterError("Fault injection is only available in simulation")

    def sim_truth(self) -> Optional[dict]:
        return None

    def set_external_frame(self, jpeg):
        raise AdapterError("External frames are only accepted in simulation (camera.source=webcam)")


# =====================================================================================
# Mock adapter: today's remote development target
# =====================================================================================
class MockAs2Adapter(BaseRobotAdapter):
    """Kinematic AS2 stand-in living in SimWorld. Mirrors the real robot's behaviour that the
    stack relies on: velocity commands time out, contact stops the body, Euler tilts the camera."""
    kind = "mock"
    STAND_HEIGHT, LIE_HEIGHT = 0.35, 0.12
    LINEAR_ACCEL, YAW_ACCEL = 1.5, 3.0

    def __init__(self, config, clock=None, world=None):
        super().__init__(config, clock)
        self.world = world or SimWorld()
        self.motion_enabled = True
        self.armed = True
        hx, hy, hyaw = config.home_pose
        self._pose = [float(hx), float(hy), self.STAND_HEIGHT, float(hyaw)]
        self._vel = [0.0, 0.0, 0.0]
        self._cmd = [0.0, 0.0, 0.0]
        self._last_cmd = -math.inf
        self._standing = True
        self._damped = False
        self._battery = 87.0
        self._blocked_at = -math.inf
        self._blocked_info = None
        self._lock = threading.RLock()
        self._running = False
        self._thread = None
        self._frame_cache = (None, -math.inf)
        self._scan_cache = (None, -math.inf)
        self._frame_counter = 0
        self._external = (None, -math.inf)
        self.speech_log = []

    def start(self):
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._physics_loop, name="mock-as2-physics", daemon=True)
        self._thread.start()

    def close(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=1)

    def _physics_loop(self):
        last = self.clock.now()
        while self._running:
            now = self.clock.now()
            dt = min(now - last, 0.1)
            last = now
            while dt > 1e-6:
                step = min(dt, 0.02)
                self._step(step, now)
                dt -= step
            time.sleep(0.01 / self.clock.speedup if self.clock.speedup > 1 else 0.01)

    def _step(self, dt, now):
        lim = self.limits
        with self._lock:
            cmd = self._cmd if now - self._last_cmd <= lim.command_timeout_s else [0.0, 0.0, 0.0]
            if not self._standing or self._damped:
                cmd = [0.0, 0.0, 0.0]
            for i, accel in enumerate((self.LINEAR_ACCEL, self.LINEAR_ACCEL, self.YAW_ACCEL)):
                delta = cmd[i] - self._vel[i]
                self._vel[i] += max(-accel * dt, min(accel * dt, delta))
            x, y, z, yaw = self._pose
            vx, vy, vyaw = self._vel
            nx = x + (vx * math.cos(yaw) - vy * math.sin(yaw)) * dt
            ny = y + (vx * math.sin(yaw) + vy * math.cos(yaw)) * dt
            nyaw = wrap(yaw + vyaw * dt)
            if (abs(vx) > 1e-4 or abs(vy) > 1e-4):
                hit = self.world.collides(nx, ny, lim.robot_radius_m)
                if hit:
                    self._vel[0] = self._vel[1] = 0.0
                    self._blocked_at, self._blocked_info = now, hit
                    nx, ny = x, y
            target_z = self.STAND_HEIGHT if self._standing else self.LIE_HEIGHT
            nz = z + max(-0.3 * dt, min(0.3 * dt, target_z - z))
            self._pose = [nx, ny, nz, nyaw]
            moving = abs(vx) + abs(vy) + 0.3 * abs(vyaw)
            self._battery = max(0.0, self._battery - dt * (0.002 + 0.02 * moving))

    # ---- sensing ------------------------------------------------------------------
    def _pose_model(self):
        x, y, z, yaw = self._pose
        return Pose(x=x, y=y, z=z, yaw=yaw)

    def get_telemetry(self):
        with self._lock:
            now = self.clock.now()
            if self._damped:
                gait, fsm = "damped", "DAMP"
            elif not self._standing:
                gait, fsm = "lying", "STAND_DOWN"
            elif any(abs(v) > 0.02 for v in self._vel):
                gait, fsm = "trot", "LOCOMOTION"
            else:
                gait, fsm = "stand", "BALANCE_STAND"
            notes = [] if self.last_refusal is None else [self.last_refusal]
            return Telemetry(source="mock", connected=True, pose=self._pose_model(),
                             velocity=[round(v, 4) for v in self._vel], body_euler=list(self._euler),
                             battery_pct=round(self._battery, 1), gait_mode=gait, fsm_mode=fsm,
                             standing=self._standing and not self._damped,
                             blocked=now - self._blocked_at < 0.5, motion_enabled=self.motion_enabled,
                             armed=self.armed, emergency_latched=self.emergency_latched,
                             timestamp=time.time(), notes=notes)

    def get_frame(self):
        frame, made = self._frame_cache
        if frame is not None and self.clock.now() - made < 0.1:  # simulated seconds
            return frame
        cam = self.config.camera
        with self._lock:
            pose = list(self._pose)
            pitch = self._euler[1]
            pose_model = self._pose_model()
        jpeg, truth, depth_fn = self.world.render(pose, pitch, cam, quality=cam.jpeg_quality)
        source = "synthetic"
        external, received = self._external
        if cam.source == "webcam" and external is not None and time.monotonic() - received < 3.0:
            jpeg, truth, source = external, [], "webcam"
        self._frame_counter += 1
        meta = {"blocking": self.world.blocking_ahead(pose, self.limits.robot_radius_m,
                                                      self.limits.robot_half_width_m)}
        frame = CameraFrame(frame_id=self._frame_counter, timestamp=time.time(), width=cam.width,
                            height=cam.height, hfov_rad=math.radians(cam.hfov_deg), pose=pose_model,
                            body_pitch=pitch, camera_offset=list(cam.offset_xyz), jpeg=jpeg,
                            depth_fn=depth_fn, source=source, truth=truth, meta=meta)
        self._frame_cache = (frame, self.clock.now())
        return frame

    def get_scan(self):
        scan, made = self._scan_cache
        now = self.clock.now()
        if scan is not None and now - made < 0.05:
            return scan
        with self._lock:
            pose = list(self._pose)
        angles, ranges = self.world.scan(pose)
        scan = LaserScan(angles=angles, ranges=ranges, timestamp=time.time())
        self._scan_cache = (scan, now)
        return scan

    # ---- actuation ----------------------------------------------------------------
    def _move_impl(self, vx, vy, vyaw):
        with self._lock:
            if not self._standing or self._damped:
                self.last_refusal = "Robot is not standing"
                return False
            self._cmd = [vx, vy, vyaw]
            self._last_cmd = self.clock.now()
            self.last_refusal = None
            return True

    def _stop_impl(self):
        with self._lock:
            self._cmd = [0.0, 0.0, 0.0]
            self._last_cmd = -math.inf

    def _euler_impl(self, roll, pitch, yaw):
        with self._lock:
            return self._standing and not self._damped

    def _sport_call(self, name):
        with self._lock:
            if name in ("StandUp", "RecoveryStand", "BalanceStand"):
                self._standing, self._damped = True, False
            elif name == "StandDown":
                self._cmd = [0.0, 0.0, 0.0]
                self._standing = False
            elif name == "Damp":
                self._cmd = [0.0, 0.0, 0.0]
                self._damped = True
                self._standing = False
            else:
                raise AdapterError(f"Unsupported sport call {name}")
            self._vel = [0.0, 0.0, 0.0] if name in ("StandDown", "Damp") else self._vel
            return True

    def _speak_impl(self, text):
        self.speech_log.append((time.time(), text))
        return True

    # ---- simulation-only capabilities ------------------------------------------------
    def static_map(self):
        return self.world.static_map()

    def seed_nodes(self):
        return self.world.seed_nodes()

    def sim_truth(self):
        return self.world.truth_overlay()

    def set_external_frame(self, jpeg):
        if not (isinstance(jpeg, bytes) and jpeg.startswith(b"\xff\xd8\xff")):
            raise ValueError("Expected JPEG bytes")
        self._external = (jpeg, time.monotonic())

    def inject_anomaly(self, kind, **params):
        world = self.world
        with self._lock:
            if kind == "close_door":
                door_id = params.get("door_id")
                world.set_door(door_id, "closed")
                return f"{world.doors[door_id].label} closed"
            if kind == "open_door":
                door_id = params.get("door_id")
                world.set_door(door_id, "open")
                return f"{world.doors[door_id].label} opened"
            if kind == "box":
                x, y = float(params["x"]), float(params["y"])
                rx, ry = self._pose[0], self._pose[1]
                if math.hypot(x - rx, y - ry) < self.limits.robot_radius_m + 0.45:
                    raise ValueError("Box would overlap the robot")
                world.add_box(x, y)
                return f"cardboard box placed at ({x:.1f}, {y:.1f})"
            if kind == "object_state":
                world.set_object_state(params["object_id"], params["state"])
                return f"{world.objects[params['object_id']].label} is now {params['state']}"
            if kind == "clear":
                world.clear_anomalies()
                return "all anomalies cleared (doors restored, boxes removed)"
        raise ValueError(f"Unknown anomaly {kind!r}")


# =====================================================================================
# Real adapter: tomorrow's hardware target
# =====================================================================================
class RealUnitreeAs2Adapter(BaseRobotAdapter):
    """unitree_sdk2py high-level client for the AS2 EDU.

    Verified against the public SDK: `unitree_sdk2py.as2.sport.sport_client.SportClient`
    (Move, StopMove, StandUp, StandDown, Damp, Euler, RecoveryStand, BalanceStand, GetState)
    and state topic `rt/lf/sportmodestate` (unitree_go SportModeState_). Camera, depth,
    LiDAR, battery and TTS differ per EDU build; plug them in with the register_* hooks.
    """
    kind = "as2"

    def __init__(self, config, clock=None):
        super().__init__(config, clock)
        self.motion_enabled = bool(config.robot.motion_enabled)
        self.armed = False  # the operator arms motion from the dashboard each session
        self._sport = None
        self._subscriber = None
        self._state = None
        self._state_time = -math.inf
        self._lock = threading.RLock()
        self._last_move = -math.inf
        self._moving = False
        self._running = False
        self._threads = []
        self._frame_provider = None     # () -> (jpeg_bytes, width, height) | None
        self._depth_provider = None     # (u, v) -> metres | None, normalized image coordinates
        self._scan_provider = None      # () -> (angles, ranges) | None, body frame
        self._battery_provider = None   # () -> percent | None
        self._tts_provider = None       # (text) -> bool
        self._latest_frame = None
        self._frame_counter = 0
        self._static_map = None

    # ---- hooks for site-specific / proprietary code ---------------------------------
    def register_frame_provider(self, fn): self._frame_provider = fn
    def register_depth_provider(self, fn): self._depth_provider = fn
    def register_scan_provider(self, fn): self._scan_provider = fn
    def register_battery_provider(self, fn): self._battery_provider = fn
    def register_tts(self, fn): self._tts_provider = fn

    def start(self):
        try:
            from unitree_sdk2py.as2.sport.sport_client import SportClient
            from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
            from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_
        except ImportError as exc:
            raise AdapterError("unitree_sdk2py with AS2 support is not installed. On the robot-side Linux "
                               "machine: pip install cyclonedds==0.10.2 && pip install -e unitree_sdk2_python") from exc
        rc = self.config.robot
        ChannelFactoryInitialize(rc.dds_domain, rc.network_interface)
        self._sport = SportClient()
        if hasattr(self._sport, "SetTimeout"):
            self._sport.SetTimeout(rc.sdk_timeout_s)
        self._sport.Init()
        self._subscriber = ChannelSubscriber(rc.state_topic, SportModeState_)
        self._subscriber.Init(self._on_state, 1)
        if rc.static_map_path:
            import json
            from pathlib import Path
            self._static_map = json.loads(Path(self.config.resolve_path(rc.static_map_path)).read_text("utf-8"))
        # TODO: Plug in existing VLM/comms hook here tomorrow: register_frame_provider(),
        # register_depth_provider() (RealSense/Orbbec), register_scan_provider() (Mid-360 / Unitree
        # LiDAR), register_battery_provider() and register_tts() from your onboard code.
        source = self.config.camera.source
        if self._frame_provider is None and source.startswith("opencv:"):
            self._frame_provider = self._opencv_provider(source.split(":", 1)[1])
        self._running = True
        for target, name in ((self._watchdog_loop, "as2-watchdog"), (self._camera_loop, "as2-camera")):
            thread = threading.Thread(target=target, name=name, daemon=True)
            thread.start()
            self._threads.append(thread)

    def close(self):
        if self._sport is not None:
            self._stop_impl()
        self._running = False
        for thread in self._threads:
            thread.join(timeout=1)

    def _on_state(self, msg):
        try:
            state = {"position": list(msg.position), "velocity": list(msg.velocity),
                     "rpy": list(msg.imu_state.rpy), "mode": int(msg.mode),
                     "gait_type": int(getattr(msg, "gait_type", -1)),
                     "body_height": float(getattr(msg, "body_height", 0.0))}
        except (AttributeError, TypeError, ValueError):
            log.exception("Unexpected SportModeState_ layout")
            return
        with self._lock:
            self._state, self._state_time = state, time.monotonic()

    def _watchdog_loop(self):
        # Stops the robot when the engine stops streaming Move() (crash, hang, network loss).
        while self._running:
            with self._lock:
                stale = self._moving and time.monotonic() - self._last_move > self.limits.command_timeout_s
            if stale:
                log.warning("Move() stream stale; sending StopMove()")
                self._stop_impl()
            time.sleep(0.05)

    def _opencv_provider(self, spec):
        try:
            import cv2
        except ImportError as exc:
            raise AdapterError("camera.source=opencv:* needs opencv-python") from exc
        capture = cv2.VideoCapture(int(spec) if spec.isdigit() else spec)
        quality = [int(cv2.IMWRITE_JPEG_QUALITY), int(self.config.camera.jpeg_quality)]

        def provider():
            ok, image = capture.read()
            if not ok:
                return None
            ok, encoded = cv2.imencode(".jpg", image, quality)
            return (encoded.tobytes(), image.shape[1], image.shape[0]) if ok else None
        return provider

    def _camera_loop(self):
        while self._running:
            if self._frame_provider is not None:
                try:
                    result = self._frame_provider()
                except Exception:  # provider code is site-specific; never kill the loop
                    log.exception("Frame provider failed")
                    result = None
                if result:
                    jpeg, width, height = result
                    tel = self.get_telemetry()
                    self._frame_counter += 1
                    cam = self.config.camera
                    self._latest_frame = CameraFrame(
                        frame_id=self._frame_counter, timestamp=time.time(), width=width, height=height,
                        hfov_rad=math.radians(cam.hfov_deg), pose=tel.pose, body_pitch=self._euler[1],
                        camera_offset=list(cam.offset_xyz), jpeg=jpeg, depth_fn=self._depth_provider,
                        source="as2")
            time.sleep(0.1)

    # ---- sensing ------------------------------------------------------------------
    def get_telemetry(self):
        with self._lock:
            state, age = self._state, time.monotonic() - self._state_time
        notes = [] if self.last_refusal is None else [self.last_refusal]
        if state is None or age > 1.0:
            notes.append("No fresh SportModeState on " + self.config.robot.state_topic)
            return Telemetry(source="as2", connected=False, pose=Pose(), velocity=[0, 0, 0],
                             body_euler=list(self._euler), motion_enabled=self.motion_enabled,
                             armed=self.armed, emergency_latched=self.emergency_latched,
                             timestamp=time.time(), notes=notes)
        x, y, z = state["position"][:3]
        yaw = state["rpy"][2]
        # TODO(hardware): confirm SportModeState velocity frame on AS2 (Go2 reports odom frame).
        wvx, wvy = state["velocity"][:2]
        bvx = wvx * math.cos(yaw) + wvy * math.sin(yaw)
        bvy = -wvx * math.sin(yaw) + wvy * math.cos(yaw)
        battery = None
        if self._battery_provider:
            try:
                battery = self._battery_provider()
            except Exception:
                log.exception("Battery provider failed")
        standing = state["body_height"] > 0.2 if state["body_height"] else True
        return Telemetry(source="as2", connected=True, pose=Pose(x=x, y=y, z=z, yaw=yaw),
                         velocity=[bvx, bvy, 0.0], body_euler=list(self._euler), battery_pct=battery,
                         gait_mode=f"gait {state['gait_type']}", fsm_mode=f"mode {state['mode']}",
                         standing=standing, motion_enabled=self.motion_enabled, armed=self.armed,
                         emergency_latched=self.emergency_latched, timestamp=time.time(), notes=notes)

    def get_frame(self):
        return self._latest_frame

    def get_scan(self):
        if self._scan_provider is None:
            return None
        try:
            result = self._scan_provider()
        except Exception:
            log.exception("Scan provider failed")
            return None
        if not result:
            return None
        angles, ranges = result
        return LaserScan(angles=list(angles), ranges=list(ranges), timestamp=time.time())

    # ---- actuation ----------------------------------------------------------------
    def _checked(self, name, code):
        if code != 0:
            log.error("SportClient.%s returned %s", name, code)
            self.last_refusal = f"SDK {name} returned {code}"
            return False
        return True

    def _move_impl(self, vx, vy, vyaw):
        code = self._sport.Move(vx, vy, vyaw)
        with self._lock:
            self._last_move = time.monotonic()
            self._moving = any(abs(v) > 1e-3 for v in (vx, vy, vyaw))
        return self._checked("Move", code)

    def _stop_impl(self):
        with self._lock:
            self._moving = False
        if self._sport is not None:
            self._checked("StopMove", self._sport.StopMove())

    def _euler_impl(self, roll, pitch, yaw):
        if not self.motion_enabled:
            self.last_refusal = "Motion disabled in config"
            return False
        return self._checked("Euler", self._sport.Euler(roll, pitch, yaw))

    def _sport_call(self, name):
        if name not in {"StandUp", "StandDown", "Damp", "RecoveryStand", "BalanceStand"}:
            raise AdapterError(f"Unsupported sport call {name}")
        if self._sport is None:
            return False
        return self._checked(name, getattr(self._sport, name)())

    def _speak_impl(self, text):
        if self._tts_provider:
            return bool(self._tts_provider(text))
        binary = self.config.robot.tts_binary
        if binary:
            # robot/bridge_cpp/as2_tts.cpp wraps unitree::robot::as2::AudioClient::TtsMaker.
            args = [self.config.resolve_path(binary), self.config.robot.network_interface, text,
                    str(self.config.robot.tts_speaker_id)]
            try:
                subprocess.Popen(args, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                return True
            except OSError:
                log.exception("TTS binary failed")
                return False
        # TODO: Plug in existing VLM/comms hook here tomorrow (onboard speaker / AudioClient).
        log.warning("No TTS configured; speech shown in the dashboard only: %s", text)
        return False

    def static_map(self):
        return self._static_map
