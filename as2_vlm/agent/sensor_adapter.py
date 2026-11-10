"""Read-only C++/WSL sensor transport for the verified AS2 configuration."""
import io
import json
import logging
import math
import os
from pathlib import Path
import re
import shutil
import subprocess
import tempfile
import threading
import time

from .models import Pose, Telemetry
from .robot_adapter import AdapterError, BaseRobotAdapter, CameraFrame
from .lidar_geometry import LocalLidarGeometry, decode_xyz
from .obstacle_guard import ObstacleGuard, movement_permission


class ReadOnlySensorAdapter(BaseRobotAdapter):
    kind = "as2"
    read_only = True

    def __init__(self, config, clock=None):
        super().__init__(config, clock)
        if config.robot.motion_enabled:
            raise AdapterError("cpp_readonly transport cannot enable motion")
        self._process = None
        self._thread = None
        self._temporary = None
        self._lock = threading.RLock()
        self._ready = threading.Event()
        self._snapshot = {}
        self._received = -math.inf
        self._frame = None
        self._frame_received = -math.inf
        self._frame_sequence = 0
        self._cloud_raw = None
        self._cloud_sequence = 0
        self._error = None
        self.local_geometry = LocalLidarGeometry()
        path = config.robot.lidar_body_calibration_path
        self.obstacle_guard = ObstacleGuard.load(config.resolve_path(path) if path else None)

    @staticmethod
    def _linux_path(path):
        path = Path(path).resolve()
        if os.name != "nt":
            return str(path)
        if not re.fullmatch(r"[A-Za-z]:", path.drive):
            raise AdapterError("Sensor paths must be on a local Windows drive")
        return "/mnt/" + path.drive[0].lower() + path.as_posix()[2:]

    def start(self):
        rc = self.config.robot
        if rc.dds_domain != 0:
            raise AdapterError("Verified sensor bridge supports DDS domain 0 only")
        for text in (rc.network_interface, rc.wsl_distro):
            if not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_.:-]{0,63}", text):
                raise AdapterError("Invalid interface or WSL distribution")
        binary = Path(self.config.resolve_path(rc.sensor_binary))
        if not binary.is_file():
            raise AdapterError("Build as2_sensor_stream in WSL before live mode")
        root = Path(self.config.resolve_path(".runtime/agent-sensors"))
        root.mkdir(parents=True, exist_ok=True)
        self._temporary = tempfile.TemporaryDirectory(dir=root)
        self._image_path = Path(self._temporary.name) / "frame.jpg"
        args = [self._linux_path(binary), rc.network_interface, self._linux_path(self._image_path)]
        if os.name == "nt":
            wsl = shutil.which("wsl.exe")
            if not wsl:
                self.close()
                raise AdapterError("WSL is unavailable")
            args = [wsl, "-d", rc.wsl_distro, "--", *args]
        try:
            log = Path(self.config.resolve_path("logs/sensor-bridge.stderr.log"))
            log.parent.mkdir(parents=True, exist_ok=True)
            with log.open("ab") as errors:
                self._process = subprocess.Popen(args, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                                 stderr=errors, shell=False)
            self._thread = threading.Thread(target=self._read_loop, name="as2-sensors", daemon=True)
            self._thread.start()
            if not self._ready.wait(12) or self._error:
                raise AdapterError(self._error or "No sensor bridge response; check Ethernet and DDS")
        except Exception:
            self.close()
            raise

    def _accept(self, snapshot):
        if not isinstance(snapshot, dict) or not {"state", "lidar", "camera"} <= set(snapshot) or set(snapshot) - {"state", "lidar", "camera", "recording"}:
            raise ValueError("Invalid sensor envelope")
        if any(not isinstance(value, dict) for value in snapshot.values()):
            raise ValueError("Invalid sensor objects")
        for name in ("state", "lidar", "camera"):
            age = snapshot[name].get("age_s")
            if type(age) not in (int, float) or not math.isfinite(age) or age < -1:
                raise ValueError("Invalid sensor age")
        state = snapshot["state"]
        if state["age_s"] >= 0:
            for name in ("position", "velocity", "rpy"):
                values = state.get(name)
                if (not isinstance(values, list) or len(values) != 3 or
                        any(type(v) not in (int, float) or not math.isfinite(v) for v in values)):
                    raise ValueError("Invalid odometry")
        with self._lock:
            self._snapshot = snapshot
            self._received = time.monotonic()

    def _read_loop(self):
        try:
            for raw in iter(lambda: self._process.stdout.readline(65537), b""):
                if len(raw) > 65536 or not raw.endswith(b"\n"):
                    raise ValueError("Oversized sensor response")
                snapshot = json.loads(raw, parse_constant=lambda _: (_ for _ in ()).throw(ValueError("Nonfinite JSON")))
                self._accept(snapshot)
                lidar = snapshot["lidar"]
                if "cloud_sequence" in lidar:
                    try:
                        with Path(str(self._image_path) + ".cloud").open("rb") as source:
                            raw_cloud = source.read(16 * 1024 * 1024 + 9)
                        xyz = decode_xyz(raw_cloud, lidar)
                        if xyz is not None:
                            with self._lock:
                                self._cloud_raw = raw_cloud
                                self._cloud_sequence = lidar["cloud_sequence"]
                            self.obstacle_guard.ingest(xyz, lidar, snapshot["state"])
                            self.local_geometry.ingest(xyz, lidar, snapshot["state"])
                    except (OSError, ValueError) as exc:
                        self.local_geometry.invalidate(str(exc))
                self._ready.set()
                camera = snapshot["camera"]
                if camera.get("sequence", 0) > self._frame_sequence and camera["age_s"] < 3:
                    with self._image_path.open("rb") as source:
                        jpeg = source.read(10 * 1024 * 1024 + 1)
                    if len(jpeg) > 10 * 1024 * 1024:
                        raise ValueError("Oversized camera frame")
                    from PIL import Image
                    with Image.open(io.BytesIO(jpeg)) as image:
                        if image.width * image.height > 25_000_000:
                            raise ValueError("Oversized camera dimensions")
                        image.verify()
                        width, height = image.size
                    telemetry = self.get_telemetry()
                    with self._lock:
                        self._frame_sequence = camera["sequence"]
                        self._frame_received = time.monotonic() - camera["age_s"]
                        self._frame = CameraFrame(
                            frame_id=self._frame_sequence, timestamp=time.time() - camera["age_s"],
                            width=width, height=height, hfov_rad=math.radians(self.config.camera.hfov_deg),
                            pose=telemetry.pose, body_pitch=telemetry.body_euler[1], camera_offset=[],
                            jpeg=jpeg, source="as2_videohub", meta={"calibrated": False,
                                "pose_synchronized": False, "receive_time_only": True})
        except Exception as exc:
            with self._lock:
                self._error = str(exc)
            logging.exception("AS2 sensor reader failed")
        finally:
            with self._lock:
                if self._error is None:
                    self._error = "Sensor acquisition stream ended; inspect logs/sensor-bridge.stderr.log"
            self._ready.set()

    def sensor_status(self):
        with self._lock:
            elapsed = time.monotonic() - self._received
            result = dict(self._snapshot.get("lidar", {}))
            age = result.get("age_s", -1)
            result["age_s"] = age + elapsed if age >= 0 else None
            result["connected"] = age >= 0 and age + elapsed <= 1.0
            result["body_transform_verified"] = False
            result["scan_for_navigation"] = False
            result["error_message"] = self._error
            result["geometry"] = self.local_geometry.status()
            result["collision_guard"] = self.collision_status()
            return result

    def collision_status(self):
        return self.obstacle_guard.status()

    def movement_permission(self, mode, amount):
        return movement_permission(self.collision_status(), mode, amount)

    def get_telemetry(self):
        with self._lock:
            state = dict(self._snapshot.get("state", {}))
            elapsed = time.monotonic() - self._received
        age = state.get("age_s", -1)
        fresh = age >= 0 and age + elapsed <= 1.0
        position = state.get("position", [0, 0, 0])
        rpy = state.get("rpy", [0, 0, 0])
        return Telemetry(source="as2_cpp_readonly", connected=fresh,
                         pose=Pose(x=position[0], y=position[1], z=position[2], yaw=rpy[2]),
                         velocity=[0, 0, 0], body_euler=rpy,
                         gait_mode="unknown", fsm_mode=f"mode {state.get('mode', 'unknown')}",
                         standing=fresh and position[2] > 0.2, timestamp=time.time(),
                         notes=["Read-only sensors; no motion API. LiDAR/body and camera calibration unverified.",
                                "Velocity frame unverified; command-axis velocity not exposed."] +
                               ([] if fresh else ["Odometry missing or stale"]))

    def get_frame(self):
        with self._lock:
            return self._frame if time.monotonic() - self._frame_received < 3 else None

    def get_scan(self):
        # Raw lidar frame is not a calibrated body-frame obstacle scan.
        return None

    def set_armed(self, armed):
        self.armed = False
        self.last_refusal = "Read-only sensor transport cannot arm motion"
        return False

    def _move_impl(self, vx, vy, vyaw): return False
    def _stop_impl(self): pass
    def _euler_impl(self, roll, pitch, yaw): return False
    def _sport_call(self, name): return False
    def _speak_impl(self, text): return False

    def close(self):
        process = self._process
        if process:
            if process.poll() is None:
                try:
                    process.stdin.write(b"Q\n")
                    process.stdin.flush()
                    process.wait(timeout=6)
                except (OSError, subprocess.TimeoutExpired):
                    process.terminate()
                    process.wait(timeout=3)
            if self._thread:
                self._thread.join(timeout=3)
            process.stdin.close()
            process.stdout.close()
            self._process = None
        if self._temporary:
            self._temporary.cleanup()
            self._temporary = None
