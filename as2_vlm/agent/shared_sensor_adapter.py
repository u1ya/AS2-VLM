"""Panel client of the loopback sensor hub. Never launches DDS or sends motion."""
import copy
import io
import json
import threading
import time
import urllib.request
import urllib.error

from PIL import Image

from .robot_adapter import AdapterError, CameraFrame
from .sensor_adapter import ReadOnlySensorAdapter


class HubClient:
    def __init__(self, url):
        from urllib.parse import urlparse
        parsed = urlparse(url)
        if parsed.scheme != "http" or parsed.hostname != "127.0.0.1" or not parsed.port or parsed.path or parsed.username or parsed.query or parsed.fragment:
            raise ValueError("Numeric loopback hub URL required")
        self.url = url
        self.opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))

    def request(self, path, body=None, binary=False):
        data = None if body is None else json.dumps(body, allow_nan=False).encode()
        request = urllib.request.Request(self.url + path, data=data,
            headers={"Content-Type": "application/json", "Origin": self.url})
        limit = 16 * 1024 * 1024 + 8 if binary and path.startswith("/cloud?") else 12 * 1024 * 1024
        with self.opener.open(request, timeout=3) as response:
            raw = response.read(limit + 1)
        if len(raw) > limit:
            raise ValueError("Oversized sensor hub response")
        return raw if binary else json.loads(raw, parse_constant=lambda _: (_ for _ in ()).throw(ValueError("Nonfinite hub JSON")))


class SharedGeometry:
    def __init__(self, adapter):
        self.adapter = adapter

    def payload(self):
        with self.adapter._lock:
            payload = copy.deepcopy(self.adapter._geometry)
            elapsed = time.monotonic() - self.adapter._received
        age = payload.get("age_s")
        payload["age_s"] = age + elapsed if age is not None else None
        payload["connected"] = bool(payload.get("connected") and age is not None and age + elapsed <= 1)
        return payload

    def status(self):
        result = self.payload()
        result.pop("image_png_base64", None)
        return result

    def set_slice(self, z_min, z_max):
        self.adapter._hub.request("/geometry/filter", {"z_min": z_min, "z_max": z_max})

    def export(self, directory):
        # The server owns the fixed private export directory, never the caller.
        return self.adapter._hub.request("/geometry/export", {})


class SharedSensorAdapter(ReadOnlySensorAdapter):
    def __init__(self, config, clock=None):
        super().__init__(config, clock)
        self._hub = HubClient(config.robot.sensor_hub_url)
        self._stop = threading.Event()
        self._geometry = {}
        self._hub_id = None
        self._guard_status = {}
        self.local_geometry = SharedGeometry(self)

    def start(self):
        try:
            health = self._hub.request("/health")
            expected = {"interface": self.config.robot.network_interface,
                        "dds_domain": self.config.robot.dds_domain, "distro": self.config.robot.wsl_distro,
                        "binary": self.config.resolve_path(self.config.robot.sensor_binary)}
            if health.get("read_only") is not True or health.get("profile") != expected:
                raise AdapterError("Sensor hub profile does not match this panel")
            self._hub_id = health["instance"]
        except Exception as exc:
            raise AdapterError(f"Shared sensor hub unavailable; start sensor-hub.ps1: {exc}") from exc
        self._thread = threading.Thread(target=self._poll, name="shared-sensors", daemon=True)
        self._thread.start()
        if not self._ready.wait(8) or self._error:
            self.close()
            raise AdapterError(self._error or "No shared sensor snapshot")

    def _poll(self):
        while not self._stop.is_set():
            try:
                data = self._hub.request("/snapshot")
                if data.get("instance") != self._hub_id:
                    raise ValueError("Hub restarted; reconnect panel to verify profile")
                self._accept({key: data[key] for key in ("state", "lidar", "camera", "recording")})
                with self._lock:
                    self._geometry = data["geometry"]
                    self._guard_status = data.get("collision_guard", {})
                    self._error = None
                camera = data["camera"]
                if camera.get("sequence", 0) > self._frame_sequence and 0 <= camera["age_s"] < 3:
                    # Bind camera bytes to their sequence; retry a newer frame on the next poll.
                    try:
                        jpeg = self._hub.request("/frame?sequence=" + str(camera["sequence"]), binary=True)
                    except urllib.error.HTTPError as exc:
                        if exc.code != 409:
                            raise
                        self._ready.set()
                        self._stop.wait(.2)
                        continue
                    with Image.open(io.BytesIO(jpeg)) as image:
                        if image.width * image.height > 25_000_000:
                            raise ValueError("Oversized shared camera frame")
                        image.verify()
                        width, height = image.size
                    telemetry = self.get_telemetry()
                    with self._lock:
                        self._frame_sequence = camera["sequence"]
                        self._frame_received = self._received - camera["age_s"]
                        self._frame = CameraFrame(self._frame_sequence, time.time()-camera["age_s"], width, height,
                            self.config.camera.hfov_deg * 3.141592653589793 / 180,
                            telemetry.pose, telemetry.body_euler[1], [], jpeg=jpeg, source="as2_videohub",
                            meta={"calibrated": False, "pose_synchronized": False, "receive_time_only": True,
                                  "shared_acquisition": True})
                self._ready.set()
            except Exception as exc:
                with self._lock:
                    self._error = str(exc)
                self._ready.set()
            self._stop.wait(.2)

    def sensor_status(self):
        result = super().sensor_status()
        result["shared_acquisition"] = True
        result["hub_instance"] = self._hub_id
        return result

    def capture_bundle(self, frame):
        """Bind an image to a sequence-checked cloud before slow inference starts.

        This measures receipt-time proximity only, not exposure synchronization.
        A race returns no pair; stale data never silently replaces the named scan.
        """
        from .camera_lidar import capture_receipt_bundle
        with self._lock:
            snapshot = copy.deepcopy(self._snapshot)
            elapsed = time.monotonic() - self._received
            snapshot["instance"] = self._hub_id
        return capture_receipt_bundle(self._hub, snapshot, frame.frame_id, elapsed)

    def collision_status(self):
        with self._lock:
            status = copy.deepcopy(self._guard_status)
            elapsed = time.monotonic() - self._received
        age = status.get("age_s")
        if age is not None:
            status["age_s"] = age + elapsed
        if age is None or status["age_s"] > .3:
            status.update(forward=False, reverse=False, rotate=False)
            if status.get("configured"):
                status["reason"] = "stale_lidar"
        return status

    def close(self):
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=4)
