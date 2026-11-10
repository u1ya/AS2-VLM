"""Single loopback owner of read-only AS2 acquisition and bounded raw recording."""
import argparse
import copy
import json
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
import re
import struct
import threading
import time
from urllib.parse import parse_qs, urlparse
import uuid

from .config import load_config
from .sensor_adapter import ReadOnlySensorAdapter


class SensorHub:
    def __init__(self, config):
        if config.robot.motion_enabled or config.robot.transport != "cpp_readonly":
            raise ValueError("Hub supports read-only C++ sensors only")
        self.config = config
        self.owner = ReadOnlySensorAdapter(config)
        self.instance = uuid.uuid4().hex
        self.lock = threading.Lock()
        self.pending = None

    def health(self):
        rc = self.config.robot
        return {"schema": 1, "instance": self.instance, "read_only": True,
                "profile": {"interface": rc.network_interface, "dds_domain": rc.dds_domain,
                            "distro": rc.wsl_distro, "binary": self.config.resolve_path(rc.sensor_binary)},
                "connected": self.owner.get_telemetry().connected,
                "acquisition_running": self.owner._process is not None and self.owner._process.poll() is None,
                "acquisition_error": self.owner.sensor_status().get("error_message")}

    def snapshot(self):
        with self.owner._lock:
            data = copy.deepcopy(self.owner._snapshot)
            elapsed = time.monotonic() - self.owner._received
        if not data:
            # Open the panel honestly disconnected; never synthesize fresh robot telemetry.
            data = {"state": {"age_s": -1}, "lidar": {"age_s": -1},
                    "camera": {"age_s": -1, "sequence": 0}, "recording": {}}
        for key in ("state", "lidar", "camera"):
            age = data[key].get("age_s", -1)
            if age >= 0:
                data[key]["age_s"] = age + elapsed
        data.setdefault("recording", {})
        data["geometry"] = self.owner.local_geometry.payload()
        data["collision_guard"] = self.owner.collision_status()
        data["instance"] = self.instance
        return data

    def record(self, body):
        if set(body) != {"name", "seconds"}:
            raise ValueError("Expected only name and seconds")
        name, seconds = body["name"], body["seconds"]
        if not isinstance(name, str) or not re.fullmatch(r"[A-Za-z0-9_-]{1,80}", name):
            raise ValueError("Invalid recording name")
        if type(seconds) is not int or not 3 <= seconds <= 30:
            raise ValueError("Duration must be 3-30 seconds")
        root = Path(self.config.resolve_path("recordings")).resolve()
        path = (root / name).resolve()
        if not path.is_relative_to(root) or path.exists():
            raise ValueError("New private recording directory required")
        with self.lock:
            status = self.snapshot()["recording"]
            if status.get("command_error") and not status.get("active"):
                self.pending = None
            if status.get("active") or (self.pending and not (
                    status.get("name") == self.pending and status.get("finished"))):
                raise ValueError("Recording already active or awaiting acknowledgement")
            process = self.owner._process
            if not process or process.poll() is not None:
                raise ValueError("Acquisition process unavailable")
            process.stdin.write(f"RECORD\t{seconds}\t{self.owner._linux_path(path)}\n".encode())
            process.stdin.flush()
            self.pending = name
        return {"accepted": True, "name": name, "seconds": seconds,
                "note": "Check /snapshot recording.finished and recording.ok before using data"}

    def cloud(self, sequence):
        """Return exactly the named latest cloud, never substitute a newer scan."""
        with self.owner._lock:
            lidar = self.owner._snapshot.get("lidar", {})
            age = lidar.get("age_s", -1)
            if (sequence != str(lidar.get("cloud_sequence")) or age < 0 or
                    age + time.monotonic() - self.owner._received > .5):
                return None
            # Reuse the immutable bytes already validated by the single owner;
            # an HTTP reader must not reopen/lock the WSL writer's live file.
            raw = self.owner._cloud_raw
            if raw is None or sequence != str(self.owner._cloud_sequence):
                return None
        if len(raw) < 8 or len(raw) > 16 * 1024 * 1024 + 8:
            raise ValueError("Invalid cloud size")
        if str(struct.unpack_from("<Q", raw)[0]) != sequence:
            return None
        return raw


class Handler(BaseHTTPRequestHandler):
    hub = None

    def log_message(self, *args):
        pass

    def send(self, status, data, content_type="application/json"):
        raw = json.dumps(data, allow_nan=False).encode() if content_type == "application/json" else data
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(raw)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(raw)

    def do_GET(self):
        if self.headers.get("Host") != "127.0.0.1:" + str(self.server.server_port):
            self.send(403, {"error": "Host rejected"})
            return
        try:
            parsed = urlparse(self.path)
            if parsed.path == "/health":
                self.send(200, self.hub.health())
            elif parsed.path == "/snapshot":
                self.send(200, self.hub.snapshot())
            elif parsed.path == "/frame":
                frame = self.hub.owner.get_frame()
                if frame is None or parse_qs(parsed.query).get("sequence") != [str(frame.frame_id)]:
                    self.send(409, {"error": "Frame changed or stale"})
                else:
                    self.send(200, frame.jpeg, "image/jpeg")
            elif parsed.path == "/cloud":
                sequences = parse_qs(parsed.query).get("sequence", [])
                raw = self.hub.cloud(sequences[0]) if len(sequences) == 1 else None
                if raw is None:
                    self.send(409, {"error": "Cloud changed or stale"})
                else:
                    self.send(200, raw, "application/octet-stream")
            else:
                self.send(404, {"error": "Unknown sensor endpoint"})
        except (ValueError, KeyError, TypeError, OSError) as exc:
            self.send(503, {"error": str(exc)})

    def do_POST(self):
        expected = "http://127.0.0.1:" + str(self.server.server_port)
        if self.headers.get("Origin") not in (None, expected) or self.headers.get("Host") != expected[7:]:
            self.send(403, {"error": "Origin or Host rejected"})
            return
        try:
            size = int(self.headers.get("Content-Length", "0"))
            if not 0 < size <= 2048 or self.headers.get_content_type() != "application/json":
                raise ValueError("Small JSON body required")
            body = json.loads(self.rfile.read(size))
            if not isinstance(body, dict):
                raise ValueError("Expected JSON object")
            if self.path == "/record":
                self.send(202, self.hub.record(body))
            elif self.path == "/geometry/filter":
                if set(body) != {"z_min", "z_max"}:
                    raise ValueError("Expected z_min and z_max")
                self.hub.owner.local_geometry.set_slice(body["z_min"], body["z_max"])
                self.send(200, {"ok": True})
            elif self.path == "/geometry/export":
                if body:
                    raise ValueError("Export does not accept paths")
                self.send(200, self.hub.owner.local_geometry.export(self.hub.config.resolve_path("data/geometry")))
            else:
                self.send(404, {"error": "Unknown sensor endpoint"})
        except (ValueError, TypeError, OSError) as exc:
            self.send(400, {"error": str(exc)})


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default="configs/agent.as2-sensors.json")
    parser.add_argument("--port", type=int, default=8780)
    args = parser.parse_args(argv)
    hub = SensorHub(load_config(args.config))
    handler = type("BoundSensorHandler", (Handler,), {"hub": hub})
    # Bind before DDS starts: a second hub on the same port cannot create a reader.
    server = ThreadingHTTPServer(("127.0.0.1", args.port), handler)
    server.daemon_threads = True
    try:
        hub.owner.start()
        print(f"Read-only shared sensor hub: http://127.0.0.1:{server.server_port}", flush=True)
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        hub.owner.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
