"""Read-only AS2 mini-map: shared live sensors or explicitly labelled private replay.

Camera observations stay in image coordinates until extrinsics and timing are known.
This module never constructs an execution engine or a motion transport.
"""
import argparse
import base64
import copy
from dataclasses import replace
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import io
import json
from pathlib import Path
import threading
import time

from PIL import Image

from .config import load_config
from .camera_lidar import CameraLidarCalibration, associate
from .lidar_geometry import LocalLidarGeometry
from .models import Pose
from .robot_adapter import CameraFrame
from .sensor_audit import SensorRecording, host_ns
from .shared_sensor_adapter import SharedSensorAdapter
from .vlm_backends import DETECT_PROMPT, VLMPerception, build_chat

MINIMAP_PROMPT = DETECT_PROMPT + """
For this mini-map, focus on discrete objects: people, doors, boxes, chairs, tables,
cabinets, equipment and floor hazards. Do not list floor, ceiling, walls or ceiling
lights. An empty floor is not an obstacle. Cardboard boxes have category obstacle,
not furniture. Bound the actual visible object tightly, not empty space around it.
"""
BACKGROUND_LABELS = {"floor", "empty floor", "ceiling", "wall", "walls", "light", "lights", "ceiling lights"}
TARGET_PROMPT = """
Focus ONLY on visible people and waste/trash/rubbish bins (including small office
bins). Use label person, category person for a human; label trash_bin, category
other for a waste bin. Do not report doors, floors, chairs, cabinets or ordinary
boxes. A partially visible object is allowed only if its type is actually visible.
Return an empty objects list if neither target is present. Do not infer an object
from context. Boxes must enclose the visible object, with no invented hidden parts.
"""


def target_kind(detection):
    label = detection.label.strip().lower().replace("_", " ").replace("-", " ")
    if detection.category == "person" or label in {"person", "human", "man", "woman", "человек", "мужчина", "женщина"}:
        return "person"
    if label in {"trash bin", "waste bin", "wastebin", "trash can", "garbage bin", "garbage can",
                 "rubbish bin", "dustbin", "bin", "мусорка", "урна", "мусорное ведро"}:
        return "trash_bin"
    return None


def small_frame(frame):
    """Preserve aspect ratio and normalized box coordinates; bound laptop VRAM load."""
    with Image.open(io.BytesIO(frame.jpeg)) as image:
        if image.width * image.height > 25_000_000:
            raise ValueError("Camera dimensions exceed limit")
        image = image.convert("RGB")
        image.thumbnail((768, 768))
        output = io.BytesIO()
        image.save(output, "JPEG", quality=88)
        return replace(frame, jpeg=output.getvalue(), width=image.width, height=image.height)


def inference_frame(frame):
    """Square model input; normalized boxes map back by independent X/Y scaling.

    On this AS2 16:9 still, the installed 3B model misplaced boxes with rectangular
    input. Square resampling improved the manually reviewed box overlap. It is an
    empirical input convention, not a guarantee of grounding accuracy.
    """
    with Image.open(io.BytesIO(frame.jpeg)) as image:
        output = io.BytesIO()
        image.convert("RGB").resize((768, 768), Image.Resampling.LANCZOS).save(output, "JPEG", quality=88)
    return replace(frame, jpeg=output.getvalue(), width=768, height=768)


class RecordedSensors:
    """Replay cloud/odometry receipt times, with an explicitly UNPAIRED camera still."""
    replay = True

    def __init__(self, directory, image):
        self.recording = SensorRecording(directory)
        self.now = 0.0
        self.local_geometry = LocalLidarGeometry(clock=lambda: self.now)
        self.complete = False
        self.error = None
        self.index = 0
        self.stop = threading.Event()
        self.thread = None
        raw = Path(image).read_bytes()
        if len(raw) > 10 * 1024 * 1024:
            raise ValueError("Oversized replay image")
        self.frame = small_frame(CameraFrame(1, Path(image).stat().st_mtime, 0, 0, 0,
            Pose(), 0, [], jpeg=raw, source="as2_recorded_unpaired_still",
            meta={"calibrated": False, "pose_synchronized": False}))

    def start(self):
        if not self.recording.clouds or not self.recording.odometry:
            raise ValueError("Replay requires clouds and odometry")
        self.thread = threading.Thread(target=self._run, daemon=True, name="minimap-replay")
        self.thread.start()

    def _run(self):
        try:
            origin = host_ns(self.recording.clouds[0])
            odometry = self.recording.odometry
            previous = 0.0
            for metadata, xyz in self.recording.iter_clouds():
                elapsed = (host_ns(metadata) - origin) / 1e9
                if self.stop.wait(min(1.0, max(0, elapsed - previous))):
                    return
                self.now = elapsed
                previous = elapsed
                pose = min(odometry, key=lambda row: abs(host_ns(row) - host_ns(metadata)))
                age = abs(host_ns(pose) - host_ns(metadata)) / 1e9
                self.local_geometry.ingest(xyz, {**metadata, "age_s": 0, "error": 0},
                    {**pose, "age_s": age})
                self.index += 1
            self.complete = True
        except Exception as exc:
            self.error = str(exc)
            self.local_geometry.invalidate(self.error)

    def get_frame(self):
        return self.frame

    def close(self):
        self.stop.set()
        if self.thread:
            self.thread.join(3)


class MiniMap:
    def __init__(self, source, perception, interval=8, threshold=.5, calibration=None, targets_only=False, person_detector=None):
        self.source, self.perception = source, perception
        self.interval, self.threshold = interval, threshold
        self.stop = threading.Event()
        self.lock = threading.Lock()
        self.observation = None
        self.status = "waiting_camera"
        self.error = None
        self.thread = None
        self.calibration = calibration
        self.targets_only = targets_only
        self.person_stream = None
        if person_detector is not None:
            from .person_detection import PersonStream
            self.person_stream = PersonStream(source,person_detector,small_frame)

    def start(self):
        self.source.start()
        if self.person_stream:
            self.person_stream.start()
        self.thread = threading.Thread(target=self._recognize, daemon=True, name="minimap-vision")
        self.thread.start()

    def _recognize(self):
        last = None
        while not self.stop.is_set():
            frame = self.source.get_frame()
            if frame is None:
                with self.lock:
                    self.status = "waiting_camera"
                self.stop.wait(.5)
                continue
            if frame.frame_id == last:
                self.stop.wait(.5)
                continue
            try:
                with self.lock:
                    self.status, self.error = "recognizing", None
                started = time.monotonic()
                bundle, pair_error = None, None
                if hasattr(self.source, "capture_bundle"):
                    # Camera is 1 Hz; wait briefly for its next fresh frame instead
                    # of always choosing an old image after a slow model response.
                    pair_deadline = time.monotonic() + 1.5
                    while not self.stop.is_set():
                        try:
                            bundle = self.source.capture_bundle(frame)
                        except (ValueError, OSError) as exc:
                            pair_error = str(exc)
                        if bundle is not None or time.monotonic() >= pair_deadline:
                            break
                        self.stop.wait(.05)
                        newest = self.source.get_frame()
                        if newest is not None:
                            frame = newest
                prepared = small_frame(frame)
                detections = self.perception.detect_objects(inference_frame(prepared))
                rejected = [d.label for d in detections if d.label.strip().lower() in BACKGROUND_LABELS]
                accepted = [d for d in detections if d.confidence >= self.threshold
                            and d.label.strip().lower() not in BACKGROUND_LABELS
                            and (not self.targets_only or target_kind(d) is not None)]
                try:
                    fusion = associate(accepted, bundle, self.calibration, (frame.width, frame.height))
                except ValueError as exc:
                    fusion = {"status": "calibration_or_cloud_error", "error": str(exc),
                              "navigation_ready": False, "association_verified": False, "objects": []}
                observation = {"frame_id": frame.frame_id, "source": frame.source,
                    "received_at": frame.timestamp, "inference_s": time.monotonic() - started,
                    "image_jpeg_base64": base64.b64encode(prepared.jpeg).decode("ascii"),
                    "objects": [{**d.model_dump(), "target_kind": target_kind(d)} for d in accepted],
                    "rejected_background_labels": rejected,
                    "coordinates": "normalized_image_only", "metric_positions": False,
                    "grounding_verified": False, "model_input": "square_768",
                    "pair": bundle.status() if bundle is not None else None,
                    "pair_error": pair_error, "fusion": fusion,
                    "targets": ["person", "trash_bin"] if self.targets_only else "all"}
                with self.lock:
                    self.observation = observation
                    self.status, self.error = "ready", None
                last = frame.frame_id
            except Exception as exc:
                with self.lock:
                    self.status, self.error = "error", str(exc)
            self.stop.wait(self.interval)

    def payload(self):
        replay = bool(getattr(self.source, "replay", False))
        geometry = self.source.local_geometry.payload()
        if replay:
            geometry = {**geometry, "connected": False, "replay": True,
                        "replay_complete": self.source.complete, "replayed_clouds": self.source.index,
                        "replay_error": self.source.error}
        frame = self.source.get_frame()
        camera = None
        if frame is not None:
            camera = {"frame_id": frame.frame_id, "source": frame.source,
                      "image_jpeg_base64": base64.b64encode(small_frame(frame).jpeg).decode("ascii")}
        with self.lock:
            observation = copy.deepcopy(self.observation)
            status, error = self.status, self.error
        if observation:
            observation["age_s"] = max(0, time.time() - observation["received_at"])
            observation["current"] = not replay and frame is not None and observation["age_s"] <= 3
        return {"mode": "recorded_as2" if replay else "live_as2", "motion_enabled": False,
                "fast_person": self.person_stream.payload() if self.person_stream else None,
                "collision_guard": self.source.collision_status() if hasattr(self.source, "collision_status") else None,
                "camera_lidar_synchronized": False, "geometry": geometry, "camera": camera,
                "recognition": {"status": status, "error": error, "observation": observation}}

    def close(self):
        self.stop.set()
        if self.person_stream:
            self.person_stream.close()
        self.source.close()
        if self.thread:
            self.thread.join(1)


class Handler(BaseHTTPRequestHandler):
    scene = None

    def log_message(self, *args):
        pass

    def do_GET(self):
        if self.headers.get("Host") != f"127.0.0.1:{self.server.server_port}":
            self.send_error(403)
            return
        if self.path == "/":
            raw = Path(__file__).with_name("static").joinpath("minimap.html").read_bytes()
            content_type = "text/html; charset=utf-8"
        elif self.path == "/api/scene":
            raw = json.dumps(self.scene.payload(), ensure_ascii=False, allow_nan=False).encode()
            content_type = "application/json; charset=utf-8"
        else:
            self.send_error(404)
            return
        self.send_response(200)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(raw)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.end_headers()
        self.wfile.write(raw)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default="configs/agent.as2-sensors.json")
    parser.add_argument("--port", type=int, default=8772)
    parser.add_argument("--recording", help="Private AS2 recording directory (offline mode)")
    parser.add_argument("--image", help="Unpaired real AS2 camera still, required for offline mode")
    parser.add_argument("--calibration", help="Measured camera intrinsics and T_camera_from_lidar JSON")
    parser.add_argument("--all-objects", action="store_true", help="Detect all classes instead of person/trash bin focus")
    parser.add_argument("--fast-person-model", help="Pinned local OpenCV Zoo YOLOX int8 model; enables independent person stream")
    args = parser.parse_args(argv)
    if bool(args.recording) != bool(args.image):
        parser.error("--recording and --image must be used together")
    config = load_config(args.config)
    if config.robot.motion_enabled or config.robot.transport != "cpp_readonly":
        parser.error("Mini-map requires cpp_readonly sensors with motion disabled")
    config.vlm.perception_backend = "ollama"
    config.vlm.timeout_s = 90
    source = RecordedSensors(args.recording, args.image) if args.recording else SharedSensorAdapter(config)
    calibration = CameraLidarCalibration.load(args.calibration) if args.calibration else None
    prompt = MINIMAP_PROMPT if args.all_objects else MINIMAP_PROMPT + TARGET_PROMPT
    perception = VLMPerception(build_chat("ollama", config.vlm), "ollama", detection_prompt=prompt)
    person_detector = None
    if args.fast_person_model:
        from .person_detection import PersonDetector
        person_detector = PersonDetector(args.fast_person_model)
    scene = MiniMap(source, perception, interval=2, calibration=calibration, targets_only=not args.all_objects,
                   person_detector=person_detector)
    handler = type("MiniMapHandler", (Handler,), {"scene": scene})
    server = ThreadingHTTPServer(("127.0.0.1", args.port), handler)
    server.daemon_threads = True
    try:
        scene.start()
        print(f"AS2 mini-map ({'REPLAY' if args.recording else 'LIVE'}): http://127.0.0.1:{server.server_port}/", flush=True)
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        scene.close()
        server.server_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
