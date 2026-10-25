import time
import unittest
import threading
from http.client import HTTPConnection
from http.server import ThreadingHTTPServer
from types import SimpleNamespace

from as2_vlm.agent.config import load_config
from as2_vlm.agent.models import Detection
from as2_vlm.agent.robot_adapter import AdapterError, CameraFrame
from as2_vlm.agent.sensor_adapter import ReadOnlySensorAdapter
from as2_vlm.agent.semantic_map import SemanticSceneGraph
from as2_vlm.agent.vlm_backends import build_perception


def snapshot():
    return {"state": {"age_s": 0.02, "position": [1, 2, .31], "velocity": [0, 0, 0],
                      "rpy": [0, 0, .1], "mode": 3, "error": "0"},
            "lidar": {"age_s": .03, "frame": "hesai_lidar", "points": 81633, "hz": 10},
            "camera": {"age_s": .01, "sequence": 1, "code": 0}}


class SensorAdapterTests(unittest.TestCase):
    def setUp(self):
        self.config = load_config(path="", env={})
        self.adapter = ReadOnlySensorAdapter(self.config)

    def test_fresh_and_expired_state(self):
        self.adapter._accept(snapshot())
        self.assertTrue(self.adapter.get_telemetry().connected)
        self.assertTrue(self.adapter.sensor_status()["connected"])
        self.assertEqual(self.adapter.get_telemetry().pose.x, 1)
        self.adapter._received -= 2
        self.assertFalse(self.adapter.get_telemetry().connected)
        self.assertFalse(self.adapter.sensor_status()["connected"])

    def test_no_navigation_scan_without_transform(self):
        self.adapter._accept(snapshot())
        self.assertIsNone(self.adapter.get_scan())
        self.assertFalse(self.adapter.sensor_status()["body_transform_verified"])

    def test_no_command_channel(self):
        self.assertFalse(self.adapter.set_armed(True))
        self.assertFalse(self.adapter.move(.1, 0, 0))
        self.assertFalse(self.adapter.euler(0, .1, 0))
        self.assertFalse(self.adapter.stand_up())
        self.assertFalse(self.adapter.stand_down())
        self.adapter.stop_move()
        self.adapter.close()

    def test_motion_cannot_be_enabled(self):
        with self.assertRaises(ValueError):
            load_config(path="", env={}, overrides={"robot": {"motion_enabled": True}})
        self.config.robot.motion_enabled = True
        with self.assertRaises(AdapterError):
            ReadOnlySensorAdapter(self.config)

    def test_invalid_sensor_responses(self):
        bad = [{}, {"state": [], "lidar": {}, "camera": {}}]
        for field in ("position", "velocity", "rpy"):
            value = snapshot()
            value["state"][field] = [float("nan"), 0, 0]
            bad.append(value)
        value = snapshot(); value["lidar"]["age_s"] = float("inf"); bad.append(value)
        for value in bad:
            with self.subTest(value=value), self.assertRaises(ValueError):
                self.adapter._accept(value)

    def test_camera_expires(self):
        self.adapter._frame = object()
        self.adapter._frame_received = time.monotonic()
        self.assertIsNotNone(self.adapter.get_frame())
        self.adapter._frame_received -= 4
        self.assertIsNone(self.adapter.get_frame())

    def test_uncalibrated_frames_cannot_create_metric_nodes(self):
        frame = CameraFrame(1, time.time(), 1920, 1080, 1.5,
                            self.adapter.get_telemetry().pose, 0, [], meta={"calibrated": False})
        graph = SemanticSceneGraph()
        try:
            detection = Detection(label="door", bbox=(.1, .1, .5, .9), confidence=.9, depth_m=2)
            self.assertIsNone(graph.ingest_detection(detection, frame))
            self.assertTrue(graph.is_empty())
        finally:
            graph.close()

    def test_sensor_profile_needs_no_model_server(self):
        cfg = load_config("configs/agent.as2-sensors.json", env={})
        self.assertEqual(build_perception(cfg.vlm, False).detect_objects(None), [])
        self.assertEqual(cfg.vlm.model, "qwen2.5vl:3b")
        self.assertFalse(cfg.robot.motion_enabled)

    def test_http_commands_are_rejected_without_touching_engine(self):
        from as2_vlm.agent.app import Handler
        class TestHandler(Handler):
            runtime = SimpleNamespace(adapter=SimpleNamespace(read_only=True))
        server = ThreadingHTTPServer(("127.0.0.1", 0), TestHandler)
        worker = threading.Thread(target=server.serve_forever, daemon=True)
        worker.start()
        try:
            for path in ("/api/command", "/api/override"):
                connection = HTTPConnection("127.0.0.1", server.server_port, timeout=3)
                try:
                    connection.request("POST", path, b'{"action":"emergency_sit"}',
                                       {"Content-Type": "application/json"})
                    response = connection.getresponse()
                    self.assertEqual(response.status, 403)
                    response.read()
                finally:
                    connection.close()
        finally:
            server.shutdown()
            server.server_close()
            worker.join(timeout=2)
