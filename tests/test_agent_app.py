import json
import threading
import time
import unittest
from http.client import HTTPConnection
from http.server import ThreadingHTTPServer

try:
    import pydantic  # noqa: F401
except ImportError:
    raise unittest.SkipTest("agent extra (pydantic) is not installed")

from as2_vlm.agent.app import AgentRuntime, Handler
from as2_vlm.agent.config import load_config


class DashboardApiTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        config = load_config(path="", overrides={"simulation_mode": True, "sim_speedup": 20,
                                                 "sim_map_db_path": ":memory:"})
        cls.runtime = AgentRuntime(config)
        Handler.runtime = cls.runtime
        cls.server = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
        cls.server.daemon_threads = True
        cls.port = cls.server.server_address[1]
        threading.Thread(target=cls.server.serve_forever, daemon=True).start()

    @classmethod
    def tearDownClass(cls):
        cls.server.shutdown()
        cls.server.server_close()
        cls.runtime.shutdown()

    def request(self, method, path, body=None, headers=None):
        connection = HTTPConnection("127.0.0.1", self.port, timeout=10)
        payload = None if body is None else (body if isinstance(body, bytes) else json.dumps(body).encode())
        connection.request(method, path, payload, {"Content-Type": "application/json", **(headers or {})})
        response = connection.getresponse()
        data = response.read()
        connection.close()
        return response.status, data, response

    def test_dashboard_and_state(self):
        status, page, _ = self.request("GET", "/")
        self.assertEqual(status, 200)
        self.assertIn(b"AS2 Mission Control", page)
        status, data, _ = self.request("GET", "/api/state")
        state = json.loads(data)
        self.assertEqual(state["mode"], "mock")
        self.assertIn("battery_pct", state["telemetry"])

    def test_map_payload_is_seeded(self):
        status, data, _ = self.request("GET", "/api/map")
        payload = json.loads(data)
        labels = {n["label"] for n in payload["nodes"]}
        self.assertTrue({"stove", "back door", "charging dock", "kitchen"} <= labels)
        self.assertEqual(len(payload["static"]["rooms"]), 5)

    def test_cross_origin_post_rejected(self):
        status, _, _ = self.request("POST", "/api/override", {"action": "pause"}, {"Origin": "http://evil.example"})
        self.assertEqual(status, 403)

    def test_invalid_requests_rejected(self):
        self.assertEqual(self.request("POST", "/api/override", {"action": "LowCmd"})[0], 400)
        self.assertEqual(self.request("POST", "/api/command", {"text": 5})[0], 400)
        self.assertEqual(self.request("POST", "/api/frame", b"not a jpeg", {"Content-Type": "image/jpeg"})[0], 400)

    def test_command_round_trip(self):
        status, data, _ = self.request("POST", "/api/command", {"text": "go to the kitchen", "source": "voice"})
        self.assertEqual(status, 200)
        self.assertTrue(json.loads(data)["accepted"])
        deadline = time.time() + 30
        while self.runtime.engine.is_busy() and time.time() < deadline:
            time.sleep(0.05)
        self.assertEqual(self.runtime.engine.state, "SUCCEEDED")

    def test_camera_frame(self):
        status, data, response = self.request("GET", "/api/camera.jpg")
        if status == 204:
            self.skipTest("Pillow not installed")
        self.assertEqual(status, 200)
        self.assertTrue(data.startswith(b"\xff\xd8\xff"))
        self.assertTrue(response.getheader("X-Frame-Id"))

    def test_event_stream_sends_state(self):
        connection = HTTPConnection("127.0.0.1", self.port, timeout=10)
        connection.request("GET", "/api/events")
        response = connection.getresponse()
        self.assertEqual(response.getheader("Content-Type"), "text/event-stream")
        seen = b""
        while b"event: state" not in seen:
            seen += response.fp.readline()
        connection.close()


if __name__ == "__main__":
    unittest.main()
