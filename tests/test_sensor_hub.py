import copy
import io
import json
import struct
from http.client import HTTPConnection
from http.server import ThreadingHTTPServer
from pathlib import Path
import tempfile
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from as2_vlm.agent.config import load_config
from as2_vlm.agent.sensor_hub import Handler, SensorHub
from as2_vlm.agent.shared_sensor_adapter import HubClient, SharedSensorAdapter


def snapshot():
    return {"state": {"age_s": .01, "position": [1, 2, .3], "rpy": [0, 0, .1],
                       "velocity": [0, 0, 0], "mode": 3, "error": "0"},
            "lidar": {"age_s": .01, "frame": "unverified"},
            "camera": {"age_s": -1, "sequence": 0}, "recording": {"active": False}}


class SensorHubTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.cfg = load_config(path="", env={})
        self.cfg.resolve_path = lambda value: str(Path(self.temp.name) / value)
        self.hub = SensorHub(self.cfg)
        self.hub.owner._accept(snapshot())
        self.hub.owner._process = SimpleNamespace(poll=lambda: None, stdin=io.BytesIO())
        handler = type("TestHubHandler", (Handler,), {"hub": self.hub})
        self.server = ThreadingHTTPServer(("127.0.0.1", 0), handler)
        self.server.daemon_threads = True
        self.worker = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.worker.start()
        self.url = f"http://127.0.0.1:{self.server.server_port}"
        self.cfg.robot.sensor_hub_url = self.url
        self.clients = []

    def tearDown(self):
        for client in self.clients:
            client.close()
        self.server.shutdown()
        self.server.server_close()
        self.worker.join(timeout=2)
        self.temp.cleanup()

    def request(self, path, body, origin=None):
        connection = HTTPConnection("127.0.0.1", self.server.server_port, timeout=3)
        connection.request("POST", path, json.dumps(body), {"Content-Type": "application/json",
                           "Origin": origin or self.url})
        response = connection.getresponse()
        code, data = response.status, json.loads(response.read())
        connection.close()
        return code, data

    def test_two_panels_do_not_launch_or_close_acquisition(self):
        with patch("as2_vlm.agent.sensor_adapter.subprocess.Popen") as popen:
            for _ in range(2):
                client = SharedSensorAdapter(self.cfg)
                self.clients.append(client)
                client.start()
                self.assertTrue(client.get_telemetry().connected)
                self.assertEqual(client.get_telemetry().pose.x, 1)
                self.assertFalse(client.set_armed(True))
                self.assertFalse(client.move(.2, 0, 0))
            self.clients[0].close()
            self.assertTrue(self.clients[1].get_telemetry().connected)
            self.assertFalse(self.hub.owner._process.stdin.closed)
            popen.assert_not_called()

    def test_stale_source_cannot_become_fresh_from_client_polling(self):
        self.hub.owner._received -= 3
        client = SharedSensorAdapter(self.cfg)
        self.clients.append(client)
        client.start()
        self.assertFalse(client.get_telemetry().connected)
        self.assertFalse(client.sensor_status()["connected"])

    def test_profile_mismatch_fails_without_fallback(self):
        self.cfg.robot.network_interface = "other0"
        # Hub has its own fixed profile, as a separate service would.
        self.hub.config = copy.deepcopy(self.cfg)
        self.hub.config.robot.network_interface = "eth0"
        with self.assertRaisesRegex(Exception, "profile"):
            SharedSensorAdapter(self.cfg).start()

    def test_recording_uses_existing_pipe_and_rejects_competing_request(self):
        code, data = self.request("/record", {"name": "safe-record", "seconds": 3})
        self.assertEqual(code, 202)
        self.assertTrue(data["accepted"])
        self.assertIn(b"RECORD\t3\t", self.hub.owner._process.stdin.getvalue())
        self.assertEqual(self.request("/record", {"name": "second", "seconds": 3})[0], 400)

    def test_completed_recording_allows_another_request(self):
        self.hub.pending = "first"
        self.hub.owner._snapshot["recording"] = {"name": "first", "active": False, "finished": True}
        self.assertEqual(self.request("/record", {"name": "second", "seconds": 3})[0], 202)

    def test_invalid_requests_never_reach_the_bridge(self):
        for body in ({"name": "../escape", "seconds": 3}, {"name": "safe", "seconds": True},
                     {"name": "safe", "seconds": 31}, {"name": "safe", "seconds": 3, "path": "/tmp"}):
            self.assertEqual(self.request("/record", body)[0], 400)
        self.assertEqual(self.request("/record", {"name": "safe", "seconds": 3}, "https://untrusted.example")[0], 403)
        self.assertEqual(self.request("/move", {"vx": 1})[0], 404)
        self.assertEqual(self.request("/geometry/export", {"directory": "elsewhere"})[0], 400)
        self.assertEqual(self.hub.owner._process.stdin.getvalue(), b"")

    def test_geometry_proxy_expires(self):
        client = SharedSensorAdapter(self.cfg)
        client._geometry = {"age_s": .1, "connected": True, "slam": False, "navigation_ready": False}
        client._received = time.monotonic() - 2
        self.assertFalse(client.local_geometry.status()["connected"])

    def test_hub_restart_requires_reconnection_and_expires_old_state(self):
        client = SharedSensorAdapter(self.cfg)
        self.clients.append(client)
        client.start()
        self.hub.instance = "different-instance"
        time.sleep(1.3)
        self.assertFalse(client.get_telemetry().connected)
        self.assertIn("restarted", client.sensor_status()["error_message"])

    def test_non_loopback_hubs_are_rejected(self):
        for value in ("http://example.com:8780", "https://127.0.0.1:8780", "http://127.0.0.1:8780?x=1"):
            with self.assertRaises(ValueError):
                HubClient(value)

    def test_configuration_rejects_motion_and_bad_hub_urls(self):
        for value in ("http://localhost:8780", "http://192.168.123.161:8780", "http://127.0.0.1:99999"):
            with self.assertRaises(ValueError):
                load_config(path="", env={}, overrides={"robot": {"sensor_hub_url": value}})

    def test_raw_cloud_is_sequence_bound_and_expired_cloud_is_refused(self):
        raw = struct.pack("<Q", 23)+b"synthetic points"
        self.hub.owner._cloud_raw = raw
        self.hub.owner._cloud_sequence = 23
        self.hub.owner._snapshot["lidar"].update(cloud_sequence=23)
        client = HubClient(self.url)
        self.assertEqual(client.request("/cloud?sequence=23", binary=True), raw)
        for path in ("/cloud?sequence=22", "/cloud?sequence=../23", "/cloud"):
            with self.assertRaises(Exception) as error:
                client.request(path, binary=True)
            self.assertEqual(error.exception.code, 409)
        self.hub.owner._cloud_raw = struct.pack("<Q", 24)+b"new cloud"
        self.hub.owner._cloud_sequence = 24
        with self.assertRaises(Exception) as error:
            client.request("/cloud?sequence=23", binary=True)
        self.assertEqual(error.exception.code, 409)
        self.hub.owner._cloud_raw = raw
        self.hub.owner._cloud_sequence = 23
        self.hub.owner._received -= 1
        with self.assertRaises(Exception) as error:
            client.request("/cloud?sequence=23", binary=True)
        self.assertEqual(error.exception.code, 409)
