import json
import threading
import unittest
from http.server import BaseHTTPRequestHandler, HTTPServer

from as2_vlm.vlm import OllamaBackend, VLMError, plan_instruction


class LocalTransportTests(unittest.TestCase):
    def setUp(self):
        self.requests = []
        requests = self.requests

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *args):
                pass

            def do_POST(self):
                payload = json.loads(self.rfile.read(int(self.headers["Content-Length"])))
                requests.append(payload)
                response = {"done": True, "message": {"content": json.dumps({"steps": [{"skill": "stop"}]})}}
                self.send_response(200)
                self.end_headers()
                self.wfile.write(json.dumps(response).encode())

            def do_GET(self):
                if self.path == "/api/tags":
                    self.send_response(200)
                    self.end_headers()
                    self.wfile.write(b'{"models":[{"name":"qwen2.5vl:3b"}]}')
                else:
                    self.send_response(302)
                    self.send_header("Location", "https://example.com/")
                    self.end_headers()

        self.server = HTTPServer(("127.0.0.1", 0), Handler)
        self.thread = threading.Thread(target=lambda: self.server.serve_forever(poll_interval=0.01), daemon=True)
        self.thread.start()
        self.backend = OllamaBackend("http://127.0.0.1:" + str(self.server.server_port))

    def tearDown(self):
        self.server.shutdown()
        self.thread.join(timeout=2)
        self.server.server_close()

    def test_utf8_instruction_over_http(self):
        result = plan_instruction(self.backend, "Остановись")
        self.assertEqual(result["steps"], [{"skill": "stop"}])
        user = self.requests[0]["messages"][0]["content"]
        self.assertEqual(json.loads(user.split("TASK INPUT:\n", 1)[1])["instruction"], "Остановись")

    def test_healthcheck_not_inference(self):
        report = self.backend.healthcheck()
        self.assertTrue(report["model_installed"])
        self.assertFalse(report["inference_tested"])

    def test_redirect_not_followed(self):
        with self.assertRaises(VLMError):
            self.backend._request("/redirect")
