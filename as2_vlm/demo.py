"""Loopback-only simulation server: python -m as2_vlm.demo."""
import argparse
import json
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

from .parser import parse_instruction
from .simulator import WORLD, simulate
from .vlm import OllamaBackend, plan_instruction


class Handler(BaseHTTPRequestHandler):
    def reply(self, code, body, content_type="application/json; charset=utf-8"):
        raw = body.encode("utf-8") if isinstance(body, str) else json.dumps(body, ensure_ascii=False).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(raw)))
        self.send_header("X-Content-Type-Options", "nosniff")
        self.end_headers()
        self.wfile.write(raw)

    def do_GET(self):
        if self.path == "/":
            self.reply(200, Path(__file__).with_name("demo.html").read_text(encoding="utf-8"), "text/html; charset=utf-8")
        elif self.path == "/world":
            self.reply(200, WORLD)
        else:
            self.reply(404, {"error": "Not found"})

    def do_POST(self):
        if self.path != "/simulate":
            self.reply(404, {"error": "Not found"})
            return
        # Reject browser requests originating outside this demo.
        if self.headers.get("Origin") not in (None, "http://" + self.headers.get("Host", "")):
            self.reply(403, {"error": "Origin rejected"})
            return
        try:
            size = int(self.headers.get("Content-Length", "0"))
            if not 0 < size <= 8192:
                raise ValueError("Invalid request size")
            request = json.loads(self.rfile.read(size))
            instruction = request["instruction"]
            if not isinstance(instruction, str) or len(instruction) > 2000:
                raise ValueError("Invalid instruction")
            backend = request.get("backend", "rules")
            if backend not in {"rules", "ollama"}:
                raise ValueError("Unknown backend")
            if type(request.get("obstacle", False)) is not bool:
                raise ValueError("Invalid obstacle flag")
            plan = (plan_instruction(OllamaBackend(), instruction) if backend == "ollama"
                    else parse_instruction(instruction))
            result = simulate(plan, request.get("pose"), request.get("obstacle", False))
            result["backend"] = backend
            self.reply(200, result)
        except (ValueError, KeyError, TypeError) as exc:
            self.reply(400, {"error": str(exc)})


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", type=int, default=8765)
    args = parser.parse_args()
    server = ThreadingHTTPServer(("127.0.0.1", args.port), Handler)
    print(f"Simulation: http://127.0.0.1:{args.port}", flush=True)
    server.serve_forever()


if __name__ == "__main__":
    main()
