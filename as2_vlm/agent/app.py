"""Explainable two-way control dashboard: python -m as2_vlm.agent [--real] [--port 8770].

Standard library only (ThreadingHTTPServer + Server-Sent Events), loopback by default.
POST endpoints reject cross-origin browser requests. The page itself is static/dashboard.html.
"""
import argparse
import json
import logging
import queue
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

from ..contracts import PlanError
from ..vlm import VLMError
from .clock import Clock
from .config import load_config
from .execution_engine import EventLog, ExecutionEngine
from .navigation import Navigator
from .robot_adapter import AdapterError, MockAs2Adapter, RealUnitreeAs2Adapter
from .sensor_adapter import ReadOnlySensorAdapter
from .semantic_map import SemanticSceneGraph
from .task_planner import TaskPlanner
from .vlm_backends import build_perception

log = logging.getLogger(__name__)
STATIC = Path(__file__).with_name("static")
MAX_JSON_BYTES = 16 * 1024
MAX_FRAME_BYTES = 3 * 1024 * 1024
OVERRIDES = {"pause", "stop", "resume", "abort_return", "emergency_sit", "stand_up", "arm", "disarm"}
ANOMALIES = {"close_next_door", "box_on_path", "close_door", "open_door", "object_state", "clear"}


class AgentRuntime:
    """Builds and owns adapter + scene graph + planner + engine; supports switching mode at runtime."""

    def __init__(self, config):
        self.config = config
        self.log = EventLog()
        self._lock = threading.RLock()
        self.adapter = self.graph = self.engine = None
        self.manual = None
        self._build(config.simulation_mode)

    def _build(self, simulation):
        cfg = self.config
        clock = Clock(cfg.sim_speedup if simulation else 1.0)
        if simulation:
            adapter = MockAs2Adapter(cfg, clock)
        elif cfg.robot.transport == "cpp_readonly":
            if cfg.robot.sensor_hub_url:
                from .shared_sensor_adapter import SharedSensorAdapter
                adapter = SharedSensorAdapter(cfg, clock)
            else:
                adapter = ReadOnlySensorAdapter(cfg, clock)
        else:
            adapter = RealUnitreeAs2Adapter(cfg, clock)
        adapter.on_speech = lambda text: self.log.emit("speech", text, data={"text": text})
        adapter.start()
        try:
            graph = SemanticSceneGraph(cfg.resolve_path(cfg.sim_map_db_path if simulation else cfg.map_db_path))
            static = adapter.static_map()
            if graph.is_empty() and static:
                graph.seed({r["name"]: r["bounds"] for r in static["rooms"]}, adapter.seed_nodes())
            navigator = Navigator(cfg.motion, static, graph)
            perception = build_perception(cfg.vlm, simulation)
            planner = TaskPlanner(graph, navigator, cfg)
            engine = ExecutionEngine(cfg, adapter, graph, planner, perception, navigator, self.log)
            engine.start()
            manual = None
            if cfg.manual_control and not simulation:
                from .manual_control import ManualControl
                manual = ManualControl(cfg, adapter)
                manual.reserve()
        except Exception:
            if 'engine' in locals():
                engine.shutdown()
            adapter.close()
            raise
        self.adapter, self.graph, self.engine = adapter, graph, engine
        self.manual = manual
        mode = "MOCK SIMULATION" if simulation else "LIVE AS2 EDU"
        self.log.emit("system", f"{mode} ready: perception={perception.name}, planner={cfg.vlm.planner_backend}, "
                                f"motion_enabled={adapter.motion_enabled}, armed={adapter.armed}")

    def _teardown(self):
        if self.manual:
            self.manual.close()
            self.manual = None
        self.engine.shutdown()
        self.adapter.close()
        self.graph.close()

    def switch_mode(self, simulation):
        with self._lock:
            if (self.adapter.kind == "mock") == simulation:
                return {"ok": True}
            if self.engine.is_busy() or (self.manual and self.manual.status()["busy"]):
                return {"ok": False, "reason": "Stop or finish the current task before switching mode"}
            previous = self.adapter.kind == "mock"
            self._teardown()
            try:
                self._build(simulation)
                return {"ok": True}
            except (AdapterError, VLMError, OSError, ValueError) as exc:
                self.log.emit("system", f"Mode switch failed: {exc}", level="error")
                self._build(previous)
                return {"ok": False, "reason": str(exc)}

    def shutdown(self):
        with self._lock:
            self._teardown()

    def snapshot(self):
        return {**self.engine.snapshot(), "manual_control": self.manual.status() if self.manual else {"enabled": False}}

    def map_payload(self):
        nodes = [n.model_dump() for n in self.graph.get_all_nodes()]
        return {"version": self.graph.version, "static": self.adapter.static_map(), "nodes": nodes,
                "edges": self.graph.get_edges(), "rooms": self.graph.rooms(), "sim_truth": self.adapter.sim_truth(),
                "home_pose": self.config.home_pose}

    def geometry_payload(self):
        geometry = getattr(self.adapter, "local_geometry", None)
        return geometry.payload() if geometry else None

    def public_config(self):
        cfg = self.config.to_dict()
        cfg["vlm"].pop("api_key_env", None)
        return cfg


class Handler(BaseHTTPRequestHandler):
    runtime: AgentRuntime = None
    server_version = "AS2Agent/1.0"

    def log_message(self, fmt, *args):
        log.debug("%s - %s", self.address_string(), fmt % args)

    def _send(self, code, body, content_type="application/json; charset=utf-8", extra=None):
        raw = body if isinstance(body, bytes) else (
            body.encode("utf-8") if isinstance(body, str) else json.dumps(body, ensure_ascii=False).encode("utf-8"))
        self.send_response(code)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(raw)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        for key, value in (extra or {}).items():
            self.send_header(key, value)
        self.end_headers()
        self.wfile.write(raw)

    def _same_origin(self):
        origin = self.headers.get("Origin")
        return origin in (None, "http://" + self.headers.get("Host", ""))

    def _json_body(self):
        size = int(self.headers.get("Content-Length", "0"))
        if not 0 < size <= MAX_JSON_BYTES:
            raise ValueError("Invalid request size")
        body = json.loads(self.rfile.read(size))
        if not isinstance(body, dict):
            raise ValueError("Expected a JSON object")
        return body

    # ---- GET ----------------------------------------------------------------------------
    def do_GET(self):
        rt = self.runtime
        path = self.path.split("?", 1)[0]
        if path == "/":
            self._send(200, (STATIC / "dashboard.html").read_bytes(), "text/html; charset=utf-8")
        elif path == "/api/state":
            self._send(200, rt.snapshot())
        elif path == "/api/map":
            self._send(200, {**rt.map_payload(), "geometry": rt.geometry_payload()})
        elif path == "/api/geometry":
            self._send(200, rt.geometry_payload())
        elif path == "/api/config":
            self._send(200, rt.public_config())
        elif path == "/api/events":
            self._stream()
        elif path == "/api/camera.jpg":
            frame = rt.adapter.get_frame()
            if frame is None or not frame.jpeg:
                self._send(204, b"", "image/jpeg")
            else:
                self._send(200, frame.jpeg, "image/jpeg", {"X-Frame-Id": str(frame.frame_id),
                                                           "X-Frame-Source": frame.source})
        elif path == "/api/lidar":
            self._send(200, rt.adapter.sensor_status() if hasattr(rt.adapter, "sensor_status") else {})
        else:
            self._send(404, {"error": "Not found"})

    def _stream(self):
        rt = self.runtime
        self.send_response(200)
        self.send_header("Content-Type", "text/event-stream")
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.end_headers()
        subscriber = rt.log.subscribe()

        def write(event, payload):
            data = json.dumps(payload, ensure_ascii=False, default=str)
            self.wfile.write(f"event: {event}\ndata: {data}\n\n".encode("utf-8"))
            self.wfile.flush()
        try:
            for event in rt.log.recent(120):
                write("log", event)
            last_state = 0.0
            while True:
                try:
                    write("log", subscriber.get(timeout=0.1))
                except queue.Empty:
                    pass
                if time.monotonic() - last_state >= 0.2:
                    last_state = time.monotonic()
                    write("state", rt.snapshot())
        except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError, OSError):
            pass
        finally:
            rt.log.unsubscribe(subscriber)

    # ---- POST ---------------------------------------------------------------------------
    def do_POST(self):
        rt = self.runtime
        path = self.path.split("?", 1)[0]
        if not self._same_origin():
            # Drain only a bounded body before closing HTTP/1.0; otherwise Windows
            # can reset the socket before the browser receives the rejection.
            try:
                size = int(self.headers.get("Content-Length", "0"))
                if 0 < size <= MAX_JSON_BYTES:
                    self.connection.settimeout(1)
                    self.rfile.read(size)
            except (ValueError, OSError):
                pass
            self._send(403, {"error": "Origin rejected"})
            return
        try:
            if path == "/api/frame":
                size = int(self.headers.get("Content-Length", "0"))
                if not 0 < size <= MAX_FRAME_BYTES:
                    raise ValueError("Invalid frame size")
                rt.adapter.set_external_frame(self.rfile.read(size))
                self._send(200, {"ok": True})
                return
            body = self._json_body()
            if path in {"/api/manual/start", "/api/manual/heartbeat", "/api/manual/stop", "/api/manual/parse", "/api/manual/text", "/api/manual/plan", "/api/manual/execute-plan"}:
                expected_host = "127.0.0.1:" + str(self.server.server_port)
                if self.headers.get("Host") != expected_host or self.headers.get("Origin") != "http://" + expected_host or self.headers.get_content_type() != "application/json":
                    self._send(403, {"error": "Manual control requires same-origin loopback JSON"})
                    return
                manual = getattr(rt, "manual", None)
                if manual is None:
                    self._send(403, {"error": "Manual control not enabled on this panel"})
                    return
                if path.endswith("/plan"):
                    self._send(200, manual.prepare_plan(body))
                elif path.endswith("/execute-plan"):
                    self._send(202, manual.execute_plan(body))
                elif path.endswith("/parse"):
                    if set(body) != {"text"}:
                        raise ValueError("Expected text only")
                    from .manual_control import parse_manual_text
                    self._send(200, parse_manual_text(body["text"]))
                elif path.endswith("/text"):
                    self._send(202, manual.text(body))
                elif path.endswith("/start"):
                    self._send(202, manual.start(body))
                elif path.endswith("/heartbeat"):
                    if set(body) != {"lease"}:
                        raise ValueError("Expected session lease")
                    self._send(200, manual.heartbeat(body["lease"]))
                else:
                    if body:
                        raise ValueError("Stop does not accept parameters")
                    self._send(202, manual.stop())
            elif path in {"/api/geometry/export", "/api/geometry/filter"}:
                geometry = getattr(rt.adapter, "local_geometry", None)
                if geometry is None:
                    raise ValueError("Real LiDAR geometry is not available in this mode")
                if path.endswith("/filter"):
                    geometry.set_slice(body.get("z_min"), body.get("z_max"))
                    self._send(200, {"ok": True})
                else:
                    saved = geometry.export(rt.config.resolve_path("data/geometry"))
                    self._send(200, {"ok": True, **saved})
            elif path == "/api/command":
                if getattr(rt.adapter, "read_only", False):
                    self._send(403, {"error": "Live sensor mode is read-only; use simulation for goals"})
                    return
                text, source = body.get("text"), body.get("source", "text")
                if not isinstance(text, str) or source not in ("text", "voice", "map"):
                    raise ValueError("Expected text and source text|voice|map")
                planner = body.get("planner")
                if planner is not None and not isinstance(planner, str):
                    raise ValueError("Invalid planner")
                self._send(200, rt.engine.submit_goal(text, source, planner))
            elif path == "/api/override":
                if getattr(rt.adapter, "read_only", False):
                    self._send(403, {"error": "No robot command channel in live sensor mode"})
                    return
                action = body.get("action")
                if action not in OVERRIDES:
                    raise ValueError("Unknown override")
                self._send(200, rt.engine.override(action))
            elif path == "/api/anomaly":
                kind = body.get("kind")
                params = body.get("params", {})
                if kind not in ANOMALIES or not isinstance(params, dict):
                    raise ValueError("Unknown anomaly")
                self._send(200, rt.engine.inject_anomaly(kind, **params))
            elif path == "/api/mode":
                if type(body.get("simulation")) is not bool:
                    raise ValueError("Expected simulation: true|false")
                self._send(200, rt.switch_mode(body["simulation"]))
            elif path == "/api/node_state":
                node_id, state = body.get("id"), body.get("state")
                if not isinstance(node_id, str) or not isinstance(state, str) or not 0 < len(state) <= 40:
                    raise ValueError("Expected id and state")
                node = rt.graph.update_node_state(node_id, state, source="operator")
                rt.log.emit("map", f"Operator set {node.label} -> {node.state}")
                self._send(200, {"ok": True})
            else:
                self._send(404, {"error": "Not found"})
        except (ValueError, KeyError, TypeError, PlanError, VLMError, AdapterError, json.JSONDecodeError) as exc:
            self._send(400, {"error": str(exc)})
        except TimeoutError:
            self._send(504, {"error": "Engine did not respond in time"})


def main(argv=None):
    parser = argparse.ArgumentParser(description="AS2 EDU agent dashboard")
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--sim", action="store_true", help="force MockAs2Adapter")
    mode.add_argument("--real", action="store_true", help="force RealUnitreeAs2Adapter (unitree_sdk2py)")
    parser.add_argument("--config", help="JSON config (default configs/agent.local.json if present)")
    parser.add_argument("--host")
    parser.add_argument("--port", type=int)
    parser.add_argument("--speedup", type=float, help="simulation speed factor")
    parser.add_argument("--perception", choices=["auto", "disabled", "simulated", "ollama", "openai_compatible"])
    parser.add_argument("--planner", choices=["rules", "ollama", "openai_compatible"])
    parser.add_argument("--fresh-map", action="store_true", help="discard the stored semantic map before start")
    parser.add_argument("--verbose", action="store_true")
    parser.add_argument("--manual-control", action="store_true", help="enable bounded physical forward test and network stop on loopback; independent remote stop required")
    parser.add_argument("--relative-language", action="store_true", help="use the physical relative-skill language contract in simulation")
    args = parser.parse_args(argv)
    logging.basicConfig(level=logging.DEBUG if args.verbose else logging.INFO,
                        format="%(asctime)s %(levelname)s %(name)s: %(message)s")
    overrides = {}
    if args.relative_language:
        overrides["relative_language_planner"] = True
    if args.manual_control:
        overrides["manual_control"] = True
    if args.sim or args.real:
        overrides["simulation_mode"] = bool(args.sim)
    for key, value in (("ui_host", args.host), ("ui_port", args.port), ("sim_speedup", args.speedup)):
        if value is not None:
            overrides[key] = value
    vlm = {k: v for k, v in (("perception_backend", args.perception), ("planner_backend", args.planner)) if v}
    if vlm:
        overrides["vlm"] = vlm
    config = load_config(args.config, overrides=overrides)
    if args.fresh_map:
        for path in (config.map_db_path, config.sim_map_db_path):
            resolved = Path(config.resolve_path(path))
            if resolved.exists() and resolved.name.endswith(".sqlite"):
                resolved.unlink()
    if config.ui_host not in ("127.0.0.1", "localhost", "::1"):
        log.warning("Dashboard bound to %s: anyone on that network can command the robot", config.ui_host)
    runtime = AgentRuntime(config)
    Handler.runtime = runtime
    server = ThreadingHTTPServer((config.ui_host, config.ui_port), Handler)
    server.daemon_threads = True
    print(f"AS2 agent dashboard: http://{config.ui_host}:{config.ui_port}  "
          f"({'simulation' if runtime.adapter.kind == 'mock' else 'LIVE AS2'})", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        runtime.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
