"""Opt-in loopback UI for the bounded, independently stopped physical meter test."""
import os
from pathlib import Path
import re
import secrets
import shutil
import subprocess
import threading
import time

from .robot_adapter import AdapterError
from .sensor_adapter import ReadOnlySensorAdapter
from .task_planner import primitive_step
from ..contracts import PlanError


def parse_manual_text(text):
    if not isinstance(text, str) or not 0 < len(text.strip()) <= 500:
        raise ValueError("Expected a command of 1-500 characters")
    command = " ".join(text.lower().replace("ё", "е").strip().rstrip(".!?").split())
    if command in {"стоп", "остановись", "останови команду", "останови движение", "отмена", "stop", "stop moving", "halt", "cancel"}:
        return {"action": "stop", "requires_confirmation": False}
    # Normalize distance-first wording, then use the existing strict primitive parser.
    match = re.fullmatch(r"(пройди|иди)\s+(?:на\s+)?(1(?:[.,]0+)?|один)\s+(метр|м)(?:\s+(вперед|назад))?", command)
    if match:
        command = f"пройди {match[4] or 'вперед'} 1 метр"
    try:
        step = primitive_step(command)
    except (PlanError, ValueError):
        step = None
    if step is not None and step.tool == "stop":
        return {"action": "stop", "requires_confirmation": False}
    if step is not None and step.tool == "walk_relative" and step.distance_m == 1:
        return {"action": "walk_meter", "target_m": 1, "requires_confirmation": True}
    raise ValueError("Live text currently supports forward 1 metre and stop only; no turn, reverse or compound commands")


class ManualControl:
    def __init__(self, config, adapter, popen=subprocess.Popen):
        self.config, self.adapter, self.popen = config, adapter, popen
        self.lock = threading.RLock()
        self.process = None
        self.thread = None
        self.token = None
        self.last_heartbeat = 0
        self.ready = False
        self.stopping = False
        self.output = ""
        self.exit_code = None
        self.started = 0
        self._lease = None
        self._planning = False
        self._generation = 0
        self._prepared = None
        self._planner = None
        self._step = None
        self._visible_plan = None
        self._movement = ("walk", 1)

    def _movement_permission(self, mode, amount):
        try:
            monitor = getattr(self.adapter, "movement_permission", None)
            result = monitor(mode, amount) if callable(monitor) else {}
            if not isinstance(result, dict):
                raise ValueError("Invalid obstacle monitor response")
            return result
        except Exception:
            return {"allowed": False, "reason": "obstacle_monitor_failed"}

    def _require_obstacle_clearance(self, mode, amount):
        result = self._movement_permission(mode, amount)
        if result.get("allowed") is not True:
            raise ValueError("LiDAR movement gate: " + str(result.get("reason", "obstacle_monitor_unavailable")))

    def reserve(self):
        path = Path(self.config.resolve_path(".runtime/manual-control.lock"))
        path.parent.mkdir(parents=True, exist_ok=True)
        self._lease = path.open("a+b")
        try:
            if os.name == "nt":
                import msvcrt
                self._lease.seek(0); self._lease.write(b"0"); self._lease.flush(); self._lease.seek(0)
                msvcrt.locking(self._lease.fileno(), msvcrt.LK_NBLCK, 1)
            else:
                import fcntl
                fcntl.flock(self._lease, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError:
            self._lease.close(); self._lease = None
            raise AdapterError("Another panel already owns manual control")

    def status(self):
        with self.lock:
            return {"enabled": True, "busy": self.token is not None, "stopping": self.stopping,
                    "planning": self._planning, "step": self._step, "model": self.config.vlm.model,
                    "language_planning": True,
                    "language_plan": self._visible_plan,
                    "skills": ["walk_relative", "rotate_relative", "stop"], "plan_max_duration_s": 120,
                    "result": self.output[-4000:], "exit_code": self.exit_code,
                    "max_speed_m_s": .15, "max_duration_s": 12, "target_m": 1,
                    "collision_guard": self.adapter.collision_status() if hasattr(self.adapter, "collision_status") else {"configured": False},
                    "network_stop_is_emergency_stop": False}

    def prepare_plan(self, body):
        if set(body) != {"text"}:
            raise ValueError("Expected text only")
        try:
            is_stop = parse_manual_text(body["text"])["action"] == "stop"
        except ValueError:
            is_stop = False
        if is_stop:
            return {"action": "stop", "requires_confirmation": False}
        from .physical_planner import PhysicalPlanner, primitive_arguments
        with self.lock:
            if self._planning or self.token:
                raise ValueError("Another plan/command is active; stop or finish it first")
            self._planning = True
            self._prepared = None
            generation = self._generation
            if self._planner is None:
                self._planner = PhysicalPlanner(self.config)
        try:
            plan = self._planner.plan(body["text"])
            if plan.status == "ok":
                primitive_arguments(plan)
            with self.lock:
                if generation != self._generation:
                    raise ValueError("Planning cancelled; stale result discarded")
                identifier = secrets.token_hex(24) if plan.status == "ok" else None
                self._visible_plan = plan.model_dump()
                self._prepared = {"id": identifier, "plan": plan.model_dump(), "at": time.monotonic()} if identifier else None
                return {"plan_id": identifier, "plan": plan.model_dump(), "model": self.config.vlm.model,
                        "requires_confirmation": plan.status == "ok"}
        finally:
            with self.lock:
                self._planning = False

    def execute_plan(self, body):
        if set(body) != {"plan_id", "remote_ready", "path_clear"} or body["remote_ready"] is not True or body["path_clear"] is not True:
            raise ValueError("Confirm independent stop and clear space before executing the reviewed plan")
        if not self.adapter.read_only or self.adapter.kind != "as2" or not self.adapter.get_telemetry().connected:
            raise ValueError("Fresh live AS2 sensors required")
        from .physical_planner import primitive_arguments
        with self.lock:
            prepared = self._prepared
            if self.token or self._planning:
                raise ValueError("A command/planning operation is active")
            if not prepared or not isinstance(body["plan_id"], str) or not secrets.compare_digest(body["plan_id"], prepared["id"]) or time.monotonic()-prepared["at"] > 60:
                raise ValueError("Plan missing, expired or cancelled; request a new plan")
            steps = primitive_arguments(prepared["plan"])
            for mode, amount in steps:
                if mode != "stop":
                    self._require_obstacle_clearance(mode, amount)
            # Resolve all fixed executable paths before accepting physical execution.
            arguments = [(self._args("as2_stop_only", "--stop"), False, mode, amount) if mode == "stop" else
                         (self._relative_args(mode, amount), True, mode, amount) for mode, amount in steps]
            self._prepared = None
            self._generation += 1
            self.token = secrets.token_hex(24)
            token = self.token
            self.output = ""; self.exit_code = None; self.stopping = False; self.ready = False
            self.last_heartbeat = self.started = time.monotonic()
            self.thread = threading.Thread(target=self._run_plan, args=(arguments,), daemon=True, name="as2-physical-plan")
            self.thread.start()
            return {"accepted": True, "lease": token, "steps": len(steps)}

    def _relative_args(self, mode, amount):
        args = self._args("as2_relative_move", "--execute-ui")
        return [*args[:-1], mode, format(amount, ".16f"), args[-1]]

    def _run_plan(self, arguments):
        final_code = 1
        try:
            for index, (args, moving, mode, amount) in enumerate(arguments):
                with self.lock:
                    if self.stopping or time.monotonic()-self.last_heartbeat > 1 or time.monotonic()-self.started > 120:
                        self.stopping = True
                        break
                    if moving:
                        self._require_obstacle_clearance(mode, amount)
                        self._movement = (mode, amount)
                    self.ready = False
                    self._step = index + 1
                    self.output = (self.output + f"\nStep {index+1}/{len(arguments)}\n")[-8000:]
                    self.process = self.popen(args, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, shell=False)
                    if moving:
                        self._write(b"RELATIVE\n")
                self._run(moving, release=False)
                with self.lock:
                    final_code = self.exit_code
                    self.process = None
                    if final_code != 0 or self.stopping:
                        break
        except Exception as exc:
            final_code = 1
            with self.lock:
                self.output = (self.output + "\nPlan failed: " + str(exc))[-8000:]
        finally:
            with self.lock:
                self.output += "\nPlan completed.\n" if final_code == 0 and not self.stopping else "\nPlan stopped; remaining steps skipped.\n"
                self.exit_code = final_code
                self.token = None; self._step = None; self.ready = False

    def _args(self, binary, mode):
        rc = self.config.robot
        for value in (rc.network_interface, rc.wsl_distro):
            if not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_.:-]{0,63}", value):
                raise ValueError("Invalid transport setting")
        path = Path(self.config.resolve_path("build/as2-probe/" + binary))
        if not path.is_file():
            raise ValueError("Build " + binary + " first")
        args = [ReadOnlySensorAdapter._linux_path(path), rc.network_interface, mode]
        if os.name == "nt":
            wsl = shutil.which("wsl.exe")
            if not wsl:
                raise ValueError("WSL unavailable")
            args = [wsl, "-d", rc.wsl_distro, "--", *args]
        return args

    def start(self, body):
        if set(body) != {"remote_ready", "path_clear"} or body["remote_ready"] is not True or body["path_clear"] is not True:
            raise ValueError("Confirm independent remote stop and clear level path with cable slack")
        if not self.adapter.read_only or self.adapter.kind != "as2" or not self.adapter.get_telemetry().connected:
            raise ValueError("Fresh live AS2 sensor transport required")
        self._require_obstacle_clearance("walk", 1)
        return self._launch("as2_walk_meter", "--execute-ui", True)

    def text(self, body):
        if not isinstance(body, dict) or "text" not in body:
            raise ValueError("Expected text")
        action = parse_manual_text(body["text"])
        if action["action"] == "stop":
            if set(body) != {"text"}:
                raise ValueError("Stop accepts text only")
            return {**self.stop(), "action": "stop"}
        if set(body) != {"text", "remote_ready", "path_clear"}:
            raise ValueError("Physical movement requires remote_ready and path_clear confirmations")
        return {**self.start({key: body[key] for key in ("remote_ready", "path_clear")}), "action": "walk_meter"}

    def _launch(self, binary, mode, walk):
        args = self._args(binary, mode)
        with self.lock:
            if self.token is not None:
                raise ValueError("A manual command is already active")
            if walk and self._planning:
                raise ValueError("Planning is active; cancel or finish it first")
            if walk:
                self._visible_plan = None
            self._generation += 1
            self._prepared = None
            self.token = secrets.token_hex(24)
            token = self.token
            self.ready = False; self.stopping = False; self.output = ""; self.exit_code = None
            self.last_heartbeat = self.started = time.monotonic()
            try:
                if walk:
                    self._require_obstacle_clearance("walk", 1)
                    self._movement = ("walk", 1)
                self.process = self.popen(args, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT, shell=False)
                if walk:
                    self.process.stdin.write(b"METER\n"); self.process.stdin.flush()
                self.thread = threading.Thread(target=self._run, args=(walk,), daemon=True, name="manual-as2")
                self.thread.start()
            except Exception:
                if self.process and self.process.stdin:
                    self.process.stdin.close()
                self.token = None
                raise
            return {"accepted": True, "lease": token}

    def _write(self, message):
        if self.process is None:
            return
        try:
            self.process.stdin.write(message); self.process.stdin.flush()
        except (OSError, ValueError):
            self.stopping = True

    def heartbeat(self, token):
        with self.lock:
            if not isinstance(token, str) or self.token is None or not secrets.compare_digest(token, self.token) or self.stopping:
                raise ValueError("Manual session expired or stopped")
            if time.monotonic() - self.last_heartbeat > 1:
                self.stopping = True; self._write(b"STOP\n")
                raise ValueError("Manual session heartbeat expired")
            self.last_heartbeat = time.monotonic()
        return {"ok": True}

    def stop(self):
        with self.lock:
            self._generation += 1
            self._prepared = None
            if self.token is not None:
                self.stopping = True
                self._write(b"STOP\n")
                return {"accepted": True, "note": "Stop requested; verify physically, use remote if needed"}
        result = self._launch("as2_stop_only", "--stop", False)
        result.pop("lease", None)
        return result

    def _run(self, walk, release=True):
        process = self.process
        step_started = time.monotonic()
        def read_output():
            try:
                for raw in iter(lambda: process.stdout.readline(4097), b""):
                    text = raw.decode("utf-8", errors="replace")
                    with self.lock:
                        self.output = (self.output + text)[-8000:]
                        if "UI_READY" in text:
                            self.ready = True
            finally:
                process.stdout.close()
        reader = threading.Thread(target=read_output, daemon=True)
        reader.start()
        try:
            while process.poll() is None:
                with self.lock:
                    if walk:
                        obstacle = self._movement_permission(*self._movement)
                        if obstacle.get("allowed") is not True and not self.stopping:
                            self.output = (self.output + "\nLiDAR stop: " + str(obstacle.get("reason", "unknown obstacle status")))[-8000:]
                            self.stopping = True
                        if self.stopping or time.monotonic() - self.last_heartbeat > 1 or time.monotonic() - step_started > 24 or time.monotonic() - self.started > 120:
                            self.stopping = True; self._write(b"STOP\n")
                        elif self.ready:
                            self._write(b"KEEPALIVE\n")
                time.sleep(.15)
            reader.join(timeout=2)
        finally:
            with self.lock:
                process.stdin.close()
                self.exit_code = process.poll()
                if release:
                    self.token = None
                self.ready = False

    def close(self):
        with self.lock:
            self._generation += 1; self._prepared = None
            if self.token:
                self.stopping = True; self._write(b"STOP\n")
        if self.thread:
            self.thread.join(timeout=8)
        if self.token:
            raise RuntimeError("Manual process still active; use independent remote stop")
        if self._lease:
            self._lease.close(); self._lease = None
