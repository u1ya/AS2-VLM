"""Closed-loop execution with monitoring, VLM diagnosis and failure recovery.

The engine owns an asyncio loop in a background thread. Navigation streams Move() at
control_hz from the deterministic follower and watches four failure signals:
contact (robot/simulator), LiDAR corridor, stall (commanded but not moving), and lack of
progress. On any of them it sends StopMove(), asks the VLM *why*, writes the diagnosis into
the scene graph and recovers: (1) re-plan an alternate route, (2) wait for a transient
obstacle, (3) ask a person for help and re-verify before continuing.
"""
import asyncio
import logging
import math
import queue
import threading
import time
from collections import deque

from pydantic import ValidationError

from ..contracts import PlanError
from ..vlm import VLMError
from .config import PLANNER_BACKENDS
from .models import BlockageDiagnosis, VisualVerdict
from .navigation import advance_waypoint, corridor_obstacle, follow_command, path_length, turn_rate
from .robot_adapter import AdapterError
from .sim_world import ray_segment, wrap
from .task_planner import PlanStep, TaskPlan

log = logging.getLogger(__name__)
MODEL_ERRORS = (VLMError, ValidationError, ValueError)


class EventLog:
    """Thread-safe reasoning log with fan-out to dashboard subscribers."""

    def __init__(self, maxlen=500):
        self._events = deque(maxlen=maxlen)
        self._subscribers = set()
        self._lock = threading.Lock()
        self._seq = 0

    def emit(self, kind, message, *, reason=None, confidence=None, level="info", data=None):
        with self._lock:
            self._seq += 1
            event = {"seq": self._seq, "time": time.time(), "kind": kind, "level": level, "message": message,
                     "reason": reason, "confidence": None if confidence is None else round(float(confidence), 3),
                     "data": data}
            self._events.append(event)
            for subscriber in list(self._subscribers):
                try:
                    subscriber.put_nowait(event)
                except queue.Full:
                    pass
        getattr(log, "warning" if level == "warning" else "error" if level == "error" else "info")(
            "[%s] %s%s", kind, message, f" -> {reason}" if reason else "")
        return event

    def subscribe(self):
        subscriber = queue.Queue(maxsize=1000)
        with self._lock:
            self._subscribers.add(subscriber)
        return subscriber

    def unsubscribe(self, subscriber):
        with self._lock:
            self._subscribers.discard(subscriber)

    def recent(self, count=100):
        with self._lock:
            return list(self._events)[-count:]


class ExecutionEngine:
    BUSY = {"PLANNING", "EXECUTING", "RECOVERING", "WAITING_FOR_HUMAN", "PAUSED"}

    def __init__(self, config, adapter, graph, planner, perception, navigator, event_log):
        self.config = config
        self.adapter = adapter
        self.graph = graph
        self.planner = planner
        self.perception = perception
        self.navigator = navigator
        self.log = event_log
        self.clock = adapter.clock
        self.limits = config.motion
        self.state = "IDLE"
        self.goal = None
        self.plan = None
        self.step_status = []
        self.current_step = None
        self.activity = "Idle"
        self.reason = ""
        self.confidence = None
        self.confidence_source = ""
        self.path = []
        self.nav_goal = None
        self.detections = []
        self.detection_frame = None
        self.findings = []
        self.waiting_for = None
        self.planner_backend = config.vlm.planner_backend
        self._vlm_lock = threading.Lock()
        self._loop = None
        self._thread = None
        self._task = None
        self._paused = False
        self._resume = None
        self._ready = threading.Event()
        self._last_perception_warning = 0.0

    # ---- lifecycle --------------------------------------------------------------------
    def start(self):
        self._thread = threading.Thread(target=self._run_loop, name="as2-engine", daemon=True)
        self._thread.start()
        if not self._ready.wait(5):
            raise RuntimeError("Engine loop failed to start")

    def _run_loop(self):
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        self._loop = loop
        self._resume = asyncio.Event()
        self._resume.set()
        loop.create_task(self._perception_loop())
        self._ready.set()
        try:
            loop.run_forever()
        finally:
            pending = asyncio.all_tasks(loop)
            for task in pending:
                task.cancel()
            loop.run_until_complete(asyncio.gather(*pending, return_exceptions=True))
            loop.close()

    def shutdown(self):
        if self._loop is None or not self._loop.is_running():
            return
        try:
            self.call(self._cancel_task(), timeout=5)
        except Exception:
            log.exception("Engine shutdown: cancel failed")
        self._loop.call_soon_threadsafe(self._loop.stop)
        self._thread.join(5)
        self.adapter.stop_move()

    def call(self, coro, timeout=10):
        return asyncio.run_coroutine_threadsafe(coro, self._loop).result(timeout)

    async def _in_thread(self, fn, *args):
        return await asyncio.get_running_loop().run_in_executor(None, fn, *args)

    async def _vlm(self, fn, *args):
        def run():
            with self._vlm_lock:
                return fn(*args)
        return await self._in_thread(run)

    # ---- thread-safe public API -------------------------------------------------------
    def submit_goal(self, text, source="text", planner_backend=None):
        return self.call(self._submit(text, source, planner_backend))

    def override(self, action):
        return self.call(self._override(action), timeout=30)

    def inject_anomaly(self, kind, **params):
        return self.call(self._inject(kind, params))

    def is_busy(self):
        return self._task is not None and not self._task.done()

    def snapshot(self):
        telemetry = self.adapter.get_telemetry()
        plan = None
        if self.plan is not None:
            plan = self.plan.model_dump()
            for index, step in enumerate(plan["steps"]):
                step["status"] = self.step_status[index] if index < len(self.step_status) else "pending"
        truth = self.adapter.sim_truth() or {}
        return {"mode": self.adapter.kind, "state": self.state, "goal": self.goal, "plan": plan,
                "current_step": self.current_step, "activity": self.activity, "reason": self.reason,
                "confidence": self.confidence, "confidence_source": self.confidence_source,
                "telemetry": telemetry.model_dump(), "path": self.path, "nav_goal": self.nav_goal,
                "detections": self.detections, "detection_frame": self.detection_frame,
                "findings": self.findings[-20:], "waiting_for": self.waiting_for, "paused": self._paused,
                "map_version": self.graph.version, "perception_backend": self.perception.name,
                "planner_backend": self.planner_backend, "sim_anomalies": truth.get("anomalies", []),
                "time": time.time()}

    # ---- goal handling ----------------------------------------------------------------
    async def _submit(self, text, source, backend):
        text = (text or "").strip()
        if not text or len(text) > 500:
            return {"accepted": False, "reason": "Command must be 1-500 characters"}
        if self.adapter.emergency_latched:
            return {"accepted": False, "reason": "Emergency stop is latched. Press STAND UP to recover first."}
        if self.is_busy():
            return {"accepted": False, "reason": f"Busy ({self.state}). Pause or abort the current task first."}
        backend = backend or self.config.vlm.planner_backend
        if backend not in PLANNER_BACKENDS:
            return {"accepted": False, "reason": f"Unknown planner {backend!r}"}
        self.planner_backend = backend
        self._task = self._loop.create_task(self._run_goal(text, source, backend))
        return {"accepted": True}

    def _reset_view(self, goal):
        self.goal, self.plan, self.step_status, self.current_step = goal, None, [], None
        self.findings, self.path, self.nav_goal, self.waiting_for = [], [], None, None
        self.confidence, self.confidence_source = None, ""

    def _set(self, state, activity, reason, kind="step"):
        if state:
            self.state = state
        self.activity, self.reason = activity, reason or ""
        self.log.emit(kind, activity, reason=reason)

    async def _run_goal(self, text, source, backend):
        self._reset_view(text)
        self.log.emit("goal", f'New goal via {source}: "{text}"', data={"source": source, "planner": backend})
        self._set("PLANNING", f'Planning "{text}" with the {backend} planner',
                  "Turn the goal into executable AS2 primitives grounded in the scene graph", kind="planning")
        try:
            pose = self.adapter.get_telemetry().pose
            plan = await self._in_thread(self.planner.plan, text, backend, pose)
        except (PlanError, *MODEL_ERRORS) as exc:
            self._finish("FAILED", f"Planning failed: {exc}")
            return
        self.plan = plan
        self.step_status = ["pending"] * len(plan.steps)
        self.log.emit("plan", f"Plan ({plan.planner}, {len(plan.steps)} steps): {plan.summary}", data=plan.model_dump())
        await self._execute_plan(plan)

    async def _execute_plan(self, plan):
        self.state = "EXECUTING"
        failed, unreachable = [], set()
        try:
            for index, step in enumerate(plan.steps):
                self.current_step = index
                key = (step.target_name or "").lower()
                if step.tool == "verify_visual_state" and key in unreachable:
                    self.step_status[index] = "skipped"
                    self._record_finding(step.target_name, step.question, VisualVerdict(
                        answer="uncertain", state="unknown", confidence=0.0, evidence="Target was not reached."))
                    continue
                self.step_status[index] = "running"
                ok = await self._execute_step(step)
                self.step_status[index] = "done" if ok else "failed"
                if not ok:
                    failed.append(index)
                    if step.tool == "navigate_to" and key:
                        unreachable.add(key)
            if failed:
                self._finish("FAILED", f"{len(failed)} of {len(plan.steps)} steps failed: "
                             + ", ".join(f"#{i + 1} {plan.steps[i].tool}" for i in failed))
            else:
                self._finish("SUCCEEDED", f"Goal completed ({len(plan.steps)} steps)")
        except asyncio.CancelledError:
            self.adapter.stop_move()
            self.step_status = ["cancelled" if s in ("pending", "running") else s for s in self.step_status]
            if self.state != "EMERGENCY":
                self._finish("ABORTED", "Task aborted by the operator")
            raise

    def _finish(self, state, message):
        self.adapter.stop_move()
        telemetry = self.adapter.get_telemetry()
        if telemetry.standing and any(abs(v) > 1e-3 for v in telemetry.body_euler):
            self.adapter.euler(0.0, 0.0, 0.0)
        self.state, self.current_step, self.path, self.nav_goal, self.waiting_for = state, None, [], None, None
        self.activity, self.reason = message, ""
        level = "info" if state == "SUCCEEDED" else "error" if state == "FAILED" else "warning"
        self.log.emit("task", f"{state}: {message}", level=level)

    async def _execute_step(self, step):
        tool = step.tool
        if tool == "navigate_to":
            return await self._step_navigate(step)
        if tool == "adjust_posture":
            roll, pitch, yaw = step.roll or 0.0, step.pitch or 0.0, step.yaw or 0.0
            self._set(None, f"Adjusting posture via Euler(roll {math.degrees(roll):.0f}°, pitch "
                            f"{math.degrees(pitch):.0f}°, yaw {math.degrees(yaw):.0f}°)", step.reasoning)
            ok = self.adapter.euler(roll, pitch, yaw)
            await self.clock.sleep(0.8)
            return ok
        if tool == "verify_visual_state":
            return await self._step_verify(step)
        if tool == "speak":
            template = step.message if step.message.count("{findings}") <= 1 else "Inspection report: {findings}"
            message = template.replace("{findings}", self._findings_text())
            self._set(None, f'Speaking: "{message}"', step.reasoning)
            self.adapter.speak(message)
            await self.clock.sleep(min(6.0, 0.5 + len(message) / 15))
            return True
        if tool == "stand_down":
            self._set(None, "Standing down (StandDown)", step.reasoning)
            ok = await self._in_thread(self.adapter.stand_down)
            await self.clock.sleep(1.5)
            return ok
        if tool == "return_home":
            return await self._return_home(step.reasoning)
        return False

    # ---- navigation -----------------------------------------------------------------
    async def _step_navigate(self, step):
        pose = self.adapter.get_telemetry().pose
        if step.target_name:
            node = self.planner.resolve(step.target_name)
            if node is None:
                self.log.emit("step", f"Target '{step.target_name}' is no longer in the map", level="error")
                return False
            try:
                x, y, yaw = self.planner.approach_pose(node, (pose.x, pose.y))
            except PlanError as exc:
                self.log.emit("navigation", str(exc), level="error")
                return False
            label = node.label
        else:
            x, y, yaw, label = step.x, step.y, step.yaw, f"map point ({step.x:.1f}, {step.y:.1f})"
        return await self._navigate(x, y, yaw, label, step.reasoning)

    async def _return_home(self, reason):
        node = self.planner.home_node()
        if node is not None:
            x, y, yaw = self.planner.approach_pose(node)
            label = node.label
        else:
            x, y, yaw = self.config.home_pose
            label = "home pose"
        return await self._navigate(x, y, yaw, label, reason or "Return to the charging dock")

    async def _ensure_ready_to_walk(self):
        telemetry = self.adapter.get_telemetry()
        if any(abs(v) > 1e-3 for v in telemetry.body_euler):
            self.log.emit("safety", "Resetting body posture to neutral before walking")
            self.adapter.euler(0.0, 0.0, 0.0)
            await self.clock.sleep(0.5)
        if not telemetry.standing:
            self.log.emit("safety", "Standing up before walking (RecoveryStand)")
            await self._in_thread(self.adapter.stand_up)
            await self.clock.sleep(1.5)

    async def _navigate(self, x, y, yaw, label, reason):
        if self.adapter.kind != "mock" and self.adapter.get_scan() is None \
                and not self.config.robot.allow_walking_without_scan:
            self.log.emit("safety", "Refusing autonomous walking: no LiDAR/depth scan source is registered "
                                    "(RealUnitreeAs2Adapter.register_scan_provider)", level="error")
            return False
        await self._ensure_ready_to_walk()
        self.nav_goal = [round(x, 3), round(y, 3), yaw]
        recoveries, ignore = 0, set()
        while True:
            pose = self.adapter.get_telemetry().pose
            path = await self._in_thread(self.navigator.plan, (pose.x, pose.y), (x, y), frozenset(ignore))
            if path is None:
                blocker = await self._in_thread(self._believed_blocker, (pose.x, pose.y), (x, y), frozenset(ignore))
                if blocker is None:
                    self.log.emit("navigation", f"No route to the {label} in the current map", level="error")
                    return False
                ignore.add(blocker.id)
                self.log.emit("replan", f"Every route to the {label} passes the {blocker.label} (believed "
                                        f"{blocker.state}); going to re-check it",
                              reason="Map beliefs can be stale; verify before giving up")
                continue
            self.path = [list(p) for p in path]
            self._set("EXECUTING", f"Navigating to {label} [x: {x:.1f}, y: {y:.1f}]", reason)
            result = await self._follow(path, yaw, label, (x, y), frozenset(ignore))
            kind = result["kind"]
            if kind == "arrived":
                self.path = []
                self.log.emit("navigation", f"Arrived at the {label}")
                return True
            if kind == "refused":
                self.log.emit("safety", f"Motion refused: {result['reason']}", level="error")
                return False
            if kind == "map_changed":
                self.log.emit("replan", f"Scene graph update blocks the current route; re-planning to the {label}",
                              reason=result["reason"])
                continue
            recoveries += 1
            if recoveries > self.config.max_recoveries_per_step:
                self.log.emit("recovery", f"Giving up on the {label} after {recoveries - 1} recoveries", level="error")
                return False
            outcome = await self._recover(result, (x, y), label)
            ignore.clear()
            if outcome == "abort":
                return False

    def _believed_blocker(self, start, goal, ignore):
        candidates = sorted(self.navigator.blockers(ignore),
                            key=lambda n: math.hypot(n.coords[0] - start[0], n.coords[1] - start[1]))
        for node in candidates:
            if self.navigator.plan(start, goal, frozenset(ignore | {node.id})) is not None:
                return node
        return None

    async def _checkpoint(self):
        if not self._paused:
            return
        self.adapter.stop_move()
        previous, self.state = self.state, "PAUSED"
        await self._resume.wait()
        self.state = previous if previous != "PAUSED" else "EXECUTING"

    async def _follow(self, path, final_yaw, label, goal_xy, ignore):
        lim = self.limits
        period = 1.0 / lim.control_hz
        index = 1
        start = self.clock.now()
        budget = path_length(path) / max(0.3 * lim.max_vx, 0.05) + 30.0
        best_remaining, progress_time, stall_since = math.inf, start, None
        signature, checked = self.navigator.signature(ignore), start
        while True:
            await self._checkpoint()
            now = self.clock.now()
            telemetry = self.adapter.get_telemetry()
            pose = telemetry.pose
            index = advance_waypoint(path, index, pose)
            last = index == len(path) - 1
            vx, vy, vyaw, status = follow_command(pose, path[index], final_yaw if last else None, last, lim)
            if status == "arrived":
                self.adapter.stop_move()
                return {"kind": "arrived"}
            if telemetry.blocked:
                self.adapter.stop_move()
                return {"kind": "blocked", "source": "contact", "distance": 0.0, "bearing": 0.0}
            to_waypoint = math.hypot(path[index][0] - pose.x, path[index][1] - pose.y)
            if vx > 0.05:
                scan = self.adapter.get_scan()
                heading = wrap(math.atan2(path[index][1] - pose.y, path[index][0] - pose.x) - pose.yaw)
                hit = corridor_obstacle(scan, lim, heading=heading) if scan else None
                if hit and hit[0] < to_waypoint + lim.robot_radius_m + 0.05:
                    self.adapter.stop_move()
                    return {"kind": "blocked", "source": "lidar", "distance": max(0.0, hit[0] - lim.robot_radius_m),
                            "bearing": hit[1]}
            if not self.adapter.move(vx, vy, vyaw):
                return {"kind": "refused", "reason": self.adapter.last_refusal or "motion refused"}
            speed = math.hypot(telemetry.velocity[0], telemetry.velocity[1])
            if status == "driving" and vx > 0.12 and speed < 0.03:
                stall_since = now if stall_since is None else stall_since
                if now - stall_since > lim.stall_time_s:
                    self.adapter.stop_move()
                    return {"kind": "stalled", "source": "odometry", "distance": None, "bearing": 0.0}
            else:
                stall_since = None
            remaining = to_waypoint + path_length(path[index:])
            if status in ("turning", "aligning") or remaining < best_remaining - lim.min_progress_m:
                best_remaining, progress_time = min(best_remaining, remaining), now
            elif now - progress_time > lim.progress_window_s:
                self.adapter.stop_move()
                return {"kind": "no_progress", "source": "odometry", "distance": None, "bearing": 0.0}
            if now - start > budget:
                self.adapter.stop_move()
                return {"kind": "timeout", "source": "timer", "distance": None, "bearing": 0.0}
            if now - checked > 1.0:
                checked = now
                current = self.navigator.signature(ignore)
                if current != signature:
                    added = {entry[0] for entry in current} - {entry[0] for entry in signature}
                    signature = current
                    valid = await self._in_thread(self.navigator.path_still_valid, path, index, ignore)
                    if not valid:
                        self.adapter.stop_move()
                        names = [f"{n.label} ({n.state}, via {n.extra.get('state_source', 'map')})"
                                 for n in map(self.graph.get_node, added) if n is not None]
                        return {"kind": "map_changed", "reason": "Newly believed blocker on the route: "
                                + (", ".join(names) or "updated obstacle geometry")}
            self.activity = f"Navigating to {label} [x: {goal_xy[0]:.1f}, y: {goal_xy[1]:.1f}] · {remaining:.1f} m left"
            await self.clock.sleep(period)

    async def _face(self, coords, timeout=8.0):
        deadline = self.clock.now() + timeout
        while self.clock.now() < deadline:
            await self._checkpoint()
            pose = self.adapter.get_telemetry().pose
            error = wrap(math.atan2(coords[1] - pose.y, coords[0] - pose.x) - pose.yaw)
            if abs(error) < self.limits.yaw_tolerance_rad * 1.5:
                break
            if not self.adapter.move(0.0, 0.0, turn_rate(error, self.limits)):
                break
            await self.clock.sleep(1.0 / self.limits.control_hz)
        self.adapter.stop_move()

    # ---- recovery -------------------------------------------------------------------
    async def _recover(self, result, goal_xy, label):
        self.adapter.stop_move()
        self.state = "RECOVERING"
        pose = self.adapter.get_telemetry().pose
        distance, bearing = result.get("distance"), result.get("bearing") or 0.0
        reach = (distance if distance is not None else 0.3) + self.limits.robot_radius_m + 0.3  # ~object centre
        point = (pose.x + reach * math.cos(pose.yaw + bearing), pose.y + reach * math.sin(pose.yaw + bearing))
        cause = {"blocked": "path blocked", "stalled": "velocity dropped to zero unexpectedly",
                 "no_progress": "no progress towards the waypoint", "timeout": "navigation timed out"}[result["kind"]]
        detail = f", obstacle {distance:.2f} m ahead" if distance else ""
        self.activity = f"Recovering: {cause} ({result['source']})"
        self.log.emit("anomaly", f"StopMove(): {cause} on the way to the {label}{detail}", level="warning",
                      reason=f"Monitor signal: {result['source']}")
        frame = await self._in_thread(self.adapter.get_frame)
        nearby = [{"label": n.label, "category": n.category, "state": n.state}
                  for n in self.graph.get_all_nodes(include_rooms=False)
                  if math.hypot(n.coords[0] - point[0], n.coords[1] - point[1]) < 1.5][:6]
        context = {"goal": label, "trigger": result["kind"], "distance_m": distance, "nearby_map_objects": nearby}
        try:
            diagnosis = await self._vlm(self.perception.diagnose_blockage, frame, context)
        except MODEL_ERRORS as exc:
            diagnosis = BlockageDiagnosis(cause="unknown", blocking_label="obstacle", requires_human=False,
                                          confidence=0.0, description=f"VLM diagnosis unavailable: {exc}"[:300])
        self.confidence, self.confidence_source = diagnosis.confidence, "VLM blockage diagnosis"
        self.log.emit("diagnosis", f"VLM diagnosis: {diagnosis.cause.replace('_', ' ')} - {diagnosis.description}",
                      reason=f"Asked the VLM why the path to the {label} is blocked", confidence=diagnosis.confidence,
                      data=diagnosis.model_dump())
        if (diagnosis.cause == "unknown" and result["source"] != "lidar") or (
                diagnosis.cause == "permanent_obstacle" and diagnosis.blocking_label.lower() in ("wall", "a wall")):
            # Clipped corner / known structure: the floor plan already has it; back off and re-plan.
            self.log.emit("recovery", "Blocked by known structure or no visible cause; backing off 0.3 m and re-planning",
                          reason="Recovery 0: path-following error, not a new obstacle")
            await self._back_off(0.3)
            self.state = "EXECUTING"
            return "retry"
        node = self._record_blockage(diagnosis, point)

        path = await self._in_thread(self.navigator.plan, (pose.x, pose.y), goal_xy)
        if path is not None:
            self.log.emit("replan", f"Alternate route to the {label} found ({path_length(path):.1f} m) "
                                    f"avoiding the {node.label}", reason="Recovery 1: re-plan around the blockage")
            self.state = "EXECUTING"
            return "replanned"
        if diagnosis.cause == "permanent_obstacle" and not diagnosis.requires_human:
            self.log.emit("recovery", f"The {node.label} is permanent and no alternate route exists; skipping this step",
                          reason="A person cannot reasonably move fixed structure", level="error")
            await self._back_off(0.3)
            return "abort"
        if diagnosis.cause in ("temporary_obstacle", "person_in_way", "unknown") and not diagnosis.requires_human:
            wait = self.config.obstacle_wait_s
            self.state = "WAITING_FOR_HUMAN" if diagnosis.cause == "person_in_way" else "RECOVERING"
            self.activity = f"Waiting up to {wait:.0f} s for the {node.label} to clear"
            self.log.emit("recovery", self.activity, reason="Recovery 2: no alternate route; the obstacle may be temporary")
            if await self._wait_until_clear(node, wait):
                self._clear_blocker(node)
                self.log.emit("recovery", f"The {node.label} cleared by itself; continuing")
                self.state = "EXECUTING"
                return "retry"
        return await self._ask_human(node, diagnosis)

    async def _back_off(self, distance):
        speed = 0.15
        end = self.clock.now() + distance / speed
        while self.clock.now() < end:
            scan = self.adapter.get_scan()
            if scan is not None and any(r < self.limits.robot_radius_m + 0.15 and math.cos(a) < -0.7
                                        for a, r in zip(scan.angles, scan.ranges)):
                break  # something right behind us
            if not self.adapter.move(-speed, 0.0, 0.0):
                break
            await self.clock.sleep(1.0 / self.limits.control_hz)
        self.adapter.stop_move()

    def _record_blockage(self, diagnosis, point):
        if diagnosis.cause == "door_closed":
            door = self.graph.nearest(point[0], point[1], category="door", max_distance=1.5)
            if door is None:
                door = self.graph.add_node(diagnosis.blocking_label or "door", "door", (*point, 1.0),
                                           max(diagnosis.confidence, 0.3), "closed", source="diagnosis",
                                           extra={"radius": 0.55})
            state = "locked" if door.state == "locked" else "closed"
            node = self.graph.update_node_state(door.id, state, diagnosis.confidence, diagnosis.description,
                                                source="diagnosis")
        else:
            category = "person" if diagnosis.cause == "person_in_way" else "obstacle"
            existing = self.graph.nearest(point[0], point[1], category=category, max_distance=0.6)
            if existing is not None:
                node = self.graph.update_node_state(existing.id, "blocking", diagnosis.confidence,
                                                    diagnosis.description, source="diagnosis")
            else:
                node = self.graph.add_node(diagnosis.blocking_label or "obstacle", category, (*point, 0.3),
                                           max(diagnosis.confidence, 0.3), "blocking", source="diagnosis",
                                           extra={"radius": 0.4,
                                                  "transient": diagnosis.cause != "permanent_obstacle"},
                                           notes=diagnosis.description)
        self.log.emit("map", f"Scene graph updated: {node.label} ({node.id}) -> {node.state}",
                      data={"id": node.id, "coords": node.coords, "state": node.state})
        return node

    def _clear_blocker(self, node):
        if node.category == "door":
            self.graph.update_node_state(node.id, "open", source="verification")
        else:
            self.graph.remove_node(node.id)

    def _aimed_range(self, node):
        """Median LiDAR range towards a blocker (door: floor-plan segment midpoint, else node centre)."""
        scan = self.adapter.get_scan()
        if scan is None:
            return None
        segment = node.extra.get("segment")
        tx, ty = ((segment[0] + segment[2]) / 2, (segment[1] + segment[3]) / 2) if segment else node.coords[:2]
        pose = self.adapter.get_telemetry().pose
        bearing = wrap(math.atan2(ty - pose.y, tx - pose.x) - pose.yaw)
        aimed = sorted(r for a, r in zip(scan.angles, scan.ranges) if abs(wrap(a - bearing)) <= math.radians(8))
        return aimed[len(aimed) // 2] if aimed else None

    @staticmethod
    def _opened(baseline, current):
        """LiDAR now sees well past where the blocker was when we stopped."""
        return baseline is not None and current is not None and current > baseline + 0.5

    async def _wait_until_clear(self, node, timeout):
        baseline = self._aimed_range(node)
        deadline = self.clock.now() + timeout
        while self.clock.now() < deadline:
            await self._checkpoint()
            await self.clock.sleep(1.0)
            if self._opened(baseline, self._aimed_range(node)):
                return True
        return False

    async def _ask_human(self, node, diagnosis):
        is_door = node.category == "door"
        if is_door:
            message = f"Excuse me, the {node.label} is closed. Could you open it for me?"
            question = f"Is the {node.label} open?"
        else:
            article = "an" if node.label[:1] in "aeiou" else "a"
            message = f"Excuse me, {article} {node.label} is blocking my way. Could you please move it?"
            question = f"Is the path clear of the {node.label}?"
        self.state = "WAITING_FOR_HUMAN"
        self.waiting_for = ("open the " if is_door else "move the ") + node.label
        self._set(None, f"Waiting for a person to {self.waiting_for}",
                  "Recovery 3: no alternate route; a person must act", kind="recovery")
        self.adapter.speak(message)
        start = self.clock.now()
        deadline, reminder = start + self.config.human_wait_timeout_s, start + self.config.human_reminder_s
        baseline, next_vlm_check = self._aimed_range(node), start + 6.0
        while self.clock.now() < deadline:
            await self._checkpoint()
            await self.clock.sleep(1.5)
            lidar_clear = self._opened(baseline, self._aimed_range(node))
            verdict = None
            if lidar_clear or (is_door and self.clock.now() >= next_vlm_check):
                next_vlm_check = self.clock.now() + 6.0
                frame = await self._in_thread(self.adapter.get_frame)
                try:
                    verdict = await self._vlm(self.perception.verify_visual_state, frame, node.label, question)
                except MODEL_ERRORS:
                    verdict = None
            if is_door:
                vlm_open = verdict is not None and verdict.answer == "yes" and verdict.confidence >= 0.6
                cleared = vlm_open or (lidar_clear and (verdict is None or verdict.answer != "no"))
            else:
                cleared = lidar_clear
            if not cleared:
                if self.clock.now() >= reminder:
                    reminder += self.config.human_reminder_s
                    self.adapter.speak("Sorry to bother you again. " + message.split(". ", 1)[-1])
                continue
            self._clear_blocker(node)
            evidence = [] if not lidar_clear else ["LiDAR sees past it"]
            if verdict is not None:
                evidence.append(f"VLM says {verdict.answer}")
            confirmed_by_vlm = verdict is not None and verdict.answer == "yes"
            self.log.emit("recovery", f"Path clear again: the {node.label} is {'open' if is_door else 'gone'} "
                                      f"({', '.join(evidence)})", confidence=verdict.confidence if confirmed_by_vlm else None)
            self.adapter.speak("Thank you!")
            self.waiting_for = None
            self.state = "EXECUTING"
            return "retry"
        self.waiting_for = None
        self.adapter.speak(f"I could not get past the {node.label}. I will skip this part of the task.")
        self.log.emit("recovery", f"Timed out waiting for help with the {node.label}", level="error")
        return "abort"

    # ---- verification -----------------------------------------------------------------
    async def _step_verify(self, step):
        node = self.planner.resolve(step.target_name)
        label = node.label if node else step.target_name
        self._set(None, f"Verifying the {label}: {step.question}", step.reasoning)
        if node is not None:
            await self._face(node.coords)
        verdict, frame, tried = None, None, set()
        viewpoints = [] if node is None else [self.planner.inspection_pitch(node, self.adapter.get_telemetry().pose),
                                              -self.limits.max_pitch, self.limits.max_pitch]
        while True:
            frame = await self._in_thread(self.adapter.get_frame)
            try:
                verdict = await self._vlm(self.perception.verify_visual_state, frame, label, step.question)
            except MODEL_ERRORS as exc:
                verdict = VisualVerdict(answer="uncertain", state="unknown", confidence=0.0,
                                        evidence=f"VLM error: {exc}"[:300])
            current = self.adapter.get_telemetry().body_euler[1]
            tried.add(round(current, 2))
            options = [p for p in viewpoints if all(abs(p - t) >= 0.05 for t in tried)]
            if verdict.answer != "uncertain" or not options:
                break
            pitch = options[0]
            self.log.emit("posture", f"The {label} is not clearly visible ({verdict.evidence}); Euler pitch "
                                     f"{math.degrees(pitch):.0f}° and look again",
                          reason="Closed-loop perception: change viewpoint before giving up")
            self.adapter.euler(0.0, pitch, 0.0)
            await self.clock.sleep(0.8)
        if node is not None and verdict.answer != "uncertain":
            self.graph.update_node_state(node.id, verdict.state, verdict.confidence, verdict.evidence,
                                         source="verification")
        self._record_finding(label, step.question, verdict, step.reasoning)
        if verdict.bbox and frame is not None:
            self.detections = [{"label": label, "bbox": list(verdict.bbox), "confidence": verdict.confidence,
                                "state": verdict.state, "highlight": True}]
            self.detection_frame = frame.frame_id
        return True

    def _record_finding(self, target, question, verdict, reason=None):
        finding = {"target": target, "question": question, "answer": verdict.answer, "state": verdict.state,
                   "confidence": verdict.confidence, "evidence": verdict.evidence, "time": time.time()}
        self.findings.append(finding)
        self.confidence, self.confidence_source = verdict.confidence, "VLM visual verification"
        self.log.emit("verification", f"{target}: {verdict.answer.upper()} ({verdict.state}) - {verdict.evidence}",
                      reason=reason, confidence=verdict.confidence, data=finding,
                      level="warning" if verdict.answer != "yes" else "info")

    def _findings_text(self):
        if not self.findings:
            return "I have nothing to report."
        parts = []
        for item in self.findings:
            if item["answer"] == "uncertain":
                parts.append(f"I could not verify the {item['target']}")
            else:
                parts.append(f"the {item['target']} is {item['state']}" + (", attention needed" if item["answer"] == "no" else ""))
        text = "; ".join(parts) + "."
        return text[0].upper() + text[1:]

    # ---- perception loop --------------------------------------------------------------
    async def _perception_loop(self):
        while True:
            try:
                await self.clock.sleep(self.config.vlm.perception_interval_s)
                if self.state == "EMERGENCY" or not self._vlm_lock.acquire(blocking=False):
                    continue
                try:
                    frame = await self._in_thread(self.adapter.get_frame)
                    detections = await self._in_thread(self.perception.detect_objects, frame) if frame else []
                finally:
                    self._vlm_lock.release()
                if not frame:
                    continue
                threshold = self.config.vlm.min_detection_confidence
                detections = [d for d in detections if d.confidence >= threshold]
                fused = await self._in_thread(self._fuse, detections, frame)
                self.detections = [{"label": d.label, "bbox": list(d.bbox), "confidence": d.confidence,
                                    "state": d.state, "category": d.category,
                                    "node_id": result[0].id if result else None}
                                   for d, result in fused]
                self.detection_frame = frame.frame_id
                for detection, result in fused:
                    if result and result[1]:
                        node = result[0]
                        x, y, z = node.coords
                        self.log.emit("discovery", f"New object mapped: {node.label} at [{x:.1f}, {y:.1f}, {z:.1f}]"
                                                   f" in the {node.room or 'unknown room'}",
                                      confidence=detection.confidence, data={"id": node.id})
            except asyncio.CancelledError:
                raise
            except (*MODEL_ERRORS, AdapterError) as exc:
                if time.monotonic() - self._last_perception_warning > 30:
                    self._last_perception_warning = time.monotonic()
                    self.log.emit("perception", f"Perception update failed: {exc}", level="warning")
            except Exception:
                log.exception("Perception loop error")

    def _fuse(self, detections, frame):
        return [(d, self.graph.ingest_detection(d, frame)) for d in detections]

    # ---- operator overrides -----------------------------------------------------------
    async def _cancel_task(self):
        self._paused = False
        self._resume.set()
        task = self._task
        if task is not None and not task.done():
            task.cancel()
            await asyncio.wait({task}, timeout=3)
        self._task = None
        self.adapter.stop_move()

    async def _override(self, action):
        if action in ("pause", "stop"):
            self.adapter.stop_move()
            if self.is_busy():
                self._paused = True
                self._resume.clear()
                self.log.emit("override", "PAUSE / STOP MOVE: StopMove() sent, task paused", level="warning")
            else:
                self.log.emit("override", "STOP MOVE: StopMove() sent", level="warning")
            return {"ok": True}
        if action == "resume":
            if not self._paused:
                return {"ok": False, "reason": "Not paused"}
            self._paused = False
            self._resume.set()
            self.log.emit("override", "Resumed by the operator")
            return {"ok": True}
        if action == "abort_return":
            await self._cancel_task()
            if self.adapter.emergency_latched:
                return {"ok": False, "reason": "Emergency latched; press Stand Up first"}
            self.log.emit("override", "ABORT & RETURN: task cancelled, returning to the charging dock", level="warning")
            self._task = self._loop.create_task(self._run_return_home())
            return {"ok": True}
        if action == "emergency_sit":
            await self._cancel_task()
            self.state, self.activity = "EMERGENCY", "Emergency: StopMove -> StandDown -> Damp"
            self.reason = "Operator pressed EMERGENCY DAMP / SIT DOWN"
            self.log.emit("override", "EMERGENCY: StopMove() -> StandDown() -> Damp()", level="error")
            ok = await self._in_thread(self.adapter.emergency_sit)
            return {"ok": bool(ok)}
        if action == "stand_up":
            if self.is_busy():
                return {"ok": False, "reason": "Busy"}
            ok = await self._in_thread(self.adapter.stand_up)
            if ok:
                self.state, self.activity, self.reason = "IDLE", "Standing; ready for commands", ""
                self.log.emit("override", "Stand up (RecoveryStand); emergency latch cleared")
            return {"ok": bool(ok), "reason": self.adapter.last_refusal}
        if action in ("arm", "disarm"):
            armed = self.adapter.set_armed(action == "arm")
            self.log.emit("override", "Motion ARMED by the operator" if armed else "Motion disarmed",
                          level="warning" if armed else "info")
            return {"ok": True, "armed": armed}
        return {"ok": False, "reason": f"Unknown override {action!r}"}

    async def _run_return_home(self):
        self._reset_view("Abort & return home")
        self.plan = TaskPlan(goal="Return home", summary="Operator abort: return to the charging dock",
                             planner="override", steps=[PlanStep(tool="return_home",
                                                                 reasoning="Operator pressed ABORT & RETURN")])
        self.step_status = ["pending"]
        await self._execute_plan(self.plan)

    # ---- simulation fault injection ---------------------------------------------------
    async def _inject(self, kind, params):
        if self.adapter.kind != "mock":
            return {"ok": False, "reason": "Fault injection is only available in simulation"}
        if kind == "close_next_door":
            door = self._next_door_on_path()
            if door is None:
                return {"ok": False, "reason": "No interior door on the current route (start a navigation task first)"}
            kind, params = "close_door", {"door_id": door}
        elif kind == "box_on_path":
            point = self._point_on_path(1.6)
            if point is None:
                return {"ok": False, "reason": "No active route (start a navigation task first)"}
            kind, params = "box", {"x": point[0], "y": point[1]}
        try:
            description = self.adapter.inject_anomaly(kind, **params)
        except (ValueError, KeyError, TypeError, AdapterError) as exc:
            return {"ok": False, "reason": str(exc)}
        self.log.emit("anomaly", f"[SIM] Injected: {description}", level="warning")
        return {"ok": True, "description": description}

    def _remaining_path(self):
        if len(self.path) < 2:
            return None
        pose = self.adapter.get_telemetry().pose
        nearest = min(range(len(self.path)), key=lambda i: math.hypot(self.path[i][0] - pose.x, self.path[i][1] - pose.y))
        return [[pose.x, pose.y]] + self.path[max(nearest, 1):]

    def _next_door_on_path(self):
        remaining = self._remaining_path()
        static = self.navigator.static_map or {}
        if remaining is None:
            return None
        for a, b in zip(remaining, remaining[1:]):
            dx, dy = b[0] - a[0], b[1] - a[1]
            length = math.hypot(dx, dy)
            if length < 1e-6:
                continue
            hits = []
            for door in static.get("doors", []):
                if door.get("exterior"):
                    continue
                t = ray_segment(a[0], a[1], dx / length, dy / length, *door["segment"])
                if t is not None and t <= length:
                    hits.append((t, door["id"]))
            if hits:
                return min(hits)[1]
        return None

    def _point_on_path(self, ahead):
        remaining = self._remaining_path()
        if remaining is None:
            return None
        travelled = 0.0
        for a, b in zip(remaining, remaining[1:]):
            length = math.hypot(b[0] - a[0], b[1] - a[1])
            if travelled + length >= ahead:
                k = (ahead - travelled) / length
                return a[0] + (b[0] - a[0]) * k, a[1] + (b[1] - a[1]) * k
            travelled += length
        return None
