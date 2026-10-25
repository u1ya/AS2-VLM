"""Closed-loop scenarios on the mock AS2 at 20x speed (SimulatedVLM, no model required)."""
import math
import sys
import time
import types
import unittest

try:
    import pydantic  # noqa: F401
except ImportError:
    raise unittest.SkipTest("agent extra (pydantic) is not installed")

from as2_vlm.agent.clock import Clock
from as2_vlm.agent.config import load_config
from as2_vlm.agent.execution_engine import EventLog, ExecutionEngine
from as2_vlm.agent.navigation import Navigator
from as2_vlm.agent.robot_adapter import AdapterError, MockAs2Adapter, RealUnitreeAs2Adapter
from as2_vlm.agent.semantic_map import SemanticSceneGraph
from as2_vlm.agent.task_planner import TaskPlanner
from as2_vlm.agent.vlm_backends import SimulatedVLM

SPEEDUP = 20


class Rig:
    def __init__(self, **overrides):
        self.config = load_config(path="", overrides={"sim_speedup": SPEEDUP, "human_wait_timeout_s": 120, **overrides})
        self.adapter = MockAs2Adapter(self.config, Clock(SPEEDUP))
        self.adapter.start()
        self.graph = SemanticSceneGraph()
        self.graph.seed(self.adapter.world.rooms, self.adapter.seed_nodes())
        navigator = Navigator(self.config.motion, self.adapter.static_map(), self.graph)
        self.log = EventLog()
        self.engine = ExecutionEngine(self.config, self.adapter, self.graph, TaskPlanner(self.graph, navigator, self.config),
                                      SimulatedVLM(), navigator, self.log)
        self.engine.start()

    def run(self, goal, timeout=60, on_tick=None):
        self.assert_accepted(self.engine.submit_goal(goal, "text", "rules"))
        deadline = time.time() + timeout
        while self.engine.is_busy() and time.time() < deadline:
            if on_tick:
                on_tick()
            time.sleep(0.05)
        return self.engine.state

    @staticmethod
    def assert_accepted(result):
        if not result.get("accepted"):
            raise AssertionError(result)

    def kinds(self):
        return [e["kind"] for e in self.log.recent(500)]

    def speech(self):
        return [text for _, text in self.adapter.speech_log]

    def close(self):
        self.engine.shutdown()
        self.adapter.close()


class EngineScenarioTests(unittest.TestCase):
    def setUp(self):
        self.rig = Rig()

    def tearDown(self):
        self.rig.close()

    def test_walk_one_meter_relative_to_rotated_start(self):
        adapter = self.rig.adapter
        with adapter._lock:
            adapter._pose = [1.2, 2.9, .35, .2]
        origin = adapter.get_telemetry().pose
        self.assertEqual(self.rig.run("go forward 1 meter"), "SUCCEEDED")
        pose = adapter.get_telemetry().pose
        progress = (pose.x - origin.x) * math.cos(origin.yaw) + (pose.y - origin.y) * math.sin(origin.yaw)
        self.assertAlmostEqual(progress, 1, delta=.04)
        self.assertAlmostEqual(pose.yaw, origin.yaw, delta=.03)

    def test_full_turn_accumulates_rotation_across_wrap(self):
        adapter = self.rig.adapter
        original_move = adapter._move_impl
        calls = []
        def record(vx, vy, vyaw):
            calls.append(vyaw)
            return original_move(vx, vy, vyaw)
        adapter._move_impl = record
        start = adapter.get_telemetry().pose
        self.assertEqual(self.rig.run("rotate by 360 degrees"), "SUCCEEDED")
        final = adapter.get_telemetry().pose
        self.assertGreater(len(calls), 50)
        self.assertTrue(all(yaw > 0 for yaw in calls))
        self.assertAlmostEqual(math.atan2(math.sin(final.yaw-start.yaw), math.cos(final.yaw-start.yaw)), 0, delta=.05)
        self.assertLess(math.hypot(final.x-start.x, final.y-start.y), .01)

    def test_turn_then_walk_uses_heading_at_step_execution(self):
        adapter = self.rig.adapter
        with adapter._lock:
            adapter._pose = [1.2, 1.2, .35, 0]
        start = adapter.get_telemetry().pose
        self.assertEqual(self.rig.run("turn left and go forward 0.5 meters"), "SUCCEEDED")
        final = adapter.get_telemetry().pose
        self.assertAlmostEqual(final.x, start.x, delta=.04)
        self.assertAlmostEqual(final.y - start.y, .5, delta=.04)

    def test_relative_motion_stops_at_obstacle_without_recovery(self):
        self.rig.adapter.inject_anomaly("box", x=1.6, y=.8)
        self.assertEqual(self.rig.run("go forward 1 meter"), "FAILED")
        self.assertNotIn("recovery", self.rig.kinds())
        self.assertEqual(self.rig.adapter._cmd, [0, 0, 0])

    def test_failed_relative_step_does_not_execute_the_rest(self):
        calls = []
        async def refuse(step):
            calls.append(step.tool)
            return False
        self.rig.engine._step_relative = refuse
        self.assertEqual(self.rig.run("turn left and go forward 1 meter"), "FAILED")
        self.assertEqual(calls, ["rotate_relative"])
        self.assertEqual(self.rig.engine.step_status, ["failed", "skipped"])

    def test_text_stop_has_priority_over_busy_motion(self):
        rig = self.rig
        rig.assert_accepted(rig.engine.submit_goal("rotate by 360 degrees"))
        time.sleep(.1)
        rig.assert_accepted(rig.engine.submit_goal("stop"))
        time.sleep(.1)
        self.assertTrue(rig.engine._paused)
        self.assertEqual(rig.adapter._cmd, [0, 0, 0])

    def test_multi_step_inspection_succeeds_and_reports(self):
        state = self.rig.run("Check if the back door is locked and make sure the stove is off")
        self.assertEqual(state, "SUCCEEDED")
        findings = {f["target"]: (f["answer"], f["state"]) for f in self.rig.engine.findings}
        self.assertEqual(findings, {"back door": ("yes", "locked"), "stove": ("yes", "off")})
        self.assertIn("Inspection report: The back door is locked; the stove is off.", self.rig.speech())
        self.assertEqual(self.rig.graph.resolve_target("stove").state, "off")

    def test_detects_unsafe_state(self):
        self.rig.adapter.inject_anomaly("object_state", object_id="stove", state="on")
        self.rig.run("make sure the stove is off")
        self.assertEqual(self.rig.engine.findings[-1]["answer"], "no")
        self.assertIn("attention needed", self.rig.speech()[-1])

    def test_closed_door_with_alternate_route_is_bypassed(self):
        self.rig.adapter.inject_anomaly("close_door", door_id="kitchen_door")
        self.assertEqual(self.rig.run("make sure the stove is off"), "SUCCEEDED")
        self.assertIn("replan", self.rig.kinds())
        self.assertEqual(self.rig.graph.resolve_target("kitchen door").state, "closed")

    def test_closed_only_route_asks_human_then_continues(self):
        rig = self.rig
        rig.adapter.inject_anomaly("close_door", door_id="utility_door")
        asked, helped = [], []

        def human():  # the person needs ~20 simulated seconds to come and open the door
            if rig.engine.state == "WAITING_FOR_HUMAN" and not asked:
                asked.append(time.time())
            if asked and not helped and time.time() - asked[0] > 1.0:
                self.assertEqual(rig.engine.state, "WAITING_FOR_HUMAN")  # no false "path clear"
                helped.append(True)
                rig.adapter.inject_anomaly("open_door", door_id="utility_door")
        self.assertEqual(rig.run("check if the back door is locked", on_tick=human), "SUCCEEDED")
        self.assertTrue(helped)
        self.assertEqual(rig.speech().count("Excuse me, the utility room door is closed. Could you open it for me?"), 1)
        self.assertIn("Thank you!", rig.speech())
        self.assertEqual(rig.engine.findings[-1]["state"], "locked")

    def test_box_on_route_is_handled(self):
        rig = self.rig
        placed = []

        def drop_box():
            if not placed and len(rig.engine.path) > 1 and rig.engine.state == "EXECUTING":
                time.sleep(0.1)
                result = rig.engine.inject_anomaly("box_on_path")
                if result["ok"]:
                    placed.append(result)
            if rig.engine.state == "WAITING_FOR_HUMAN":
                rig.adapter.inject_anomaly("clear")
        self.assertEqual(rig.run("go to the bedroom", on_tick=drop_box), "SUCCEEDED")
        self.assertTrue(placed)
        self.assertTrue({"replan", "diagnosis"} & set(rig.kinds()))

    def test_pause_resume_and_emergency(self):
        rig = self.rig
        rig.assert_accepted(rig.engine.submit_goal("go to the kitchen", "text", "rules"))
        time.sleep(0.4)
        self.assertTrue(rig.engine.override("pause")["ok"])
        time.sleep(0.3)
        telemetry = rig.adapter.get_telemetry()
        self.assertEqual(rig.engine.state, "PAUSED")
        self.assertLess(abs(telemetry.velocity[0]), 0.05)
        rig.engine.override("resume")
        time.sleep(0.2)
        self.assertNotEqual(rig.engine.state, "PAUSED")
        rig.engine.override("emergency_sit")
        self.assertEqual(rig.engine.state, "EMERGENCY")
        self.assertEqual(rig.adapter.get_telemetry().gait_mode, "damped")
        self.assertFalse(rig.engine.submit_goal("go to the kitchen")["accepted"])
        self.assertTrue(rig.engine.override("stand_up")["ok"])
        self.assertEqual(rig.engine.state, "IDLE")

    def test_abort_returns_home(self):
        rig = self.rig
        rig.assert_accepted(rig.engine.submit_goal("go to the bedroom", "text", "rules"))
        time.sleep(0.6)
        rig.engine.override("abort_return")
        deadline = time.time() + 40
        while rig.engine.is_busy() and time.time() < deadline:
            time.sleep(0.05)
        pose = rig.adapter.get_telemetry().pose
        self.assertEqual(rig.engine.state, "SUCCEEDED")
        self.assertLess(math.hypot(pose.x - 0.8, pose.y - 0.8), 0.4)


class AdapterSafetyTests(unittest.TestCase):
    def test_mock_clamps_and_watchdog(self):
        config = load_config(path="")
        adapter = MockAs2Adapter(config, Clock(10))
        adapter.start()
        try:
            self.assertTrue(adapter.move(5.0, 0.0, 9.0))
            self.assertEqual(adapter._cmd, [config.motion.max_vx, 0.0, config.motion.max_vyaw])
            time.sleep(0.15)  # 1.5 simulated seconds without a new command
            self.assertLess(max(abs(v) for v in adapter.get_telemetry().velocity), 0.05)
            adapter.set_armed(False)
            self.assertFalse(adapter.move(0.1, 0, 0))
            with self.assertRaises(ValueError):
                adapter.move(float("nan"), 0, 0)
        finally:
            adapter.close()


def install_fake_sdk(calls):
    class SportClient:
        def SetTimeout(self, t): calls.append(("SetTimeout", t))
        def Init(self): calls.append(("Init",))
        def __getattr__(self, name):
            def call(*args):
                calls.append((name, *args))
                return 0
            return call

    class ChannelSubscriber:
        def __init__(self, topic, kind): calls.append(("Subscribe", topic))
        def Init(self, handler, depth): calls.append(("SubscriberInit", depth))

    modules = {
        "unitree_sdk2py": {}, "unitree_sdk2py.as2": {}, "unitree_sdk2py.as2.sport": {},
        "unitree_sdk2py.as2.sport.sport_client": {"SportClient": SportClient},
        "unitree_sdk2py.core": {},
        "unitree_sdk2py.core.channel": {"ChannelFactoryInitialize": lambda d, i: calls.append(("DDS", d, i)),
                                        "ChannelSubscriber": ChannelSubscriber},
        "unitree_sdk2py.idl": {}, "unitree_sdk2py.idl.unitree_go": {}, "unitree_sdk2py.idl.unitree_go.msg": {},
        "unitree_sdk2py.idl.unitree_go.msg.dds_": {"SportModeState_": object},
    }
    saved = {name: sys.modules.get(name) for name in modules}
    for name, attrs in modules.items():
        module = types.ModuleType(name)
        module.__dict__.update(attrs)
        sys.modules[name] = module
    return saved


class RealAdapterTests(unittest.TestCase):
    def test_missing_sdk_is_reported(self):
        if "unitree_sdk2py" in sys.modules:
            self.skipTest("SDK importable here")
        with self.assertRaises(AdapterError):
            RealUnitreeAs2Adapter(load_config(path="")).start()

    def test_real_adapter_guards_and_watchdog(self):
        calls = []
        saved = install_fake_sdk(calls)
        try:
            config = load_config(path="", overrides={"robot": {"network_interface": "enp3s0"}})
            adapter = RealUnitreeAs2Adapter(config)
            adapter.start()
            self.assertIn(("DDS", 0, "enp3s0"), calls)
            self.assertIn(("Subscribe", "rt/lf/sportmodestate"), calls)
            self.assertFalse(adapter.move(0.2, 0, 0))          # motion_enabled defaults to False
            adapter.motion_enabled = True
            self.assertFalse(adapter.move(0.2, 0, 0))          # not armed yet
            adapter.set_armed(True)
            self.assertTrue(adapter.move(3.0, 0.0, -3.0))
            self.assertIn(("Move", 0.5, 0.0, -0.8), calls)
            time.sleep(0.6)                                    # stop streaming -> watchdog StopMove
            self.assertIn(("StopMove",), calls[-3:])
            adapter.emergency_latched = False
            adapter.config.robot.damp_delay_s = 0.0
            adapter.emergency_sit()
            names = [c[0] for c in calls]
            self.assertLess(names.index("StandDown"), names.index("Damp"))
            self.assertFalse(adapter.move(0.1, 0, 0))          # latched
            adapter.close()
        finally:
            for name, module in saved.items():
                if module is None:
                    sys.modules.pop(name, None)
                else:
                    sys.modules[name] = module


if __name__ == "__main__":
    unittest.main()
