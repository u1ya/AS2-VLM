import math
import queue
import tempfile
from pathlib import Path
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

from as2_vlm.agent.config import load_config
from as2_vlm.agent.manual_control import ManualControl
from as2_vlm.agent.physical_planner import PhysicalPlan, PhysicalPlanner, PhysicalStep, primitive_arguments
from as2_vlm.vlm import VLMError


def step(tool, distance=None, angle=None):
    return {"tool": tool, "distance_m": distance, "angle_rad": angle, "reasoning": "test instruction"}


def plan(steps):
    return PhysicalPlan(status="ok", summary="test", steps=steps)


class PhysicalPlannerTests(unittest.TestCase):
    def test_same_language_contract_is_used_by_simulation(self):
        from as2_vlm.agent.task_planner import TaskPlanner
        from as2_vlm.agent.semantic_map import SemanticSceneGraph
        from as2_vlm.agent.navigation import Navigator
        config=load_config(path="",env={},overrides={"relative_language_planner":True})
        graph=SemanticSceneGraph()
        try:
            planner=TaskPlanner(graph,Navigator(config.motion,None,graph),config)
            planner._relative_planner=Mock()
            planner._relative_planner.plan.return_value=plan([step("rotate_relative",angle=-math.pi/4),step("walk_relative",distance=1)])
            result=planner.plan("a natural instruction","ollama")
            self.assertEqual([s.tool for s in result.steps],["rotate_relative","walk_relative"])
            self.assertEqual(result.steps[1].distance_m,1)
        finally:graph.close()
    def test_model_not_templates_interprets_free_text(self):
        chat=Mock()
        chat.chat_json.return_value={"status":"ok","summary":"test","steps":[
            {"tool":"turn_right","value":45,"reasoning":"test"},
            {"tool":"walk_backward","value":.5,"reasoning":"test"}]}
        planner=PhysicalPlanner(load_config(path="",env={}),chat)
        result=planner.plan("a free paraphrase outside the command vocabulary")
        self.assertEqual(len(result.steps),2)
        self.assertAlmostEqual(result.steps[0].angle_rad,-math.pi/4)
        chat.chat_json.assert_called_once()
        self.assertIn("a free paraphrase",chat.chat_json.call_args.args[1])
        self.assertIsNone(chat.chat_json.call_args.args[2])

    def test_finite_numbers_and_skill_arguments_are_strict(self):
        for value in (step("walk_relative",distance=float("nan")),step("walk_relative",distance=True),
                      step("walk_relative",distance=3),step("rotate_relative",angle=8),
                      step("walk_relative",distance=.5,angle=1),step("stop",distance=1),
                      step("navigate_to"),dict(step("stop"),shell="anything")):
            with self.subTest(value=value),self.assertRaises(ValueError):
                PhysicalStep.model_validate(value)

    def test_sequence_budget_and_terminal_stop(self):
        for steps in ([step("walk_relative",distance=1.5)]*2,
                      [step("rotate_relative",angle=math.pi)]*3,
                      [step("stop"),step("walk_relative",distance=.5)], [step("stop")]*5):
            with self.assertRaises(ValueError): plan(steps)
        with self.assertRaises(ValueError): PhysicalPlan(status="unsupported",summary="no map",steps=[step("stop")])

    def test_native_splitting_preserves_amount_sign_and_order(self):
        result=primitive_arguments(plan([step("rotate_relative",angle=-2*math.pi),step("walk_relative",distance=1.5),step("stop")]))
        self.assertEqual([mode for mode,_ in result],["rotate"]*4+["walk"]*2+["stop"])
        self.assertAlmostEqual(sum(value for mode,value in result if mode=="rotate"),-2*math.pi)
        self.assertAlmostEqual(sum(value for mode,value in result if mode=="walk"),1.5)
        self.assertTrue(all(abs(value)<=1 for mode,value in result if mode=="walk"))
        self.assertTrue(all(abs(value)<=math.pi/2 for mode,value in result if mode=="rotate"))

    def test_common_rounded_angles_are_canonicalized(self):
        self.assertAlmostEqual(PhysicalStep.model_validate(step("rotate_relative",angle=-6.2832)).angle_rad,-2*math.pi)

    def test_invalid_model_output_has_no_execution_fallback(self):
        chat=Mock();chat.chat_json.return_value={"status":"ok","summary":"go","steps":[{"tool":"walk_forward","value":100,"reasoning":"test"}]}
        with self.assertRaises(VLMError): PhysicalPlanner(load_config(path="",env={}),chat).plan("move")


class PlanControlTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory()
        self.cfg=load_config(path="",env={})
        self.cfg.resolve_path=lambda value:str(Path(self.temp.name)/value)
        root=Path(self.temp.name)/"build/as2-probe";root.mkdir(parents=True)
        for name in ("as2_relative_move","as2_stop_only"):(root/name).touch()
        adapter=SimpleNamespace(kind="as2",read_only=True,get_telemetry=lambda:SimpleNamespace(connected=True),
                                movement_permission=lambda mode, amount: {"allowed": True})
        self.spawn=Mock(side_effect=RuntimeError("mock spawn refused"))
        self.control=ManualControl(self.cfg,adapter,self.spawn)
        self.control._planner=Mock()
        self.control._planner.plan.return_value=plan([step("walk_relative",distance=.5)])

    def tearDown(self):
        self.control.close();self.temp.cleanup()

    def test_preview_does_not_create_any_native_process(self):
        result=self.control.prepare_plan({"text":"half a metre"})
        self.assertTrue(result["plan_id"])
        self.spawn.assert_not_called()
        self.assertFalse(self.control.status()["planning"])

    def test_stop_bypasses_model(self):
        self.assertEqual(self.control.prepare_plan({"text":"стоп"})["action"],"stop")
        self.control._planner.plan.assert_not_called();self.spawn.assert_not_called()

    def test_plan_cannot_be_replaced_by_client_supplied_steps(self):
        preview=self.control.prepare_plan({"text":"half a metre"})
        body={"plan_id":preview["plan_id"],"remote_ready":True,"path_clear":True,"steps":[step("walk_relative",distance=100)]}
        with self.assertRaises(ValueError):self.control.execute_plan(body)
        self.spawn.assert_not_called()

    def test_expired_and_unknown_plans_are_rejected(self):
        preview=self.control.prepare_plan({"text":"half a metre"})
        for identifier in ("unknown",preview["plan_id"]):
            self.control._prepared["at"]-=61
            with self.assertRaises(ValueError):self.control.execute_plan({"plan_id":identifier,"remote_ready":True,"path_clear":True})
        self.spawn.assert_not_called()

    def test_stop_invalidates_outstanding_inference(self):
        entered=threading.Event();release=threading.Event();errors=[]
        def slow(text):entered.set();release.wait(3);return plan([step("walk_relative",distance=.5)])
        self.control._planner.plan.side_effect=slow
        def prepare():
            try:self.control.prepare_plan({"text":"half a metre"})
            except ValueError as exc:errors.append(str(exc))
        thread=threading.Thread(target=prepare);thread.start();self.assertTrue(entered.wait(1))
        # Emulate stop invalidation without a real network StopMove subprocess.
        with self.control.lock:self.control._generation+=1;self.control._prepared=None
        release.set();thread.join(2)
        self.assertTrue(errors);self.assertIsNone(self.control._prepared)

    def test_clarification_never_produces_executable_plan(self):
        self.control._planner.plan.return_value=PhysicalPlan(status="clarify",summary="How far?",steps=[])
        result=self.control.prepare_plan({"text":"a little forward"})
        self.assertIsNone(result["plan_id"]);self.spawn.assert_not_called()

    def test_spawn_failure_does_not_continue_sequence_or_leave_busy(self):
        self.control._planner.plan.return_value=plan([step("walk_relative",distance=.5),step("rotate_relative",angle=.5)])
        preview=self.control.prepare_plan({"text":"walk then turn"})
        self.control.execute_plan({"plan_id":preview["plan_id"],"remote_ready":True,"path_clear":True})
        self.control.thread.join(2)
        self.assertEqual(self.spawn.call_count,1)
        self.assertFalse(self.control.status()["busy"])
        self.assertEqual(self.control.status()["exit_code"],1)

    def test_native_failure_skips_later_actions(self):
        class Job:
            def __init__(self, code):
                self.code=None;self.result=code;self.stdin=self;self.lines=queue.Queue()
                self.stdout=SimpleNamespace(readline=lambda _:self.lines.get(timeout=4),close=lambda:None)
            def write(self, value):
                if value==b'RELATIVE\n':self.lines.put(b'UI_READY\n')
                if value in (b'KEEPALIVE\n',b'STOP\n') and self.code is None:
                    self.code=self.result if value==b'KEEPALIVE\n' else 1
                    self.lines.put(b'Result=distance_reached\n' if self.code==0 else b'Result=no_progress\n')
                    self.lines.put(b'StopMove=0 telemetry_stationary=1\n');self.lines.put(b'')
            def flush(self):pass
            def close(self):pass
            def poll(self):return self.code
        spawned=[]
        def spawn(*args,**kwargs):
            job=Job(0 if not spawned else 1);spawned.append(job);return job
        self.control.popen=spawn
        self.control._planner.plan.return_value=plan([step('walk_relative',distance=.5),step('rotate_relative',angle=.5),step('walk_relative',distance=.5)])
        preview=self.control.prepare_plan({'text':'three actions'})
        result=self.control.execute_plan({'plan_id':preview['plan_id'],'remote_ready':True,'path_clear':True})
        until=time.monotonic()+3
        while self.control.status()['busy'] and time.monotonic()<until:
            try:self.control.heartbeat(result['lease'])
            except ValueError:break
            time.sleep(.04)
        self.control.thread.join(2)
        self.assertEqual(len(spawned),2)
        self.assertFalse(self.control.status()['busy'])
        self.assertEqual(self.control.status()['exit_code'],1)
        self.assertIn('remaining steps skipped',self.control.status()['result'])
