import json
import math
import unittest

try:
    import pydantic  # noqa: F401
except ImportError:
    raise unittest.SkipTest("agent extra (pydantic) is not installed")

from as2_vlm.agent.config import load_config
from as2_vlm.agent.models import Pose
from as2_vlm.agent.navigation import Navigator
from as2_vlm.agent.semantic_map import SemanticSceneGraph
from as2_vlm.agent.sim_world import SimWorld
from as2_vlm.agent.task_planner import LLMPlanResponse, PlanStep, TaskPlanner
from as2_vlm.agent.vlm_backends import llm_schema
from as2_vlm.contracts import PlanError
from as2_vlm.vlm import VLMError


class FakeChat:
    def __init__(self, answer):
        self.answer = answer
        self.calls = []

    def chat_json(self, system, user, image=None, schema=None, num_predict=768):
        self.calls.append({"system": system, "user": json.loads(user), "schema": schema})
        return self.answer


def step(tool, reasoning="because", **fields):
    base = {k: None for k in ("target_name", "x", "y", "yaw", "roll", "pitch", "question", "message")}
    return {"tool": tool, **base, **fields, "reasoning": reasoning}


class PlannerTests(unittest.TestCase):
    def setUp(self):
        self.config = load_config(path="")
        world = SimWorld()
        self.graph = SemanticSceneGraph()
        self.graph.seed(world.rooms, world.seed_nodes())
        self.planner = TaskPlanner(self.graph, Navigator(self.config.motion, world.static_map(), self.graph),
                                   self.config)

    def use_llm(self, answer):
        chat = FakeChat(answer)
        self.planner._chats["ollama"] = chat
        return chat

    def test_rule_plan_for_multi_step_goal(self):
        plan = self.planner.plan("Check if the back door is locked and make sure the stove is off", "rules",
                                 Pose(x=0.8, y=0.8))
        self.assertEqual([s.tool for s in plan.steps],
                         ["navigate_to", "adjust_posture", "verify_visual_state", "adjust_posture",
                          "navigate_to", "verify_visual_state", "speak"])
        questions = [s.question for s in plan.steps if s.question]
        self.assertEqual(questions, ["Is the back door locked?", "Is the stove off?"])
        self.assertLess(plan.steps[1].pitch, 0)  # look up at the lock
        self.assertTrue(all(s.reasoning for s in plan.steps))

    def test_rule_plan_navigation_coordinates_and_home(self):
        plan = self.planner.plan("Go to 3.5, 2.0 and then return home", "rules")
        self.assertEqual([s.tool for s in plan.steps], ["navigate_to", "return_home"])
        self.assertEqual((plan.steps[0].x, plan.steps[0].y), (3.5, 2.0))

    def test_ambiguous_and_unknown_targets_rejected(self):
        with self.assertRaisesRegex(PlanError, "ambiguous"):
            self.planner.plan("check the door", "rules")
        with self.assertRaises(PlanError):
            self.planner.plan("check the helicopter", "rules")
        with self.assertRaises(PlanError):
            self.planner.plan("go to 40, 40", "rules")

    def test_llm_plan_is_validated_and_grounded(self):
        chat = self.use_llm({"status": "ok", "summary": "Inspect the stove.", "steps": [
            step("navigate_to", target_name="stove"),
            step("verify_visual_state", target_name="stove", question="Is the stove off?"),
            step("speak", message="Report: {findings}")]})
        plan = self.planner.plan("is the stove off?", "ollama", Pose(x=1, y=1))
        self.assertEqual(plan.planner, "ollama")
        self.assertIn("stove", chat.calls[0]["user"]["scene_graph"])
        self.assertNotIn("$ref", json.dumps(chat.calls[0]["schema"]))

    def test_llm_hallucinated_target_rejected(self):
        self.use_llm({"status": "ok", "summary": "x", "steps": [step("navigate_to", target_name="garage")]})
        with self.assertRaisesRegex(PlanError, "garage"):
            self.planner.plan("go to the garage", "ollama")

    def test_llm_malformed_output_rejected(self):
        self.use_llm({"status": "ok", "summary": "x", "steps": [step("LowCmd", target_name="stove")]})
        with self.assertRaises(VLMError):
            self.planner.plan("do something", "ollama")
        self.use_llm({"status": "ok", "summary": "x", "steps": [step("navigate_to", target_name="stove", message="hi")]})
        with self.assertRaises(VLMError):
            self.planner.plan("do something", "ollama")

    def test_llm_posture_beyond_config_rejected(self):
        self.use_llm({"status": "ok", "summary": "x", "steps": [step("adjust_posture", roll=0, pitch=-0.5, yaw=0)]})
        with self.assertRaisesRegex(PlanError, "posture"):
            self.planner.plan("look up", "ollama")

    def test_llm_refusal_surfaces_reason(self):
        self.use_llm({"status": "unsupported", "summary": "Cannot climb stairs", "steps": []})
        with self.assertRaisesRegex(PlanError, "stairs"):
            self.planner.plan("climb the stairs", "ollama")

    def test_step_validator(self):
        with self.assertRaises(ValueError):
            PlanStep(tool="navigate_to", reasoning="no target")
        with self.assertRaises(ValueError):
            PlanStep(tool="adjust_posture", pitch=math.inf, reasoning="bad")
        with self.assertRaises(ValueError):
            PlanStep(tool="speak", message="hi", reasoning="ok", motor_torque=5)

    def test_llm_schema_requires_every_field(self):
        schema = llm_schema(LLMPlanResponse)
        item = schema["properties"]["steps"]["items"]
        self.assertEqual(set(item["required"]), set(item["properties"]))
        self.assertFalse(item["additionalProperties"])

    def test_approach_poses_face_targets_from_inside(self):
        door = self.planner.resolve("back door")
        x, y, yaw = self.planner.approach_pose(door, (0.8, 0.8))
        self.assertEqual(self.graph.room_of(x, y), "utility room")
        self.assertAlmostEqual(yaw, math.pi / 2, delta=0.3)
        stove = self.planner.resolve("stove")
        x, y, yaw = self.planner.approach_pose(stove)
        self.assertEqual(self.graph.room_of(x, y), "kitchen")
        self.assertLess(abs(yaw), 0.6)


if __name__ == "__main__":
    unittest.main()
