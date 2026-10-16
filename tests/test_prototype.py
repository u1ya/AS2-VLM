import copy
import math
import unittest

from as2_vlm.bridge import MockBridge
from as2_vlm.contracts import PlanError, validate_plan
from as2_vlm.dry_run import preview_plan
from as2_vlm.parser import parse_instruction


class PrototypeTests(unittest.TestCase):
    def test_requested_examples(self):
        examples = {
            "Дойди до белой стены": "approach_target",
            "Дойди до двери": "approach_target",
            "Оглянись вокруг": "look_around",
            "Повернись в сторону мужчины": "face_target",
        }
        for text, skill in examples.items():
            with self.subTest(text=text):
                self.assertEqual(parse_instruction(text)["steps"][0]["skill"], skill)

    def test_two_actions_reuse_target(self):
        steps = parse_instruction("Повернись к двери и подойди к ней")["steps"]
        self.assertEqual([s["skill"] for s in steps], ["face_target", "approach_target"])
        self.assertEqual(steps[0]["target"], steps[1]["target"])

    def test_walk_and_stop(self):
        steps = parse_instruction("Пройди вперёд на метр и остановись")["steps"]
        self.assertEqual(steps[0]["distance_m"], 1.0)
        self.assertEqual(steps[1]["skill"], "stop")

    def test_unknown_negation_and_excess_steps_rejected(self):
        for text in ["Не иди к двери", "Прыгни", "Подойди к ней", "Стоп и стоп и стоп", "Пройди вперед на 50 метров"]:
            with self.subTest(text=text), self.assertRaises(PlanError):
                parse_instruction(text)

    def test_invalid_model_output_rejected(self):
        base = parse_instruction("Пройди вперед на метр")
        for value in [float("nan"), float("inf"), True, -1, 3, 10 ** 400]:
            plan = copy.deepcopy(base)
            plan["steps"][0]["distance_m"] = value
            with self.subTest(value=value), self.assertRaises(PlanError):
                validate_plan(plan)
        base["steps"][0]["speed"] = 10
        with self.assertRaises(PlanError):
            validate_plan(base)

    def test_right_turn_sign(self):
        self.assertAlmostEqual(parse_instruction("Повернись направо")["steps"][0]["angle_rad"], -math.pi / 2)

    def test_malformed_target_rejected(self):
        plan = parse_instruction("Дойди до двери")
        plan["steps"][0]["target"]["kind"] = ["door"]
        with self.assertRaises(PlanError):
            validate_plan(plan)

    def test_preview_does_not_execute(self):
        bridge = MockBridge()
        result = preview_plan(parse_instruction("Дойди до двери"), bridge)
        self.assertEqual(result["status"], "PREVIEW_ONLY")
        self.assertFalse(result["executed"])
        self.assertFalse(result["robot"]["hardware_connected"])
        self.assertEqual(bridge.stop_count, 0)


if __name__ == "__main__":
    unittest.main()
