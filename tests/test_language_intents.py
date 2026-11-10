import json
import math
from pathlib import Path
import unittest
from unittest.mock import Mock

from as2_vlm.agent.config import load_config
from as2_vlm.agent.language_intents import parse_clause
from as2_vlm.agent.physical_planner import PhysicalPlanner, primitive_arguments
from as2_vlm.vlm import VLMError


class LanguageIntentTests(unittest.TestCase):
    def setUp(self):
        self.chat = Mock()
        self.planner = PhysicalPlanner(load_config(path="", env={}), self.chat)

    def test_bilingual_corpus_without_model(self):
        cases = json.loads((Path(__file__).parents[1]/"as2_vlm/agent/language_cases/language_commands.json").read_text(encoding="utf-8"))
        for case in cases:
            with self.subTest(text=case["text"]):
                result = self.planner.plan(case["text"])
                self.assertEqual(result.status, case["status"])
                self.assertEqual(len(result.steps), len(case["steps"]))
                for actual, (tool, value) in zip(result.steps, case["steps"]):
                    self.assertEqual(actual.tool, tool)
                    self.assertAlmostEqual(actual.angle_rad if tool == "rotate_relative" else actual.distance_m or 0,
                                           math.radians(value) if tool == "rotate_relative" else value)
        self.chat.chat_json.assert_not_called()

    def test_full_turn_is_four_quarters_never_zero(self):
        native = primitive_arguments(self.planner.plan("Оглянись вокруг"))
        self.assertEqual(len(native), 4)
        self.assertTrue(all(mode == "rotate" for mode, _ in native))
        self.assertAlmostEqual(sum(value for _, value in native), 2*math.pi)

    def test_partial_phrases_and_typo_are_not_fuzzy_extracted(self):
        for text in ('say look around', 'не оглянись вокруг', 'look around the room',
                     'оглянис вокруг', 'look around 45 degrees', 'turn around twice'):
            with self.subTest(text=text):
                self.assertIsNone(parse_clause(text))

    def test_model_must_preserve_known_clause(self):
        for actions in (
            [("turn_left", 180), ("walk_backward", .5)],
            [("turn_right", 360), ("walk_backward", .5)],
            [("look_around", 360)],
        ):
            self.chat.chat_json.return_value = {"status":"ok", "summary":"test", "steps":[
                {"tool":tool, "value":value, "reasoning":"test"} for tool,value in actions]}
            with self.assertRaises(VLMError):
                self.planner.plan("look around then retreat half a metre")

    def test_semantic_model_intents_and_units(self):
        for tool, value, angle in (("look_around",360,360), ("turn_around",180,180), ("turn_right",45,-45)):
            self.chat.chat_json.return_value = {"status":"ok","summary":"test","steps":[
                {"tool":tool,"value":value,"reasoning":"test"}]}
            result = self.planner.plan("a free movement paraphrase")
            self.assertAlmostEqual(result.steps[0].angle_rad, math.radians(angle))

    def test_invalid_fixed_intents_and_requests_are_rejected(self):
        self.chat.chat_json.return_value = {"status":"ok","summary":"test","steps":[
            {"tool":"look_around","value":180,"reasoning":"test"}]}
        with self.assertRaises(VLMError):
            self.planner.plan("a free movement paraphrase")
        for text in ("walk forward 3 meters", "turn left 720 degrees", "stop then turn left",
                     "look around then turn left", "walk forward 0 meters"):
            with self.subTest(text=text), self.assertRaises(VLMError):
                self.planner.plan(text)

    def test_unknown_clause_is_not_silently_discarded(self):
        self.chat.chat_json.return_value = {"status":"unsupported","summary":"No map", "steps":[]}
        result = self.planner.plan("turn left then do a backflip")
        self.assertEqual(result.status,"unsupported")
        self.assertEqual(result.steps,[])
        self.chat.chat_json.assert_called_once()

    def test_unfamiliar_turn_phrases_still_preserve_angle_and_direction(self):
        for text,tool,value in (
            ("Выполни полный оборот оставаясь на месте","turn_left",180),
            ("Повернись в обратную сторону плавно через левый бок","turn_left",90),
            ("Face the opposite direction slowly with a clockwise half turn","turn_left",180),
        ):
            self.chat.chat_json.return_value = {"status":"ok","summary":"test","steps":[
                {"tool":tool,"value":value,"reasoning":"test"}]}
            with self.subTest(text=text), self.assertRaises(VLMError):
                self.planner.plan(text)

    def test_valid_unfamiliar_full_turn_is_allowed(self):
        self.chat.chat_json.return_value = {"status":"ok","summary":"test","steps":[
            {"tool":"turn_left","value":360,"reasoning":"test"}]}
        result = self.planner.plan("Выполни полный оборот оставаясь на месте")
        self.assertAlmostEqual(result.steps[0].angle_rad,2*math.pi)

    def test_units_and_expanded_bilingual_vocabulary(self):
        for text,tool,value in (
            ("Сдай назад на пятьдесят сантиметров","walk_relative",-.5),
            ("walk forward fifty centimeters","walk_relative",.5),
            ("Сделай полный оборот на месте","rotate_relative",2*math.pi),
            ("Повернись в обратную сторону через правый бок","rotate_relative",-math.pi),
            ("Face the opposite direction with a clockwise half turn","rotate_relative",-math.pi),
            ("Please, look around","rotate_relative",2*math.pi),
            ("Пожалуйста, оглянись вокруг","rotate_relative",2*math.pi),
        ):
            with self.subTest(text=text):
                result = self.planner.plan(text)
                self.assertEqual(result.steps[0].tool,tool)
                self.assertAlmostEqual(result.steps[0].angle_rad if tool == "rotate_relative" else result.steps[0].distance_m,value)
        self.chat.chat_json.assert_not_called()

    def test_after_that_preserves_sequence(self):
        result = self.planner.plan("walk forward 0.5 meters, after that turn right")
        self.assertEqual([s.tool for s in result.steps],["walk_relative","rotate_relative"])
        self.assertAlmostEqual(result.steps[1].angle_rad,-math.pi/2)
        self.chat.chat_json.assert_not_called()

    def test_unsupported_navigation_does_not_invent_distance(self):
        for text in ("Approach the nearest window", "Дойди до кухни", "turn left then go to the door"):
            result = self.planner.plan(text)
            self.assertEqual(result.status,"unsupported")
            self.assertEqual(result.steps,[])
        self.chat.chat_json.assert_not_called()


if __name__ == "__main__":
    unittest.main()
