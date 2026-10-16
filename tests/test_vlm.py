import json
import base64
import io
import unittest
from unittest.mock import Mock

from as2_vlm.contracts import PlanError
from as2_vlm.vlm import OllamaBackend, VLMError, ground_target, plan_instruction, strict_json


class VLMTests(unittest.TestCase):
    def prepared_image(self):
        try:
            from PIL import Image
        except ImportError:
            self.skipTest("Pillow vision extra is not installed")
        output = io.BytesIO()
        Image.new("RGB", (100, 100)).save(output, format="PNG")
        return base64.b64encode(output.getvalue()).decode("ascii")

    def test_plan_is_validated(self):
        backend = Mock()
        backend.chat_json.return_value = {"steps": [{"skill": "walk_distance", "distance_m": 99}]}
        with self.assertRaises(PlanError):
            plan_instruction(backend, "go")

    def test_plan_refusal(self):
        backend = Mock()
        backend.chat_json.return_value = {"error": "ambiguous target"}
        with self.assertRaises(PlanError):
            plan_instruction(backend, "go there")

    def test_two_action_model_plan(self):
        backend = Mock()
        backend.chat_json.return_value = {"steps": [
            {"skill": "face_target", "target": {"kind": "door"}},
            {"skill": "approach_target", "target": {"kind": "door"}}]}
        self.assertEqual(len(plan_instruction(backend, "door")["steps"]), 2)

    def test_compound_primitive_plan_completeness(self):
        backend = Mock()
        turn = {"skill": "turn_relative", "angle_rad": -1.5708}
        walk = {"skill": "walk_distance", "distance_m": 1}
        instruction = "Повернись направо и пройди вперед на метр"
        for steps in ([turn], [walk, turn], [turn, {**walk, "distance_m": 2}],
                      [{**turn, "angle_rad": 1.5708}, walk]):
            backend.chat_json.return_value = {"steps": steps}
            with self.subTest(steps=steps), self.assertRaises(PlanError):
                plan_instruction(backend, instruction)
        backend.chat_json.return_value = {"steps": [turn, walk]}
        self.assertEqual(len(plan_instruction(backend, instruction)["steps"]), 2)

    def test_found_box_is_only_proposal(self):
        backend = Mock()
        backend.chat_json.return_value = {"status": "found", "coordinate_system": "normalized_1000", "bbox_1000": [100, 200, 500, 800]}
        result = ground_target(backend, {"kind": "door"}, self.prepared_image())
        self.assertEqual(result["bbox"], [0.1, 0.2, 0.5, 0.8])
        self.assertFalse(result["metric_depth_available"])
        self.assertFalse(result["tracking_verified"])

    def test_invalid_grounding(self):
        backend = Mock()
        for response in [
            {"status": "found", "bbox_1000": [0, 0, 2000, 1000]},
            {"status": "found", "bbox_1000": [700, 0, 200, 1000]},
            {"status": "found", "bbox_1000": [False, 0, 1, 1]},
            {"status": "not_found", "bbox_1000": [0, 0, 1, 1]},
            {"status": "found", "bbox_1000": None},
            {"status": [], "bbox_1000": None},
            {"status": "found", "bbox_1000": [0.1, 0.2, 0.5, 0.8]},
        ]:
            backend.chat_json.return_value = {"coordinate_system": "normalized_1000", **response}
            with self.subTest(response=response), self.assertRaises(VLMError):
                ground_target(backend, {"kind": "door"}, self.prepared_image())

    def test_ambiguity_retained(self):
        backend = Mock()
        backend.chat_json.return_value = {"status": "ambiguous", "coordinate_system": "normalized_1000", "bbox_1000": None}
        self.assertEqual(ground_target(backend, {"kind": "person"}, self.prepared_image())["status"], "ambiguous")

    def test_strict_json(self):
        for raw in ['{"skill":"stop","skill":"go"}', '{"n":NaN}', '```json\n{}\n```']:
            with self.subTest(raw=raw), self.assertRaises(VLMError):
                strict_json(raw)

    def test_no_remote_endpoint(self):
        for endpoint in ["https://example.com", "http://192.168.1.2:11434", "http://localhost@evil.com"]:
            with self.subTest(endpoint=endpoint), self.assertRaises(VLMError):
                OllamaBackend(endpoint)

    def test_cloud_model_rejected(self):
        with self.assertRaises(VLMError):
            OllamaBackend(model="unknown-cloud")

    def test_chat_wire_format(self):
        backend = OllamaBackend()
        backend._request = Mock(return_value={"done": True, "message": {"content": '{"steps": [{"skill": "stop"}]}'}})
        answer = backend.chat_json("system", "user", "base64")
        self.assertEqual(answer["steps"][0]["skill"], "stop")
        path, payload = backend._request.call_args.args
        self.assertEqual(path, "/api/chat")
        self.assertEqual(payload["messages"][0]["images"], ["base64"])
        self.assertFalse(payload["stream"])

    def test_truncated_response_rejected(self):
        backend = OllamaBackend()
        backend._request = Mock(return_value={"done": True, "done_reason": "length", "message": {"content": "{}"}})
        with self.assertRaises(VLMError):
            backend.chat_json("s", "u")
