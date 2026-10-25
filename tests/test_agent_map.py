import math
import os
import tempfile
import unittest

try:
    import pydantic  # noqa: F401
except ImportError:
    raise unittest.SkipTest("agent extra (pydantic) is not installed")

from as2_vlm.agent.config import load_config
from as2_vlm.agent.models import Detection, Pose
from as2_vlm.agent.robot_adapter import MockAs2Adapter
from as2_vlm.agent.semantic_map import SemanticSceneGraph, label_similarity, project_detection
from as2_vlm.agent.vlm_backends import SimulatedVLM


class ProjectionTests(unittest.TestCase):
    OFFSET = [0.3, 0.0, 0.12]

    def test_centre_pixel_projects_straight_ahead(self):
        x, y, z = project_detection((0.45, 0.45, 0.55, 0.55), 2.0, Pose(), 640, 480, math.radians(90), self.OFFSET)
        self.assertAlmostEqual(x, 2.3)
        self.assertAlmostEqual(y, 0.0)
        self.assertAlmostEqual(z, 0.12)

    def test_left_of_centre_and_rotated_pose(self):
        x, y, _ = project_detection((0.2, 0.4, 0.3, 0.6), 2.0, Pose(yaw=math.pi / 2), 640, 480,
                                    math.radians(90), self.OFFSET)
        self.assertAlmostEqual(x, -1.0, places=6)  # 1 m left of a robot facing +y is -x
        self.assertAlmostEqual(y, 2.3, places=6)

    def test_invalid_depth_rejected(self):
        self.assertIsNone(project_detection((0.4, 0.4, 0.6, 0.6), None, Pose(), 640, 480, 1.5, self.OFFSET))
        self.assertIsNone(project_detection((0.4, 0.4, 0.6, 0.6), 50.0, Pose(), 640, 480, 1.5, self.OFFSET))


class SceneGraphTests(unittest.TestCase):
    def setUp(self):
        self.graph = SemanticSceneGraph()
        self.graph.add_room("kitchen", [6, 0, 10, 4.5])
        self.stove = self.graph.add_node("stove", "appliance", [9.6, 2.0, 0.45], 0.6, aliases=["cooker"])
        self.graph.add_node("kitchen door", "door", [6.0, 1.85, 1.0], 0.6)

    def test_fusion_merges_nearby_same_label(self):
        node, created = self.graph.fuse("stove", "appliance", (9.8, 2.1, 0.5), 0.9, "off")
        self.assertFalse(created)
        self.assertEqual(node.id, self.stove.id)
        self.assertEqual(node.observations, 2)
        self.assertGreater(node.confidence, 0.6)
        self.assertEqual(node.state, "off")
        self.assertTrue(9.6 < node.coords[0] < 9.8)

    def test_fusion_creates_new_node_for_new_object(self):
        node, created = self.graph.fuse("fire extinguisher", "safety", (6.5, 0.3, 0.35), 0.8)
        self.assertTrue(created)
        self.assertEqual(node.room, "kitchen")

    def test_new_obstacle_blocks(self):
        node, created = self.graph.fuse("cardboard box", "obstacle", (7.0, 3.0, 0.3), 0.8)
        self.assertTrue(created)
        self.assertEqual(node.state, "blocking")

    def test_query_by_label_alias_room_and_category(self):
        self.assertEqual(self.graph.query_map("the cooker")[0].id, self.stove.id)
        self.assertEqual(self.graph.query_map("kitchen")[0].category, "room")
        self.assertIn(self.stove.id, [n.id for n in self.graph.query_map("kitchen")])
        self.assertEqual(self.graph.query_map("doors")[0].label, "kitchen door")
        self.assertIsNone(self.graph.resolve_target("helicopter"))

    def test_verification_state_beats_detection_state(self):
        self.graph.update_node_state(self.stove.id, "on", 0.9, source="verification")
        self.graph.fuse("stove", "appliance", (9.6, 2.0, 0.45), 0.9, "off")
        self.assertEqual(self.graph.get_node(self.stove.id).state, "on")

    def test_persistence_round_trip(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, "map.sqlite")
            graph = SemanticSceneGraph(path)
            graph.add_room("kitchen", [6, 0, 10, 4.5])
            node = graph.add_node("stove", "appliance", [9.6, 2.0, 0.45], 0.7)
            graph.update_node_state(node.id, "off", 0.9)
            graph.close()
            reloaded = SemanticSceneGraph(path)
            self.assertEqual(reloaded.get_node(node.id).state, "off")
            self.assertEqual(reloaded.room_of(8, 1), "kitchen")
            self.assertEqual(reloaded.add_node("sink", "appliance", [7.6, 0.3, 0.45], 0.5).id, "obj_02")
            reloaded.close()

    def test_label_similarity(self):
        self.assertEqual(label_similarity("The Back-Door", "back door"), 1.0)
        self.assertGreater(label_similarity("door", "back door"), 0.8)
        self.assertEqual(label_similarity("stove", "window"), 0.0)


class PerceptionFusionTests(unittest.TestCase):
    def test_synthetic_camera_detections_land_on_true_objects(self):
        config = load_config(path="")
        adapter = MockAs2Adapter(config)
        adapter._pose = [8.75, 1.9, 0.35, 0.1]
        frame = adapter.get_frame()
        detections = SimulatedVLM().detect_objects(frame)
        self.assertIn("stove", [d.label for d in detections])
        graph = SemanticSceneGraph()
        for detection in detections:
            graph.ingest_detection(Detection(**{**detection.model_dump(), "depth_m": None}), frame)
        stove = graph.resolve_target("stove")
        self.assertLess(math.hypot(stove.coords[0] - 9.65, stove.coords[1] - 2.0), 0.35)


if __name__ == "__main__":
    unittest.main()
