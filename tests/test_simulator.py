import math
import unittest

from as2_vlm.contracts import PlanError
from as2_vlm.parser import parse_instruction
from as2_vlm.simulator import WORLD, free, simulate, wrap


class SimulatorTests(unittest.TestCase):
    def test_door_standoff(self):
        result = simulate(parse_instruction("Дойди до двери"))
        self.assertEqual(result["status"], "completed")
        self.assertAlmostEqual(math.dist(result["final_pose"][:2], WORLD["targets"]["door"]), 0.65)

    def test_two_steps(self):
        result = simulate(parse_instruction("Повернись к двери и подойди к ней"))
        self.assertEqual({f["step"] for f in result["frames"]}, {0, 1, 2})

    def test_collision(self):
        result = simulate(parse_instruction("Дойди до двери"), obstacle=True)
        self.assertEqual(result["status"], "blocked")
        self.assertTrue(all(free(*f["pose"][:2], True) for f in result["frames"]))

    def test_scan(self):
        result = simulate(parse_instruction("Оглянись вокруг"))
        self.assertAlmostEqual(wrap(result["final_pose"][2]), 0)
        self.assertEqual(result["final_pose"][:2], WORLD["start"][:2])
        self.assertGreater(len(result["frames"]), 100)

    def test_stop_cancels_remaining_step(self):
        result = simulate(parse_instruction("Остановись и пройди вперед на метр"))
        self.assertEqual(result["status"], "stopped")
        self.assertEqual(result["final_pose"], WORLD["start"])

    def test_boundary(self):
        result = simulate(parse_instruction("Пройди вперед на метр"), [7.5, 1, 0])
        self.assertEqual(result["status"], "blocked")

    def test_right_turn_then_walk_in_new_heading(self):
        result = simulate(parse_instruction("Повернись направо и пройди вперед на метр"), [4, 2, 0])
        self.assertEqual(result["status"], "completed")
        self.assertAlmostEqual(result["final_pose"][0], 4)
        self.assertAlmostEqual(result["final_pose"][1], 1)
        self.assertAlmostEqual(result["final_pose"][2], -math.pi / 2)
        self.assertTrue(any(f["step"] == 2 and f["skill"] == "walk_distance" for f in result["frames"]))

    def test_right_turn_then_walk_stops_at_start_boundary(self):
        result = simulate(parse_instruction("Повернись направо и пройди вперед на метр"))
        self.assertEqual(result["status"], "blocked")
        self.assertLess(result["final_pose"][1], WORLD["start"][1] - 0.9)

    def test_invalid_pose(self):
        for pose in ([float("nan"), 1, 0], [0, 0, 0], [1, 2]):
            with self.assertRaises(PlanError):
                simulate(parse_instruction("Остановись"), pose)

    def test_person_heading(self):
        result = simulate(parse_instruction("Повернись в сторону мужчины"))
        self.assertAlmostEqual(result["final_pose"][2], math.atan2(3.5, 0.1))
        self.assertFalse(result["robot_connected"])


if __name__ == "__main__":
    unittest.main()
