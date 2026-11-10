import unittest

import numpy as np

from as2_vlm.agent.obstacle_guard import ObstacleGuard, movement_permission


def profile():
    return {"schema": 1, "lidar_frame": "synthetic", "length_unit": "m",
            "T_body_from_lidar": np.eye(4).tolist(), "floor_z_m": -.3,
            "half_length_m": .4, "half_width_m": .25, "clearance_m": .6,
            "evidence": "synthetic test only; not AS2 calibration"}


def ring():
    theta = np.linspace(-np.pi, np.pi, 720, endpoint=False)
    return np.column_stack([3*np.cos(theta), 3*np.sin(theta), np.zeros(len(theta))])


class ObstacleGuardTests(unittest.TestCase):
    def setUp(self):
        self.now = 10
        self.guard = ObstacleGuard(profile(), clock=lambda: self.now)
        self.lidar = {"cloud_sequence": 1, "frame": "synthetic", "error": 0, "age_s": .02}
        self.state = {"error": "0", "age_s": .02}

    def ingest(self, points):
        self.guard.ingest(points, self.lidar, self.state)
        return self.guard.status()

    def test_missing_calibration_and_scan_never_allow_movement(self):
        for guard in (ObstacleGuard(), self.guard):
            for mode, amount in (("walk", 1), ("walk", -1), ("rotate", .5)):
                self.assertFalse(movement_permission(guard.status(), mode, amount)["allowed"])

    def test_known_far_returns_support_directions_in_synthetic_geometry(self):
        result = self.ingest(ring())
        for mode, amount in (("walk", 1), ("walk", -1), ("rotate", .5)):
            self.assertTrue(movement_permission(result, mode, amount)["allowed"])

    def test_front_rear_and_turning_envelopes_stop_correct_directions(self):
        result = self.ingest(np.vstack([ring(), [.6, 0, 0]]))
        self.assertFalse(result["forward"])
        self.assertTrue(result["reverse"])
        self.assertFalse(result["rotate"])
        self.lidar["cloud_sequence"] += 1
        result = self.ingest(np.vstack([ring(), [-.6, 0, 0]]))
        self.assertTrue(result["forward"])
        self.assertFalse(result["reverse"])
        self.assertFalse(result["rotate"])

    def test_calibrated_transform_changes_which_obstacle_is_in_front(self):
        p = profile()
        p["T_body_from_lidar"] = [[0,-1,0,0],[1,0,0,0],[0,0,1,0],[0,0,0,1]]
        self.guard = ObstacleGuard(p, clock=lambda: self.now)
        result = self.ingest(np.vstack([ring(), [0,-.6,0]]))
        self.assertFalse(result["forward"])
        self.assertTrue(result["reverse"])

    def test_unknown_empty_sparse_or_floor_only_never_means_clear(self):
        for points in (np.empty((0,3)), ring()[:20], ring()[:130], ring()+[0,0,-.3]):
            self.lidar["cloud_sequence"] += 1
            result = self.ingest(points)
            self.assertFalse(result["forward"])
            self.assertFalse(result["reverse"])
            self.assertFalse(result["rotate"])

    def test_stale_duplicate_scan_does_not_refresh_or_allow_motion(self):
        self.ingest(ring())
        self.now += .31
        self.ingest(ring())
        result = self.guard.status()
        self.assertEqual(result["reason"], "stale_lidar")
        self.assertFalse(movement_permission(result, "walk", 1)["allowed"])

    def test_sensor_errors_wrong_frames_and_sequence_regression_deny(self):
        for changes in ({"error": 2}, {"frame": "other"}, {"age_s": .4}, {"cloud_sequence": 0}):
            lidar = {**self.lidar, **changes}
            self.guard.ingest(ring(), lidar, self.state)
            self.assertFalse(movement_permission(self.guard.status(), "walk", 1)["allowed"])
        self.ingest(ring())
        self.guard.ingest(ring(), {**self.lidar, "cloud_sequence": 0}, self.state)
        self.assertFalse(movement_permission(self.guard.status(), "walk", 1)["allowed"])

    def test_old_or_missing_guard_age_cannot_be_accepted_from_another_service(self):
        for age in (None, -.1, .31, float("nan")):
            status = {"configured": True, "forward": True, "age_s": age}
            self.assertFalse(movement_permission(status, "walk", 1)["allowed"])

    def test_profile_needs_actual_dimensions_units_and_proper_rotation(self):
        for key, value in (("T_body_from_lidar", None), ("half_width_m", True),
                           ("length_unit", "cm"), ("floor_z_m", 0), ("clearance_m", .1)):
            p = profile(); p[key] = value
            with self.assertRaises(ValueError):
                ObstacleGuard(p)

    def test_invalid_movement_request_is_refused_even_with_fresh_clear_scan(self):
        result = self.ingest(ring())
        for mode, amount in (("fly",1), ("walk",0), ("walk",True), ("walk",float("nan"))):
            self.assertFalse(movement_permission(result, mode, amount)["allowed"])


if __name__ == "__main__":
    unittest.main()
