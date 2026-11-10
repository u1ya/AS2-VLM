import copy
import json
from pathlib import Path
import unittest
from types import SimpleNamespace
from unittest.mock import patch

from as2_vlm.agent.calibration_check import check_recording, profile_issues


def profile():
    return {"schema": 1, "sensor_model": "test device", "hardware_binding": "test fixture, not real robot",
            "lidar_length_unit": "m", "imu_acceleration_unit": "m/s^2", "imu_angular_velocity_unit": "rad/s",
            "point_time_unit": "s", "point_time_reference": "absolute_source_time",
            "imu_time_minus_lidar_time_s": 0,
            "T_imu_from_lidar": {"rotation": [[1, 0, 0], [0, 1, 0], [0, 0, 1]], "translation_m": [0, 0, 0]},
            "provenance": "synthetic test; identity is not an AS2 calibration"}


class CalibrationCheckTests(unittest.TestCase):
    def test_template_does_not_default_to_identity(self):
        template = json.loads((Path(__file__).resolve().parents[1] / "configs/calibration.example.json").read_text())
        self.assertIsNone(template["T_imu_from_lidar"])
        self.assertIsNone(template["imu_time_minus_lidar_time_s"])
        self.assertGreater(len(profile_issues(template)), 5)

    def test_complete_profile_is_only_numerically_consistent(self):
        self.assertEqual(profile_issues(profile()), [])

    def test_bad_schema_and_unknown_keys_are_rejected(self):
        for value in (None, {}, dict(profile(), schema=True), dict(profile(), assume_identity=True)):
            self.assertTrue(profile_issues(value))

    def test_bad_rotations_are_rejected(self):
        for matrix in ([[1, 0, 0], [0, 1, 0], [0, 0, -1]], [[2, 0, 0], [0, 1, 0], [0, 0, 1]],
                       [[1, 0], [0, 1]], [[True, 0, 0], [0, 1, 0], [0, 0, 1]],
                       [[float("nan"), 0, 0], [0, 1, 0], [0, 0, 1]]):
            value = profile()
            value["T_imu_from_lidar"]["rotation"] = matrix
            self.assertTrue(profile_issues(value))

    def test_bad_translations_and_offsets_are_rejected(self):
        for translation in ([0, 0], [0, float("inf"), 0], ["0", 0, 0]):
            value = profile()
            value["T_imu_from_lidar"]["translation_m"] = translation
            self.assertTrue(profile_issues(value))
        for offset in (None, True, "0", float("nan")):
            self.assertTrue(profile_issues(dict(profile(), imu_time_minus_lidar_time_s=offset)))

    def test_units_and_provenance_must_be_explicit(self):
        for key in ("sensor_model", "hardware_binding", "provenance", "point_time_unit", "point_time_reference"):
            value = copy.deepcopy(profile())
            value[key] = None
            self.assertTrue(profile_issues(value))

    def test_complete_profile_never_certifies_or_enables_navigation(self):
        recording = SimpleNamespace(imu=[{"linear_acceleration": [0, 9.8, 0]}])
        audit = {"imu": {"frames": ["same_frame"], "dominant_static_acceleration_axis": "+Y"},
                 "cloud": {"frames": ["same_frame"]}, "warnings": [],
                 "stationarity": {"odometry_consistent_with_stationary": True}}
        with patch("as2_vlm.agent.calibration_check.audit_recording", return_value=audit):
            report = check_recording(recording, profile())
        self.assertTrue(report["profile_complete"])
        self.assertFalse(report["calibration_verified"])
        self.assertFalse(report["navigation_ready"])
        self.assertFalse(report["slam_ready"])
        self.assertFalse(report["same_frame_name_proves_extrinsics"])
        self.assertEqual(report["static_acceleration_observation"]["unit_direction_in_imu_message_axes"], [0, 1, 0])
