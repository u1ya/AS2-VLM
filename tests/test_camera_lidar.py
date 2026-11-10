import copy
import struct
import urllib.error
import unittest
from unittest.mock import Mock

import numpy as np

from as2_vlm.agent.camera_lidar import CameraLidarCalibration, ObservationBundle, associate, capture_receipt_bundle
from as2_vlm.agent.models import Detection


def profile():
    # Synthetic calibrated optical frame for analytic tests, NOT hardware values.
    return {"schema": 1, "lidar_frame": "synthetic", "length_unit": "m",
            "image_size": [640, 480], "K": [[400, 0, 320], [0, 400, 240], [0, 0, 1]],
            "distortion": [0, 0, 0, 0, 0], "T_camera_from_lidar": np.eye(4).tolist(),
            "evidence": "synthetic analytic test only"}


def bundle(xyz=None):
    if xyz is None:
        xyz = np.array([[x, y, 2] for x in np.linspace(-.1, .1, 5) for y in np.linspace(-.1, .1, 5)])
    return ObservationBundle(xyz, 7, 20, "synthetic", .02, .1, .08, True)


class CameraLidarTests(unittest.TestCase):
    def test_capture_pair_preserves_frame_cloud_and_hub_identity(self):
        raw = struct.pack('<Qffffff',23,1,2,3,4,5,6)
        snapshot = {'instance':'source-a','camera':{'sequence':7,'age_s':.08},
                    'state':{'age_s':.01,'error':'0','velocity':[0,0,0]},
                    'lidar':{'age_s':.1,'cloud_sequence':23,'error':0,'frame':'test',
                             'width':2,'height':1,'row_step':24,'point_step':12,'bigendian':False,
                             'fields':[{'name':axis,'count':1,'offset':i*4,'datatype':7} for i,axis in enumerate('xyz')]}}
        hub = Mock()
        hub.request.return_value=raw
        pair = capture_receipt_bundle(hub,snapshot,7)
        self.assertEqual(pair.frame_id,7)
        self.assertEqual(pair.cloud_sequence,23)
        self.assertEqual(pair.hub_instance,'source-a')
        np.testing.assert_array_equal(pair.xyz,[[1,2,3],[4,5,6]])
        hub.request.assert_called_once_with('/cloud?sequence=23',binary=True)
        for expected, elapsed in ((8,0),(7,.5)):
            hub.reset_mock()
            self.assertIsNone(capture_receipt_bundle(hub,snapshot,expected,elapsed))
            hub.request.assert_not_called()
        hub.request.side_effect=urllib.error.HTTPError('http://127.0.0.1:8780',409,'changed',{},None)
        self.assertIsNone(capture_receipt_bundle(hub,snapshot,7))

    def setUp(self):
        self.calibration = CameraLidarCalibration(profile())
        self.person = Detection(label="person", category="person", bbox=(.3, .2, .7, .8), confidence=.8)

    def test_projection_has_analytic_optical_coordinates_and_rejects_behind_camera(self):
        xyz = np.array([[0, 0, 2], [1, .5, 2], [0, 0, -2], [np.nan, 0, 2]])
        uv, depth, indices = self.calibration.project(xyz, (640, 480))
        np.testing.assert_allclose(uv, [[.5, .5], [520/640, 340/480]])
        np.testing.assert_equal(indices, [0, 1])
        np.testing.assert_equal(depth, [2, 2])

    def test_transform_direction_and_translation_are_applied(self):
        p = profile()
        p["T_camera_from_lidar"] = [[0, -1, 0, 1], [1, 0, 0, 0], [0, 0, 1, 2], [0, 0, 0, 1]]
        uv, depth, _ = CameraLidarCalibration(p).project(np.array([[0, 1, 0]]), (640, 480))
        np.testing.assert_allclose(uv, [[.5, .5]])
        np.testing.assert_allclose(depth, [2])

    def test_distortion_is_applied_before_intrinsics(self):
        p = profile()
        p["distortion"] = [.1, 0, 0, 0, 0]
        uv, _, _ = CameraLidarCalibration(p).project(np.array([[.5, 0, 1]]), (640, 480))
        self.assertAlmostEqual(uv[0, 0], (400*.5*1.025+320)/640)

    def test_no_identity_or_fov_fallback_for_missing_calibration(self):
        result = associate([self.person], bundle(), None, (640, 480))
        self.assertEqual(result["status"], "calibration_required")
        self.assertFalse(result["navigation_ready"])
        self.assertEqual(result["objects"], [])

    def test_pair_requires_matching_frame_and_stationary_nearby_receipts(self):
        for key, value in (("stationary", False), ("receipt_skew_s", .151),
                           ("camera_age_s", .6), ("lidar_age_s", -.1)):
            data = bundle()
            setattr(data, key, value)
            self.assertEqual(associate([self.person], data, self.calibration, (640, 480))["status"],
                             "pair_rejected_motion_or_timing")
        data = bundle()
        data.lidar_frame = "other"
        self.assertEqual(associate([self.person], data, self.calibration, (640, 480))["status"],
                         "calibration_frame_mismatch")

    def test_resolution_mismatch_is_not_silently_scaled(self):
        with self.assertRaisesRegex(ValueError, "resolution"):
            self.calibration.project(bundle().xyz, (768, 432))

    def test_candidate_retains_cloud_identity_and_does_not_claim_verified_object(self):
        data = bundle()
        original = data.xyz.copy()
        result = associate([self.person], data, self.calibration, (640, 480))
        self.assertEqual(result["objects"][0]["status"], "candidate")
        np.testing.assert_allclose(result["objects"][0]["position_lidar_m"], [0, 0, 2], atol=1e-8)
        self.assertEqual(result["pair"]["cloud_sequence"], 20)
        self.assertFalse(result["association_verified"])
        self.assertFalse(result["hardware_synchronized"])
        self.assertFalse(result["navigation_ready"])
        np.testing.assert_equal(data.xyz, original)

    def test_two_depth_surfaces_are_ambiguous_instead_of_assigning_wall_to_person(self):
        data = bundle()
        data.xyz = np.concatenate((data.xyz, data.xyz*2))
        result = associate([self.person], data, self.calibration, (640, 480))
        self.assertEqual(result["objects"][0]["status"], "ambiguous_depth")
        self.assertIsNone(result["objects"][0]["position_lidar_m"])

    def test_too_few_points_or_diffuse_surface_does_not_invent_a_position(self):
        for xyz in (np.zeros((0, 3)), bundle().xyz[:4],
                    np.array([[0, 0, z] for z in np.linspace(1, 3, 30)])):
            result = associate([self.person], bundle(xyz), self.calibration, (640, 480))
            self.assertIsNone(result["objects"][0]["position_lidar_m"])

    def test_invalid_calibration_is_rejected(self):
        cases = []
        for key, value in (("K", None), ("distortion", [0]*4), ("length_unit", "cm"),
                           ("evidence", ""), ("image_size", [True, 480])):
            p = profile(); p[key] = value; cases.append(p)
        for coefficient in (-1, True, "1", float("nan")):
            p = profile(); p["T_camera_from_lidar"][0][0] = coefficient; cases.append(p)
        p = profile(); p["T_camera_from_lidar"][3][3] = 2; cases.append(p)
        for p in cases:
            with self.subTest(profile=p), self.assertRaises(ValueError):
                CameraLidarCalibration(p)


if __name__ == "__main__":
    unittest.main()
