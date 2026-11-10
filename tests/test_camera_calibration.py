import io
import tempfile
from pathlib import Path
import time
import unittest
from unittest.mock import patch

import numpy as np

from as2_vlm.agent.camera_calibration import CalibrationSession, detect_board, solve_intrinsics, board_pose_diversity
from as2_vlm.agent.camera_lidar import ObservationBundle

try:
    import cv2
except ImportError:
    cv2 = None


@unittest.skipUnless(cv2 is not None, "Optional calibration OpenCV environment required")
class CameraCalibrationTests(unittest.TestCase):
    def test_upright_only_board_poses_request_missing_forward_backward_tilts(self):
        K=np.array([[600.,0,320],[0,600,240],[0,0,1]])
        intrinsics={'K':K.tolist(),'distortion':[0]*5}
        obj=np.zeros((54,3));obj[:,:2]=np.mgrid[0:9,0:6].T.reshape(-1,2)*.023
        def images(tilted):
            result=[]
            for i in range(12):
                rv=np.array([(-.45+(i%3)*.45) if tilted else 0.,-.6+i*.11,0.])
                p,_=cv2.projectPoints(obj,rv,np.array([-.1,-.05,1.]),K,np.zeros(5))
                result.append(p)
            return result
        flat=board_pose_diversity(images(False),intrinsics,.023)
        self.assertFalse(flat['ready'])
        self.assertIn('верхним краем',flat['hint'])
        diverse=board_pose_diversity(images(True),intrinsics,.023)
        self.assertTrue(diverse['ready'],diverse)
        self.assertFalse(diverse['navigation_ready'])

    def test_printed_pattern_is_detected_with_expected_corner_count(self):
        canvas = np.full((700,1000), 255, np.uint8)
        for row in range(7):
            for col in range(10):
                if (row+col)%2 == 0:
                    canvas[100+row*60:100+(row+1)*60, 160+col*60:160+(col+1)*60] = 0
        _, jpeg = cv2.imencode('.jpg',canvas)
        corners, size, preview = detect_board(jpeg.tobytes())
        self.assertEqual(corners.shape, (54,1,2))
        self.assertEqual(size, (1000,700))
        self.assertGreater(len(preview),1000)

    def test_absent_pattern_does_not_invent_corners(self):
        _, jpeg = cv2.imencode('.jpg',np.full((480,640,3),150,np.uint8))
        corners, _, _ = detect_board(jpeg.tobytes())
        self.assertIsNone(corners)

    def test_calibration_recovers_synthetic_intrinsics_with_holdout_views(self):
        K = np.array([[900.,0,960],[0,910,540],[0,0,1]])
        distortion = np.array([-.08,.02,.001,-.001,0.])
        obj = np.zeros((54,3),np.float32)
        obj[:,:2] = np.mgrid[0:9,0:6].T.reshape(-1,2)*.025
        samples = []
        rng = np.random.default_rng(32)
        for i in range(20):
            r = np.array([-.5+(i%5)*.25,-.5+(i%4)*.3,.1*np.sin(i)])
            t = np.array([-.5+(i%5)*.2,-.35+(i//5)*.18,.8+(i%3)*.15])
            image,_ = cv2.projectPoints(obj,r,t,K,distortion)
            samples.append((image+rng.normal(0,.03,image.shape)).astype(np.float32))
        result = solve_intrinsics(samples,(1920,1080))
        np.testing.assert_allclose(np.array(result['K'])[[0,1],[0,1]],[900,910],rtol=.015)
        self.assertLess(max(result['holdout_rms_px']),.15)
        self.assertFalse(result['navigation_ready'])
        self.assertFalse(result['lidar_extrinsics_available'])

    def test_duplicate_and_stale_views_are_not_saved(self):
        with tempfile.TemporaryDirectory() as root:
            session = CalibrationSession(None, Path(root)/'session')
            corners = np.mgrid[0:9,0:6].T.reshape(-1,1,2).astype(np.float32)*20+50
            session.frame = {'id':1, 'received':time.time(),'jpeg':b'synthetic',
                             'corners':corners,'size':(640,480)}
            self.assertEqual(session.capture()['views'],1)
            with self.assertRaisesRegex(ValueError,'ракурс'):
                session.capture()
            session.frame['corners'] = corners[::-1].copy()
            with self.assertRaises(ValueError):
                session.capture()
            session.frame['received'] -= 3
            with self.assertRaisesRegex(ValueError,'Свежая'):
                session.capture()

    def test_repeated_or_insufficient_views_cannot_produce_intrinsics(self):
        sample = np.mgrid[0:9,0:6].T.reshape(-1,1,2).astype(np.float32)*20+50
        for samples in ([sample]*5,[sample]*15):
            with self.assertRaises(ValueError):
                solve_intrinsics(samples,(640,480))

    def test_saved_cloud_matches_capture_frame_and_session_resumes_without_losing_views(self):
        import json
        with tempfile.TemporaryDirectory() as root:
            directory = Path(root)/'session'
            session = CalibrationSession(None, directory, square_m=.022)
            points = np.array([[1.,2.,3.],[4.,5.,6.]])
            pair = ObservationBundle(points, 7, 23, 'test_lidar', .02, .1, .08, True, 'source-a')
            session.frame = {'id':7,'received':time.time(),'jpeg':b'synthetic',
                'corners':np.mgrid[0:9,0:6].T.reshape(-1,1,2).astype(np.float32)*20+50,
                'size':(640,480),'pair':pair,'hub_instance':'source-a'}
            result = session.capture()
            self.assertEqual(result['paired_views'],1)
            saved = json.loads((directory/'view-001.json').read_text())
            self.assertEqual(saved['square_m'],.022)
            self.assertEqual(saved['lidar_pair']['cloud_sequence'],23)
            with np.load(directory/'view-001.cloud.npz',allow_pickle=False) as cloud:
                np.testing.assert_array_equal(cloud['xyz'], points)
            resumed = CalibrationSession(None,directory,square_m=.022,resume=True)
            self.assertEqual(len(resumed.samples),1)
            self.assertEqual(resumed.paired_views,1)
            self.assertEqual(resumed.hub_instance,'source-a')

    def test_mismatched_pair_leaves_no_partial_saved_view(self):
        with tempfile.TemporaryDirectory() as root:
            session = CalibrationSession(None,Path(root)/'session')
            pair = ObservationBundle(np.ones((10,3)),99,23,'test',0,.1,.1,True)
            session.frame = {'id':7,'received':time.time(),'jpeg':b'image','size':(640,480),'pair':pair,
                'corners':np.mgrid[0:9,0:6].T.reshape(-1,1,2).astype(np.float32)*20+50}
            with self.assertRaisesRegex(ValueError,'Несовпадение'):
                session.capture()
            self.assertEqual(list(session.directory.glob('view-*')),[])

    def test_lower_half_only_views_are_rejected_despite_sufficient_count(self):
        base = np.mgrid[0:9,0:6].T.reshape(-1,1,2).astype(np.float32)*5
        samples = [base + [20+(i%4)*140,260+(i//4)*35] for i in range(16)]
        with self.assertRaisesRegex(ValueError,'верхней'):
            solve_intrinsics(samples,(640,480),square_m=.022)

    def test_new_board_size_can_reuse_lens_intrinsics_but_requires_paired_cloud(self):
        import json
        with tempfile.TemporaryDirectory() as root:
            reference=Path(root)/'reference.json'
            reference.write_text(json.dumps({'K':[[400,0,320],[0,400,240],[0,0,1]],'distortion':[0]*5,
                'image_size':[640,480],'square_m':.022,'quality_checks_passed':True,
                'focal_stability_relative_range':[.01,.02]}))
            session=CalibrationSession(None,Path(root)/'new-board',square_m=.023,intrinsics_path=reference)
            self.assertEqual(session.status()['stage'],'camera_lidar')
            self.assertEqual(session.status()['square_mm'],23)
            session.frame={'id':7,'received':time.time(),'jpeg':b'image','size':(640,480),
                'corners':np.mgrid[0:9,0:6].T.reshape(-1,1,2).astype(np.float32)*20+50}
            with self.assertRaisesRegex(ValueError,'LiDAR'):
                session.capture()
            session.frame['pair']=ObservationBundle(np.ones((50,3)),7,23,'test',0,.1,.1,True)
            self.assertEqual(session.capture()['paired_views'],1)
            saved=json.loads((session.directory/'view-001.json').read_text())
            self.assertEqual(saved['square_m'],.023)
            self.assertEqual(session.reference_intrinsics['square_m'],.022)
            with self.assertRaises(ValueError):session.solve()


if __name__ == '__main__':
    unittest.main()
