import json
from pathlib import Path
import tempfile
import unittest

import numpy as np

from as2_vlm.agent.lidar_camera_extrinsics import fit_plane, solve_planes, solve_session

try:
    import cv2
except ImportError:
    cv2 = None


def rotation():
    theta = .32
    return np.array([[np.cos(theta),0,np.sin(theta)],[0,1,0],[-np.sin(theta),0,np.cos(theta)]])


def planes():
    R, t = rotation(),np.array([.12,-.08,.04])
    rng = np.random.default_rng(32)
    pairs = []
    for _ in range(12):
        n = np.r_[rng.uniform(-.7,.7,2),-1.]
        n /= np.linalg.norm(n)
        d = rng.uniform(1,2)
        pairs.append({'camera_normal':n,'camera_offset_m':d,
                      'lidar_normal':R.T@n,'lidar_offset_m':d+n@t})
    return pairs,R,t


class ExtrinsicsTests(unittest.TestCase):
    def test_known_rotation_translation_and_transform_direction_are_recovered(self):
        pairs,R,t = planes()
        transform,report = solve_planes(pairs)
        np.testing.assert_allclose(transform[:3,:3],R,atol=1e-9)
        np.testing.assert_allclose(transform[:3,3],t,atol=1e-9)
        self.assertEqual(report['holdout_views'],2)
        self.assertFalse(report['navigation_ready'])
        self.assertFalse(report['calibration_verified'])

    def test_inconsistent_heldout_plane_is_rejected(self):
        pairs,_,_ = planes()
        pairs[-1]['camera_offset_m'] += .15
        with self.assertRaisesRegex(ValueError,'held-out'):
            solve_planes(pairs)

    def test_parallel_views_cannot_invent_unobservable_transform(self):
        pairs = [{'camera_normal':[0,0,-1],'lidar_normal':[0,0,-1],
                  'camera_offset_m':1,'lidar_offset_m':1} for _ in range(12)]
        with self.assertRaisesRegex(ValueError,'parallel'):
            solve_planes(pairs)

    def test_insufficient_and_nonfinite_planes_refuse_solution(self):
        pairs,_,_ = planes()
        with self.assertRaises(ValueError):
            solve_planes(pairs[:5])
        pairs[0]['camera_offset_m'] = float('nan')
        with self.assertRaises(ValueError):
            solve_planes(pairs)

    def test_plane_fit_tolerates_some_outliers_without_accepting_line_or_noise(self):
        rng = np.random.default_rng(2)
        p = np.column_stack([rng.uniform(-.12,.12,500),rng.uniform(-.09,.09,500),rng.normal(1,.0005,500)])
        points = np.vstack([p,rng.uniform(-2,2,(30,3))])
        n,d,stats = fit_plane(points)
        np.testing.assert_allclose(n,[0,0,-1],atol=.002)
        self.assertAlmostEqual(d,1,places=3)
        self.assertGreater(stats['inliers'],490)
        for bad in (p[:10],rng.uniform(-1,1,(500,3)),np.column_stack([np.linspace(-1,1,100),np.zeros((100,2))])):
            with self.assertRaises(ValueError):
                fit_plane(bad)

    @unittest.skipUnless(cv2 is not None,'Optional OpenCV calibration environment required')
    def test_saved_paired_views_produce_candidate_with_independent_holdouts(self):
        rng = np.random.default_rng(5)
        R,t = rotation(),np.array([.12,-.08,.04])
        K = np.array([[900.,0,960],[0,910,540],[0,0,1]])
        square = .022
        obj = np.zeros((54,3))
        obj[:,:2] = np.mgrid[0:9,0:6].T.reshape(-1,2)*square
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            # The camera may have been calibrated with a different physical board.
            camera = {'image_size':[1920,1080],'K':K.tolist(),'distortion':[0]*5,'square_m':.025,
                      'quality_checks_passed':True,'focal_stability_relative_range':[0,0]}
            (root/'camera-intrinsics.json').write_text(json.dumps(camera))
            regions = {'schema':1,'lidar_length_unit':'m','unit_evidence':'synthetic test metre geometry',
                       'square_m':square,'views':[]}
            for i in range(12):
                rv = np.array([-.6+(i%4)*.4,-.6+(i//4)*.6,.04*np.sin(i)])
                rb,_ = cv2.Rodrigues(rv)
                tb = np.array([-.15,-.1,1.3+i*.04])
                corners,_ = cv2.projectPoints(obj,rv,tb,K,np.zeros(5))
                board = np.column_stack([rng.uniform(-square,9*square,600),
                                         rng.uniform(-square,6*square,600),np.zeros(600)])
                lidar = (board@rb.T+tb-t)@R
                lidar += rng.normal(0,.0001,lidar.shape)
                name = f'view-{i+1:03d}'
                pair = {'stationary':True,'frame_id':i+1,'cloud_sequence':i+10,'camera_age_s':.1,
                        'lidar_age_s':.1,'receipt_skew_s':0,'hub_instance':'synthetic','lidar_frame':'test_lidar'}
                metadata = {'frame_id':i+1,'image_size':[1920,1080],'square_m':square,
                            'corners':corners.tolist(),'lidar_pair':pair}
                (root/(name+'.json')).write_text(json.dumps(metadata))
                np.savez_compressed(root/(name+'.cloud.npz'),xyz=lidar)
                bounds = np.r_[lidar.min(0)-.001,lidar.max(0)+.001]
                regions['views'].append({'view':name,'same_board_confirmed':True,'bounds_xyz_m':bounds.tolist()})
            candidate,report = solve_session(root,regions)
            np.testing.assert_allclose(np.array(candidate['T_camera_from_lidar'])[:3,:3],R,atol=.002)
            np.testing.assert_allclose(np.array(candidate['T_camera_from_lidar'])[:3,3],t,atol=.003)
            self.assertTrue(report['candidate_only'])
            self.assertFalse(report['navigation_ready'])
            regions['views'][0]['same_board_confirmed'] = False
            with self.assertRaisesRegex(ValueError,'identification'):
                solve_session(root,regions)


if __name__ == '__main__':
    unittest.main()
