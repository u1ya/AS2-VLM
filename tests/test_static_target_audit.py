import json
import struct
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import numpy as np

from as2_vlm.agent.static_target_audit import collect, corner_motion


class StaticTargetAuditTests(unittest.TestCase):
    def test_corner_order_and_motion(self):
        points = np.arange(108).reshape(54, 2)
        self.assertEqual(corner_motion(points, points[::-1]), 0)
        self.assertGreater(corner_motion(points, points+10), 3)

    def test_preserves_exact_raw_fields_and_sequence_for_three_stable_frames(self):
        class Hub:
            sequence = 0
            def request(self, path, binary=False):
                if path == '/health':
                    return {'read_only': True, 'instance': 'test'}
                if path == '/snapshot':
                    self.sequence += 1
                    return {'instance': 'test', 'camera': {'sequence': self.sequence, 'age_s': .02},
                            'state': {'age_s': .01, 'error': '0', 'velocity': [0, 0, 0]},
                            'lidar': {'age_s': .02, 'error': 0, 'cloud_sequence': self.sequence,
                                      'frame': 'test', 'width': 1, 'height': 1, 'point_step': 16,
                                      'row_step': 16, 'bigendian': False,
                                      'fields': [{'name': a, 'offset': i*4, 'count': 1, 'datatype': 7}
                                                 for i, a in enumerate(['x', 'y', 'z', 'intensity'])]}}
                if path.startswith('/frame?'):
                    return b'test-image'
                if path.startswith('/cloud?'):
                    return struct.pack('<Qffff', self.sequence, 1, 2, 3, 177)
                raise AssertionError(path)
        clock = [0.]
        def advance(_):
            clock[0] += 1.1
        with tempfile.TemporaryDirectory() as directory, \
                patch('as2_vlm.agent.static_target_audit.time.monotonic', side_effect=lambda: clock[0]), \
                patch('as2_vlm.agent.static_target_audit.time.sleep', side_effect=advance), \
                patch('as2_vlm.agent.static_target_audit.detect_board',
                      return_value=(np.zeros((54, 1, 2)), (1920, 1080), b'')):
            root = collect(Hub(), directory, 10)
            self.assertEqual(len(list(root.glob('*.cloud'))), 3)
            for i in range(1, 4):
                raw = (root / f'sample-{i:02d}.cloud').read_bytes()
                metadata = json.loads((root / f'sample-{i:02d}.json').read_text())
                self.assertEqual(struct.unpack('<Qffff', raw), (i, 1, 2, 3, 177))
                self.assertEqual(metadata['pair']['cloud_sequence'], i)
                self.assertFalse(metadata['calibration_verified'])


if __name__ == '__main__':
    unittest.main()
