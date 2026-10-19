import json
import subprocess
import unittest
from unittest.mock import Mock, patch

from as2_vlm.state_bridge import ReadOnlyAS2Bridge, StateReadError, decode_state


class StateBridgeTests(unittest.TestCase):
    def test_strict_state_response(self):
        state = {'fsm_name': 'NORMAL_WALK', 'speed_level': '1'}
        self.assertEqual(decode_state(json.dumps({'sdk_return_code': 0, 'state': state})), state)
        invalid = [
            '{"sdk_return_code":0,"sdk_return_code":3104,"state":{}}',
            '{"sdk_return_code":true,"state":{"mode":"x"}}',
            '{"sdk_return_code":3104,"state":{}}',
            '{"sdk_return_code":0,"state":{"mode":NaN}}',
            '{"sdk_return_code":0,"state":{}}',
            '{"sdk_return_code":0,"state":{"speed_level":1}}',
            '{"sdk_return_code":0,"state":{"x":"y"},"extra":1}',
            'not JSON', 'x' * 65537,
        ]
        for raw in invalid:
            with self.subTest(raw=raw[:80]), self.assertRaises(StateReadError):
                decode_state(raw)

    def bridge(self):
        bridge = ReadOnlyAS2Bridge()
        bridge.command = Mock(return_value=['timeout', '12', '/probe', 'eth1', '--json'])
        return bridge

    @patch('as2_vlm.state_bridge.subprocess.run')
    def test_snapshot_is_read_only(self, run):
        run.return_value = subprocess.CompletedProcess([], 0, '{"sdk_return_code":0,"state":{"fsm_name":"NORMAL_WALK"}}', '')
        result = self.bridge().get_state()
        self.assertTrue(result['hardware_connected'])
        self.assertFalse(result['motion_enabled'])
        self.assertFalse(result['pose_available'])
        self.assertIn('observed_at', result)
        self.assertFalse(run.call_args.kwargs['shell'])
        self.assertEqual(run.call_args.kwargs['timeout'], 18)

    @patch('as2_vlm.state_bridge.subprocess.run')
    def test_timeout_and_failure(self, run):
        bridge = self.bridge()
        run.side_effect = subprocess.TimeoutExpired('probe', 18)
        with self.assertRaises(StateReadError):
            bridge.get_state()
        run.side_effect = None
        run.return_value = subprocess.CompletedProcess([], 1, '{"sdk_return_code":0,"state":{"fsm_name":"NORMAL_WALK"}}', '')
        with self.assertRaises(StateReadError):
            bridge.get_state()

    def test_busy_and_no_motion(self):
        bridge = self.bridge()
        bridge.lock.acquire()
        with self.assertRaises(StateReadError):
            bridge.get_state()
        bridge.lock.release()
        with self.assertRaises(StateReadError):
            bridge.stop()

    def test_invalid_interface_or_distro(self):
        for interface in ('--help', 'eth1; reboot', '', 'x' * 33):
            with self.subTest(interface=interface), self.assertRaises(StateReadError):
                ReadOnlyAS2Bridge(interface)
        with self.assertRaises(StateReadError):
            ReadOnlyAS2Bridge(distro='--exec')


if __name__ == '__main__':
    unittest.main()
