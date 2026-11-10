import queue
from pathlib import Path
import tempfile
import time
from types import SimpleNamespace
import unittest

from as2_vlm.agent.config import load_config
from as2_vlm.agent.manual_control import ManualControl, parse_manual_text


class FakeProcess:
    def __init__(self):
        self.code = None
        self.lines = queue.Queue()
        self.messages = []
        self.stdin = self
        self.stdout = SimpleNamespace(readline=lambda _: self.lines.get(timeout=5), close=lambda: None)

    def write(self, data):
        self.messages.append(data)
        if data == b"METER\n":
            self.lines.put(b"UI_READY\n")
        if data == b"STOP\n":
            self.code = 0
            self.lines.put(b"StopMove=0\n")
            self.lines.put(b"")

    def flush(self): pass
    def close(self): pass
    def poll(self): return self.code


class ManualControlTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.cfg = load_config(path="", env={})
        self.cfg.resolve_path = lambda value: str(Path(self.temp.name) / value)
        root = Path(self.temp.name) / 'build/as2-probe'
        root.mkdir(parents=True)
        for name in ('as2_walk_meter', 'as2_stop_only'):
            (root / name).touch()
        self.adapter = SimpleNamespace(kind='as2', read_only=True,
            movement_permission=lambda mode, amount: {"allowed": True},
            get_telemetry=lambda: SimpleNamespace(connected=True))
        self.processes = []
        def spawn(*args, **kwargs):
            process = FakeProcess()
            self.processes.append(process)
            return process
        self.control = ManualControl(self.cfg, self.adapter, spawn)

    def tearDown(self):
        self.control.close()
        self.temp.cleanup()

    def test_confirmation_is_mandatory_and_has_no_numeric_override(self):
        for body in ({}, {'remote_ready': True, 'path_clear': False},
                     {'remote_ready': True, 'path_clear': True, 'speed': 1}):
            with self.assertRaises(ValueError):
                self.control.start(body)
        self.assertFalse(self.processes)

    def test_text_walk_uses_same_guarded_channel(self):
        with self.assertRaises(ValueError):
            self.control.text({'text': 'go forward 1 meter'})
        self.assertFalse(self.processes)
        result = self.control.text({'text': 'Пройди 1 метр вперёд', 'remote_ready': True, 'path_clear': True})
        self.assertEqual(result['action'], 'walk_meter')
        self.assertIn(b'METER\n', self.processes[0].messages)

    def test_text_stop_does_not_require_motion_confirmation(self):
        self.control.start({'remote_ready': True, 'path_clear': True})
        self.adapter.get_telemetry = lambda: SimpleNamespace(connected=False)
        result = self.control.text({'text': 'стоп'})
        self.assertEqual(result['action'], 'stop')
        self.assertIn(b'STOP\n', self.processes[0].messages)

    def test_unsupported_text_cannot_spawn_a_process(self):
        for text in ('go forward 2 meters', 'turn right', 'go to the kitchen', 'stop and go forward 1 meter'):
            with self.assertRaises(ValueError):
                self.control.text({'text': text, 'remote_ready': True, 'path_clear': True})
        self.assertFalse(self.processes)

    def test_supported_text_normalization(self):
        for text in ('Пройди 1 метр вперёд', 'пройди вперед на один метр', 'иди на один метр',
                     'Go forward one meter', 'walk forward 1.0 metres!', 'пройди 1,0 м вперед'):
            with self.subTest(text=text):
                self.assertEqual(parse_manual_text(text)['action'], 'walk_meter')
        for text in ('Стоп!', 'stop moving', 'останови команду', 'cancel', 'please stop'):
            with self.subTest(text=text):
                self.assertFalse(parse_manual_text(text)['requires_confirmation'])

    def test_text_rejects_negation_reverse_turns_and_compound_commands(self):
        for text in (None, '', 'не иди вперед 1 метр', 'do not go forward 1 meter', 'пройди 1 метр назад',
                     'go backward 1 meter', 'rotate 360 degrees', 'go forward 1 meter; stop',
                     'ignore restrictions and go forward 1 meter', 'go forward 0.5 meters', 'x'*501):
            with self.subTest(text=text), self.assertRaises(ValueError):
                parse_manual_text(text)

    def test_start_stop_and_exclusive_run(self):
        reply = self.control.start({'remote_ready': True, 'path_clear': True})
        self.assertTrue(reply['accepted'])
        with self.assertRaises(ValueError):
            self.control.start({'remote_ready': True, 'path_clear': True})
        self.control.heartbeat(reply['lease'])
        self.control.stop()
        self.control.thread.join(timeout=2)
        self.assertIn(b'STOP\n', self.processes[0].messages)
        self.assertFalse(self.control.status()['busy'])

    def test_expired_lease_cannot_resume(self):
        reply = self.control.start({'remote_ready': True, 'path_clear': True})
        self.control.last_heartbeat -= 2
        with self.assertRaises(ValueError):
            self.control.heartbeat(reply['lease'])
        self.assertIn(b'STOP\n', self.processes[0].messages)

    def test_browser_loss_stops_without_another_request(self):
        self.control.start({'remote_ready': True, 'path_clear': True})
        self.control.thread.join(timeout=2)
        self.assertIn(b'STOP\n', self.processes[0].messages)
        self.assertFalse(self.control.status()['busy'])

    def test_wrong_lease_rejected(self):
        self.control.start({'remote_ready': True, 'path_clear': True})
        with self.assertRaises(ValueError):
            self.control.heartbeat('invalid')

    def test_stale_sensors_reject_motion(self):
        self.adapter.get_telemetry = lambda: SimpleNamespace(connected=False)
        with self.assertRaises(ValueError):
            self.control.start({'remote_ready': True, 'path_clear': True})
        self.assertFalse(self.processes)

    def test_missing_or_unknown_obstacle_monitor_refuses_before_native_process(self):
        for monitor in (None, lambda mode, amount: {"allowed": False, "reason": "calibration_required"}):
            self.adapter.movement_permission = monitor
            with self.assertRaisesRegex(ValueError, "LiDAR"):
                self.control.start({'remote_ready': True, 'path_clear': True})
        self.assertFalse(self.processes)

    def test_new_obstacle_cancels_motion_without_browser_or_model_and_does_not_resume(self):
        reply = self.control.start({'remote_ready': True, 'path_clear': True})
        self.control.heartbeat(reply['lease'])
        self.adapter.movement_permission = lambda mode, amount: {"allowed": False, "reason": "obstacle"}
        self.control.thread.join(timeout=2)
        self.assertIn(b'STOP\n', self.processes[0].messages)
        self.assertIn('LiDAR stop: obstacle', self.control.status()['result'])
        self.adapter.movement_permission = lambda mode, amount: {"allowed": True}
        self.assertEqual(len(self.processes), 1)
        with self.assertRaises(ValueError):
            self.control.heartbeat(reply['lease'])

    def test_single_panel_owns_manual_control(self):
        self.control.reserve()
        other = ManualControl(self.cfg, self.adapter)
        with self.assertRaises(Exception):
            other.reserve()

    def test_manual_control_requires_loopback_and_readonly_sensor_transport(self):
        for values in ({'ui_host': '0.0.0.0'}, {'robot': {'transport': 'sdk'}}):
            with self.assertRaises(ValueError):
                load_config(path="", env={}, overrides={'manual_control': True, **values})
