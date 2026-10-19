import json
import threading
import unittest
from http.server import ThreadingHTTPServer
from urllib.error import HTTPError
from urllib.request import ProxyHandler, Request, build_opener
from unittest.mock import Mock

from as2_vlm.demo import Handler
from as2_vlm.state_bridge import StateReadError


class QuietHandler(Handler):
    def log_message(self, *args):
        pass


class DemoStateTests(unittest.TestCase):
    def setUp(self):
        self.server = ThreadingHTTPServer(('127.0.0.1', 0), QuietHandler)
        self.server.robot_bridge = Mock()
        self.server.robot_bridge.get_state.return_value = {'motion_enabled': False, 'state': {'fsm_name': 'NORMAL_WALK'}}
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()
        self.url = f'http://127.0.0.1:{self.server.server_port}'
        self.opener = build_opener(ProxyHandler({}))

    def tearDown(self):
        self.server.shutdown()
        self.server.server_close()
        self.thread.join()

    def request(self, headers=None):
        request = Request(self.url + '/robot-state', data=b'', method='POST',
                          headers=headers if headers is not None else {'X-AS2-Read-Only': '1', 'Origin': self.url})
        try:
            response = self.opener.open(request, timeout=3)
        except HTTPError as exc:
            response = exc
        with response:
            return response.status, json.loads(response.read())

    def test_state_request_is_separate(self):
        status, data = self.request()
        self.assertEqual(status, 200)
        self.assertFalse(data['motion_enabled'])
        self.server.robot_bridge.get_state.assert_called_once_with()
        self.server.robot_bridge.stop.assert_not_called()

    def test_disabled_and_failed_state(self):
        self.server.robot_bridge.get_state.side_effect = StateReadError('SDK GetState failed: 3104')
        status, data = self.request()
        self.assertEqual(status, 503)
        self.assertFalse(data['hardware_connected'])
        self.server.robot_bridge = None
        self.assertEqual(self.request()[0], 503)

    def test_untrusted_requests_never_call_hardware(self):
        for headers in ({}, {'X-AS2-Read-Only': '1', 'Origin': 'https://example.com'},
                        {'X-AS2-Read-Only': '1', 'Host': 'attacker.example'}):
            with self.subTest(headers=headers):
                self.assertEqual(self.request(headers)[0], 403)
        self.server.robot_bridge.get_state.assert_not_called()
