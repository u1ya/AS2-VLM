import json
from http.client import HTTPConnection
from http.server import ThreadingHTTPServer
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

from as2_vlm.agent.app import Handler


class ManualApiTests(unittest.TestCase):
    def setUp(self):
        self.manual = SimpleNamespace(start=Mock(return_value={"accepted": True, "lease": "fake"}),
            text=Mock(return_value={"accepted": True, "lease": "fake"}),
            prepare_plan=Mock(return_value={"plan_id":"draft","plan":{"status":"ok","steps":[]}}),
            execute_plan=Mock(return_value={"accepted":True,"lease":"fake"}),
            stop=Mock(return_value={"accepted": True}), heartbeat=Mock(return_value={"ok": True}))
        runtime = SimpleNamespace(manual=self.manual, adapter=SimpleNamespace(read_only=True))
        handler = type('ManualTestHandler', (Handler,), {'runtime': runtime})
        self.runtime = runtime
        self.server = ThreadingHTTPServer(('127.0.0.1', 0), handler)
        self.worker = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.worker.start()
        self.origin = f'http://127.0.0.1:{self.server.server_port}'

    def tearDown(self):
        self.server.shutdown(); self.server.server_close(); self.worker.join(timeout=2)

    def post(self, path, body, origin=None):
        connection = HTTPConnection('127.0.0.1', self.server.server_port, timeout=3)
        connection.request('POST', path, json.dumps(body), {'Content-Type': 'application/json',
            'Origin': origin if origin is not None else self.origin})
        response = connection.getresponse()
        code, data = response.status, json.loads(response.read())
        connection.close()
        return code, data

    def test_manual_start_and_stop_are_separate_from_planner(self):
        self.assertEqual(self.post('/api/manual/start', {'remote_ready': True, 'path_clear': True})[0], 202)
        self.manual.start.assert_called_once()
        self.assertEqual(self.post('/api/manual/stop', {})[0], 202)
        self.manual.stop.assert_called_once()
        self.assertEqual(self.post('/api/command', {'text': 'go forward'})[0], 403)
        self.assertEqual(self.post('/api/override', {'action': 'emergency_sit'})[0], 403)

    def test_remote_origins_do_not_reach_commands(self):
        self.assertEqual(self.post('/api/manual/start', {}, 'https://untrusted.example')[0], 403)
        self.assertEqual(self.post('/api/manual/stop', {}, 'null')[0], 403)
        self.manual.start.assert_not_called(); self.manual.stop.assert_not_called()

    def test_disabled_panel_has_no_manual_channel(self):
        self.runtime.manual = None
        self.assertEqual(self.post('/api/manual/start', {})[0], 403)
        self.assertEqual(self.post('/api/manual/stop', {})[0], 403)

    def test_stop_rejects_extra_parameters(self):
        self.assertEqual(self.post('/api/manual/stop', {'action': 'Damp'})[0], 400)
        self.manual.stop.assert_not_called()

    def test_text_preview_is_non_actuating(self):
        code, data = self.post('/api/manual/parse', {'text': 'go forward 1 meter'})
        self.assertEqual(code, 200)
        self.assertEqual(data['action'], 'walk_meter')
        self.assertTrue(data['requires_confirmation'])
        self.manual.start.assert_not_called(); self.manual.text.assert_not_called(); self.manual.stop.assert_not_called()
        self.assertEqual(self.post('/api/manual/parse', {'text': 'go forward 2 meters'})[0], 400)

    def test_text_endpoint_uses_guarded_manual_channel(self):
        body = {'text': 'go forward 1 meter', 'remote_ready': True, 'path_clear': True}
        self.assertEqual(self.post('/api/manual/text', body)[0], 202)
        self.manual.text.assert_called_once_with(body)
        self.assertEqual(self.post('/api/manual/text', body, 'https://untrusted.example')[0], 403)

    def test_language_planning_is_separate_from_execution(self):
        self.assertEqual(self.post('/api/manual/plan', {'text':'a free instruction'})[0],200)
        self.manual.prepare_plan.assert_called_once_with({'text':'a free instruction'})
        self.manual.execute_plan.assert_not_called();self.manual.start.assert_not_called()
        self.assertEqual(self.post('/api/manual/execute-plan',{'plan_id':'draft','remote_ready':True,'path_clear':True})[0],202)
        self.manual.execute_plan.assert_called_once()
