"""Actual replay decoding, bounded inference and image/box identity regressions."""
import base64
import io
import json
from http.client import HTTPConnection
from http.server import ThreadingHTTPServer
import threading
import time
import unittest
from types import SimpleNamespace

from PIL import Image

from as2_vlm.agent.minimap import Handler, MiniMap, RecordedSensors, small_frame, inference_frame, target_kind
from as2_vlm.agent.models import Detection, Pose
from as2_vlm.agent.robot_adapter import CameraFrame


def frame(sequence=1):
    output = io.BytesIO()
    Image.new('RGB', (1920, 1080), 'red' if sequence == 1 else 'blue').save(output, 'JPEG')
    return CameraFrame(sequence, time.time(), 1920, 1080, 0, Pose(), 0, [], jpeg=output.getvalue())


class MiniMapTests(unittest.TestCase):
    def test_person_bin_focus_does_not_relabel_unrelated_objects(self):
        for label, category, expected in (("man", "person", "person"), ("trash_bin", "other", "trash_bin"),
                                          ("waste bin", "other", "trash_bin"), ("chair", "furniture", None),
                                          ("cabinet", "furniture", None), ("box", "obstacle", None)):
            self.assertEqual(target_kind(Detection(label=label, category=category, bbox=(0,0,1,1), confidence=.9)), expected)

    def test_cloud_is_captured_before_slow_model_and_bound_to_original_frame(self):
        events = []
        class Pair:
            def status(self):
                return {"frame_id": 1, "cloud_sequence": 42}
        class Source:
            def get_frame(self):
                return frame()
            def capture_bundle(self, supplied):
                events.append(("cloud", supplied.frame_id))
                return Pair()
        class Perception:
            def detect_objects(self, supplied):
                events.append(("inference", supplied.frame_id))
                return [Detection(label='person', category='person', bbox=(.1,.2,.5,.7), confidence=.8),
                        Detection(label='trash_bin', category='other', bbox=(.6,.6,.7,.8), confidence=.7),
                        Detection(label='cabinet', category='furniture', bbox=(0,0,1,1), confidence=.9)]
        scene = MiniMap(Source(), Perception(), interval=100, targets_only=True)
        worker = threading.Thread(target=scene._recognize)
        worker.start()
        try:
            deadline = time.monotonic()+3
            while scene.observation is None and time.monotonic()<deadline:
                time.sleep(.01)
            observation = scene.observation
            self.assertIsNotNone(observation, scene.error)
            self.assertEqual(events, [("cloud", 1), ("inference", 1)])
            self.assertEqual(observation["pair"]["cloud_sequence"], 42)
            self.assertEqual([d["target_kind"] for d in observation["objects"]], ["person", "trash_bin"])
            self.assertEqual(observation["fusion"]["status"], "calibration_required")
        finally:
            scene.stop.set(); worker.join(3)

    def test_image_resize_preserves_aspect_and_frame_identity(self):
        source = frame()
        prepared = small_frame(source)
        self.assertEqual((prepared.width, prepared.height), (768, 432))
        self.assertEqual(prepared.frame_id, source.frame_id)
        self.assertEqual(source.width, 1920)
        squared = inference_frame(prepared)
        self.assertEqual((squared.width, squared.height), (768, 768))
        self.assertEqual(squared.frame_id, source.frame_id)
        self.assertEqual(prepared.height, 432)

    def test_boxes_remain_bound_to_analyzed_frame_when_camera_advances(self):
        initial, advanced = frame(), frame(2)
        started, release = threading.Event(), threading.Event()
        class Perception:
            def detect_objects(self, supplied):
                started.set()
                release.wait(3)
                return [Detection(label='box', category='obstacle', bbox=(.1,.2,.5,.7), confidence=.8),
                        Detection(label='floor', category='obstacle', bbox=(0,0,1,1), confidence=1),
                        Detection(label='uncertain', category='other', bbox=(.1,.2,.5,.7), confidence=.1)]
        source = SimpleNamespace(get_frame=lambda: initial)
        scene = MiniMap(source, Perception(), interval=100)
        worker = threading.Thread(target=scene._recognize)
        worker.start()
        try:
            self.assertTrue(started.wait(2))
            source.get_frame = lambda: advanced
            release.set()
            deadline = time.monotonic()+3
            while scene.observation is None and time.monotonic()<deadline:
                time.sleep(.01)
            self.assertEqual(scene.observation['frame_id'], 1)
            self.assertEqual(len(scene.observation['objects']), 1)
            self.assertEqual(scene.observation['rejected_background_labels'], ['floor'])
            with Image.open(io.BytesIO(base64.b64decode(scene.observation['image_jpeg_base64']))) as im:
                self.assertGreater(im.getpixel((0,0))[0], 200)
            self.assertFalse(scene.observation['metric_positions'])
        finally:
            release.set(); scene.stop.set(); worker.join(3)

    def test_model_failure_is_visible_and_recovers_without_queue(self):
        supplied = frame()
        class Perception:
            calls = 0
            def detect_objects(self, _):
                self.calls += 1
                if self.calls == 1:
                    raise ValueError('bad model response')
                return []
        perception = Perception()
        scene = MiniMap(SimpleNamespace(get_frame=lambda: supplied), perception, interval=.05)
        worker = threading.Thread(target=scene._recognize)
        worker.start()
        try:
            deadline=time.monotonic()+3
            while scene.observation is None and time.monotonic()<deadline:
                time.sleep(.01)
            self.assertIsNotNone(scene.observation)
            self.assertEqual(scene.status, 'ready')
            self.assertIsNone(scene.error)
            self.assertEqual(perception.calls, 2)
        finally:
            scene.stop.set(); worker.join(3)

    def test_private_as2_replay_uses_real_clouds_and_never_claims_live(self):
        from pathlib import Path
        root = Path(__file__).resolve().parents[1]
        recording = root/'recordings/slam-shared-cli-20261025-06'
        photo = root/'logs/camera-20261025-videohub.jpg'
        if not recording.exists() or not photo.exists():
            self.skipTest('Private AS2 recordings unavailable')
        source = RecordedSensors(recording, photo)
        source.start()
        try:
            source.thread.join(15)
            self.assertTrue(source.complete, source.error)
            self.assertEqual(source.index, len(source.recording.clouds))
            scene = MiniMap(source, None)
            payload = scene.payload()
            self.assertGreater(payload['geometry']['return_cells'], 0)
            self.assertFalse(payload['geometry']['connected'])
            self.assertFalse(payload['motion_enabled'])
            self.assertFalse(payload['camera_lidar_synchronized'])
            self.assertEqual(payload['mode'], 'recorded_as2')
        finally:
            source.close()

    def test_api_is_read_only_and_rejects_foreign_host(self):
        scene = SimpleNamespace(payload=lambda: {'motion_enabled': False})
        handler = type('TestHandler', (Handler,), {'scene': scene})
        server = ThreadingHTTPServer(('127.0.0.1', 0), handler)
        worker = threading.Thread(target=server.serve_forever, daemon=True)
        worker.start()
        def request(method, path, headers=None):
            connection = HTTPConnection('127.0.0.1', server.server_port, timeout=3)
            try:
                connection.request(method, path, headers=headers or {})
                result=connection.getresponse()
                return result.status,result.read()
            finally:
                connection.close()
        try:
            code, raw=request('GET','/api/scene')
            self.assertEqual(code,200)
            self.assertFalse(json.loads(raw)['motion_enabled'])
            self.assertEqual(request('POST','/api/move')[0],501)
            self.assertEqual(request('GET','/api/scene',{'Host':'evil.example'})[0],403)
            self.assertEqual(request('GET','/')[0],200)
        finally:
            server.shutdown(); server.server_close(); worker.join(3)


if __name__ == '__main__':
    unittest.main()
