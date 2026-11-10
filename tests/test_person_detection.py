import base64
import io
from pathlib import Path
import threading
import time
from types import SimpleNamespace
import unittest

import numpy as np
from PIL import Image

from as2_vlm.agent.person_detection import PersonDetector, PersonStream, decode_people
from as2_vlm.agent.models import Detection


def frame(sequence):
    image=io.BytesIO();Image.new('RGB',(40,20),'red' if sequence==1 else 'blue').save(image,'JPEG')
    return SimpleNamespace(frame_id=sequence,timestamp=time.time(),source='test',jpeg=image.getvalue(),width=40,height=20)


class PersonDetectionTests(unittest.TestCase):
    def output(self):
        raw=np.zeros((1,8400,85),np.float32)
        index=22*80+40
        raw[0,index,:4]=[0,.5,np.log(40),np.log(22.5)]
        raw[0,index,4:6]=[.9,.9]
        return raw,index

    def test_letterbox_decode_keeps_rectangular_image_coordinates(self):
        raw,_=self.output()
        people=decode_people(raw,(768,432))
        self.assertEqual(len(people),1)
        np.testing.assert_allclose(people[0].bbox,[.25,.25,.75,.75],atol=1e-6)
        self.assertIsNone(people[0].depth_m)

    def test_non_person_or_padding_only_box_never_becomes_human(self):
        raw,index=self.output();raw[0,index,6]=.99
        self.assertEqual(decode_people(raw,(768,432)),[])
        raw,index=self.output();raw[0,index,1]=80
        self.assertEqual(decode_people(raw,(768,432)),[])

    def test_duplicate_boxes_are_suppressed_and_invalid_layout_is_visible(self):
        raw,index=self.output()
        raw[0,index+1]=raw[0,index];raw[0,index+1,0]-=1
        self.assertEqual(len(decode_people(raw,(768,432))),1)
        with self.assertRaises(ValueError):decode_people(np.zeros((1,80)),(768,432))

    def test_worker_updates_without_qwen_and_keeps_boxes_with_exact_source_image(self):
        source=SimpleNamespace(get_frame=lambda:frame(1))
        detector=SimpleNamespace(detect_objects=lambda _: [Detection(label='person',category='person',bbox=(.1,.1,.5,.9),confidence=.8)])
        worker=PersonStream(source,detector,lambda f:f)
        worker.start()
        try:
            deadline=time.monotonic()+3
            while worker.observation is None and time.monotonic()<deadline:time.sleep(.01)
            source.get_frame=lambda:frame(2)
            while worker.observation['frame_id']!=2 and time.monotonic()<deadline:time.sleep(.01)
            result=worker.payload();o=result['observation']
            self.assertEqual(o['frame_id'],2)
            with Image.open(io.BytesIO(base64.b64decode(o['image_jpeg_base64']))) as image:
                self.assertGreater(image.getpixel((0,0))[2],200)
            self.assertTrue(o['current'])
            self.assertFalse(o['metric_positions'])
            self.assertFalse(result['motion_enabled'])
        finally:worker.close()

    def test_replay_or_old_image_cannot_be_claimed_live(self):
        worker=PersonStream(SimpleNamespace(replay=True),None,None)
        worker.observation={'received_at':time.time(),'objects':[]}
        self.assertFalse(worker.payload()['observation']['current'])
        worker.source.replay=False
        worker.observation['received_at']-=4
        self.assertFalse(worker.payload()['observation']['current'])


if __name__=='__main__':unittest.main()
