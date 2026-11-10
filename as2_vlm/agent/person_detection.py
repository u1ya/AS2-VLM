"""Local YOLOX person observations; no motion or emergency-stop responsibility.

Uses OpenCV Zoo's Apache-2.0 YOLOX-S int8 ONNX model and its published RGB,
top-left letterbox and grid/stride conventions. Model provenance stays local.
Reference: https://github.com/opencv/opencv_zoo/tree/main/models/object_detection_yolox
"""
import base64
import copy
import hashlib
from pathlib import Path
import threading
import time

import numpy as np

from .models import Detection

MODEL_SHA256 = "01a3b0f400b30bc1e45230e991b2e499ab42622485a330021947333fbaf03935"


def decode_people(output, image_size, threshold=.5):
    """Decode this fixed model while retaining original rectangular image axes."""
    if not 0 < threshold < 1:
        raise ValueError("Invalid confidence threshold")
    width,height = image_size
    if min(width,height) <= 0:
        raise ValueError("Invalid image dimensions")
    raw = np.asarray(output)
    if raw.shape != (1,8400,85):
        raise ValueError("Unexpected YOLOX output layout")
    rows = raw[0]
    # COCO class 0 is person; another winning class must not become a person.
    scores = rows[:,4]*rows[:,5]
    selected = (np.isfinite(rows).all(axis=1) & (scores >= threshold) & (scores <= 1) &
                (np.argmax(rows[:,5:],axis=1) == 0) & np.all(np.abs(rows[:,2:4]) < 10,axis=1))
    indices = np.flatnonzero(selected)
    if not len(indices):
        return []
    grids,strides = [],[]
    for stride in (8,16,32):
        y,x = np.mgrid[:640//stride,:640//stride]
        grids.append(np.column_stack([x.ravel(),y.ravel()]))
        strides.append(np.full((x.size,1),stride))
    grid = np.concatenate(grids)[indices]
    stride = np.concatenate(strides)[indices]
    center = (rows[indices,:2]+grid)*stride
    size = np.exp(rows[indices,2:4])*stride
    scale = min(640/width,640/height)
    boxes = np.column_stack([center-size/2,center+size/2])/scale
    boxes[:,[0,2]] = np.clip(boxes[:,[0,2]],0,width)
    boxes[:,[1,3]] = np.clip(boxes[:,[1,3]],0,height)
    valid = (boxes[:,2] > boxes[:,0]) & (boxes[:,3] > boxes[:,1])
    boxes,score = boxes[valid],scores[indices][valid]
    order = np.argsort(-score)
    kept = []
    while len(order):
        index = int(order[0]);kept.append(index)
        others = order[1:]
        if not len(others):break
        lo = np.maximum(boxes[index,:2],boxes[others,:2])
        hi = np.minimum(boxes[index,2:],boxes[others,2:])
        intersection = np.prod(np.maximum(0,hi-lo),axis=1)
        area = np.prod(boxes[index,2:]-boxes[index,:2])
        other_area = np.prod(boxes[others,2:]-boxes[others,:2],axis=1)
        iou = intersection/(area+other_area-intersection)
        order = others[iou < .5]
    return [Detection(label="person",category="person",confidence=float(score[i]),
                      bbox=tuple((boxes[i]/[width,height,width,height]).tolist())) for i in kept[:20]]


class PersonDetector:
    square_input = False
    model_input = "rgb_letterbox_640"

    def __init__(self, model_path):
        import cv2
        cv2.setNumThreads(4)
        path = Path(model_path)
        if not path.is_file() or not 1000 < path.stat().st_size < 64*1024*1024:
            raise ValueError("Local YOLOX model file missing or invalid")
        if hashlib.sha256(path.read_bytes()).hexdigest() != MODEL_SHA256:
            raise ValueError("Person model checksum differs from the pinned OpenCV Zoo model")
        self.net = cv2.dnn.readNetFromONNX(str(path))
        self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
        self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)

    def detect_objects(self, frame):
        import cv2
        image = cv2.imdecode(np.frombuffer(frame.jpeg,np.uint8),cv2.IMREAD_COLOR)
        if image is None or image.shape[:2] != (frame.height,frame.width):
            raise ValueError("Image bytes and frame dimensions disagree")
        ratio = min(640/frame.width,640/frame.height)
        width,height = int(frame.width*ratio),int(frame.height*ratio)
        rgb = cv2.cvtColor(image,cv2.COLOR_BGR2RGB)
        padded = np.full((640,640,3),114,dtype=np.float32)
        padded[:height,:width] = cv2.resize(rgb,(width,height),interpolation=cv2.INTER_LINEAR)
        self.net.setInput(np.ascontiguousarray(padded.transpose(2,0,1)[None]))
        output = self.net.forward()
        return decode_people(output,(frame.width,frame.height))


class PersonStream:
    """Independent latest-frame worker; never waits for Qwen's slower output."""
    def __init__(self, source, detector, prepare):
        self.source,self.detector,self.prepare = source,detector,prepare
        self.lock = threading.Lock()
        self.stop = threading.Event()
        self.thread = None
        self.observation = None
        self.error = None
        self.status = "waiting_camera"

    def start(self):
        self.thread = threading.Thread(target=self._run,daemon=True,name="as2-fast-person")
        self.thread.start()

    def _run(self):
        last = None
        while not self.stop.is_set():
            frame = self.source.get_frame()
            if frame is None or frame.frame_id == last:
                self.stop.wait(.05);continue
            try:
                with self.lock:self.status="recognizing"
                started = time.monotonic()
                prepared = self.prepare(frame)
                detections = self.detector.detect_objects(prepared)
                observation = {"frame_id":frame.frame_id,"received_at":frame.timestamp,
                    "source":frame.source,"backend":"yolox_s_int8","inference_s":time.monotonic()-started,
                    "image_jpeg_base64":base64.b64encode(prepared.jpeg).decode("ascii"),
                    "objects":[{**d.model_dump(),"target_kind":"person"} for d in detections],
                    "metric_positions":False,"grounding_verified":False,"coordinates":"normalized_image_only"}
                # No cloud is paired after inference; a later cloud must not be
                # used as if it described this observation. Metric fusion is pending.
                with self.lock:
                    self.observation=observation;self.status="ready";self.error=None
                last=frame.frame_id
            except Exception as exc:
                with self.lock:self.status="error";self.error=str(exc)
                self.stop.wait(.5)

    def payload(self):
        with self.lock:
            observation=copy.deepcopy(self.observation);status,error=self.status,self.error
        if observation:
            observation["age_s"]=max(0,time.time()-observation["received_at"])
            observation["current"]=not bool(getattr(self.source,"replay",False)) and observation["age_s"]<=3
        return {"status":status,"error":error,"observation":observation,"motion_enabled":False}

    def close(self):
        self.stop.set()
        if self.thread:self.thread.join(3)
