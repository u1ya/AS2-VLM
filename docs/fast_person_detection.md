# Independent local person detector

The live mini-map now has a separate YOLOX person stream beside Qwen's existing
person/bin observations. It uses the same shared camera feed and has no actuator,
velocity, map-goal or StopMove interface. LiDAR acquisition and the physical movement
gate stay separate. No camera/LiDAR extrinsics are invented to show a person.

## Model and installation

The model is [OpenCV Zoo YOLOX-S int8](https://github.com/opencv/opencv_zoo/tree/main/models/object_detection_yolox),
Apache-2.0, pinned to OpenCV Zoo commit
`48b10fa505aeb51fbd127fc60a5c660040e823b7`. File size is 9,079,452 bytes; SHA-256:

```text
01a3b0f400b30bc1e45230e991b2e499ab42622485a330021947333fbaf03935
```

Weights, license and provenance are kept in ignored `.models/yolox/`. To restore:

```powershell
./deploy/install_person_detector.ps1
```

The installer downloads only the pinned model and license; the Python detector
checks the known model digest before loading it. It does not download at runtime.
OpenCV 4.12.0.88 already exists in `.runtime/calibration-python`, separate from the
default bundled interpreter. The mini-map launcher automatically selects this
environment when the known local weights are present. `./minimap.ps1 -NoFastPerson`
keeps the older Qwen-only profile. An explicit `AS2_VLM_PYTHON` remains authoritative
and must include OpenCV if the fast detector is enabled.

## Pipeline

The dedicated worker processes the latest unseen frame without waiting for Qwen.
It applies the model's RGB, 640×640 top-left letterbox convention and decodes its
grid/stride outputs, then maps corners back to the original rectangular frame.
Only winning COCO person-class predictions above 0.5 are returned; duplicate boxes
are suppressed. There is no trash-bin class on this path. Qwen's original stream
continues to display bin hypotheses on its own analyzed frame.

Every fast result stores the exact analyzed image and frame ID, receive age,
inference duration and backend. Its boxes are displayed on that image, never on a
newer live image. Old and replay observations cannot be labelled current.
`metric_positions=false`, `grounding_verified=false`, `depth_m=null` stay explicit.
No cloud is fetched after inference and presented as if it belonged to that image.

GET `/api/scene` adds `fast_person` with status, error and observation. The original
recognition output remains in `recognition`. The live camera sampling is still
approximately 1 Hz; faster inference does not establish a higher camera frame rate
or timely collision protection.

## Verification on this machine

Three actual 1920×1080 AS2 saved frames ran at 0.28–0.31 seconds per image with four
OpenCV CPU threads. The detector returned a person on the standing/cropped image
and crouching image with boxes consistent with manual visual inspection. A third
image showing only a largely occluded person/foot returned no person. This is a
small smoke test, not an accuracy or absence benchmark.

The live endpoint returned a current frame with approximately 0.30-second inference
and no detector error. Unit checks cover rectangular letterbox inversion, class
filtering, padded-area rejection, duplicate suppression, independent frame updates,
image/box identity and stale/replay marking. Existing mini-map tests still pass.

Camera/LiDAR board matching remains unverified. The actual 22-cloud patch audit
found no candidate passing its held-out correspondence checks; its best hypothesis
matched only 1/20 training views and 0/2 holdouts under the strict criteria, and was
not installed. The owner currently has no larger matte target. The 42-view camera
calibration and all paired data remain saved for future geometric work.
