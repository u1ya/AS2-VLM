# AS2 Camera: Read-Only Verification

Verified on the owner's AS2 EDU on 2026-10-25. No motion, pose changes,
camera reconfiguration, process termination on the robot or login attempts were used.

## Connection

Robot: 192.168.123.161, laptop: 192.168.123.100/24. Windows Ethernet link is up.
WSL Ubuntu-22.04 mirrored now exposes the robot link as **eth0**, not the old eth1.
Always check `ip route get 192.168.123.161` before choosing an interface.
Both existing peer-scoped Windows/Hyper-V UDP firewall permissions remain enabled.
Ping: 3/3 replies, approximately 1.2-1.9 ms. GetState returned 0,
NORMAL_WALK / process_state=0. Read-only odometry preflight received 60 samples
and passed. No StopMove or Move was requested.

## Confirmed Image Interface

The pinned SDK contains no AS2-specific camera header. Its Go2 VideoClient
successfully retrieves images from this AS2 through service `videohub`,
API version 1.0.0.0, GetImageSample API 1001. This is empirical compatibility
on this unit, not a guarantee for all AS2 firmware variants.

- Four requests returned sdk_return_code=0, each with about 133 kB of JPEG data.
- All four decoded as RGB, 1920 x 1080, with distinct hashes and pixel content.
- Measured request durations: 48.1, 40.9, 19.0 and 19.1 ms, excluding discovery,
  file output and decoding. These are NOT measured FPS or end-to-end camera latency.
- Each process waits 2 seconds for discovery; this diagnostic is not a live viewer.
- Source capture timestamps and frame counters are not provided by this helper.
  Changed images support updating content, but do not prove capture age or synchronization.
- The first frame shows an indoor room with boxes and a white wall. A nearby object
  obscures part of the right side; the operator must inspect the obstruction.
- The B2 `front_videohub` alternative returned 3104 with no bytes. This is a failed
  request, not proof of absent camera hardware or a known firmware fault.
- A subsequent two-camera check queried all three available SDK image clients:
  `videohub` returned a new JPEG (177219 bytes, about 22.7 ms); both
  `front_videohub` and `back_videohub` returned 3104 with no bytes. The new image
  contains a hand close to the lens, unlike the earlier frames. Only one accessible
  RGB service is confirmed; the physical number of cameras remains unknown.
  A second sensor could use another interface or be part of a stereo/depth module.
  External photos of the sensor assembly or onboard device inventory are needed
  to distinguish these possibilities. The helper now accepts `back_videohub` too.
- TCP probes on the known .161 board at 22/80/443/554/8554 did not connect within
  0.8 seconds. This does not establish whether another onboard computer exposes them.

## Build and Capture

The existing SDK build remains pinned to d1abf001220d3e5f59e00a0c4aaa595e00089a4a.
Reconfigure CMake after pulling this change, or use `bash deploy/build_state_probe.sh`.
From the project root inside Ubuntu:

```bash
cmake -S robot/bridge_cpp -B build/as2-probe \
  -DCMAKE_PREFIX_PATH="$PWD/.runtime/unitree-sdk-install"
cmake --build build/as2-probe --target as2_camera_probe --parallel 2
./build/as2-probe/as2_camera_probe --help
mkdir -p logs
timeout 10 ./build/as2-probe/as2_camera_probe eth0 videohub logs/as2-camera.jpg
```

Choose a new output name for each invocation. Existing files are rejected before
SDK initialization. Only one GetImageSample request is made per invocation, with
a bounded outer process timeout. Responses must have a JPEG signature and be at
most 10 MiB before being saved; use a real decoder to verify saved image content.
No continuous capture starts, and the utility does not enable application motion.
The help and invalid-service CTest cases do not contact the robot.

Private session frames are in ignored `logs/camera-20261025-*.jpg` locally.
They are not committed or uploaded. Do not publish indoor camera data by default.

## VLM Readiness

The existing `as2_vlm.vlm.read_image` successfully decoded a real camera JPEG,
converted it to RGB and resized it to 768 x 432 for the model. This verifies input
preparation only: no new inference or detection-accuracy test was performed.
The model is still `qwen2.5vl:3b` served through local Ollama when started.

To run an explicitly requested preview later, start the local model as described
in RESUME.md, capture a fresh image, then pass it using the existing `--image`
option. VLM boxes are unvalidated proposals; they must not directly generate Move.

## Required Before Physical Visual Navigation

1. Inspect and remove the right-side obstruction without touching an active robot's legs.
2. Add a persistent read-only camera worker, measure sustained request rate, failures,
   frame freshness, decode time and inference time. Keep only the latest frame;
   bound queues, use a monotonic receive timestamp and reject stale observations.
3. Obtain camera model, intrinsics/distortion and camera-to-body transform from
   the vendor or a calibration procedure. None have been retrieved yet.
4. Identify whether depth is actually available on this configuration, or associate
   RGB with calibrated LiDAR. A JPEG and bounding box do not supply metric distance.
5. Measure RGB/odometry/LiDAR timing and coordinate conventions before fusion.
6. Validate wall/door/person perception on this camera, including ambiguity,
   occlusion and stale/missing frames. Add operator-reviewed failure cases.
7. Connect goals to a separate obstacle-aware planner and safety-supervised
   controller. Keep camera/VLM failure from producing continued blind movement.

ROS, OpenCV and SSH were not needed for the verified JPEG capture. Do not install
or stop onboard services speculatively; first identify the actual sensor interface.

Reference implementation: the local pinned SDK headers
`include/unitree/robot/go2/video/video_client.hpp`, `video_api.hpp` and
`example/go2/go2_video_client.cpp` in the official
[unitree_sdk2 repository](https://github.com/unitreerobotics/unitree_sdk2).
