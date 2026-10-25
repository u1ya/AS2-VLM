# AS2 LiDAR: Read-Only Verification

Update: real XYZ projection is now available in the live dashboard. See
[Local Geometry](local_geometry.md) for the stationary window, filters and private
snapshot export. This adds a sensor-frame density view, not global SLAM.

Verified on the owner's AS2 EDU on 2026-10-25 over Ethernet, WSL2 Ubuntu-22.04
mirrored, interface eth0, laptop 192.168.123.100/24, robot 192.168.123.161.
Existing peer-scoped firewall permissions were unchanged. No movement, StopMove,
sensor switch, calibration, SLAM commands or onboard service changes were requested.

## Discovery and Received Data

A DDS domain 0 publisher-announcement reader found 89 distinct topic/type pairs.
Relevant advertised topics:

| Topic | DDS type |
| --- | --- |
| rt/utlidar/cloud_livox_mid360 | sensor_msgs::msg::dds_::PointCloud2_ |
| rt/utlidar/imu_livox_mid360 | sensor_msgs::msg::dds_::Imu_ |
| rt/utlidar/lidar_state | unitree_go::msg::dds_::LidarState_ |
| rt/utlidar/voxel_map_compressed | unitree_go::msg::dds_::VoxelMapCompressed_ |

The voxel map was discovered, not read or decoded. Publisher announcements do
not themselves prove data delivery. Cloud, IMU and lidar-state subscriptions did:

```text
Observation window: 8 seconds
cloud_messages=78
state_messages=8
imu_messages=1130
observed_cloud_hz=9.71372
last_cloud_age_s=0.00852349
firmware=1.b.0028
software=15.AF.B0.00.02.L
sdk=0
error_state=0
dirty_percentage=0
reported_cloud_hz=10.5461
reported_cloud_loss=0
reported_cloud_size=81567
frame=hesai_lidar
stamp=1279:400226000
width=81633 height=1 point_step=26 row_step=2122458
bytes=2122458 bigendian=0
finite_xyz=81633 nonzero_xyz=81633
range_raw_min=0.024 range_raw_max=31.808
```

Latest cloud fields: x/y/z/intensity float32 at offsets 0/4/8/12,
ring uint16 at offset 16, timestamp float64 at offset 18. The parser respects
row padding, endianness and field offsets; it does not cast packed records to
an aligned C++ struct. All XYZ values in the final received cloud were finite
and nonzero. Radial range statistics are raw coordinate norms, NOT validated
free-space clearances; units, self-returns and extrinsics need independent checks.

**Do not identify the device from the topic name.** The cloud frame is
`hesai_lidar`, while the topic retains a Livox name. This suggests a reused
interface for a different sensor, but neither the exact model nor beam count
has been confirmed. Request the device label/configuration from the supplier.
The SDK state's zero error and loss fields are reported values, not independent
hardware certification or a complete network-loss measurement.

The reported firmware/software belong to the lidar-state message; they do not
identify the robot motion-controller firmware. IMU messages were counted but
their numerical contents were not independently validated.

## Repeat Without Motion

From the project root in Ubuntu after installing the pinned SDK:

```bash
ip route get 192.168.123.161
cmake -S robot/bridge_cpp -B build/as2-probe \
  -DCMAKE_PREFIX_PATH="$PWD/.runtime/unitree-sdk-install"
cmake --build build/as2-probe --target as2_dds_discover as2_lidar_probe --parallel 2
timeout 15 ./build/as2-probe/as2_dds_discover eth0
timeout 15 ./build/as2-probe/as2_lidar_probe eth0
ctest --test-dir build/as2-probe --output-on-failure
```

Use the current route's interface, not a historical name. The discovery helper
only reads built-in publisher announcements for 8 seconds. The lidar helper
only subscribes to the three confirmed topics for 8 seconds, keeps the latest
cloud in memory and prints metadata/range statistics. Raw point clouds are not
written or published. Help tests do not connect to the robot.

No ROS installation, direct sensor IP, new driver, SSH or sensor-enable command
was needed for this check. Connection is confirmed; mapping/navigation is not
implemented or validated by this test.

## Before Mapping or Obstacle Avoidance

1. Verify sensor model, coordinate units/axes and LiDAR-to-body extrinsics.
2. Check per-point time units, source clocks and synchronization with odometry,
   IMU and camera; this stamp is not established as UTC or laptop time.
3. Inspect a point-cloud visualization, remove body/self returns, isolate ground
   and obstacles, and record private time-synchronized datasets for replay.
4. Use bounded latest-frame processing: the observed stream is approximately
   2.1 MB per cloud at 10 Hz before transport overhead. Never queue indefinitely.
5. Validate an existing LiDAR SLAM/localization pipeline and then use a static map
   plus live local obstacle updates. Do not replace live obstacle sensing with
   an old map, and do not send the complete cloud to the VLM on every frame.

Generated DDS message definitions come from the locally pinned official
[unitree_sdk2](https://github.com/unitreerobotics/unitree_sdk2) checkout;
topic names and measurements above were discovered on this robot, not assumed
from examples for another model.
