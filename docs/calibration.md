# Calibration Evidence and Remaining Inputs

## Scope

The offline checker validates the structure of a supplied calibration profile and
reports observations from private recordings. It is not a calibration estimator,
does not apply transforms to live data and never enables motion or navigation.
Unknown values are `null`, not zero or an identity matrix.

Run from the project root in the configured Python environment:

```powershell
python -m as2_vlm.agent.calibration_check recordings/slam-shared-20261025-05
```

The default template is `configs/calibration.example.json`. Use `--profile PATH`
for supplied parameters. The report is written to the ignored recording directory
as `calibration-report.json`. Exit zero means analysis completed, not calibration
success. Even a numerically complete profile does not prove it matches the robot:
`calibration_verified`, `slam_ready` and `navigation_ready` stay false.

## What Was Actually Checked: 2026-10-25

- Local SDK sources expose messages and interfaces but no verified transform for
  this LiDAR installation was found.
- The official `unitreerobotics/unitree_ros` AS2 description was inspected at commit
  `15ebe53f4cd77e7d745ae83b9d7ff90e685e8998`, using GitHub API and XML parsing.
  [AS2 URDF](https://github.com/unitreerobotics/unitree_ros/blob/15ebe53f4cd77e7d745ae83b9d7ff90e685e8998/robots/as2_description/urdf/as2.urdf)
  has world/base and leg links, but no LiDAR or IMU link/joint supplying the needed
  sensor-to-sensor transform. Its Git blob SHA is
  `ab96a46046fea7b5bd7f027bbd458efabb1121aa`.
- The accompanying
  [AS2 MJCF](https://github.com/unitreerobotics/unitree_ros/blob/15ebe53f4cd77e7d745ae83b9d7ff90e685e8998/robots/as2_description/as2.xml)
  contains a simulation IMU site at zero local offset. That is not factory
  LiDAR-IMU calibration for the physical EDU hardware, and was not imported.
- TCP port 22 on the known robot address `192.168.123.161` did not connect within
  two seconds. No login, password attempts, address sweep or onboard changes were
  made. This does not prove there is no separately addressed onboard computer.
- The owner has neither onboard access nor factory calibration/packing files.

## Recorded Observation, Not an Estimated Transform

For `slam-shared-20261025-05`, mean accelerometer readings are approximately
`[-0.8073, 9.5214, -1.2021]` in unverified units. Their normalized direction is
`[-0.08383, 0.98863, -0.12482]`, dominated by +Y. The reported body odometry is
consistent with remaining stationary. LiDAR and IMU messages both name
`hesai_lidar`; identical labels alone do not establish physical extrinsics or
that the publisher has already transformed the measurements.

A stationary specific-force observation cannot determine yaw, relative sensor
translation or a clock offset. It also does not prove the LiDAR axes are the same
as the IMU-message axes. No 90-degree rotation was invented from the +Y finding.
There is no measured LiDAR-to-body or camera transform in this stage.

## Profile Contract

The transform convention is explicit:

```text
p_imu = rotation @ p_lidar + translation_m
t_imu = t_lidar + imu_time_minus_lidar_time_s
```

This is a requested convention for supplied evidence, not a statement about the
current DDS driver. A driver may already have transformed data; that must be
verified before applying anything again.

The profile requires exact sensor model, hardware binding/evidence, coordinate
units, per-point timing units/reference, explicit clock offset, and transform
provenance. Rotation must be a finite 3x3 proper orthonormal matrix with determinant
+1; translation must be three finite numeric metre values. Reflections, scaling,
bad shapes, strings/booleans and nonfinite coefficients are rejected. Numerical
validity alone does not establish calibration accuracy or the transform direction
of an external supplier file.

## Routes Forward Without Factory Files

1. A full LiDAR-inertial calibration requires independently documented transforms
   or a separately planned, authorized dynamic calibration dataset with a proven
   estimator and adequate motion excitation. No calibration motion is authorized
   or performed by this checker. Do not disable safety limits to collect it.
2. For progress toward a map without that transform, evaluate a LiDAR-only
   odometry engine offline, such as
   [KISS-ICP](https://github.com/PRBonn/kiss-icp), which is an existing LiDAR odometry
   implementation. This avoids the particular LiDAR-to-IMU dependency, not the
   need to verify length units and validate registration on moving recordings.
   Odometry is not loop-closure SLAM, a semantic map or safe navigation.

No new odometry engine was installed in this calibration-evidence stage. Existing
recordings are stationary and cannot validate moving-map accuracy. Current shared
panels keep their uncalibrated sensor-frame projection and read-only transport.

Verification: all 140 Python regression tests passed, including seven new
calibration-profile/evidence tests. The checker completed on the 87-cloud private
recording and kept calibration, SLAM and navigation readiness false.
