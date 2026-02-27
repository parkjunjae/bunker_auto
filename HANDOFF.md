# HANDOFF (2026-02-27)

This file is for resuming this project from another PC with minimal context loss.

## 1) Workspace / Repo
- Workspace: `/home/atoz/ca_ws`
- Main branch: `main`
- Remote: `origin https://github.com/parkjunjae/bunker_auto.git`
- Current sync status (at handoff time): local == origin/main

## 2) Fast Resume (new PC)
```bash
cd ~/ca_ws
git pull origin main
source /opt/ros/humble/setup.bash
source ~/ca_ws/install/setup.bash
```

If packages changed, rebuild once:
```bash
cd ~/ca_ws
colcon build --symlink-install
source ~/ca_ws/install/setup.bash
```

## 3) Main Run Command
```bash
bash ~/ca_ws/run_all.sh
```

Current run_all stack includes:
- Livox driver
- RealSense
- EC25 GPS bridge (`/gps/fix`)
- bunker_base
- sensor_sync (imu correction + madgwick + livox offset + deskew + static TF)
- EKF
- RTAB-Map + Nav2
- agent_pid

## 4) Important Recent Changes

### A) EC25 GPS bridge added
- New package: `src/ec25_gps_bridge`
- Publishes `sensor_msgs/NavSatFix` to `/gps/fix`
- Launch file: `src/ec25_gps_bridge/launch/ec25_gps_bridge.launch.py`
- Static TF (`base_link -> gps_link`) is included in launch

Standalone launch example:
```bash
ros2 launch ec25_gps_bridge ec25_gps_bridge.launch.py \
  device:=/dev/ttyUSB3 \
  topic_name:=/gps/fix \
  frame_id:=gps_link \
  parent_frame:=base_link \
  publish_static_tf:=true \
  gps_x:=0.3 gps_y:=0.0 gps_z:=0.35
```

### B) Narrow corridor in-place rotation guard
- `rl_local_controller` updated to block in-place rotate when clearance is low
- Escape behavior added (forward-first, optional reverse)
- Params in Nav2 config:
  - `rotate_min_side_clearance: 0.42`
  - `rotate_min_front_clearance: 0.32`
  - `escape_forward_speed: 0.05`
  - `escape_forward_turn_scale: 0.6`
  - `escape_use_reverse: true`
  - `escape_reverse_speed: 0.03`

### C) Smooth control tuning
- Agent PID launch tuned (`src/rl_pid_training/launch/agent_pid.launch.py`):
  - `step_dt=0.6`
  - `gain_scale=0.08`
  - `gain_lpf_alpha=0.08`
  - `action_deadzone=0.35`
  - `w_ref_lpf_alpha=0.10`
  - `w_ref_deadband=0.08`
  - `dither_w_ref_thresh=0.20`
- Nav2 velocity smoother tuned (`nav2_rtabmap_params.yaml`):
  - `smoothing_frequency: 30.0`
  - `feedback: OPEN_LOOP`
  - `max_accel: [0.25, 0.0, 0.8]`
  - `max_decel: [-0.25, 0.0, -0.8]`
  - `deadband_velocity: [0.0, 0.0, 0.02]`

### D) Livox time sync flow
- `sensor_sync.launch.py` offset node:
  - `/livox/lidar` -> `/livox/lidar/synced`
  - `/livox/lidar/synced` -> deskew -> `/livox/lidar/synced/deskewed`
- Current default offset: `lidar_offset_sec=0.003`

Check delay quickly:
```bash
ros2 topic delay /livox/lidar
ros2 topic delay /livox/lidar/synced
ros2 topic delay /livox/lidar/synced/deskewed
```

## 5) Known Operational Notes
- IMU and pointcloud topics require RealSense and related modules/drivers to be healthy.
- For GPS fusion into localization, `navsat_transform_node` + dual/global EKF integration is intentionally postponed for outdoor test.
- If `base_link` slowly rotates while stopped, check TF duplicate publishers and EKF yaw source config.

Quick checks:
```bash
ros2 topic info /tf -v
ros2 run tf2_ros tf2_echo odom base_link
ros2 topic echo /gps/fix --once
```

## 6) Pending Work (Outdoor Session)
- Enable `navsat_transform_node`
- Fuse `odometry/gps` into global EKF
- Validate map consistency with real GPS motion

## 7) Where to continue in chat
When opening this project on another PC, provide this file first:
- `~/ca_ws/HANDOFF.md`

Then continue with the latest task request directly.
