# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 빌드 및 실행

### 빌드
```bash
cd ~/ca_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source ~/ca_ws/install/setup.bash
```

특정 패키지만 빌드:
```bash
colcon build --symlink-install --packages-up-to <패키지명>
```

### 전체 시스템 실행 (권장)
```bash
cd ~/ca_ws
source /opt/ros/humble/setup.bash
source ~/ca_ws/install/setup.bash
bash ~/ca_ws/run_all.sh
```

### 개별 컴포넌트 실행

```bash
# GPS 브리지
ros2 launch ec25_gps_bridge ec25_gps_bridge.launch.py device:=/dev/ttyUSB3

# 센서 동기화 (IMU 보정 + LiDAR deskew + 정적 TF)
ros2 launch rtabmap_launch sensor_sync.launch.py lidar_offset_sec:=0.003

# EKF
ros2 launch robot_localization ekf.launch.py

# RTAB-Map + Nav2
ros2 launch rtabmap_launch rtabmap_nav2.launch.py

# Agent PID (RL 기반 PID 게인 조정)
ros2 launch rl_pid_training agent_pid.launch.py \
  model:=/home/atoz/ca_ws/rl_pid_model_new \
  odom_topic:=/odometry/filtered \
  desired_cmd_topic:=/controller_server/RLController/desired_cmd \
  desired_cmd_type:=auto \
  motor_status_topic:=/bunker_status \
  use_motor_encoder_obs:=true
```

### 시뮬레이션 (Gazebo)
```bash
ros2 launch bunker_sim bunker_gz.launch.py gui:=false  # 헤드리스
ros2 launch bunker_sim bunker_gz.launch.py gui:=true   # GUI
```

## 진단 도구

```bash
# LiDAR-카메라 시간동기 측정
python3 ~/ca_ws/tools/lidar_camera_sync_probe.py \
  --camera-topic /camera/camera/aligned_depth_to_color/image_raw \
  --lidar-topic /livox/lidar/synced/deskewed \
  --current-lidar-offset 0.003 \
  --duration-sec 30

# EKF yaw 드리프트 진단
python3 ~/ca_ws/tools/ekf_yaw_probe.py --duration 180 --rate 20

# RL PID 로그 시각화
python3 ~/ca_ws/tools/plot_pid_policy.py \
  --csv ~/ca_ws/rl_pid_logs/pid_policy_<날짜>.csv --show

# LiDAR 딜레이 확인
ros2 topic delay /livox/lidar
ros2 topic delay /livox/lidar/synced
ros2 topic delay /livox/lidar/synced/deskewed

# TF 중복 확인 (odom->base_link 발행 노드 수)
ros2 topic info /tf -v
ros2 run tf2_ros tf2_echo odom base_link
```

## 아키텍처 개요

이 프로젝트는 **Bunker 자율주행 로봇**을 위한 ROS2 Humble 기반 시스템입니다.

### 데이터 흐름

```
[하드웨어 드라이버]
  Livox MID360 → /livox/lidar
  RealSense D455 → /camera/camera/* + /camera/camera/imu_fixed
  EC25 모뎀 → /gps/fix
  Bunker CAN → /odom + /bunker_status

[센서 처리 파이프라인]
  /livox/lidar → livox_timestamp_offset → /livox/lidar/synced
              → lidar_deskewing → /livox/lidar/synced/deskewed
  /camera/camera/imu → camera_imu_bias_corrector → imu_filter_madgwick

[상태 추정]
  /odom + /camera/camera/imu_fixed → EKF → /odometry/filtered
                                         → odom → base_link TF

[SLAM + 네비게이션]
  /livox/lidar/synced/deskewed + /odometry/filtered → RTAB-Map
  RTAB-Map → map → odom TF + /rtabmap/map
  Nav2 + rl_local_controller → /cmd_vel

[RL PID 제어]
  /odometry/filtered + /bunker_status + desired_cmd → run_pid_policy.py → PID gains
```

### 핵심 패키지

| 패키지 | 역할 | 언어 |
|--------|------|------|
| `bunker_ros2/bunker_base` | CAN 버스 모터 제어, 휠 오도메트리 | C++ |
| `livox_ros_driver2` | Livox MID360 LiDAR 드라이버 | C++ |
| `livox_timestamp_offset` | LiDAR 타임스탬프 오프셋 보정 | Python |
| `camera_imu_pipeline_cpp` | RealSense IMU 바이어스 보정 | C++ |
| `ec25_gps_bridge` | EC25 모뎀 → /gps/fix 변환 | Python |
| `robot_localization` | EKF 기반 센서 퓨전 | C++ |
| `rtabmap_ros/rtabmap_launch` | SLAM 백엔드, Nav2 통합 런치 | - |
| `rl_local_controller` | Nav2 로컬 플래너 플러그인 (회전 보호 포함) | C++ |
| `rl_pid_training` | PPO 기반 실시간 PID 게인 조정 | Python |
| `traversability_layer` | Nav2 costmap 지형 통과 가능성 플러그인 | C++ |

### 주요 설정 파일 위치

- **Nav2 파라미터**: `src/rtabmap_ros/rtabmap_launch/launch/config/nav2_rtabmap_params.yaml`
- **EKF 설정**: `src/robot_localization/params/ekf.yaml`
- **센서 동기화 런치**: `src/rtabmap_ros/rtabmap_launch/launch/sensor_sync.launch.py`
- **Agent PID 런치**: `src/rl_pid_training/launch/agent_pid.launch.py`
- **전체 실행 스크립트**: `run_all.sh`

### RL PID 시스템

`src/rl_pid_training/rl_pid_training/run_pid_policy.py`가 핵심 추론 로직:
- **입력 관측** (9차원): v_ref, w_ref, v_meas, w_meas, e_v, e_w, motor_rpm_mean, motor_current_mean, encoder_pulse_rate_mean
- **출력**: kp_lin, ki_lin, kd_lin, kp_ang, ki_ang, kd_ang 6개 PID 게인의 증감값
- **학습 모델**: `rl_pid_model_new.zip` (stable-baselines3 PPO)
- **실행 환경**: `~/.venv/` Python venv

## 알려진 이슈 및 중요 설정

### TF 중복 publish 방지
`bunker_base`와 EKF가 동시에 `odom→base_link`를 publish하면 TF가 튄다.
- `bunker_base`는 반드시 `publish_odom_tf:=false`로 실행
- EKF만 `odom→base_link` TF를 publish

### 맵 뒤틀림(회전 드리프트)
EKF yaw 설정:
- `odom0_config`: `yaw=false, vyaw=false`
- `imu0_config`: `yaw=false, vyaw=true`

RTAB-Map에서 `g2o optimizer not available` 경고가 나오면 g2o를 소스 빌드 후 rtabmap_ros 재빌드 필요 (README.md §11-4 참조).

### 현재 센서 TF 오프셋
- `base_link → livox_frame`: x=0.30, y=0.00, z=0.63
- `base_link → camera_link`: x=0.30, y=0.00, z=0.55
- `base_link → gps_link`: x=0.30, y=0.00, z=0.35

### GPS 아웃도어 통합 (미완성)
`navsat_transform_node` 활성화 및 global EKF로의 `odometry/gps` 퓨전은 아직 미구현 (HANDOFF.md §6 참조).
