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
  /odom(휠 vx) + /icp_odom_filtered(ICP vyaw) → EKF → /odometry/filtered
                                                      → odom → base_link TF
  /icp_odom → icp_odom_cov_scale.py → /icp_odom_filtered  (cmd_vel 상태별 공분산 조정)

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

### 맵 뒤틀림(회전 드리프트) — 현재 진행 중인 이슈

#### 현재 EKF 설정 (ekf.yaml)
- `odom0` (휠): `vx=true`만, 나머지 false (슬립 영향, yaw/vyaw 신뢰 불가)
- `odom1` (ICP F2F `/icp_odom_filtered`): `vyaw=true`만 (cmd_vel 상태별 공분산 조정 후 입력)
- `imu0`: 전부 false (LSB=0.00107 rad/s 양자화 노이즈 → 정지 중 TF 진동 → 사용 불가)

#### ICP Odometry vyaw 계산 원리 (드리프트 해결 공식)

```
F2F ICP: P_{t-1}(이전 스캔) ↔ P_t(현재 스캔) 매칭

PointToPlane 최소화:  Σ [ (R·pᵢ + t − qᵢ) · n̂ᵢ ]² → min
  pᵢ: 현재 포인트,  qᵢ: 대응점,  n̂ᵢ: 법선벡터(K=8 이웃)

Force3DoF → R = Rz(θ) → vyaw = θ / dt
```

- 휠 슬립, IMU 양자화와 무관하게 실제 스캔 기하학에서 yaw rate를 직접 측정
- 정지 시 두 스캔 동일 → vyaw≈0 + cov[35]=9999(sentinel) → 드리프트 없음

#### icp_odom_cov_scale.py — ICP vyaw 공분산 동적 조정 ✅ 적용 완료
`src/rtabmap_ros/rtabmap_launch/scripts/icp_odom_cov_scale.py`

`/icp_odom`의 `twist.covariance[35]`(vyaw 분산)를 cmd_vel 상태에 따라 스케일링해 `/icp_odom_filtered`로 재발행:

| 상태 | 조건 | SCALE | 효과 |
|------|------|-------|------|
| 정지 | `|v|<0.01, |w|<0.01` 또는 cmd_vel 0.5초 없음 | 1000 | EKF가 ICP vyaw 사실상 무시 → 드리프트 방지 |
| 회전 | `|w|≥0.01` | 1 | ICP PointToPlane 완전 신뢰 → odom yaw 정확 추적 |
| 전진 | `|v|≥0.01` | 10 | vyaw 부분 신뢰 |

> RTAB-Map은 정지/no-motion 감지 시 cov[35]=9999(sentinel)를 출력하므로, SCALE_STOP=1000으로 곱해도 EKF가 무시하게 됨.

#### 현재 ICP Odometry 설정 (rtabmap_nav2.launch.py odom_args) ✅ 적용 완료
```
--Odom/Strategy 0               # F2F: 직전 프레임과만 비교
--Odom/GuessMotion true         # 이전 ICP 상대 움직임을 guess로 활용
--Reg/Force3DoF true            # 평면 주행 강제
--Icp/VoxelSize 0.15
--Icp/PointToPlane 1            # 법선벡터 활용 → 회전 추정 정확도 향상 (0→1)
--Icp/PointToPlaneK 8           # 법선벡터 계산에 사용할 최근접 이웃 수
--Icp/MaxCorrespondenceDistance 0.3  # 0.5에서 강화 → 엄격한 매칭
--Icp/CorrespondenceRatio 0.01
--Icp/OutlierRatio 0.7
--Icp/MaxTranslation 2.0
--Icp/MaxRotation 6.28
```

#### 현재 RTAB-Map 그래프/ICP 설정 (rtabmap.launch.py) ✅ 적용 완료
```
RGBD/ProximityBySpace: false      # 제자리 회전 노드 간 모순 링크 방지 (true→false)
RGBD/ProximityMaxGraphDepth: 0    # proximity 탐색 완전 차단
RGBD/ProximityPathMaxNeighbors: 0
RGBD/AngularUpdate: 1.0           # 360° 회전 시 최적화 횟수 42→6회 (0.15→1.0)
RGBD/LinearUpdate: 0.10

Optimizer/Robust: true            # Huber 비용함수: outlier 제약 하중 감소 → map→odom 안정화
Icp/VoxelSize: 0.15               # odom ICP와 동일 파라미터로 통일
Icp/PointToPlane: true
Icp/PointToPlaneK: 8
Icp/MaxCorrespondenceDistance: 0.3
Icp/OutlierRatio: 0.7
Icp/CorrespondenceRatio: 0.01

Rtabmap/LoopThr: 0.50             # 루프 임계 강화 (0.30→0.50): 회전 중 false loop 방지
Vis/MinInliers: 50                # 시각 매칭 강화 (30→50)
```

**ICP 파라미터 통일 이유**: 기존에 RTAB-Map 노드 ICP(맵 등록용)에는 PointToPlane만 설정되어 있었고 VoxelSize/MaxCorrespondenceDistance/OutlierRatio는 기본값 사용. odom ICP와 결과가 달라지면 매 노드마다 map→odom이 조정됨 → 통일해서 불일치 제거.

**Optimizer/Robust 이유**: 회전 중 ICP 등록이 약간 부정확해도 표준 least-squares는 그 오차 전체를 map→odom으로 전달. Huber 비용함수는 outlier 제약을 하중 감소시켜 map→odom을 안정화.

#### 해결 완료: odom→base_link yaw 드리프트 ✅
- 정지 중: SCALE_STOP=1000으로 ICP vyaw 무시 → 드리프트 없음
- 회전 중 추적: ICP PointToPlane + SCALE_ROT=1 → 개선

#### 진행 중: 제자리 회전 시 map→odom TF 불안정
- 적용 변경: Optimizer/Robust=true + ICP파라미터 통일 + AngularUpdate=1.0 + LoopThr=0.50
- **재테스트 필요**: 위 변경 후 제자리 회전 시 map 안정성 확인

#### 드리프트 원인 분리 진단
```bash
# 터미널 1: EKF 오차 확인 (odom->base_link)
ros2 run tf2_ros tf2_echo odom base_link
# 터미널 2: RTAB-Map 오차 확인 (map->odom)
ros2 run tf2_ros tf2_echo map odom
# 정지 20초 + 180도 회전 후 각각 yaw 변화량 비교
```
- `odom→base_link` yaw 변화 → EKF 문제 (icp_odom_cov_scale 조정)
- `map→odom` yaw 변화 → RTAB-Map 문제 (ProximityBySpace/루프클로저 조정)

#### 미해결: IMU vyaw 양자화 노이즈
- RealSense IMU LSB=0.00107 rad/s, 200Hz → 정지 중 TF 진동 → `imu0` 전부 false 유지

RTAB-Map에서 `g2o optimizer not available` 경고가 나오면 g2o를 소스 빌드 후 rtabmap_ros 재빌드 필요 (README.md §11-4 참조).

### 현재 센서 TF 오프셋
- `base_link → livox_frame`: x=0.30, y=0.00, z=0.63
- `base_link → camera_link`: x=0.30, y=0.00, z=0.55
- `base_link → gps_link`: x=0.30, y=0.00, z=0.35

### GPS 아웃도어 통합 (미완성)
`navsat_transform_node` 활성화 및 global EKF로의 `odometry/gps` 퓨전은 아직 미구현 (HANDOFF.md §6 참조).
