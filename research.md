# Bunker 자율주행 로봇 시스템 — 심층 연구 보고서

작성일: 2026-03-09
분석 대상: `/home/atoz/ca_ws` (ROS2 Humble 기반 Bunker 자율주행 로봇)

---

## 목차

1. [시스템 개요](#1-시스템-개요)
2. [전체 아키텍처 및 데이터 흐름](#2-전체-아키텍처-및-데이터-흐름)
3. [하드웨어 구성 및 센서 스펙](#3-하드웨어-구성-및-센서-스펙)
4. [패키지별 상세 분석](#4-패키지별-상세-분석)
   - 4.1 bunker_ros2/bunker_base (모터/오도메트리)
   - 4.2 livox_ros_driver2 (LiDAR 드라이버)
   - 4.3 livox_timestamp_offset (타임스탬프 보정)
   - 4.4 camera_imu_pipeline_cpp (IMU 바이어스 보정)
   - 4.5 robot_localization (EKF 센서 퓨전)
   - 4.6 rtabmap_ros/rtabmap_launch (SLAM + Nav2)
   - 4.7 rl_local_controller (로컬 플래너 플러그인)
   - 4.8 rl_pid_training (RL 기반 PID 게인 조정)
   - 4.9 ec25_gps_bridge (GPS 브리지)
   - 4.10 traversability_layer (지형 통과 가능성 레이어)
5. [핵심 알고리즘 심층 분석](#5-핵심-알고리즘-심층-분석)
   - 5.1 ICP Odometry & PointToPlane
   - 5.2 EKF 이중 오도메트리 전략
   - 5.3 icp_odom_cov_scale 동적 공분산 조정
   - 5.4 LiDAR Deskewing
   - 5.5 PPO 기반 실시간 PID 게인 적응
   - 5.6 RTAB-Map 그래프 최적화
6. [시작 순서 및 실행 메커니즘](#6-시작-순서-및-실행-메커니즘)
7. [설정 파일 상세 분석](#7-설정-파일-상세-분석)
8. [알려진 이슈 및 해결책](#8-알려진-이슈-및-해결책)
9. [진단 도구 분석](#9-진단-도구-분석)
10. [시스템 강점과 한계](#10-시스템-강점과-한계)

---

## 1. 시스템 개요

이 프로젝트는 **Bunker 모바일 로봇**을 위한 ROS2 Humble 기반 완전 자율주행 스택이다. Nvidia Jetson 플랫폼 위에서 동작하며, 실내외 혼합 환경에서의 자율 내비게이션을 목표로 한다.

### 핵심 기술 혁신

| 혁신 | 설명 |
|------|------|
| **ICP-EKF 이중 오도메트리** | 휠 슬립과 IMU 양자화 노이즈를 동시에 극복하는 이중 소스 퓨전 |
| **PointToPlane ICP** | 순수 회전 시 안정적인 yaw 추정 |
| **PPO 실시간 PID 조정** | stable-baselines3 학습 모델이 9차원 관측으로 PID 게인을 실시간 조정 |
| **동적 공분산 스케일링** | cmd_vel 상태에 따라 ICP vyaw 신뢰도를 동적으로 조정 |
| **좁은 통로 탈출** | Nav2 로컬 플래너가 벽 충돌 없이 좁은 공간에서 탈출 |

### 전체 패키지 목록

```
src/
├── bunker_ros2/          # CAN 모터 제어, 휠 오도메트리
├── livox_ros_driver2/    # Livox MID360 드라이버
├── livox_timestamp_offset/ # LiDAR 타임스탬프 보정
├── camera_imu_pipeline_cpp/ # IMU 바이어스 보정 + Madgwick
├── ec25_gps_bridge/      # EC25 모뎀 GPS
├── robot_localization/   # EKF (외부, 커스텀 설정)
├── rtabmap_ros/          # RTAB-Map SLAM 백엔드
│   └── rtabmap_launch/   # 런치 + 스크립트 (커스텀)
├── rl_local_controller/  # Nav2 로컬 플래너 플러그인 (C++)
├── rl_pid_training/      # PPO PID 게인 조정 (Python)
└── traversability_layer/ # Nav2 costmap 지형 레이어 (C++)
```

---

## 2. 전체 아키텍처 및 데이터 흐름

### 전체 데이터 흐름도

```
[하드웨어 드라이버 레이어]
─────────────────────────────────────────────────────────────────
Bunker CAN (50Hz)
  └─ bunker_base ──────────────────────────────► /odom
                                                  (vx만 EKF에 사용)
                                               ► /bunker_status
                                                  (rpm, current, encoder)

Livox MID360 (20Hz)
  └─ livox_ros_driver2 ────────────────────────► /livox/lidar
  └─ livox_timestamp_offset ──────────────────► /livox/lidar/synced
  └─ lidar_deskewing ─────────────────────────► /livox/lidar/synced/deskewed
  └─ dynamic_object_filter ───────────────────► /livox/lidar/static_filtered

RealSense D455 (200Hz IMU, 30fps)
  └─ camera_imu_bias_corrector ───────────────► /camera/camera/imu_bias_corrected
  └─ imu_filter_madgwick ─────────────────────► /camera/camera/imu_fixed

EC25 모뎀 (2Hz)
  └─ ec25_gps_bridge ─────────────────────────► /gps/fix (미퓨전)

[센서 퓨전 레이어]
─────────────────────────────────────────────────────────────────
/livox/lidar/synced/deskewed
  └─ rtabmap ICP 오도메트리 ──────────────────► /icp_odom
  └─ icp_odom_cov_scale.py ──────────────────► /icp_odom_filtered
                                                  (cov[35]: SCALE 적용)

/odom (vx) + /icp_odom_filtered (vyaw)
  └─ robot_localization EKF ──────────────────► /odometry/filtered
                                               ► /tf: odom→base_link

[SLAM 레이어]
─────────────────────────────────────────────────────────────────
/livox/lidar/static_filtered + /odometry/filtered
  └─ RTAB-Map ────────────────────────────────► /rtabmap/map (OccupancyGrid)
                                               ► /rtabmap/cloud_map (PointCloud2)
                                               ► /tf: map→odom

[내비게이션 레이어]
─────────────────────────────────────────────────────────────────
/rtabmap/map → Nav2 (SmacPlanner2D) → 전역 경로
  └─ RLController (rl_local_controller) ──────► /controller_server/.../desired_cmd
  └─ velocity_smoother (30Hz) ────────────────► /cmd_vel → CAN

[RL PID 적응 레이어]
─────────────────────────────────────────────────────────────────
/odometry/filtered + /bunker_status + /desired_cmd
  └─ run_pid_policy.py (PPO) ──────────────────► set_parameters 서비스
                                                  (kp_lin, ki_lin, kd_lin,
                                                   kp_ang, ki_ang, kd_ang)

[costmap 레이어]
─────────────────────────────────────────────────────────────────
/rtabmap/cloud_map
  └─ traversability_layer ────────────────────► 테이블 아래 통과 가능 영역 마킹
/livox/lidar/static_filtered
  └─ costmap obstacle_layer ──────────────────► 장애물 투영
```

### 토픽별 발행 주기 및 크기

| 토픽 | 주기 | 타입 | 설명 |
|------|------|------|------|
| `/livox/lidar` | 20Hz | PointCloud2 | 원시 LiDAR |
| `/livox/lidar/synced` | 20Hz | PointCloud2 | 타임스탬프 보정 |
| `/livox/lidar/synced/deskewed` | 20Hz | PointCloud2 | 모션 보정 |
| `/odom` | 50Hz | Odometry | 휠 오도메트리 |
| `/icp_odom` | ~4Hz | Odometry | ICP 오도메트리 |
| `/icp_odom_filtered` | ~4Hz | Odometry | 공분산 스케일 후 |
| `/odometry/filtered` | 20Hz | Odometry | EKF 퓨전 결과 |
| `/bunker_status` | 50Hz | BunkerStatus | 모터/엔코더 상태 |
| `/cmd_vel` | 30Hz | Twist | 최종 모터 명령 |
| `/gps/fix` | 2Hz | NavSatFix | GPS (미퓨전) |

---

## 3. 하드웨어 구성 및 센서 스펙

### 물리적 마운팅 위치 (base_link 기준)

```
        front
   ─────────────
   |           |
   |  [LiDAR]  |  ← x=0.30, y=0.00, z=0.63m (livox_frame)
   |  [Camera] |  ← x=0.30, y=0.00, z=0.55m (camera_link)
   |  [GPS]    |  ← x=0.30, y=0.00, z=0.35m (gps_link)
   |           |
   ─────────────
```

### 센서 상세 스펙

#### Livox MID360 LiDAR
- **출력**: 20Hz, PointCloud2
- **수평 FOV**: 360°
- **수직 FOV**: -7° ~ +52°
- **최대 사거리**: 70m
- **타임스탬프 문제**: 하드웨어 클럭이 시스템 시계보다 약 0.003초 빠름
  → `livox_timestamp_offset`으로 보정

#### Intel RealSense D455
- **RGB-D**: 30fps (1280×720)
- **IMU**: 200Hz (3축 가속도계 + 3축 자이로스코프)
- **IMU 양자화 노이즈**: LSB = 0.00107 rad/s
  → 정지 시에도 0, ±0.00107, ±0.00214... 이산값 출력
  → 200Hz 적분 시 누적 드리프트 발생
  → EKF에서 IMU vyaw 완전 비활성화

#### Bunker 모터/엔코더
- **프로토콜**: CAN 버스 (can1, 500kbps)
- **펌웨어**: AGX_V1 또는 AGX_V2 자동 감지
- **오도메트리**: 50Hz
- **문제점**: 바퀴 슬립 시 yaw 오차 발생 → EKF에서 휠 yaw 비활성화

#### EC25 LTE 모뎀 GPS
- **인터페이스**: /dev/ttyUSB3, 115200bps
- **출력**: 2Hz (HDOP 기반 공분산 추정)
- **AT 명령 시퀀스**: `AT` → `AT+QGPS?` → `AT+QGPS=1` → `AT+QGPSLOC=2`
- **상태**: 브리지 동작 중, 퓨전은 미구현

---

## 4. 패키지별 상세 분석

### 4.1 bunker_ros2/bunker_base (C++)

**경로**: `src/bunker_ros2/bunker_base/`

#### 역할
Bunker 로봇의 CAN 버스 인터페이스. 모터 명령 수신 및 오도메트리 발행.

#### 핵심 파일
- `src/bunker_base_node.cpp` — 노드 진입점
- `src/bunker_base_ros.cpp` — CAN 통신 + 메신저 로직

#### 발행/구독

| 방향 | 토픽 | 타입 | 비고 |
|------|------|------|------|
| 구독 | `/cmd_vel` | Twist | 모터 속도 명령 |
| 발행 | `/odom` | Odometry | 50Hz 휠 오도메트리 |
| 발행 | `/bunker_status` | BunkerStatus | RPM, 전류, 엔코더 |
| 발행 | `/tf` odom→base_link | Transform | `publish_odom_tf=false`로 비활성화 |

#### 중요 파라미터
```yaml
publish_odom_tf: false   # EKF가 TF 관리 → 중복 방지 필수
can_device: can1
is_bunker_mini: false
```

#### TF 충돌 문제
`bunker_base`와 EKF가 동시에 `odom→base_link` TF를 발행하면 TF가 진동(jitter)한다.
반드시 `publish_odom_tf:=false`로 실행해야 한다.

---

### 4.2 livox_ros_driver2 (C++, 외부)

**경로**: `src/livox_ros_driver2/`

- Livox SDK2 위에서 MID360 드라이버 구현
- 주요 설정: `MID360_config.json` (IP, 포트, 포인트클라우드 타입)
- 출력: `/livox/lidar` (PointCloud2, 20Hz)

---

### 4.3 livox_timestamp_offset (Python)

**경로**: `src/livox_timestamp_offset/`

#### 역할
LiDAR 하드웨어 클럭과 시스템 클럭의 오차 보정.

#### 핵심 코드 (`livox_timestamp_offset_node.py`)
```python
def callback(self, msg):
    corrected_msg = copy.deepcopy(msg)
    offset = Duration(nanoseconds=int(self.offset_sec * 1e9))
    corrected_msg.header.stamp = (
        Time.from_msg(msg.header.stamp) + offset
    ).to_msg()
    self.pub.publish(corrected_msg)
```

#### 파라미터
```yaml
lidar_offset_sec: 0.003  # 시스템이 LiDAR보다 0.003s 앞선 경우
```

**실측 방법**:
```bash
python3 ~/ca_ws/tools/lidar_camera_sync_probe.py \
  --camera-topic /camera/camera/aligned_depth_to_color/image_raw \
  --lidar-topic /livox/lidar/synced/deskewed \
  --current-lidar-offset 0.003 \
  --duration-sec 30
```

---

### 4.4 camera_imu_pipeline_cpp (C++)

**경로**: `src/camera_imu_pipeline_cpp/`

#### 역할
RealSense D455 IMU의 바이어스 보정 및 Madgwick 필터를 통한 자세 추정.

#### camera_imu_bias_corrector.cpp

**바이어스 보정 알고리즘**:
1. 초기 정지 단계에서 1000 샘플 수집
2. 평균값 계산 → 초기 바이어스 추정
3. 이후 실시간 EMA (Exponential Moving Average, α=0.001)로 온도 드리프트 추적
4. 보정된 IMU 데이터 발행

```cpp
// 초기 캘리브레이션
if (calib_count < 1000) {
    bias_x += gyro.x; bias_y += gyro.y; bias_z += gyro.z;
    calib_count++;
} else {
    // 실시간 EMA 업데이트
    bias_x = 0.001 * gyro.x + 0.999 * bias_x;
}
corrected.z = gyro.z - bias_z;
```

#### imu_filter_madgwick

- 입력: `/camera/camera/imu_bias_corrected`
- 출력: `/camera/camera/imu_fixed` (쿼터니언 자세 + 각속도)
- `zeta: 0.0` → 자이로 드리프트 보정 비활성화 (원시 각속도 보존 목적)

#### IMU 한계 (EKF에서 비활성화한 이유)
```
LSB = 0.00107 rad/s
200Hz 적분 → 200 × 0.00107 = 0.214 rad/s 최대 양자화 오차
30초 적분 → 최대 6.4 rad (360° 이상) 누적 가능
정지 중에도 ±0.00107, ±0.00214 이산값 출력 → TF 진동 발생
```

---

### 4.5 robot_localization (EKF 센서 퓨전)

**경로**: `src/robot_localization/`, **설정**: `src/robot_localization/params/ekf.yaml`

#### 역할
확장 칼만 필터(EKF)로 휠 오도메트리와 ICP 오도메트리를 퓨전.

#### 입력 구성

```yaml
# 휠 오도메트리 (50Hz) - vx만 활성화
odom0: /odom
odom0_config: [false, false, false,   # x, y, z 위치
               false, false, false,   # roll, pitch, yaw
               true,  false, false,   # vx, vy, vz  ← 이것만 활성화
               false, false, false,   # vroll, vpitch, vyaw
               false, false, false]   # ax, ay, az
odom0_queue_size: 10
odom0_differential: false

# ICP LiDAR 오도메트리 (~4Hz) - vyaw만 활성화
odom1: /icp_odom_filtered
odom1_config: [false, false, false,
               false, false, false,
               false, false, false,
               false, false, true,    # vyaw만 활성화
               false, false, false]
odom1_queue_size: 10

# IMU - 전부 비활성화 (양자화 노이즈)
imu0: /camera/camera/imu_fixed
imu0_config: [false × 15]
```

#### 출력
- `/odometry/filtered` — 20Hz, 퓨전된 (vx, wz) 상태
- `/tf: odom → base_link` — 20Hz TF

#### 설계 철학

```
문제: 단일 센서로는 정확한 yaw 추정 불가
  - 휠 오도메트리: 슬립 → yaw 오차
  - IMU: 양자화 노이즈 → 적분 드리프트
  - ICP 단독: 빠른 갱신 불가

해결: 이중 소스 분리 퓨전
  - vx 소스: 휠 엔코더 (슬립 영향 작음)
  - vyaw 소스: LiDAR ICP (슬립/양자화 무관)
  - EKF: 두 소스를 최적 결합 → 정확한 전체 상태
```

---

### 4.6 rtabmap_ros/rtabmap_launch (SLAM + Nav2)

**경로**: `src/rtabmap_ros/rtabmap_launch/`

#### 4.6.1 sensor_sync.launch.py

센서 전처리 파이프라인 런치.

**구성 요소**:
1. **livox_timestamp_offset** — 타임스탬프 보정
2. **camera_imu_bias_corrector** — IMU 바이어스 보정
3. **imu_filter_madgwick** — 자세 추정
4. **lidar_deskewing** (`rtabmap_util`) — base_link 기준 모션 보정
5. **static TF publishers** — 센서 마운팅 위치

**LiDAR deskewing 설정**:
```python
parameters=[{
    'fixed_frame_id': 'base_link',  # odom이 아닌 base_link 사용 (이유 후술)
    'wait_imu_to_init': False,
    'slerp': True,  # 회전 보간 활성화
}]
```

#### 4.6.2 rtabmap.launch.py

RTAB-Map SLAM 백엔드.

**핵심 파라미터** (맵 뒤틀림 수정 반영):
```python
RTABMAP_ARGS = {
    # 그래프 최적화 — 제자리 회전 시 맵 안정성
    'RGBD/ProximityBySpace': 'false',       # false: 공간 근접 링크 비활성화
    'RGBD/ProximityMaxGraphDepth': '0',     # 근접 탐색 완전 차단
    'RGBD/AngularUpdate': '1.0',            # 57° 간격으로 노드 생성 (기존 11.5°)
    'RGBD/LinearUpdate': '0.10',            # 10cm 이동 시 노드 생성

    # Robust 최적화 — ICP 오차 흡수
    'Optimizer/Robust': 'true',             # Huber 비용함수

    # ICP 등록 파라미터 — odom ICP와 통일
    'Icp/PointToPlane': 'true',
    'Icp/PointToPlaneK': '8',
    'Icp/VoxelSize': '0.15',
    'Icp/MaxCorrespondenceDistance': '0.3',
    'Icp/OutlierRatio': '0.7',
    'Icp/CorrespondenceRatio': '0.01',

    # 루프 클로저 강화 — 회전 중 false loop 방지
    'Rtabmap/LoopThr': '0.50',
    'Vis/MinInliers': '50',
    'Rtabmap/DetectionRate': '5.0',
}
```

**AngularUpdate 변경 이유**:
```
기존 0.15 rad (11.5°):
  360° 회전 = 42개 노드 생성
  각 노드의 ICP 등록이 조금씩 부정확 → 42번 누적 오차 → 맵 뒤틀림

변경 후 1.0 rad (57°):
  360° 회전 = 6개 노드 생성
  Huber robust + PointToPlane으로 각 노드 등록 정확
  → 누적 오차 대폭 감소 → 맵 안정
```

#### 4.6.3 rtabmap_nav2.launch.py

SLAM + Nav2 + ICP 오도메트리 통합 런치.

**ICP 오도메트리 파라미터**:
```python
odom_args = (
    '--Odom/Strategy 0 '            # F2F(Frame-to-Frame): 직전 프레임과 비교
    '--Odom/GuessMotion true '      # 이전 움직임을 초기 추정치로 활용
    '--Reg/Force3DoF true '         # 2D 평면 주행 강제 (z, roll, pitch 고정)
    '--Icp/VoxelSize 0.15 '
    '--Icp/PointToPlane 1 '
    '--Icp/PointToPlaneK 8 '
    '--Icp/MaxCorrespondenceDistance 0.3 '
    '--Icp/CorrespondenceRatio 0.01 '
    '--Icp/OutlierRatio 0.7 '
    '--Icp/MaxTranslation 2.0 '
    '--Icp/MaxRotation 6.28 '
    '--Odom/ResetCountdown 5 '      # 5회 연속 실패 시 리셋
)
```

**오도메트리 입력 선택**: `/livox/lidar/synced/deskewed` (원시 포인트클라우드)
→ `/livox/lidar/static_filtered` (동적 객체 제거 후)가 아닌 이유:
→ static_filtered는 RTAB-Map 출력 의존 → 데드락(순환 의존) 가능성

#### 4.6.4 scripts/icp_odom_cov_scale.py

ICP vyaw 공분산 동적 조정 스크립트.

```python
# 구독
/cmd_vel (Twist) → 현재 운동 상태 파악
/icp_odom (Odometry) → ICP 출력 수신

# 상태별 SCALE 적용
if abs(v) < 0.01 and abs(w) < 0.01:  # 정지
    SCALE = 1000  # ICP vyaw 사실상 무시 → 드리프트 방지
elif abs(w) >= 0.01:                  # 회전
    SCALE = 1     # ICP PointToPlane 완전 신뢰
else:                                 # 전진
    SCALE = 10    # vyaw 부분 신뢰

cov_out[35] = cov_in[35] * SCALE
publish(/icp_odom_filtered)
```

**설계 이유**:
- 정지 시 ICP는 동일 스캔 → vyaw ≈ 0이나 노이즈 존재 → EKF가 보정 시도 → 드리프트
- SCALE=1000으로 곱하면 EKF 관측 불확실성이 커져 사실상 무시
- 회전 시 SCALE=1 → ICP PointToPlane이 정확 → 완전 신뢰
- RTAB-Map 자체 no-motion sentinel (cov[35]=9999) × SCALE=1000 = 9,999,000 → EKF가 절대 무시

---

### 4.7 rl_local_controller (C++ Nav2 플러그인)

**경로**: `src/rl_local_controller/src/rl_local_controller.cpp`

#### 역할
Nav2 LocalPlanner 플러그인으로 경로 추적 + 좁은 공간 탈출.

#### 경로 추적 알고리즘

```cpp
// 1. Lookahead 포인트 추출
lookahead_dist = 0.9m (기본값)
lookahead_point = findLookahead(global_path, current_pose, lookahead_dist)

// 2. 방향 오차 계산
heading_error = atan2(lookahead_point.y - curr.y, lookahead_point.x - curr.x) - curr.yaw
heading_error = normalize_angle(heading_error)  // [-π, π]

// 3. 속도 명령 생성
cmd_vel.angular.z = heading_gain * heading_error  // 0.85 gain
cmd_vel.linear.x = compute_forward_speed(obstacles, heading_error)
```

#### 좁은 통로 탈출 로직

```cpp
// costmap에서 클리어런스 측정
float left_clearance  = measureClearance(LEFT, 90°)
float right_clearance = measureClearance(RIGHT, 90°)
float front_clearance = measureClearance(FRONT, 0°)
float side_clearance  = min(left_clearance, right_clearance)

// 탈출 조건 판단
if (side_clearance < 0.42m || front_clearance < 0.32m) {
    disable_in_place_rotation();

    if (front_clearance > 0.15m) {
        // 앞으로 서행
        cmd_vel.linear.x = 0.05 m/s
        cmd_vel.angular.z = heading_cmd * 0.6
    } else {
        // 후진
        cmd_vel.linear.x = -0.03 m/s
    }
}
```

#### PID 내부 제어

```cpp
// use_pid: true일 때 활성화
error_v = v_ref - v_meas
integral_v += error_v * dt
cmd_v = kp_lin * error_v + ki_lin * integral_v + kd_lin * (error_v - prev_error_v) / dt

// 초기값 (RL이 실시간 조정)
kp_lin=0.7, ki_lin=0.0, kd_lin=0.06
kp_ang=1.2, ki_ang=0.0, kd_ang=0.05
```

#### 설정 파라미터 (nav2_rtabmap_params.yaml)

```yaml
RLController:
  max_lin: 0.25              # 최대 선속도 (m/s)
  max_ang: 1.0               # 최대 각속도 (rad/s)
  stop_dist: 0.30            # 목표 도달 거리 (m)
  hard_stop_dist: 0.18       # 비상 정지 거리 (m)
  creep_speed: 0.04          # 서행 속도 (m/s)
  in_place_heading: 0.8      # 제자리 회전 시작 각도 오차 (rad)
  in_place_dist: 0.5         # 제자리 회전 전환 거리 임계 (m)
  rotate_min_side_clearance: 0.42  # 회전 허용 최소 측면 공간 (m)
  rotate_min_front_clearance: 0.32 # 회전 허용 최소 전방 공간 (m)
  lookahead_dist: 0.9        # 경로 추적 lookahead (m)
  heading_gain: 0.85         # 방향 오차 게인
  use_pid: true
```

---

### 4.8 rl_pid_training (Python RL 에이전트)

**경로**: `src/rl_pid_training/`

#### 역할
PPO(Proximal Policy Optimization) 기반 실시간 PID 게인 자동 조정.

#### 시스템 구조

```
[관측 수집] (0.4s 주기)
  /odometry/filtered → v_meas, w_meas
  /desired_cmd       → v_ref, w_ref
  /bunker_status     → motor_rpm, current, encoder_pulse_rate

[PPO 추론]
  obs(9D) → policy(PPO network) → action(6D)

[게인 적용]
  Δkp_lin += clip(action[0], -max_delta, max_delta)
  kp_lin = clip(kp_lin_current + Δkp_lin, kp_min, kp_max)

  → set_parameters 서비스 호출

[보상 계산 + 로깅]
  reward = -(|e_v| + |e_w|) - penalty_terms
  CSV 기록 → ~/ca_ws/rl_pid_logs/pid_policy_YYYYMMDD_HHMMSS.csv
```

#### 관측 벡터 (9차원)

| 인덱스 | 변수 | 설명 |
|--------|------|------|
| 0 | v_ref | 목표 선속도 (m/s) |
| 1 | w_ref | 목표 각속도 (rad/s) |
| 2 | v_meas | 측정 선속도 |
| 3 | w_meas | 측정 각속도 |
| 4 | e_v | 선속도 오차 (v_ref - v_meas) |
| 5 | e_w | 각속도 오차 (w_ref - w_meas) |
| 6 | motor_rpm_mean | 모터 평균 RPM |
| 7 | motor_current_mean | 모터 평균 전류 |
| 8 | encoder_pulse_rate_mean | 엔코더 펄스 레이트 평균 |

#### 행동 벡터 (6차원)

| 인덱스 | 변수 | 범위 |
|--------|------|------|
| 0 | Δkp_lin | [-max_delta, max_delta] |
| 1 | Δki_lin | [-max_delta, max_delta] |
| 2 | Δkd_lin | [-max_delta, max_delta] |
| 3 | Δkp_ang | [-max_delta, max_delta] |
| 4 | Δki_ang | [-max_delta, max_delta] |
| 5 | Δkd_ang | [-max_delta, max_delta] |

#### 신호 컨디셔닝 (떨림 방지)

```python
# w_ref 저역통과 필터
w_ref_filtered = 0.10 * w_ref_raw + 0.90 * w_ref_prev  # LPF α=0.10

# 데드밴드 적용
if abs(w_ref_filtered) < 0.08:
    w_ref = 0.0

# 저속 회전 제한
if abs(v_ref) < 0.10:
    w_ref = clip(w_ref, -0.30, 0.30)

# 방향 전환 홀드 (0.5초)
if sign(w_ref) != sign(w_ref_prev) and t - t_last_flip < 0.50:
    w_ref = w_ref_prev

# 게인 LPF
kp_lin = 0.08 * kp_new + 0.92 * kp_old  # gain_lpf_alpha=0.08

# 행동 데드존
if abs(action[i]) < 0.35:
    action[i] = 0.0
```

#### CSV 로그 형식
```
t, kp_lin, ki_lin, kd_lin, kp_ang, ki_ang, kd_ang,
v_ref, w_ref_raw, w_ref, v_meas, w_meas,
motor_rpm_mean, motor_current_mean, encoder_pulse_rate_mean,
reward
```

#### 학습 모델
- **알고리즘**: PPO (Proximal Policy Optimization, stable-baselines3)
- **모델 파일**: `~/ca_ws/rl_pid_model_new.zip`
- **실행 환경**: `~/.venv/` Python venv

---

### 4.9 ec25_gps_bridge (Python)

**경로**: `src/ec25_gps_bridge/`

#### 역할
EC25 LTE 모뎀의 GPS 데이터를 ROS NavSatFix 메시지로 변환.

#### AT 명령 시퀀스
```
1. AT\r\n              → OK 확인
2. AT+QGPS?\r\n        → GPS 상태 확인
3. AT+QGPS=1\r\n       → GPS 활성화
4. AT+QGPSLOC=2\r\n    → 위치 데이터 요청 (2Hz 폴링)
```

#### 좌표 변환 (DDMM → 십진수)
```python
def parse_gps_location(response):
    # DDMM.MMMMM 형식 파싱
    lat_raw = float(lat_str)
    lat_deg = int(lat_raw / 100)
    lat_min = lat_raw - lat_deg * 100
    lat_decimal = lat_deg + lat_min / 60.0
    if lat_dir == 'S': lat_decimal = -lat_decimal

    # HDOP 기반 공분산
    sigma_xy = hdop * 5.0  # 기본 5m
    cov = [[sigma_xy**2, 0, 0], [0, sigma_xy**2, 0], [0, 0, 9999.0]]
```

#### 현재 상태
- `/gps/fix` 발행 정상 동작
- `navsat_transform_node` + 전역 EKF 퓨전은 **미구현** (향후 과제)

---

### 4.10 traversability_layer (C++ Nav2 플러그인)

**경로**: `src/traversability_layer/src/traversability_layer.cpp`

#### 역할
3D 포인트클라우드 맵에서 로봇이 통과 가능한 낮은 공간(테이블 아래 등)을 costmap에 표시.

#### 알고리즘

```cpp
// /rtabmap/cloud_map 구독
// costmap 각 셀 (x, y)에 대해:

for each point in cloud_map within check_radius (0.25m):
    if point.z < 0.15m:
        floor_count++        // 바닥
    elif point.z <= 0.5m:    // robot_height
        collision_count++    // 로봇 충돌 영역
    elif point.z > 0.6m:     // robot_height + min_clearance
        above_count++        // 로봇 위 공간

// 통과 가능 판단
if above_count > 1.5 * collision_count:
    costmap[x][y] = FREE_SPACE    // 로봇 위에 충분한 공간
else:
    costmap[x][y] = LETHAL_OBSTACLE
```

#### 파라미터
```yaml
robot_height: 0.5m
min_clearance: 0.1m
check_radius: 0.25m
```

---

## 5. 핵심 알고리즘 심층 분석

### 5.1 ICP Odometry & PointToPlane

#### F2F ICP 원리

ICP(Iterative Closest Point)는 두 포인트클라우드 간의 최적 변환(R, t)을 반복적으로 계산:

```
입력: P_{t-1} (이전 스캔), P_t (현재 스캔)

반복:
  1. 대응점 탐색: 각 p_i ∈ P_t에 대해 최근접 q_i ∈ P_{t-1}
  2. 변환 추정: minimize Σ||R·p_i + t - q_i||²
  3. P_t 변환 적용
  4. 수렴까지 반복
```

#### PointToPlane ICP 개선

기존 PointToPoint vs PointToPlane:

```
PointToPoint:
  objective = Σ ||R·p_i + t - q_i||²
  문제: 순수 회전 시 점들이 호(arc)로 이동 → 많은 local minimum

PointToPlane:
  objective = Σ [(R·p_i + t - q_i) · n̂_i]²
  n̂_i: q_i 주변 K=8 이웃으로 추정한 법선벡터

  효과: 평면에 접선 방향 오차 허용, 법선 방향만 제약
       → 회전 시 평면이 회전축에 직교 → 정확한 회전 추정
       → local minimum 대폭 감소
```

#### Force3DoF 제약
```
Reg/Force3DoF: true
→ R = Rz(θ) (z축 회전만 허용)
→ t = [tx, ty, 0] (x, y 이동만 허용)
→ 평면 주행 로봇에 최적화

vyaw = θ / dt
```

---

### 5.2 EKF 이중 오도메트리 전략

#### 단일 센서 한계

```
휠 오도메트리만:
  vx: ±2% 오차 (저속에서 양호)
  vyaw: 슬립 시 수십% 오차 → EKF yaw 발산

IMU만:
  LSB = 0.00107 rad/s
  정지 시 ±0.00107, ±0.00214 이산값
  → 200Hz × 30초 = 6,000 샘플 적분 → 최대 6.4 rad 오차

ICP 단독:
  4Hz 업데이트 → EKF 예측 후 보정 지연
  빠른 회전 시 발산 가능
```

#### 이중 소스 퓨전 설계

```
EKF 상태 벡터: [x, y, yaw, vx, vy, vyaw, ...]

odom0 (휠, 50Hz):
  관측: [vx]
  공분산: 낮음 (엔코더 신뢰)
  → x, y 추적에 기여
  → vyaw에는 영향 없음

odom1 (ICP, ~4Hz):
  관측: [vyaw]
  공분산: cmd_vel 상태에 따라 동적 조정
  → yaw 추적에 기여
  → vx에는 영향 없음

결과: 슬립 없는 vx + 양자화 없는 vyaw
     → 정확한 odom→base_link TF
```

---

### 5.3 icp_odom_cov_scale 동적 공분산 조정

#### 상태 머신

```
cmd_vel 모니터링:
  - 마지막 cmd_vel 수신 후 0.5초 이상 경과 → STOP 상태
  - |v| < 0.01 AND |w| < 0.01 → STOP 상태
  - |w| >= 0.01 → ROTATE 상태
  - |v| >= 0.01 → TRANSLATE 상태

공분산 스케일 적용:
  STOP:       cov[35] × 1000  → EKF가 ICP vyaw 무시
  ROTATE:     cov[35] × 1     → ICP PointToPlane 완전 신뢰
  TRANSLATE:  cov[35] × 10    → 부분 신뢰
```

#### 수학적 의미

EKF Kalman 게인: `K = P·H^T / (H·P·H^T + R)`
- R: 관측 노이즈 공분산 (= cov[35])
- R 크면 → K 작음 → 관측 영향 적음

SCALE=1000이면:
```
R_new = R_original × 1000
K_new ≈ K_original / 1000
혁신 항 기여: K × (z - h(x)) ≈ 0
→ EKF가 이 관측을 사실상 무시
```

---

### 5.4 LiDAR Deskewing

#### 문제: 스캔 중 모션 왜곡

Livox MID360의 스캔 주기는 약 50ms (20Hz). 이 동안 로봇이 이동하면:
- 스캔 시작 시 점: 현재 위치의 포인트
- 스캔 끝 시 점: 50ms 후 위치의 포인트
- 두 시점에 다른 좌표계 → 포인트클라우드가 "휜" 상태

#### 해결: base_link 기준 모션 보정

```python
# sensor_sync.launch.py
parameters=[{
    'fixed_frame_id': 'base_link',  # 핵심 설정
    'slerp': True,
}]
```

**왜 base_link인가** (odom이 아닌 이유):
```
odom 기준 deskew:
  TF: odom → base_link 사용
  odom 드리프트 포함 → 잘못된 보정
  → RTAB-Map이 이미 드리프트된 클라우드를 등록

base_link 기준 deskew:
  로봇의 실제 순수 모션만 반영
  → 정확한 포인트 위치 보정
  → RTAB-Map 그래프 최적화가 잔차 처리
```

---

### 5.5 PPO 기반 실시간 PID 게인 적응

#### PPO 알고리즘 개요

```
PPO (Proximal Policy Optimization):

  정책 업데이트:
    L(θ) = min(r_t(θ) × A_t, clip(r_t(θ), 1-ε, 1+ε) × A_t)
    r_t(θ) = π_θ(a_t|s_t) / π_θ_old(a_t|s_t)  # 확률 비율
    ε = 0.2  # 클리핑 범위
    A_t: 어드밴티지 추정

  특성:
    - 정책 업데이트 시 큰 변화 방지 (안전한 학습)
    - 실제 로봇 환경에서 학습 안정성 높음
```

#### 실제 동작 사이클

```
0.4초마다:

1. 관측 수집:
   e_v = v_ref - v_meas (선속도 오차)
   e_w = w_ref - w_meas (각속도 오차)
   obs = [v_ref, w_ref, v_meas, w_meas, e_v, e_w, rpm, curr, pulse]

2. 정책 추론:
   action, _states = model.predict(obs, deterministic=True)
   # PPO network forward pass

3. 게인 업데이트:
   for i in range(6):
       if abs(action[i]) > 0.35:  # 데드존
           delta = action[i] * scale
           gains[i] = clip(gains[i] + delta * 0.08, min, max)  # LPF

4. 파라미터 서비스 호출:
   controller_server/set_parameters(kp_lin=gains[0], ...)

5. 보상 계산:
   reward = -(abs(e_v) + abs(e_w))

6. CSV 로깅
```

#### 모터/엔코더 관측의 의미

```
motor_rpm_mean:
  낮음 + v_ref 높음 → 모터 슬립 또는 부하 증가
  → 정책이 ki_lin 증가로 반응

motor_current_mean:
  높음 → 모터 포화 상태
  → 정책이 kp_lin 감소로 반응 (과부하 방지)

encoder_pulse_rate_mean:
  순수 기계적 휠 회전 속도
  → IMU/오도메트리와 비교해 슬립 감지
```

---

### 5.6 RTAB-Map 그래프 최적화

#### 그래프 SLAM 구조

```
노드: 로봇 위치 스냅샷 (스캔 + 포즈)
엣지: 두 노드 간 ICP 변환 제약

그래프 최적화:
  minimize Σ_e ||f(T_i, T_j) - z_ij||²_Ω
  T_i, T_j: 노드 포즈
  z_ij: ICP 측정 변환
  Ω: 정보 행렬 (1/공분산)
```

#### Optimizer/Robust=true (Huber 비용함수)

```
표준 최소제곱:
  L(r) = r²
  모든 잔차 동등하게 → ICP 오류 제약도 동등한 힘으로 map 왜곡

Huber 비용함수:
  L(r) = r²         if |r| ≤ δ
  L(r) = 2δ|r| - δ² if |r| > δ

  효과: 큰 잔차(ICP 오류) → 선형 페널티로 하중 감소
       → map→odom 안정화
```

#### ProximityBySpace=false

```
ProximityBySpace=true (이전 설정):
  공간적으로 가까운 노드 간 자동 링크 생성
  제자리 회전 시: 같은 위치의 다른 시간 노드가 링크됨
  ICP 등록 오류 시 → 모순된 제약 → 맵 뒤틀림

ProximityBySpace=false (현재 설정):
  근접 링크 비활성화
  루프 클로저만 (높은 임계값 Rtabmap/LoopThr=0.50)
  → 모순 제약 방지 → 안정적인 맵
```

---

## 6. 시작 순서 및 실행 메커니즘

### run_all.sh 실행 순서

```bash
#!/bin/bash
# 로그 디렉토리 생성
LOG_DIR=~/ca_ws/logs/run_$(date +%Y%m%d_%H%M%S)
mkdir -p $LOG_DIR

# 1. SHM 정리 (mutex 데드락 방지)
rm -f /dev/shm/*

# 2. CAN 버스 초기화
sudo ip link set can1 up type can bitrate 500000

# 3. Livox LiDAR
ros2 launch livox_ros_driver2 msg_MID360_launch.py &

# 4. RealSense 카메라
ros2 launch realsense2_camera rs_launch.py &

# 5. GPS 브리지
ros2 launch ec25_gps_bridge ec25_gps_bridge.launch.py device:=/dev/ttyUSB3 &

# 6. Bunker 모터 (publish_odom_tf=false 필수)
ros2 launch bunker_base bunker_base.launch.py publish_odom_tf:=false &

# 7. 센서 동기화 파이프라인
ros2 launch rtabmap_launch sensor_sync.launch.py lidar_offset_sec:=0.003 &

# ===== EKF 시작 =====
# 8. EKF (odom→base_link TF 생성)
ros2 launch robot_localization ekf.launch.py &

# 9. TF 준비 대기
wait_for_tf odom base_link 15  # 15초 타임아웃

# ===== SLAM + Nav2 시작 =====
# 10. RTAB-Map + Nav2 + ICP 오도메트리
ros2 launch rtabmap_launch rtabmap_nav2.launch.py &

# 11. 주요 토픽 준비 대기
wait_for_topic /icp_odom 30
wait_for_topic /odometry/filtered 10
wait_for_tf map odom 15
wait_for_topic /global_costmap/costmap 15

# ===== Nav2 준비 완료 =====
# 12. RL PID 에이전트 시작
ros2 launch rl_pid_training agent_pid.launch.py \
  model:=/home/atoz/ca_ws/rl_pid_model_new \
  odom_topic:=/odometry/filtered \
  desired_cmd_topic:=/controller_server/RLController/desired_cmd \
  desired_cmd_type:=auto \
  motor_status_topic:=/bunker_status \
  use_motor_encoder_obs:=true &
```

### 시작 순서의 중요성

```
순서 위반 시 발생하는 문제:

EKF 이전에 rtabmap 시작:
  → ICP 오도메트리가 deskewing에 odom→base_link TF 필요
  → TF 없음 → deskewing 실패 → ICP 오도메트리 실패 → SLAM 실패

Nav2 이전에 RL PID 시작:
  → /controller_server/RLController/desired_cmd 토픽 없음
  → RL 에이전트 타임아웃 루프 → CPU 낭비

bunker_base에 publish_odom_tf=true:
  → bunker_base + EKF 모두 odom→base_link TF 발행
  → TF 충돌 → 모든 하위 시스템 불안정
```

---

## 7. 설정 파일 상세 분석

### EKF 설정 (ekf.yaml)

```yaml
# 핵심 설정
frequency: 20.0
sensor_timeout: 0.1
two_d_mode: true  # 2D 주행 모드

# 좌표계
map_frame: map
odom_frame: odom
base_link_frame: base_link
world_frame: odom
publish_tf: true

# 예측 모델
process_noise_covariance: [...]  # 프로세스 노이즈 (Q 행렬)
initial_estimate_covariance: [...]  # 초기 불확실성 (P0 행렬)

# 관측 모델 (odom0, odom1, imu0 구성 앞 절 참조)
predict_to_current_time: false  # 현재 시간으로 외삽 비활성화
```

### Nav2 파라미터 핵심 설정 (nav2_rtabmap_params.yaml)

```yaml
# 전역 경로 계획
planner_server:
  planner_plugins: [GridBased]
  GridBased:
    plugin: nav2_smac_planner/SmacPlanner2D
    tolerance: 0.25
    use_astar: false  # Dijkstra 사용

# 로봇 형태 (footprint)
local_costmap:
  footprint: "[[0.35, 0.20], [0.35, -0.20], [-0.35, -0.20], [-0.35, 0.20]]"

# 속도 평활화
velocity_smoother:
  smoothing_frequency: 30.0
  max_velocity: [0.25, 0.0, 1.0]  # [vx, vy, wz]
  max_accel: [0.25, 0.0, 0.8]
  max_decel: [-0.25, 0.0, -0.8]
  deadband_velocity: [0.0, 0.0, 0.02]  # 미세 회전 제거

# 로컬 플래너
controller_server:
  controller_plugins: [RLController]
  RLController:
    plugin: rl_local_controller/RLController
    # (앞 절 파라미터 참조)
```

---

## 8. 알려진 이슈 및 해결책

### 8.1 IMU 양자화 노이즈 ✅ 해결됨

**증상**: 정지 중에도 `odom→base_link` yaw가 미세하게 진동
**원인**: RealSense IMU LSB=0.00107 rad/s → 이산 각속도 값
**해결**: `imu0_config` 전부 false → IMU vyaw EKF 제외
**상태**: 완전 해결

---

### 8.2 휠 슬립으로 인한 yaw 오차 ✅ 해결됨

**증상**: 곡선 주행 시 `odom→base_link` yaw가 실제와 달라짐
**원인**: 바퀴 슬립 → 엔코더 오차 → yaw 누적
**해결**: `odom0_config` vyaw=false + `odom1_config` vyaw=true (ICP)
**상태**: 완전 해결

---

### 8.3 제자리 회전 시 맵 뒤틀림 ⚠️ 최근 수정 (재테스트 필요)

**증상**: 제자리 360° 회전 후 맵이 실제 환경과 불일치
**원인 분석**:
1. AngularUpdate=0.15 → 42개 노드 → ICP 오차 누적
2. PointToPoint ICP → 회전 시 local minimum → 부정확한 등록
3. 표준 최소제곱 → ICP 오류 제약이 맵 왜곡
4. ProximityBySpace=true → 모순 링크 생성

**적용된 해결책**:
- AngularUpdate: 0.15 → 1.0 (42 → 6 노드/360°)
- Icp/PointToPlane: false → true
- Optimizer/Robust: false → true
- RGBD/ProximityBySpace: true → false
- Rtabmap/LoopThr: 0.30 → 0.50

**현재 상태**: 적용 완료, 실주행 재테스트 필요

---

### 8.4 g2o 옵티마이저 미사용 ⚠️ 진행 중

**증상**: 시작 시 "g2o optimizer not available, TORO will be used" 경고
**영향**: TORO는 g2o보다 느리고 정확도가 낮음
**해결책**: g2o 소스 빌드 후 rtabmap_ros 재빌드 (README.md §11-4 참조)
**상태**: 문서화됨, 미구현

---

### 8.5 GPS 퓨전 미완성 ⏸️ 보류

**현재**: EC25 브리지 동작, `/gps/fix` 발행
**필요**: `navsat_transform_node` + 전역 EKF 구성
**계획**: 아웃도어 테스트 환경 구축 후 구현

---

## 9. 진단 도구 분석

**경로**: `~/ca_ws/tools/`

### plot_pid_policy.py

PID 학습 로그 시각화:
```bash
python3 ~/ca_ws/tools/plot_pid_policy.py \
  --csv ~/ca_ws/rl_pid_logs/pid_policy_<날짜>.csv \
  --show
```

**4-패널 출력**:
1. 선속도/각속도 추적 (v_ref vs v_meas, w_ref vs w_meas)
2. PID 게인 변화 (kp, ki, kd 시계열)
3. 모터 신호 (RPM, 전류, 펄스 레이트)
4. 보상 함수 값

---

### ekf_yaw_probe.py

EKF yaw 드리프트 분석:
```bash
python3 ~/ca_ws/tools/ekf_yaw_probe.py --duration 180 --rate 20
```

**기능**:
- `odom→base_link` yaw와 `/odometry/filtered` yaw 동시 기록
- 운동 분류: ROTATE / STRAIGHT / STOP
- 세그먼트별 yaw 변화량 계산
- CSV 저장: `~/ca_ws/logs/ekf_yaw_probe_*.csv`

**사용법**:
1. 정지 20초 → yaw 변화 측정 (기대값: ~0°)
2. 180° 회전 → yaw 변화 측정 (기대값: ~180°)
3. odom vs map 비교:
   - `ros2 run tf2_ros tf2_echo odom base_link` → EKF 문제 진단
   - `ros2 run tf2_ros tf2_echo map odom` → RTAB-Map 문제 진단

---

### lidar_camera_sync_probe.py

LiDAR-카메라 타임스탬프 오프셋 측정:
```bash
python3 ~/ca_ws/tools/lidar_camera_sync_probe.py \
  --camera-topic /camera/camera/aligned_depth_to_color/image_raw \
  --lidar-topic /livox/lidar/synced/deskewed \
  --current-lidar-offset 0.003 \
  --duration-sec 30
```

**출력**: 권장 오프셋 조정값

---

### bunker_motor_probe.py

실시간 모터 상태 모니터:
```bash
python3 ~/ca_ws/tools/bunker_motor_probe.py
```

---

## 10. 시스템 강점과 한계

### 강점

1. **이중 오도메트리 퓨전**
   - 휠 슬립 + IMU 양자화 두 문제를 동시에 극복
   - ICP PointToPlane → 회전 정확도 향상

2. **동적 공분산 조정**
   - 운동 상태에 따라 센서 신뢰도 자동 조정
   - 정지 시 드리프트 완전 방지

3. **실시간 RL PID 적응**
   - PPO 정책으로 환경/부하 변화에 자동 적응
   - 9차원 관측으로 슬립/포화 등 이상 상태 감지

4. **Robust 그래프 최적화**
   - Huber 비용함수로 ICP 오류 제약 하중 감소
   - 맵 왜곡 최소화

5. **좁은 통로 탈출**
   - costmap 클리어런스 측정으로 충돌 예방
   - 서행/후진 탈출 전략

6. **종합 진단 도구**
   - CSV 로깅, TF 프로브, yaw 분석
   - 이슈 재현 및 튜닝 지원

### 한계 및 개선 여지

1. **g2o 미사용**: TORO 대비 최적화 정확도/속도 열위
2. **GPS 퓨전 미완성**: 실내 SLAM 의존 → GPS 신호 환경에서 글로벌 포즈 불안정
3. **카메라 미활용**: RGB-D 카메라가 설치되어 있으나 SLAM에 비전 특징 미사용 (LiDAR만)
4. **캘리브레이션 드리프트**: 센서 마운팅 오프셋이 고정값 → 진동/열팽창에 미반응
5. **RL 모델 재학습 미자동화**: 환경 변화 시 수동 재학습 필요
6. **제자리 회전 맵 안정성**: 최근 수정 후 충분한 실주행 검증 필요

---

## 부록: 주요 파일 경로 참조표

| 역할 | 파일 경로 |
|------|-----------|
| 전체 실행 스크립트 | `~/ca_ws/run_all.sh` |
| EKF 설정 | `src/robot_localization/params/ekf.yaml` |
| Nav2 파라미터 | `src/rtabmap_ros/rtabmap_launch/launch/config/nav2_rtabmap_params.yaml` |
| 센서 동기화 런치 | `src/rtabmap_ros/rtabmap_launch/launch/sensor_sync.launch.py` |
| SLAM 런치 | `src/rtabmap_ros/rtabmap_launch/launch/rtabmap.launch.py` |
| SLAM+Nav2 런치 | `src/rtabmap_ros/rtabmap_launch/launch/rtabmap_nav2.launch.py` |
| ICP 공분산 스케일 | `src/rtabmap_ros/rtabmap_launch/scripts/icp_odom_cov_scale.py` |
| RL PID 추론 | `src/rl_pid_training/rl_pid_training/run_pid_policy.py` |
| RL PID 환경 | `src/rl_pid_training/rl_pid_training/rl_pid_env_real.py` |
| RL PID 런치 | `src/rl_pid_training/launch/agent_pid.launch.py` |
| 로컬 플래너 | `src/rl_local_controller/src/rl_local_controller.cpp` |
| 지형 레이어 | `src/traversability_layer/src/traversability_layer.cpp` |
| IMU 보정 | `src/camera_imu_pipeline_cpp/src/camera_imu_bias_corrector.cpp` |
| GPS 브리지 | `src/ec25_gps_bridge/ec25_gps_bridge/ec25_navsat_bridge.py` |
| 번커 베이스 | `src/bunker_ros2/bunker_base/src/bunker_base_ros.cpp` |
| LiDAR 오프셋 | `src/livox_timestamp_offset/livox_timestamp_offset/livox_timestamp_offset_node.py` |
| PID 로그 시각화 | `tools/plot_pid_policy.py` |
| EKF yaw 진단 | `tools/ekf_yaw_probe.py` |
| LiDAR 동기 측정 | `tools/lidar_camera_sync_probe.py` |
| RL PID 학습 모델 | `~/ca_ws/rl_pid_model_new.zip` |
| PID 로그 디렉토리 | `~/ca_ws/rl_pid_logs/` |

---

*보고서 생성: 2026-03-09, Claude Code (claude-sonnet-4-6)*
