# ca_ws 실행/튜닝 가이드 (2026-03-05 업데이트)

이 문서는 반영된 변경사항 기준으로 정리합니다.

- 전체 실행(run_all)
- EC25 GPS -> /gps/fix 브리지
- Livox 시간동기(오프셋/딜레이 확인)
- TF 튐(오른쪽 점프) 원인과 해결
- 시뮬 실행(헤드리스/GUI)
- 센서 마운트/footprint 최신값
- Agent PID(9차원 관측) 실행
- 부드러운 모터 제어 파라미터(Agent PID + velocity_smoother)
- 좁은 통로 제자리 회전 방지(escape)
- PID CSV 시각화
- RealSense IMU(HID/IIO) 커널 모듈 빌드/설치
- 맵 뒤틀림(회전 드리프트) 이슈 대응 기록
- 제자리 회전 시 로컬/글로벌 맵 불일치 해결 (2026-02-28)
- **IMU 바이어스 보정 개선 / ICP odometry 활성화 / 맵 뒤틀림 근본 해결 (2026-03-05 신규)**
- **odom→base_link TF yaw 드리프트 해결 / 제자리 회전 map 불안정 개선 (2026-03-06 신규)**

## 1) 빌드

```bash
cd ~/ca_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source ~/ca_ws/install/setup.bash
```

## 2) 전체 실행 (권장)

```bash
cd ~/ca_ws
source /opt/ros/humble/setup.bash
source ~/ca_ws/install/setup.bash
bash ~/ca_ws/run_all.sh
```

실행 구성:
- Livox (`msg_MID360_launch.py`, publish_freq=20Hz)
- RealSense
- EC25 GPS bridge (`/dev/ttyUSB3` -> `/gps/fix`)
- bunker_base
- sensor_sync (IMU 보정 + Madgwick + LiDAR deskew + static TF)
- EKF (`robot_localization`)
- RTAB-Map + Nav2
- agent_pid

## 2-1) EC25 GPS 토픽 브리지 단독 실행

RTAB-Map 기본 GPS 입력은 `/gps/fix` 입니다. EC25를 이 토픽으로 발행합니다.
또한 `base_link -> gps_link` 정적 TF를 같이 publish해야 GPS 오프셋 보정이 됩니다.

```bash
source /opt/ros/humble/setup.bash
source ~/ca_ws/install/setup.bash

ros2 launch ec25_gps_bridge ec25_gps_bridge.launch.py \
  device:=/dev/ttyUSB3 \
  topic_name:=/gps/fix \
  frame_id:=gps_link \
  parent_frame:=base_link \
  publish_static_tf:=true \
  gps_x:=0.3 gps_y:=0.0 gps_z:=0.35
```

확인:
```bash
ros2 topic echo /gps/fix --once
```

## 3) TF 튐(오른쪽 점프) 해결

원인:
- `odom -> base_link`를 `bunker_base`와 `EKF`가 동시에 publish하면 TF가 간헐적으로 튑니다.

적용된 수정:
- `bunker_base.launch.py`에 `publish_odom_tf` 인자 추가
- 기본값: `false`

권장 실행:
- EKF를 사용할 때:
```bash
ros2 launch bunker_base bunker_base.launch.py publish_odom_tf:=false
```
- EKF 없이 base odom TF만 쓸 때:
```bash
ros2 launch bunker_base bunker_base.launch.py publish_odom_tf:=true
```

확인:
```bash
ros2 topic info /tf -v
```
`odom->base_link`는 한 노드만 publish해야 정상입니다.

## 4) 시뮬 실행 (Gazebo)

`bunker_gz.launch.py`가 `gui` 인자를 지원합니다.

- 헤드리스(기본):
```bash
ros2 launch bunker_sim bunker_gz.launch.py gui:=false
```
- GUI 표시:
```bash
ros2 launch bunker_sim bunker_gz.launch.py gui:=true
```

브리지 토픽:
- `/clock`
- `/cmd_vel`
- `/odom`
- `/tf`

## 5) 센서 마운트/동기화 최신 기본값

`sensor_sync.launch.py` 기준:
- Livox: `x=0.3, y=0.0, z=0.63`
- Camera: `x=0.3, y=0.0, z=0.55`
- LiDAR timestamp offset default: `0.003`

실행 예:
```bash
ros2 launch rtabmap_launch sensor_sync.launch.py
```

`imu_filter_madgwick` 패키지가 반드시 필요합니다.

## 5-1) Livox 시간동기(딜레이) 점검/튜닝

시간동기 파이프라인:
- `/livox/lidar` -> `livox_timestamp_offset_node` -> `/livox/lidar/synced`
- `/livox/lidar/synced` -> `rtabmap_util/lidar_deskewing` -> `/livox/lidar/synced/deskewed`

딜레이 확인:
```bash
ros2 topic delay /livox/lidar
ros2 topic delay /livox/lidar/synced
ros2 topic delay /livox/lidar/synced/deskewed
```

오프셋 조정 실행:
```bash
ros2 launch rtabmap_launch sensor_sync.launch.py lidar_offset_sec:=0.003
```

가이드:
- `/livox/lidar/synced`는 1~5ms 수준이면 양호
- `/livox/lidar/synced/deskewed`는 deskew 처리로 수~수십 ms 지연이 정상

라이다-카메라 상대 타임스탬프(헤더) 정합 측정:
```bash
python3 ~/ca_ws/tools/lidar_camera_sync_probe.py \
  --camera-topic /camera/camera/aligned_depth_to_color/image_raw \
  --lidar-topic /livox/lidar/synced/deskewed \
  --current-lidar-offset 0.003 \
  --duration-sec 30
```

출력의 `recommended lidar_offset_sec (median)` 값을 다음 실행에 반영:
```bash
ros2 launch rtabmap_launch sensor_sync.launch.py lidar_offset_sec:=<추천값>
```

## 6) Nav2 footprint 최신값 (Bunker 규격 반영)

현재 적용값:
```text
[[0.5115, 0.389], [0.5115, -0.389], [-0.5115, -0.389], [-0.5115, 0.389]]
```

반영 파일:
- `src/rtabmap_ros/rtabmap_launch/launch/config/nav2_rtabmap_params.yaml`
- `src/rtabmap_ros/rtabmap_launch/launch/config/nav2_rtabmap_params_train.yaml`

## 7) Agent PID 실행 (실기체, 9차원 관측)

오늘 수정으로 `RealPidGainEnv`가 9차원 관측을 지원합니다.

- 기존 6차원: `v_ref, w_ref, v_meas, w_meas, e_v, e_w`
- 추가 3차원: `motor_rpm_mean, motor_current_mean, encoder_pulse_rate_mean`

실행:
```bash
source /opt/ros/humble/setup.bash
source ~/ca_ws/install/setup.bash

ros2 launch rl_pid_training agent_pid.launch.py \
  model:=/home/atoz/ca_ws/rl_pid_model_new \
  odom_topic:=/odometry/filtered \
  desired_cmd_topic:=/controller_server/RLController/desired_cmd \
  desired_cmd_type:=auto \
  motor_status_topic:=/bunker_status \
  use_motor_encoder_obs:=true
```

CSV 로그 기본 경로:
```text
/home/atoz/ca_ws/rl_pid_logs/pid_policy_*.csv
```

## 7-1) 부드러운 모터 제어 파라미터(현재 적용값)

### Agent PID (gain 변화/저속 떨림 완화)
`src/rl_pid_training/launch/agent_pid.launch.py`

- `step_dt:=0.6`
- `gain_scale:=0.08`
- `gain_lpf_alpha:=0.08`
- `action_deadzone:=0.35`
- `w_ref_lpf_alpha:=0.10`
- `w_ref_deadband:=0.08`
- `dither_w_ref_thresh:=0.20`
- `w_ref_sign_hold_sec:=0.50`
- `w_ref_abs_max_low_speed:=0.30`

### Nav2 velocity_smoother (실기체)
`src/rtabmap_ros/rtabmap_launch/launch/config/nav2_rtabmap_params.yaml`

- `smoothing_frequency: 30.0`
- `feedback: "OPEN_LOOP"`
- `max_accel: [0.25, 0.0, 0.8]`
- `max_decel: [-0.25, 0.0, -0.8]`
- `deadband_velocity: [0.0, 0.0, 0.02]`

## 7-2) 좁은 통로 제자리 회전 방지(충돌 완화)

`rl_local_controller`에서 제자리 회전 전에 여유공간을 체크하고, 부족하면 탈출 동작을 수행합니다.

반영 파일:
- `src/rl_local_controller/src/rl_local_controller.cpp`
- `src/rtabmap_ros/rtabmap_launch/launch/config/nav2_rtabmap_params.yaml`
- `src/rtabmap_ros/rtabmap_launch/launch/config/nav2_rtabmap_params_train.yaml`

현재 파라미터:
- `rotate_min_side_clearance: 0.42`
- `rotate_min_front_clearance: 0.32`
- `escape_forward_speed: 0.05`
- `escape_forward_turn_scale: 0.6`
- `escape_use_reverse: true`
- `escape_reverse_speed: 0.03`

동작:
- 좌/우 및 전방 여유가 충분할 때만 제자리 회전
- 부족하면 전진 우선 탈출
- 전진 불가 시 후진 허용(`escape_use_reverse`)

## 8) PID 로그 시각화

추가된 스크립트:
- `tools/plot_pid_policy.py`

실행:
```bash
python3 /home/atoz/ca_ws/tools/plot_pid_policy.py \
  --csv /home/atoz/ca_ws/rl_pid_logs/pid_policy_20260226_162306.csv
```

옵션:
- `--show` : 화면에 인터랙티브 표시
- `--out <png>` : 출력 파일 경로 지정

출력 그래프:
- v/w 참조-실측
- PID gains
- motor/encoder signals
- reward(+moving average)

## 9) RealSense IMU(HID/IIO) 커널 모듈 빌드/설치

아래는 Jetson에서 HID IMU를 위해 사용한 절차입니다.

### 9-1. 빌드

```bash
cd /home/atoz/linux-jammy

zcat /proc/config.gz > .config
make olddefconfig

scripts/config --module CONFIG_HID_SENSOR_HUB
scripts/config --module CONFIG_HID_SENSOR_IIO_COMMON
scripts/config --module CONFIG_HID_SENSOR_TRIGGER
scripts/config --module CONFIG_HID_SENSOR_ACCEL_3D
scripts/config --module CONFIG_HID_SENSOR_GYRO_3D
scripts/config --module CONFIG_IIO_TRIGGERED_BUFFER
make olddefconfig

cp /lib/modules/$(uname -r)/build/Module.symvers .
make -j"$(nproc)" modules_prepare

make -j"$(nproc)" M=drivers/hid modules
make -j"$(nproc)" M=drivers/iio/common/hid-sensors modules
make -j"$(nproc)" M=drivers/iio/accel modules
make -j"$(nproc)" M=drivers/iio/gyro modules
```

### 9-2. 설치/로드

```bash
sudo install -D -m 644 /home/atoz/linux-jammy/drivers/hid/hid-sensor-hub.ko \
  /lib/modules/$(uname -r)/updates/drivers/hid/hid-sensor-hub.ko
sudo install -D -m 644 /home/atoz/linux-jammy/drivers/iio/common/hid-sensors/hid-sensor-iio-common.ko \
  /lib/modules/$(uname -r)/updates/drivers/iio/common/hid-sensors/hid-sensor-iio-common.ko
sudo install -D -m 644 /home/atoz/linux-jammy/drivers/iio/common/hid-sensors/hid-sensor-trigger.ko \
  /lib/modules/$(uname -r)/updates/drivers/iio/common/hid-sensors/hid-sensor-trigger.ko
sudo install -D -m 644 /home/atoz/linux-jammy/drivers/iio/accel/hid-sensor-accel-3d.ko \
  /lib/modules/$(uname -r)/updates/drivers/iio/accel/hid-sensor-accel-3d.ko
sudo install -D -m 644 /home/atoz/linux-jammy/drivers/iio/gyro/hid-sensor-gyro-3d.ko \
  /lib/modules/$(uname -r)/updates/drivers/iio/gyro/hid-sensor-gyro-3d.ko

sudo depmod -a

sudo modprobe hid_sensor_hub
sudo modprobe industrialio_triggered_buffer
sudo modprobe hid_sensor_iio_common
sudo modprobe hid_sensor_trigger
sudo modprobe hid_sensor_accel_3d
sudo modprobe hid_sensor_gyro_3d
```

부팅 시 자동 로드하려면 `/etc/modules-load.d/`에 모듈 목록 파일을 추가하세요.

## 10) 체크리스트 (PID 로그 품질)

좋은 로그 기준:
- `v_ref`가 변할 때 `v_meas`도 유사하게 변함
- `w_ref`가 변할 때 `w_meas`도 유사하게 변함
- 명령 활성 구간에서 `motor_rpm_mean`, `encoder_pulse_rate_mean`이 대부분 0이 아님
- reward 평균이 0에 가까워짐(덜 음수)

나쁜 로그 신호:
- 명령은 큰데 실측이 거의 0
- 모터/엔코더가 장시간 0 고정
- 엔코더 rate의 비정상 스파이크 과다

## 11) 맵 뒤틀림(회전 드리프트) 이슈 대응 기록

증상:
- 제자리 회전 또는 코너 구간 이후, 동일 벽이 기울어져 중첩되며 맵이 비틀린 것처럼 보임
- local/global costmap 문제처럼 보일 수 있으나, 실제로는 yaw 추정 + 정합(등록) + 시간동기 영향이 겹친 경우가 많음

### 11-1) 반영/확인한 항목

TF/상태추정:
- `bunker_base`의 `publish_odom_tf:=false` 유지 (중복 TF publish 방지)
- EKF가 `odom -> base_link` 단일 publish (`publish_tf: true`)
- EKF yaw 입력 정리:
- `odom0_config`: `yaw=false`, `vyaw=false`
- `imu0_config`: `yaw=false`, `vyaw=true`

IMU/센서 프레임:
- `imu_bias_corrector` 출력 프레임을 `base_link`로 통일
- `/camera/camera/imu_fixed`의 `frame_id=base_link` 확인
- 센서 정적 TF 실측 반영:
- `base_link -> livox_frame`: `x=0.30, y=0.00, z=0.63`
- `base_link -> camera_link`: `x=0.30, y=0.00, z=0.55`

RTAB-Map 파라미터(드리프트 완화용):
- `RGBD/ProximityBySpace=true`
- `RGBD/ProximityMaxGraphDepth=15`
- `RGBD/LinearUpdate=0.10` _(0.15에서 변경, 2026-02-28)_
- `RGBD/AngularUpdate=0.05` _(0.20에서 변경, 2026-02-28)_
- `Rtabmap/DetectionRate=5.0` _(3.0에서 변경, 2026-02-28)_
- `Rtabmap/LoopThr=0.30`
- `RGBD/OptimizeMaxError=0.3`
- `Reg/Force3DoF=true`
- `Optimizer/Strategy=1` 설정
- `Icp/PointToPlane=true` _(신규 추가, 2026-02-28)_
- `Icp/PointToPlaneK=8` _(신규 추가, 2026-02-28)_

### 11-2) 추가 진단 스크립트

`tools/ekf_yaw_probe.py`:
- 입력 비교: `/odometry/filtered`(EKF), `/odom`(wheel), `/camera/camera/imu_fixed`(IMU), `/cmd_vel`
- 출력: `~/ca_ws/logs/ekf_yaw_probe_*.csv`
- 목적: 제자리 회전 구간에서 EKF yaw/wz가 IMU 대비 얼마나 벗어나는지 수치로 확인

실행:
```bash
python3 ~/ca_ws/tools/ekf_yaw_probe.py --duration 180 --rate 20
```

### 11-3) 현재 로그에서 확인된 핵심 원인

- RTAB-Map 런타임에서 `g2o optimizer not available. TORO will be used instead.` 확인
- 일부 세션에서 `Messages ... arrived out of order` 경고 확인 (RGBD 동기 불안정 신호)
- 초기 기동 직후 `dynamic_object_filter`의 `livox_frame -> odom` TF 실패가 1회 발생할 수 있음 (초기화 타이밍 이슈)

### 11-4) 우선순위 조치

1. g2o 설치 후 RTAB-Map 재빌드로 `Strategy=1` 실제 적용 확인
2. 맵 안정화 1차 검증은 LiDAR-only (`rgbd_sync:=false`, `subscribe_rgbd:=false`)로 A/B 테스트
3. RGBD 재투입 전, out-of-order 경고와 센서 타임스탬프 정합 재확인
4. ~~deskew 고정 프레임 A/B 테스트~~ → **`base_link`로 확정 적용 (2026-02-28)**

g2o 소스 빌드/재빌드 예시:
```bash
sudo apt update
sudo apt install -y build-essential cmake git libeigen3-dev libsuitesparse-dev

mkdir -p ~/third_party && cd ~/third_party
git clone https://github.com/RainerKuemmerle/g2o.git
cd g2o
cmake -S . -B build \
  -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_SHARED_LIBS=ON \
  -DG2O_BUILD_APPS=OFF \
  -DG2O_BUILD_EXAMPLES=OFF \
  -DG2O_USE_OPENGL=OFF \
  -DG2O_USE_CHOLMOD=ON
cmake --build build -j"$(nproc)"
sudo cmake --install build
sudo ldconfig

cd ~/ca_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install \
  --packages-up-to rtabmap_ros \
  --cmake-clean-cache \
  --cmake-args \
    -DCMAKE_BUILD_TYPE=Release \
    -DWITH_G2O=ON \
    -DG2O_DIR=/usr/local/lib/cmake/g2o
source ~/ca_ws/install/setup.bash
```

## 12) 제자리 회전 시 로컬/글로벌 맵 불일치 해결 (2026-02-28)

### 증상

자율주행 중 제자리 회전(in-place rotation) 이후 로컬 맵이 누적 글로벌 맵과 어긋남.

### 원인 분석 (3가지 중첩)

**원인 1 — `RGBD/AngularUpdate=0.20` (11.5°) 너무 큼 (주원인)**
- 11.5°마다 맵 노드 1개 생성 → 큰 각도 변화를 ICP 1번으로 매칭해야 함
- 순수 회전 상황에서 ICP가 로컬 미니멈에 빠져 정합 실패

**원인 2 — SLAM 등록용 ICP에 PointToPlane 없음**
- `odom_args`(ICP 오도메트리)에는 설정되어 있었으나, SLAM 노드 등록(`Reg/Strategy=2`)용 ICP에는 누락
- Point-to-Point ICP는 순수 회전에서 수렴 불안정

**원인 3 — EKF 갱신 주기가 낮아 deskewing 오차 발생**
- `frequency=20Hz` → TF가 50ms 단위로 갱신
- Deskewing 노드가 각 LiDAR 포인트 타임스탬프로 TF 조회 시 보간 정밀도 부족
- `predict_to_current_time=false` → RTAB-Map이 TF 조회할 때 stale pose 반환

### 적용된 수정

**`src/rtabmap_ros/rtabmap_launch/launch/rtabmap.launch.py`**

| 파라미터 | 변경 전 | 변경 후 |
|----------|---------|---------|
| `RGBD/AngularUpdate` | `0.20` | `0.05` |
| `RGBD/LinearUpdate` | `0.15` | `0.10` |
| `Rtabmap/DetectionRate` | `3.0` | `5.0` |
| `Icp/PointToPlane` | 없음 | `true` |
| `Icp/PointToPlaneK` | 없음 | `8` |

**`src/rtabmap_ros/rtabmap_launch/launch/sensor_sync.launch.py`**

| 항목 | 변경 전 | 변경 후 |
|------|---------|---------|
| Deskewing `fixed_frame_id` | `odom` | `base_link` |

> 주석엔 "base_link 사용"이라 적혀있었으나 코드는 `odom`으로 방치된 버그 수정.
> 제자리 회전 중 EKF yaw 오차가 deskewing에 전파되던 문제 차단.

**`src/robot_localization/params/ekf.yaml`**

| 파라미터 | 변경 전 | 변경 후 |
|----------|---------|---------|
| `frequency` | `20.0` | `50.0` |
| `predict_to_current_time` | `false` | `true` |

> `frequency` 50Hz: TF 보간 정밀도 50ms → 20ms. deskewing 품질 향상.
> `predict_to_current_time true`: RTAB-Map/deskewing TF 조회 시 IMU를 현재 시점까지 추가 적분해서 최신 pose 반환.

### 재실행 방법

재빌드 불필요 (파라미터/런치 파일만 변경).

```bash
bash ~/ca_ws/run_all.sh
```

---

## 13) IMU 바이어스 보정 개선 / ICP Odometry 활성화 (2026-03-05)

### 13-1) IMU 양자화 문제 발견 및 결론

RealSense D455와 Livox MID360 IMU 모두 각속도 LSB = **~0.00107 rad/s** 로 동일한 양자화 한계가 있습니다.

- 정지 중에도 IMU z값이 이산적 값(0, ±0.00107, ±0.00214 ...)만 출력
- EKF `vyaw` 입력으로 사용하면 양자화 스텝이 적분되어 드리프트 발생
- **결론: IMU vyaw는 EKF에서 비활성화 (`imu0_config vyaw=false`)**

### 13-2) IMU Bias Corrector EMA 지속 재교정 추가

**`src/camera_imu_pipeline_cpp/src/camera_imu_bias_corrector.cpp`**

초기 1000샘플 평균 교정 완료 후, 정지 상태에서 EMA로 bias를 지속 추적합니다.

```cpp
// 파라미터 추가
continuous_calib: true   // 정지 중 EMA로 bias 지속 업데이트
ema_alpha: 0.001         // 시정수 ≈ 20초 (50Hz × 1000 스텝)

// EMA 업데이트 (정지 감지 후)
bias = (1 - alpha) * bias + alpha * current
```

**`src/rtabmap_ros/rtabmap_launch/launch/sensor_sync.launch.py`**
- `publish_during_calib: False` — 교정 완료 전 메시지 미발행
- `continuous_calib: True`
- `ema_alpha: 0.001`

### 13-3) ROR 필터 완화 (Loop Closure 복구)

**`src/rtabmap_ros/rtabmap_launch/launch/rtabmap_nav2.launch.py`**

| 파라미터 | 변경 전 | 변경 후 |
|----------|---------|---------|
| `ror_radius` | `0.25` | `0.20` |
| `ror_min_neighbors` | `4` | `2` |

원인: radius=0.25, neighbors=4 설정이 sparse한 포인트클라우드(10~60개)를 통째로 제거해 RTAB-Map loop closure 불가 → 맵 뒤틀림 발생.

### 13-4) RGBD/OptimizeFromGraphEnd 변경

**`src/rtabmap_ros/rtabmap_launch/launch/rtabmap.launch.py`**

```
RGBD/OptimizeFromGraphEnd: true → false
```

- `true`: loop closure 시 현재 위치 고정, 맵이 뒤틀림
- `false`: origin 기준 최적화, loop closure 시 로봇 위치가 보정됨 (맵 안정)

### 13-5) ICP Odometry 활성화 (yaw 드리프트 근본 해결)

IMU 양자화로 yaw를 신뢰할 수 없어 **LiDAR 스캔 매칭(ICP)으로 yaw를 직접 계산**합니다.
휠 슬립, IMU bias/양자화 모두 우회합니다.

#### 아키텍처 변경

```
이전:  /odom(휠) + IMU → EKF → /odometry/filtered
이후:  /odom(휠) + /icp_odom(LiDAR vyaw) → EKF → /odometry/filtered
```

#### EKF 설정 (`src/robot_localization/params/ekf.yaml`)

```yaml
# ICP odometry 입력 추가
odom1: /icp_odom
odom1_config: [false, false, false, false, false, false,
               false, false, false, false, false,
               true,   # vyaw ← LiDAR ICP 기반, 슬립/bias 없음
               false, false, false]

# IMU vyaw 비활성화 (양자화 노이즈로 드리프트 불가피)
imu0_config: [false×11, false, false, false, false]  # vyaw=false
```

#### ICP Odometry 파라미터 (`rtabmap_nav2.launch.py`)

```
--Reg/Force3DoF true           # 평면 3자유도 제한 (z 발산 방지)
--Icp/VoxelSize 0.10
--Icp/PointToPlane 0
--Icp/MaxCorrespondenceDistance 0.5
--Icp/MaxTranslation 2.0       # 기본값 0.2m → 초기 수렴 허용
--Icp/MaxRotation 6.28         # 기본값 0.78rad → 제한 완화
--Icp/CorrespondenceRatio 0.01 # 실내 sparse 환경 대응
--Odom/GuessMotion true
--Odom/ResetCountdown 5        # 5프레임 연속 실패 시 리셋
```

#### 데드락 해결 (`rtabmap.launch.py`, `rtabmap_nav2.launch.py`)

```
문제:
  icp_odometry → /livox/lidar/static_filtered 필요
  dynamic_filter → odom TF 필요 → EKF 필요 → /icp_odom 필요 → 데드락

해결:
  icp_odometry → /livox/lidar/synced/deskewed (원본, TF 불필요)
  rtabmap     → /livox/lidar/static_filtered  (dynamic filter 결과)
```

관련 파라미터:
- `icp_odom_scan_topic: /livox/lidar/synced/deskewed`
- `odom_guess_frame_id: ''` — guess_from_tf 비활성화
- `odom` remapping → `/icp_odom` — icp_odometry가 `/icp_odom`으로 직접 발행

#### 실행 순서 변경 (`run_all.sh`)

```
변경 전: rtabmap_nav2 → wait /icp_odom → EKF
변경 후: EKF → wait odom→base_link TF → rtabmap_nav2 → wait /icp_odom
```

EKF를 먼저 실행해 `odom→base_link` TF를 만들어야 icp_odometry가 정상 동작합니다.

### 13-6) 트러블슈팅 기록 (ICP odometry 디버깅 과정)

| 증상 | 원인 | 해결 |
|------|------|------|
| Publisher count=0 | `odom` remapping이 `/odometry/filtered`로 되어 있어 `/icp_odom`으로 미발행 | `("odom", "/icp_odom")`으로 수정 |
| `ICP correction too large` (limits=0.2m) | `Icp/MaxTranslation` 기본값 0.2m | `--Icp/MaxTranslation 2.0` 추가 |
| `guess_from_tf abort` | `odom_guess_frame_id='odom'` 기본값 → odom TF 없어 abort | `odom_guess_frame_id: ''` 설정 |
| `null guess` error | `Odom/GuessMotion false` → 2프레임 이후 guess=null | `Odom/GuessMotion true`로 복원 |
| z/roll/pitch 발산 | 3D ICP가 평면 로봇에 6자유도 최적화 → z 발산 누적 | `--Reg/Force3DoF true` 추가 |
| corrRatio 미달 | `Icp/CorrespondenceRatio 0.1` 기본값이 실내 sparse 환경에 과도 | `--Icp/CorrespondenceRatio 0.01` |
| `Icp/Strategy 1` 미지원 | libpointmatcher 없이 빌드됨 | Strategy 0(기본 PCL ICP)으로 변경 |

### 13-7) ICP Odometry 쉬운 이해 (개념 설명)

#### 왜 IMU만으로는 회전을 믿을 수 없나?

IMU(자이로 센서)는 "지금 얼마나 빠르게 회전하고 있어?"를 측정합니다.
그런데 RealSense D455, Livox MID360 IMU 모두 **최소 측정 단위(LSB)가 너무 커서**
실제로는 0.0005 rad/s로 돌고 있어도 0 또는 0.00107 rad/s 둘 중 하나만 출력합니다.

> 비유: 소수점 3자리까지 재야 하는데, 1자리밖에 없는 눈금자로 재는 것

정지해 있어도 숫자가 0과 0.00107 사이를 오락가락해서 → **시간이 지날수록 yaw 오차 누적**.

#### ICP Odometry가 하는 일

LiDAR는 매 순간 주변을 레이저로 스캔해서 "풍경 사진"을 찍습니다.
ICP(Iterative Closest Point)는 **직전 사진과 현재 사진을 비교**해서 "로봇이 얼마나 움직였는지"를 역산합니다.

```
[0.1초 전 스캔]  →  ICP 매칭  →  "얼마나 회전했나?" 계산  →  /icp_odom 발행
[현재 스캔]      ↗
```

- 정지 중: 두 사진이 똑같음 → 회전 없음 → 드리프트 없음
- 회전 중: 사진이 얼마나 돌아갔는지 계산 → 정확한 yaw rate

#### 수정한 파일과 이유

| 파일 | 수정 내용 | 이유 |
|------|----------|------|
| `rtabmap.launch.py` | icp_odometry `odom` remapping → `/icp_odom` | 주소가 잘못되어 결과가 엉뚱한 곳으로 가고 있었음 |
| `rtabmap_nav2.launch.py` | `icp_odom_scan_topic` = deskewed 원본 | 데드락 방지: filtered 토픽은 odom TF가 있어야 생성됨 |
| `rtabmap_nav2.launch.py` | `Reg/Force3DoF true` 추가 | 3D ICP가 z/roll/pitch까지 최적화해서 발산 |
| `ekf.yaml` | `odom1: /icp_odom`, `vyaw=true` | ICP yaw rate를 EKF에 입력으로 추가 |
| `ekf.yaml` | `imu0` vyaw → `false` | IMU vyaw는 양자화 문제로 비활성화 |
| `run_all.sh` | EKF 먼저 실행 → odom TF 대기 → rtabmap | icp_odom이 odom TF를 필요로 해서 순서 변경 |

#### 핵심 버그: "편지 주소 오류"

```python
# 수정 전 (잘못됨): icp_odometry가 /odometry/filtered 로 publish
("odom", "/odometry/filtered")

# 수정 후 (올바름): icp_odometry가 /icp_odom 으로 publish
("odom", "/icp_odom")
```

ROS2에서 `remappings`는 **subscribe와 publish 모두 remapping**합니다.
`odom`이라는 이름으로 보내려 했는데, 이미 `/odometry/filtered`로 redirect되어 있어서
EKF 출력 토픽을 덮어쓰는 문제가 있었습니다.

---

## 14) odom→base_link TF yaw 드리프트 해결 (2026-03-06)

### 증상 및 목표

| 문제 | 원인 레이어 | 해결 방향 |
|------|-------------|-----------|
| 정지 중 odom yaw가 천천히 드리프트 | EKF가 ICP vyaw(정지 sentinel=9999)를 그대로 신뢰 | 정지 감지 시 ICP vyaw 무시 |
| 제자리 회전 시 RViz에서 맵이 로봇 주위 회전 | map→odom TF 불안정 (RTAB-Map ProximityBySpace 모순 링크) | ProximityBySpace 비활성화 |
| 회전 중 ICP yaw 추적 정확도 부족 | PointToPoint ICP — 순수 회전 수렴 불안정 | PointToPlane ICP 활성화 |

### 14-0) ICP Odometry가 드리프트를 막는 원리 (알고리즘)

#### 기존 방법의 드리프트 원인

| 소스 | 드리프트 원인 |
|------|--------------|
| 휠 encoder vyaw | 트랙 슬립 → 실제 회전각 ≠ encoder 계산값. 스케일 오차 누적 |
| IMU gyro vyaw | LSB=0.00107 rad/s 양자화 → 적분 시 계단식 드리프트 |

#### ICP Odometry (F2F) 동작 방식

```
시각 t-1: 스캔 P_{t-1}   (N개 포인트)
시각 t:   스캔 P_t       (M개 포인트)

목표: P_{t-1}에서 P_t로 가는 변환 [R|t]를 구함
      → R(3×3 회전행렬)에서 yaw 각속도 추출
```

**PointToPlane ICP 최소화 공식** (`--Icp/PointToPlane 1`):

```
minimize Σᵢ [ (R·pᵢ + t − qᵢ) · n̂ᵢ ]²

  pᵢ : P_t의 포인트 i
  qᵢ : P_{t-1}에서 pᵢ의 대응점 (KD-tree 최근접)
  n̂ᵢ : qᵢ 주변 K=8개 이웃으로 추정한 법선벡터 (PointToPlaneK=8)
  R, t : 구하려는 회전/이동
```

PointToPoint(`n̂=1`)와 달리 **법선 방향 성분만** 최소화하므로, 순수 회전 시 평행 이동 방향 오차가 목적함수에 포함되지 않아 수렴이 안정적입니다.

**반복 과정** (Iterative):
```
1. 초기 guess = 이전 프레임 상대 움직임 (--Odom/GuessMotion true)
2. 대응점 탐색: MaxCorrespondenceDistance=0.3m 이내 최근접 점
3. 법선벡터 추정: 각 대응점 주변 K=8 이웃
4. 선형화된 최소제곱으로 dR, dt 계산
5. 포인트 변환 후 수렴 체크 (MaxIterations=30, Epsilon=0.001)
6. 수렴 시 [R|t] 확정
```

**vyaw 추출**:
```python
# 2D 강제 (--Reg/Force3DoF true): z, roll, pitch 고정
# R = Rz(θ)  →  θ = atan2(R[1,0], R[0,0])
# vyaw = θ / dt   (단위: rad/s)
```

#### 왜 드리프트가 없는가?

```
ICP는 "지금 스캔"과 "0.25초 전 스캔"을 물리적으로 비교합니다.
두 스캔 사이에 로봇이 얼마나 회전했는지를 포인트 구름의 기하학에서 직접 측정합니다.

  → 바퀴가 미끄러져도: LiDAR 스캔은 실제 환경을 봄 → 실제 회전각 계산
  → IMU가 양자화되어도: LiDAR는 무관하게 독립적 측정
```

단, F2F(Frame-to-Frame) 방식의 한계:
- 매 프레임(0.25초)의 매칭 오차가 누적됨 (`±0.003 rad/프레임 × 4Hz = 최대 ±0.72°/분`)
- 이를 공분산 스케일(icp_odom_cov_scale.py)로 보완하고, RTAB-Map 루프클로저가 장기 보정 담당

---

### 14-1) icp_odom_cov_scale.py — cmd_vel 상태별 공분산 동적 스케일링

**파일**: `src/rtabmap_ros/rtabmap_launch/scripts/icp_odom_cov_scale.py`

`/icp_odom`의 `twist.covariance[35]`(vyaw 분산)를 cmd_vel 상태에 따라 동적으로 곱한 뒤 `/icp_odom_filtered`로 재발행합니다. EKF(`odom1: /icp_odom_filtered`, `vyaw=true`)는 이 토픽만 구독하므로, 스케일이 곧 EKF의 ICP 신뢰도를 결정합니다.

```
/icp_odom ──→ icp_odom_cov_scale.py ──→ /icp_odom_filtered ──→ EKF (odom1 vyaw)
                   │
                   ├── [정지] SCALE=1000  cov[35] × 1000  → EKF가 vyaw 사실상 무시
                   ├── [회전] SCALE=1     cov[35] × 1     → EKF가 ICP vyaw 완전 신뢰
                   └── [전진] SCALE=10    cov[35] × 10    → 부분 신뢰
```

| 상태 | 조건 | SCALE | 효과 |
|------|------|-------|------|
| 정지 | `|v|<0.01, |w|<0.01` 또는 0.5초 cmd_vel 없음 | **1000** | 드리프트 방지 ✅ 효과 확인 |
| 회전 | `|w|≥0.01 rad/s` | **1** | ICP PointToPlane 완전 신뢰 |
| 전진 | `|v|≥0.01 m/s` | **10** | 부분 신뢰 |

> RTAB-Map은 no-motion 감지 시 cov[35]=9999(sentinel)를 출력합니다.
> 정지 상태에서 SCALE=1000을 곱해도 EKF가 이 큰 분산을 무시하는 방향으로 작동합니다.

**EKF 설정** (`src/robot_localization/params/ekf.yaml`):
```yaml
odom1: /icp_odom_filtered
odom1_config: [false×11, true, false, false, false]  # vyaw=true만 활성화
```

### 14-2) ICP PointToPlane 활성화

**파일**: `src/rtabmap_ros/rtabmap_launch/launch/rtabmap_nav2.launch.py`

순수 회전 시 PointToPoint ICP는 로컬 미니멈에 빠져 수렴 불안정합니다. PointToPlane은 법선벡터를 활용해 회전 추정 정확도가 더 높습니다.

| 파라미터 | 변경 전 | 변경 후 |
|----------|---------|---------|
| `Icp/PointToPlane` | `0` | `1` |
| `Icp/PointToPlaneK` | 없음 | `8` |
| `Icp/MaxCorrespondenceDistance` | `0.5` | `0.3` |

### 14-3) map→odom 안정화 — 3가지 원인 모두 제거

**파일**: `src/rtabmap_ros/rtabmap_launch/launch/rtabmap.launch.py`

#### 원인 1: ProximityBySpace 모순 링크 (이전 수정 완료)

```
제자리 회전 → 매 AngularUpdate마다 새 SLAM 노드 생성
ProximityBySpace=true → 같은 위치 노드들 사이에 ICP proximity 링크 생성
  odom: "로봇이 회전했다" ↔ proximity: "이 두 노드는 같은 위치다"
  → 그래프 최적화 모순 → map→odom 불안정
```

#### 원인 2: RTAB-Map 내부 ICP와 odom ICP 파라미터 불일치 (이번 수정)

```
odom ICP:     VoxelSize=0.15, MaxCorrespondenceDistance=0.3, OutlierRatio=0.7
RTAB-Map ICP: VoxelSize=기본값(?), MaxCorrespondenceDistance=기본값(0.1?), 나머지 기본값

→ 두 ICP가 서로 다른 결과를 냄
→ 매 노드 추가 시 odom 추정치 vs 등록 ICP 결과 불일치
→ 그래프 최적화가 불일치를 map→odom으로 흡수
```

#### 원인 3: 표준 최소제곱 최적화 — 오차 민감 (이번 수정)

```
표준 최적화: 모든 제약의 오차²를 동등하게 최소화
→ 회전 중 부정확한 ICP 등록이 outlier 제약으로 작용해도 그대로 반영
→ map→odom이 outlier에 끌려서 흔들림

Robust 최적화 (Huber): outlier 제약의 가중치를 자동으로 낮춤
→ 부정확한 ICP 제약의 영향 최소화 → map→odom 안정
```

#### 적용된 파라미터 변경

| 파라미터 | 변경 전 | 변경 후 | 효과 |
|----------|---------|---------|------|
| `RGBD/ProximityBySpace` | `true` | `false` | 모순 링크 차단 |
| `RGBD/ProximityMaxGraphDepth` | `15` | `0` | proximity 완전 차단 |
| `RGBD/AngularUpdate` | `0.15` rad | `1.0` rad | 노드 42→6개/360° |
| `Optimizer/Robust` | 없음 | `true` | Huber robust 최적화 |
| `Icp/VoxelSize` | 기본값 | `0.15` | odom ICP와 통일 |
| `Icp/MaxCorrespondenceDistance` | 기본값 | `0.3` | odom ICP와 통일 |
| `Icp/OutlierRatio` | 기본값 | `0.7` | odom ICP와 통일 |
| `Icp/CorrespondenceRatio` | 기본값 | `0.01` | odom ICP와 통일 |
| `Rtabmap/LoopThr` | `0.30` | `0.50` | false loop closure 방지 |
| `Vis/MinInliers` | `30` | `50` | 루프 검출 강화 |

### 14-4) 현재 상태 및 진단 방법

**해결 완료**:
- ✅ 정지 중 odom yaw 드리프트: SCALE_STOP=1000으로 안정화 확인
- ✅ ICP PointToPlane 활성화: 회전 추정 정확도 향상

**재테스트 필요**:
- 🔄 ProximityBySpace=false 후 제자리 회전 시 map 안정성 확인

**TF 레이어별 원인 분리 진단**:
```bash
# 터미널 1: EKF(odom→base_link) 확인
ros2 run tf2_ros tf2_echo odom base_link

# 터미널 2: RTAB-Map(map→odom) 확인
ros2 run tf2_ros tf2_echo map odom

# 정지 20초 후 제자리 180도 회전 → 각 TF의 yaw 변화량 비교
# odom→base_link 변화 → EKF/icp_odom_cov_scale 문제
# map→odom 변화     → RTAB-Map ProximityBySpace/루프클로저 문제
```

### 14-5) 수정된 파일 요약

| 파일 | 변경 내용 |
|------|----------|
| `src/rtabmap_ros/rtabmap_launch/scripts/icp_odom_cov_scale.py` | SCALE_ROT 5→1, SCALE_TRANS 30→10 |
| `src/rtabmap_ros/rtabmap_launch/launch/rtabmap_nav2.launch.py` | PointToPlane 0→1, PointToPlaneK=8, MaxCorrespondenceDistance 0.5→0.3 |
| `src/rtabmap_ros/rtabmap_launch/launch/rtabmap.launch.py` | ProximityBySpace false, AngularUpdate 0.15→1.0, **Optimizer/Robust=true(신규)**, **ICP파라미터 odom과 통일(신규)**, LoopThr 0.30→0.50, MinInliers 30→50 |
| `src/robot_localization/params/ekf.yaml` | odom1=/icp_odom_filtered, vyaw=true (변경 없음 — 이미 적용됨) |

### 15) 26/03/09 핵심적인 수정사항 

- 카메라 bias는 여기서 카메라 IMU(gyro)의 영점 오프셋입니다.
- 정지 상태에서 진짜 각속도는 0이어야 하는데, 센서는 보통 -0.42 rad/s 같은 상수 오차를 갖습니다.

수식으로 보면:
```bash
w_meas = w_true + b + noise
```
여기서 b가 bias이고, 이를 빼지 않으면 EKF가 yaw를 계속 적분해서 TF 드리프트가 납니다.

현재 소스는 이렇게 동작합니다.

- 파라미터 초기화
camera_imu_bias_corrector.cpp (line 13)
입출력 토픽, 캘리브레이션 샘플 수, 공분산, 정지 yaw 클램프 파라미터를 선언합니다.
핵심 신규 파라미터는 35~50행 (line 35)입니다.

**IMU 구독/발행**
53~56행 (line 53)
/camera/camera/imu를 받아 보정 후 /camera/camera/imu_bias_corrected로 냅니다.

**프레임 변환**
77~111행 (line 77)
target_frame이 있으면 TF로 gyro/accel 벡터를 회전시켜 기준 프레임을 맞춥니다.

**초기 bias 캘리브레이션**
115~135행 (line 115)
정지로 판단된 샘플(mag < stationary_threshold)을 calib_samples만큼 모아 평균값을 bias_x/y/z로 확정합니다.

**연속 bias 추적(EMA)**
137~142행 (line 137)
초기 보정 후에도 정지 구간에서 EMA로 bias를 천천히 업데이트합니다(온도 드리프트 대응).

**보정값 적용**
163~175행 (line 163)
out.angular_velocity = av - bias 형태로 실제 출력 각속도를 만듭니다.

**정지 시 yaw 드리프트 차단(이번 핵심 수정)**
177~198행 (line 177)
아래 3조건을 모두 만족하면 정지로 보고 angular_velocity.z = 0.0 강제:

```bash
|wz| < yaw_zero_threshold
|wx|, |wy| 작음
||a||가 g 근처
z축 공분산 상태별 적용
```
200~204행 (line 200)
- 정지면 yaw_stationary_cov(작게), 이동이면 yaw_moving_cov(크게)로 넣어 EKF 신뢰도를 분리합니다.

런치 기본값도 같이 넣어둔 상태입니다.

- sensor_sync.launch.py (line 78)
imu_pipeline_cpp.launch.py (line 32)
즉, 현재 코드는 bias 평균 보정 + 정지 yaw 강제 0 + 공분산 게이팅 3단으로 드리프트를 막는 구조입니다.



