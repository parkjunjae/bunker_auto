# ca_ws 실행/튜닝 가이드 (2026-02-28 업데이트)

이 문서는 오늘 반영된 변경사항 기준으로 정리합니다.

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
- **제자리 회전 시 로컬/글로벌 맵 불일치 해결 (2026-02-28 신규)**

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
