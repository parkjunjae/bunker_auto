# ca_ws 실행/튜닝 가이드 (2026-02-26 업데이트)

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
