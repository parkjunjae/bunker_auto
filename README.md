# ca_ws 실행 가이드

이 문서는 `Bunker` 기준으로 다음 2가지만 정리합니다.
- 시뮬레이션 실행 방법
- 모터(CAN) 통신/상태 확인 방법

# 1) 시뮬레이션 방법 (Gazebo + ROS 2)

### 1-1. 빌드
```bash
cd ~/ca_ws
colcon build --symlink-install
```

### 1-2. 환경 로드
```bash
source /opt/ros/<distro>/setup.bash
source ~/ca_ws/install/setup.bash
```

### 1-3. 시뮬레이션 실행
```bash
ros2 launch bunker_sim bunker_gz.launch.py
```

이 런치는 아래를 포함합니다.
- Gazebo 월드 로드
- `bunker` 모델 스폰
- ROS/Gazebo 브리지 (`/clock`, `/cmd_vel`, `/odom`, `/tf`)

### 1-4. 동작 확인
다른 터미널에서:
```bash
source /opt/ros/<distro>/setup.bash
source ~/ca_ws/install/setup.bash

# 속도 명령 전송
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}, angular: {z: 0.0}}" -r 10
```

또 다른 터미널에서:
```bash
source /opt/ros/<distro>/setup.bash
source ~/ca_ws/install/setup.bash

# 오도메트리 수신 확인
ros2 topic echo /odom
```

# 2) 모터 확인 방법 (실기체 CAN)

### 2-1. CAN 어댑터 준비
```bash
sudo modprobe gs_usb
cd ~/ca_ws/src/ugv_sdk/scripts

# 최초 1회
bash setup_can2usb.bash

# 이후 전원 재인가/재연결 시
bash bringup_can2usb_500k.bash
```

### 2-2. CAN 프레임 수신 확인
```bash
candump can1
```

`candump`에 프레임이 계속 보이면 CAN 통신은 정상입니다.

### 2-3. 베이스 노드 실행
```bash
source /opt/ros/<distro>/setup.bash
source ~/ca_ws/install/setup.bash

ros2 launch bunker_base bunker_base.launch.py
```

### 2-4. 모터/차량 상태 토픽 확인
다른 터미널에서:
```bash
source /opt/ros/<distro>/setup.bash
source ~/ca_ws/install/setup.bash

# 모터 상태 포함( rpm, current, driver_temperature 등 )
ros2 topic echo /bunker_status

# RC 상태 확인
ros2 topic echo /bunker_rc_state
```

### 2-5. 저속 명령 테스트
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}, angular: {z: 0.0}}" -r 5
```

안전 주의:
- 반드시 비상정지(E-Stop) 접근 가능한 상태에서 테스트
- 바퀴가 뜬 상태 또는 충분히 넓은 공간에서 저속으로 시작

# 3) PID 학습 파일 생성 (추론 전 단계, 시뮬 기준)

아래 순서대로 실행합니다.

### 3-1. 터미널 A: 시뮬 실행
```bash
cd ~/ca_ws
source /opt/ros/<distro>/setup.bash
source ~/ca_ws/install/setup.bash
ros2 launch bunker_sim bunker_gz.launch.py
```

### 3-2. 터미널 B: ros2_control 컨트롤러 활성화 확인
```bash
source /opt/ros/<distro>/setup.bash
source ~/ca_ws/install/setup.bash
ros2 control list_controllers
ros2 control set_controller_state joint_state_broadcaster active
ros2 control set_controller_state diff_drive_controller active
```

### 3-3. 터미널 C: controller_server 실행
```bash
source /opt/ros/<distro>/setup.bash
source ~/ca_ws/install/setup.bash
ros2 run nav2_controller controller_server \
  --ros-args \
  --params-file /home/atoz/ca_ws/src/rtabmap_ros/rtabmap_launch/launch/config/nav2_rtabmap_params_train.yaml \
  -p use_sim_time:=true \
  -p odom_topic:=/diff_drive_controller/odom \
  -r cmd_vel:=/diff_drive_controller/cmd_vel
```

### 3-4. 터미널 D: lifecycle manager로 controller_server 활성화
```bash
source /opt/ros/<distro>/setup.bash
source ~/ca_ws/install/setup.bash
ros2 run nav2_lifecycle_manager lifecycle_manager \
  --ros-args \
  -p use_sim_time:=true \
  -p autostart:=true \
  -p node_names:="[controller_server]"
```

### 3-5. 터미널 E: 학습 실행
```bash
source /opt/ros/<distro>/setup.bash
source ~/ca_ws/install/setup.bash
ros2 run rl_pid_training train_pid
```

### 3-6. 학습 전 필수 체크
```bash
ros2 lifecycle get /controller_server
ros2 action list | grep follow_path
ros2 topic list | grep desired_cmd
ros2 service list | grep controller_server/set_parameters
```

학습 결과 모델 저장 경로(코드 기본값):
```text
/home/atoz/ca_ws/rl_pid_model_new.zip
```

참고:
- `train_pid.py`는 현재 `PidGainEnv()` 기본값을 사용하므로, 오도메트리 기본 토픽은 `/diff_drive_controller/odom`입니다.
- 실행 전 `stable-baselines3`가 설치되어 있어야 합니다.

## 4) 시뮬 "모터 정보" 수정 방향

PID 성능 맞출 때 핵심은 상태 토픽 이름보다, 시뮬 동역학을 실차에 가깝게 맞추는 것입니다.

우선 조정할 항목:
- 차체/관성: 질량, 관성 모멘트
- 구동 특성: 최대 토크, 가감속 제한
- 바퀴/지면: 마찰 계수, 휠 반경/트레드
- 제어 응답: 명령 지연, 노이즈(필요 시)

수정 시작 파일(이 워크스페이스 기준):
- `src/bunker_sim/models/bunker_tracked/model.sdf`
- `src/bunker_sim/worlds/empty.sdf`
- `src/bunker_sim/launch/bunker_gz.launch.py`

권장 절차:
1. 기본 파라미터 1개만 변경
2. `/odom`, `/cmd_vel`, `/bunker_status` 로그 비교
3. 직진/회전/정지 응답이 실차와 비슷해질 때까지 반복
