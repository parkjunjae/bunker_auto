# plan_tfv3.md - `imu0_twist_rejection_threshold=3.0` 적용 후 결과 정리, map ghosting 판정, footprint lag 후속 계획

## 0. 문서 목적

이 문서는 아래 세 실험을 비교해서, 지금 맵 고스팅 문제와 남아 있는 footprint 추종 지연 문제를 어떤 순서로 해결해야 하는지 다시 정리한 문서다.

1. baseline clean run
   - `logs/run_20260318_142405`
   - `imu0_twist_rejection_threshold: 1.5`
2. threshold 변경 후 clean run
   - `logs/run_20260318_152622`
   - `imu0_twist_rejection_threshold: 3.0`
3. 추가 시각 검증 run
   - `logs/run_20260318_160012`
   - `imu0_twist_rejection_threshold: 3.0`
   - `spin_yaw_probe` 없이 기본 `./run_all.sh` 상태에서 수행
   - 사용자가 RViz에서 `/rtabmap/map` 과 `/rtabmap/map_stable` 를 각각 직접 확인

이번 문서의 핵심 목적은 네 가지다.

1. `imu0_twist_rejection_threshold=3.0` 변경이 실제로 효과가 있었는지 확정한다.
2. local yaw under-response가 아직 주원인인지, 아니면 이미 해결됐는지 판단한다.
3. 최신 시각 검증에서 raw map/stable map에 실제 ghosting이 남는지 다시 판정한다.
4. ghosting이 아니라면, 남은 체감 문제인 `TF는 먼저 도는데 footprint가 조금 늦게 따라오는` 현상을 어떤 계층에서 해결해야 하는지 정리한다.

---

## 0.1 핵심 결론

이번 최신 결론은 아래 세 run을 합쳐서 봐야 한다.

1. `run_20260318_152622`는 IMU, `/odometry/filtered`, `odom -> base_link TF`가 좌/우 회전량을 거의 같은 크기로 반영함을 보여준다.
2. 즉 이전 clean run에서 보였던 `local yaw under-response`는 `imu0_twist_rejection_threshold=3.0`에서 사실상 해소됐다.
3. 이후 사용자가 수행한 최신 시각 검증 `run_20260318_160012`에서는 `/rtabmap/map`, `/rtabmap/map_stable` 모두에서 맵이 돌아가거나 고스팅하는 현상이 보이지 않았다.
4. 대신 사용자가 본 남은 현상은 `TF 축은 먼저 회전하는데 footprint 또는 footprint처럼 보이는 consumer overlay가 약간 늦게 따라오는 느낌`이다.
5. 따라서 현재 주 타깃은 더 이상 raw occupancy ghosting이 아니라, `TF 대비 footprint/costmap consumer 추종 지연`이다.

한 문장으로 요약하면:

**`imu0_twist_rejection_threshold=3.0`으로 local yaw bottleneck은 거의 풀렸고, 최신 시각 검증 기준 raw/stable map ghosting은 관찰되지 않았다. 지금 남은 문제는 map 자체보다 footprint/costmap consumer의 추종 지연이다.**

---

## 0.2 최종 성공 기준

### 성공 기준 A: raw map 기준

조건:

1. RViz `Fixed Frame = map`
2. `Map topic = /rtabmap/map`
3. graph / stable map / costmap display는 모두 끔

성공 판정:

1. 좌 180 / 우 180 후 벽 외곽이 이중선처럼 오래 남지 않아야 한다.
2. 회전 종료 후 `1초` 안에 겹쳐 보이는 벽이 자연스럽게 정리되어야 한다.
3. 로봇 축이 실제 로봇 heading보다 체감상 크게 어긋나지 않아야 한다.

### 성공 기준 B: stable consumer 기준

조건:

1. RViz `Fixed Frame = map_stable`
2. `/rtabmap/map_stable` 또는 Nav2 consumer만 봄

성공 판정:

1. pure spin 후 stable map이 오래 멈추거나 warn 상태로 남지 않아야 한다.
2. `pending=1` 상태가 장시간 잔류하지 않아야 한다.
3. raw map이 깨끗하면 stable map은 raw map보다 더 나빠지면 안 된다.

### 성공 기준 C: footprint 추종 기준

조건:

1. RViz에서 TF display와 footprint 또는 footprint처럼 보이는 consumer overlay를 동시에 본다.
2. `Fixed Frame`은 비교 대상에 따라 `map` 또는 `map_stable`로 두되, 같은 조건에서 raw/stable을 비교한다.

성공 판정:

1. 제자리 회전 중 TF 축과 footprint overlay 사이에 체감상 뚜렷한 지연이 없어야 한다.
2. 회전 종료 직후 footprint overlay가 별도의 뒤늦은 snap 또는 catch-up 없이 정착해야 한다.
3. footprint overlay의 늦은 추종이 map ghosting처럼 오인되지 않아야 한다.

### 작업용 정량 게이트

이번 문서 시점의 정량 게이트는 아래다.

1. 좌/우 pure spin에서 `|odom_delta_yaw_deg / imu_delta_yaw_raw_deg| >= 0.95`
2. `odom_delta_yaw_deg`와 `tf_delta_yaw_deg`는 계속 거의 같아야 한다.

최신 run `run_20260318_152622`는 이 게이트를 통과했다.

---

## 1. 실험 조건

첫 두 run은 아래 조건에서 수행했다.

```bash
cd /home/atoz/ca_ws
ENABLE_EKF_DEBUG=0 EKF_PRINT_DIAGNOSTICS=0 ./run_all.sh
```

실험 동작:

1. 제자리에서 왼쪽으로 약 `180도`
2. 잠깐 정지
3. 제자리에서 오른쪽으로 약 `180도`

변경 사항은 `ekf.yaml`의 아래 한 줄뿐이다.

```yaml
imu0_twist_rejection_threshold: 3.0
```

현재 적용 상태는 [ekf.yaml](/home/atoz/ca_ws/src/robot_localization/params/ekf.yaml#L49) 에서 확인된다.

중요한 점:

1. 최신 run에는 `robot_localization_debug.txt`가 없다.
2. 즉 full debug 오염이 없는 상태다.
3. 따라서 이번 결과는 EKF full debug 부하 때문이 아니라 실제 파라미터 변경 효과로 보는 것이 맞다.

세 번째 run `run_20260318_160012`은 성격이 다르다.

1. `spin_yaw_probe` 없이 기본 `./run_all.sh` 상태에서 수행했다.
2. 사용자가 RViz에서 `/rtabmap/map` 과 `/rtabmap/map_stable` 를 각각 직접 비교했다.
3. 따라서 이 run은 정량 yaw 실험이 아니라, **실제 체감 증상 최종 판정용 시각 검증 run** 으로 취급해야 한다.

---

## 2. 비교 요약

## 2.1 baseline clean run `run_20260318_142405`

요약:

1. 좌회전
   - IMU `+175.237 deg`
   - odom/TF `+28.069 deg`
   - ratio `0.160178`
2. 우회전
   - IMU `-173.563 deg`
   - odom/TF `-47.419 deg`
   - ratio `0.273207`

해석:

1. IMU는 거의 180도를 정상적으로 봤다.
2. `/odometry/filtered`와 TF는 그 회전량 일부만 반영했다.
3. 이 run은 clean under-response를 재현한 기준 run이다.

## 2.2 threshold 변경 run `run_20260318_152622`

요약:

1. 좌회전
   - IMU `+170.136 deg`
   - odom/TF `+170.582 deg`
   - ratio `1.002620`
2. 우회전
   - IMU `-169.703 deg`
   - odom/TF `-170.602 deg`
   - ratio `1.005299`

해석:

1. 좌/우 모두 IMU와 odom/TF가 거의 같은 회전량을 반영했다.
2. 이 run은 기존 under-response가 해소됐음을 보여준다.

## 2.3 비교 결론

두 run을 나란히 놓으면 변화가 아주 크다.

1. baseline run
   - `0.16 ~ 0.27` 수준만 반영
2. threshold 3.0 run
   - `1.00` 수준까지 회복

따라서 이번 비교로 가장 강하게 지지되는 결론은:

**기존 local yaw under-response의 핵심 bottleneck은 `imu0_twist_rejection_threshold=1.5` 쪽이었고, `3.0`은 그 bottleneck을 충분히 완화한 값이다.**

---

## 3. 최신 run `run_20260318_152622` 상세 분석

## 3.1 `spin_yaw_probe.summary.json`

최신 summary는 아래를 보여준다.

### 세션 1: 좌회전

1. `duration_sec = 6.243875`
2. `peak_abs_imu_wz = 1.051739`
3. `mean_abs_imu_wz = 0.475931`
4. `mean_abs_odom_wz = 0.478034`
5. `imu_delta_yaw_raw_deg = +170.136`
6. `odom_delta_yaw_deg = +170.582`
7. `tf_delta_yaw_deg = +170.582`
8. `ratio_odom_over_imu_raw = 1.002620`
9. `pose_minus_tf_delta_yaw_deg = 0.0`

### 세션 2: 우회전

1. `duration_sec = 6.256423`
2. `peak_abs_imu_wz = 1.042427`
3. `mean_abs_imu_wz = 0.474048`
4. `mean_abs_odom_wz = 0.477291`
5. `imu_delta_yaw_raw_deg = -169.703`
6. `odom_delta_yaw_deg = -170.602`
7. `tf_delta_yaw_deg = -170.602`
8. `ratio_odom_over_imu_raw = 1.005299`
9. `pose_minus_tf_delta_yaw_deg = 0.0`

이 숫자가 의미하는 사실:

1. IMU 회전량과 filtered yaw 회전량이 거의 같다.
2. pose와 TF는 여전히 완전히 일치한다.
3. local yaw chain은 현재 정상 범위로 돌아왔다.

## 3.2 `spin_yaw_probe.jsonl`

순간값도 같이 봐야 한다.

좌회전 peak 구간:

1. peak IMU `1.051739 rad/s`
2. 같은 시점 odom `1.022254 rad/s`
3. peak odom `1.032279 rad/s`

우회전 peak 구간:

1. peak IMU `-1.042427 rad/s`
2. 같은 시점 odom `-0.972020 rad/s`
3. peak odom `-1.024250 rad/s`

추가 관찰:

1. 세션별 mean absolute difference는 `~0.027 ~ 0.028 rad/s`
2. 이 정도면 이전 clean run 대비 극적으로 개선된 상태다

즉 이번 run은 누적각도뿐 아니라 순간 각속도 추종도 회복됐다고 봐야 한다.

## 3.3 `sensor_sync.log`

최신 run의 `sensor_sync.log`는 아래 전환을 보여준다.

1. 좌회전 시작 시 `yaw_zeroing=OFF reason=hard_wz`
2. 좌회전 종료 후 `yaw_zeroing=ON reason=steady_hold`
3. 우회전 시작 시 `yaw_zeroing=OFF reason=yaw_evidence`
4. 우회전 종료 후 `yaw_zeroing=ON reason=steady_hold`

의미:

1. IMU pipeline은 회전을 정상적으로 감지한다.
2. yaw zeroing 상태기계는 여전히 정상적으로 동작한다.
3. 이번 결과를 "IMU가 우연히 좋아 보인 것"으로 볼 이유는 약하다.

## 3.4 `ekf.log`

최신 run의 `ekf.log`에는 `Failed to meet update rate`가 `2회` 있다.

하지만 이 값은 아래 이유로 본체 문제라고 보기 어렵다.

1. full debug 오염 run의 `145회`와는 전혀 다르다.
2. 실제 yaw 추종 결과는 이번 run에서 거의 정상이다.
3. 즉 이번 2회 경고는 결과 해석을 뒤집을 정도의 timing collapse가 아니다.

해석:

1. 이번 local yaw 회복은 timing 붕괴와 무관하게 나온 결과다.
2. 따라서 이번 개선은 실제 파라미터 변경 효과라고 봐야 한다.

## 3.5 `rtabmap_nav2.log` - `map_tf_stabilizer`

회전 중 `map_tf_stabilizer`는 아래를 보여준다.

1. `imu_wz`는 좌/우 모두 `0.9 ~ 1.0 rad/s` 수준
2. `score`는 `0.98 ~ 0.99` 수준
3. `raw_yaw`는 계속 `0.000` 근처
4. `stable_map_yaw`도 계속 `0.000` 근처

의미:

1. pure spin은 감지한다.
2. 하지만 `map -> odom` TF spike가 주요 증상은 아니다.
3. 이 부분은 이전과 동일하다.

즉 이번 시점에서도 `map_tf_stabilizer`는 주범이 아니다.

## 3.6 `rtabmap_nav2.log` - `map_topic_stabilizer`

이번 run에서도 두 회전 모두 아래 패턴이 반복된다.

1. `hold_enter`
2. 약 `4초` 후 `hold_exit reason=timeout`
3. 그 뒤 `pending=1`이 길게 남음

좌회전:

1. `hold_enter` around line `1401`
2. `hold_exit reason=timeout` around line `1512`
3. 이후 `pending=1` 계속 유지

우회전:

1. `hold_enter` around line `1680`
2. `hold_exit reason=timeout` around line `1787`
3. 이후 `pending=1` 계속 유지

의미:

1. local yaw는 살아났는데 stable relay 계층 문제는 그대로 남아 있다.
2. 따라서 사용자가 여전히 stable map이나 Nav2 consumer에서 이상을 본다면, 다음 타깃은 이 계층이다.

## 3.7 추가 시각 검증 run `run_20260318_160012`

이 run은 지금 문서에서 가장 중요하다.

이유:

1. 이전 `run_20260318_152622`는 local yaw가 회복됐음을 정량적으로 증명한 run이다.
2. 하지만 사용자가 실제로 체감하는 문제는 숫자가 아니라 화면에서 보이는 움직임이다.
3. `run_20260318_160012`은 사용자가 `/rtabmap/map` 과 `/rtabmap/map_stable` 를 직접 번갈아 보면서, "지금 남아 있는 이상 현상이 정확히 무엇인가"를 다시 정의한 run이다.

### 3.7.1 사용자 관찰

사용자 관찰은 아래 두 줄로 요약된다.

1. `/rtabmap/map`, `/rtabmap/map_stable` 둘 다에서 맵이 돌아가거나 고스팅하는 현상은 보이지 않았다.
2. 대신 제자리 180도 회전 중 TF 축은 먼저 도는데 footprint가 약간 늦게 따라오는 느낌이 있었다.

이 관찰은 중요하다.

왜냐하면 이것은 문제의 중심이 더 이상 `occupancy grid가 잘못 그려진다`가 아니라, **회전 중 footprint 또는 footprint처럼 보이는 consumer overlay가 TF를 따라가는 방식이 어색하다**로 이동했음을 뜻하기 때문이다.

### 3.7.2 로그가 보여주는 사실

이번 run의 `rtabmap_nav2.log`는 아래 사실을 보여준다.

1. startup 이후 stable map은 초기 publish만 한 것이 아니라, 주행 중 pass-through publish도 있었다.
   - [rtabmap_nav2.log#L90](/home/atoz/ca_ws/logs/run_20260318_160012/rtabmap_nav2.log#L90)
   - [rtabmap_nav2.log#L1714](/home/atoz/ca_ws/logs/run_20260318_160012/rtabmap_nav2.log#L1714)
   - [rtabmap_nav2.log#L1870](/home/atoz/ca_ws/logs/run_20260318_160012/rtabmap_nav2.log#L1870)
2. pure spin 구간에서는 `map_topic_stabilizer`가 hold에 들어간다.
   - [rtabmap_nav2.log#L2234](/home/atoz/ca_ws/logs/run_20260318_160012/rtabmap_nav2.log#L2234)
3. 첫 회전 구간은 timeout으로 hold를 빠져나오고 곧바로 `pending=1` 상태로 남는다.
   - [rtabmap_nav2.log#L2338](/home/atoz/ca_ws/logs/run_20260318_160012/rtabmap_nav2.log#L2338)
   - [rtabmap_nav2.log#L2440](/home/atoz/ca_ws/logs/run_20260318_160012/rtabmap_nav2.log#L2440)
4. 이후 다른 회전 구간에서도 같은 패턴이 반복된다.
   - [rtabmap_nav2.log#L3495](/home/atoz/ca_ws/logs/run_20260318_160012/rtabmap_nav2.log#L3495)
   - [rtabmap_nav2.log#L7510](/home/atoz/ca_ws/logs/run_20260318_160012/rtabmap_nav2.log#L7510)
5. 같은 run에서 spin detector는 회전을 분명하게 감지했다.
   - `max_wz ~= 0.746 rad/s`
   - `max_score ~= 0.993`
6. 그런데도 `map_tf_stabilizer`의 `raw_yaw`, `stable_map_yaw`는 계속 거의 `0` 근처다.
   - 즉 이번에도 `map -> odom` 급회전이 화면상 주범이라는 증거는 없다.
7. `ekf.log`에는 `Failed to meet update rate` 경고가 없다.
   - 즉 이번 시각 검증은 full-debug 오염이나 EKF timing collapse로 설명하기 어렵다.

### 3.7.3 이 run이 말해주는 것

이 run은 아래를 뜻한다.

1. raw map과 stable map이 둘 다 시각적으로 깨끗하다면, 적어도 사용자가 처음 문제 삼았던 "맵이 돈다", "벽이 이중으로 그려진다", "회전 후 고스팅이 남는다"는 현상은 현재 설정에서 크게 줄었다.
2. 그런데도 footprint가 늦게 따라온다면, **남아 있는 문제는 occupancy grid 자체가 아니라 footprint/costmap/consumer overlay 쪽이다.**

즉 이 run은 문제 정의를 다음처럼 바꾼다.

1. 이전 문제 정의
   - map ghosting / map rotation
2. 현재 문제 정의
   - TF는 빠르게 갱신되는데 footprint overlay는 한 박자 늦게 보임

### 3.7.4 footprint lag의 가장 가능성 높은 원인

이 부분은 사실과 추론을 분리해서 적어야 한다.

#### 사실 1: EKF는 현재 시각으로 미래 예측하지 않는다

[ekf.yaml](/home/atoz/ca_ws/src/robot_localization/params/ekf.yaml#L15) 에서 현재 설정은 아래와 같다.

1. `frequency: 20.0` [ekf.yaml](/home/atoz/ca_ws/src/robot_localization/params/ekf.yaml#L3)
2. `predict_to_current_time: false` [ekf.yaml](/home/atoz/ca_ws/src/robot_localization/params/ekf.yaml#L15)

의미:

1. EKF는 대략 20Hz, 즉 50ms 단위로 상태를 갱신한다.
2. RViz 렌더 시각까지 추가 예측해서 TF를 현재 시각으로 당겨오지 않는다.

이 값은 이전 under-response와 stop 후 잔류 적분 문제를 함께 고려해 유지한 값이다.

#### 사실 2: Nav2 footprint/costmap 계층은 TF보다 더 느리게 publish될 수 있다

현재 Nav2 설정은 아래와 같다.

1. behavior server는 footprint topic으로 `local_costmap/published_footprint`를 사용한다.
   - [nav2_rtabmap_params.yaml#L175](/home/atoz/ca_ws/src/rtabmap_ros/rtabmap_launch/launch/config/nav2_rtabmap_params.yaml#L175)
2. behavior server의 `transform_tolerance`는 `0.2`다.
   - [nav2_rtabmap_params.yaml#L191](/home/atoz/ca_ws/src/rtabmap_ros/rtabmap_launch/launch/config/nav2_rtabmap_params.yaml#L191)
3. local costmap은 `update_frequency: 20.0`, `publish_frequency: 10.0`, `transform_tolerance: 0.1`이다.
   - [nav2_rtabmap_params.yaml#L234](/home/atoz/ca_ws/src/rtabmap_ros/rtabmap_launch/launch/config/nav2_rtabmap_params.yaml#L234)
   - [nav2_rtabmap_params.yaml#L235](/home/atoz/ca_ws/src/rtabmap_ros/rtabmap_launch/launch/config/nav2_rtabmap_params.yaml#L235)
   - [nav2_rtabmap_params.yaml#L236](/home/atoz/ca_ws/src/rtabmap_ros/rtabmap_launch/launch/config/nav2_rtabmap_params.yaml#L236)
4. global costmap은 `update_frequency: 10.0`, `publish_frequency: 5.0`, `transform_tolerance: 0.3`이다.
   - [nav2_rtabmap_params.yaml#L342](/home/atoz/ca_ws/src/rtabmap_ros/rtabmap_launch/launch/config/nav2_rtabmap_params.yaml#L342)
   - [nav2_rtabmap_params.yaml#L343](/home/atoz/ca_ws/src/rtabmap_ros/rtabmap_launch/launch/config/nav2_rtabmap_params.yaml#L343)
   - [nav2_rtabmap_params.yaml#L344](/home/atoz/ca_ws/src/rtabmap_ros/rtabmap_launch/launch/config/nav2_rtabmap_params.yaml#L344)

의미:

1. TF는 EKF가 직접 publish하는 경로다.
2. footprint/costmap overlay는 costmap update 주기, publish 주기, transform tolerance를 한 번 더 거친다.
3. 그래서 회전 중에는 footprint/costmap overlay가 TF 축보다 느리게 보이는 것이 구조적으로 가능하다.

#### 사실 3: checked-in RViz 기본 설정에는 explicit footprint display가 없다

[rgbd.rviz](/home/atoz/ca_ws/src/rtabmap_ros/rtabmap_launch/launch/config/rgbd.rviz) 기준 기본 display에는 아래가 있다.

1. TF
2. `/rtabmap/map_stable`
3. `/rtabmap/mapData`
4. `/rtabmap/mapGraph`
5. point cloud들

즉 저장된 기본 설정만 놓고 보면 "초록 footprint" display가 명시적으로 들어가 있지 않다.

[추론]

1. 사용자가 본 footprint 또는 초록 사각형은 수동으로 켠 costmap/footprint overlay일 가능성이 높다.
2. 이 가설이 맞으면 문제는 raw occupancy grid가 아니라 consumer visualization path 쪽이다.

#### 추론 1: 현재 보이는 lag는 occupancy map lag가 아니라 consumer lag다

왜 이렇게 보는가:

1. 사용자는 raw `/rtabmap/map`, stable `/rtabmap/map_stable` 둘 다에서 ghosting이 없다고 봤다.
2. 최신 log에서도 `map_tf_stabilizer`의 `raw_yaw`, `stable_map_yaw`는 조용하다.
3. 반면 `map_topic_stabilizer`는 spin 동안 hold와 timeout, pending을 반복한다.
4. Nav2 footprint/costmap publish path는 TF보다 더 느리고 tolerance도 더 크다.

그래서 가장 일관적인 해석은 이렇다.

1. 맵 자체는 현재 크게 틀어지지 않는다.
2. TF 축은 비교적 빠르게 회전한다.
3. footprint overlay는 costmap/consumer 주기와 tolerance 때문에 약간 늦게 따라온다.

즉 지금 남은 현상은 **"맵이 늦다"가 아니라 "footprint consumer가 늦다"** 쪽으로 보는 것이 맞다.

---

## 4. 이번 결과로 확정된 것

이번 최신 run들까지 포함해서 아래는 사실상 확정됐다.

1. `imu0_twist_rejection_threshold=3.0`은 효과가 있다.
2. 기존 `local yaw under-response`는 현재 설정에서 사실상 해소됐다.
3. pose와 TF mismatch 문제는 아니다.
4. `map -> odom` TF spike가 주원인이라는 증거는 없다.
5. 최신 시각 검증 기준으로 raw `/rtabmap/map`, stable `/rtabmap/map_stable` 모두에서 map rotation / ghosting은 관찰되지 않았다.
6. stable relay 계층의 `timeout / pending=1` 문제는 아직 남아 있다.
7. 현재 체감상 남은 주된 증상은 `footprint 또는 footprint처럼 보이는 overlay가 TF보다 늦게 따라오는 현상`이다.

---

## 5. 이번 결과로 배제되거나 후순위로 밀린 것

## 5.1 지금 당장 더 만질 필요가 약한 것

현재는 아래를 더 건드릴 이유가 약하다.

1. `imu0_twist_rejection_threshold: 5.0`
2. `yaw_moving_cov`
3. `process_noise_covariance`
4. `yaw_stationary_cov`
5. `yaw_zeroing` 관련 threshold
6. `yaw_rate_scale`

이유:

1. 현재 목표는 "더 좋은 local yaw 숫자"가 아니라 "남은 ghosting의 위치 분리"다.
2. 이미 local yaw는 충분히 회복된 상태다.
3. 여기서 계속 EKF/IMU 파라미터를 만지면 downstream 문제 분리가 어려워진다.

## 5.2 여전히 열어둬야 하는 후보

다만 아래 두 후보는 아직 살아 있다.

1. raw `/rtabmap/map` 자체는 현재 충분히 안정적이고, 남은 문제는 footprint/costmap consumer가 만든다
2. raw map은 괜찮고 `map_topic_stabilizer` / stable consumer / costmap footprint 계층이 stale 또는 delayed artifact를 만든다

현재 최신 관찰까지 포함하면 2번이 더 유력하다.

즉 지금부터는 local yaw가 아니라 **consumer 계층 분리**가 핵심이다.

---

## 6. 지금 기준의 해결방안

## 6.1 1차 결정: local yaw는 현재 설정으로 고정

현재는 아래를 유지한다.

1. `imu0_twist_rejection_threshold: 3.0`
2. `predict_to_current_time: false`
3. `sensor_timeout: 0.2`
4. `frequency: 20.0`
5. `yaw_moving_cov: 0.01`
6. `yaw_stationary_cov: 1.0e-4`

이유:

1. under-response가 이미 해소됐다.
2. 지금 더 중요한 것은 ghosting이 아니라 footprint lag가 어느 계층에 남는지 분리하는 것이다.

## 6.2 2차 결정: map ghosting은 일단 해결된 것으로 취급하고, footprint lag를 주 타깃으로 전환

이 문서 시점에서는 아래처럼 판단한다.

1. raw `/rtabmap/map` 과 stable `/rtabmap/map_stable` 모두에서 사용자가 ghosting을 보지 못했다.
2. 따라서 "맵이 돈다 / 고스팅이 남는다"는 본래 문제는 현재 설정에서 일단 해결된 것으로 취급한다.
3. 대신 footprint lag는 아직 남아 있으므로, 다음 작업은 occupancy grid가 아니라 footprint/costmap/consumer chain 쪽으로 옮긴다.

## 6.3 다음 작업: footprint lag를 계층별로 쪼개서 확인

다음 확인은 아래 순서로 간다.

### Step A: raw `/rtabmap/map`

조건:

1. RViz `Fixed Frame = map`
2. `Map topic = /rtabmap/map`
3. graph / stable map / costmap display 모두 off

목표:

1. pure spin 후 wall duplication이 남는지 확인
2. 회전 종료 후 벽이 자연스럽게 정리되는지 확인
3. 사용자가 말한 "맵이 겹치거나 끌리는 느낌"이 줄었는지 확인

### Step B: stable map / consumer

조건:

1. RViz `Fixed Frame = map_stable`
2. `/rtabmap/map_stable` 또는 Nav2 consumer만 확인

목표:

1. `pending=1` 장기 잔류가 실제 체감 문제를 만드는지 확인
2. timeout 후 stale map처럼 보이는지 확인
3. raw map보다 stable map이 더 나빠지는지 확인

### Step C: footprint overlay 정체 확인

이 단계가 새로 중요해졌다.

확인 항목:

1. 화면에서 늦게 따라오는 초록 사각형/footprint가 정확히 어떤 topic/display인지 먼저 고정한다.
2. `TF`만 켠 상태에서 base frame이 느린지 본다.
3. 그 다음 footprint 또는 costmap overlay만 켜서 lag가 생기는지 본다.

[추론]

1. checked-in RViz 기본 설정에는 explicit footprint display가 없으므로, 현재 lag를 보이는 객체는 수동으로 켠 costmap/footprint overlay일 가능성이 높다.
2. 이 가설이 맞으면 map 쪽을 더 고칠 게 아니라 Nav2 consumer 갱신 주기와 tolerance를 먼저 봐야 한다.

## 6.4 footprint lag 분기 기준

### 경우 1: raw map이 이미 깨끗하다

이 경우:

1. local yaw fix가 실제 ghosting upstream에도 효과가 있었던 것
2. stable map/consumer 또는 footprint overlay에서만 남는다면 `map_topic_stabilizer`/costmap consumer가 다음 타깃

### 경우 2: raw map에서도 여전히 고스팅이 남는다

이 경우:

1. local yaw는 해결됐으므로 원인은 더 downstream이 아니라 raw occupancy/source publish 계층일 가능성이 높다
2. 다음 작업은 `/rtabmap/map` publish probe다

구체 작업:

1. `/rtabmap/map` publish마다 origin 기록
2. checksum 기록
3. size / resolution 기록
4. pure spin 중 redraw 타이밍 추적

### 경우 3: TF는 정상인데 footprint/costmap overlay만 늦다

이 경우가 현재 가장 유력하다.

이 경우:

1. 원인은 raw map이 아니라 consumer update path다.
2. 1차 타깃은 costmap publish frequency와 transform tolerance다.

우선 확인 대상:

1. local costmap `publish_frequency: 10.0`, `transform_tolerance: 0.1`
2. global costmap `publish_frequency: 5.0`, `transform_tolerance: 0.3`
3. behavior server `footprint_topic: local_costmap/published_footprint`, `transform_tolerance: 0.2`

### 경우 4: raw map은 괜찮고 stable map만 나쁘다

이 경우:

1. 다음 타깃은 `map_topic_stabilizer.py`
2. focus는 `hold_enter -> timeout -> pending=1` 경로다

구체 작업:

1. pending flush 조건 확인
2. translation reacquire 이후 refresh 정책 확인
3. stale map 유지 방식 확인

---

## 7. 다음 실무 작업 우선순위

### 우선순위 1

현재 파라미터 그대로 footprint lag의 display source를 먼저 고정한다.

1. `Fixed Frame = map`
2. `TF`만 켬
3. 그 다음 footprint 또는 costmap overlay만 추가
4. 좌 180 / 정지 / 우 180

즉 가장 먼저 필요한 것은 **새 튜닝이 아니라 lag를 만드는 display source 확인**이다.

### 우선순위 2

그 다음 `Fixed Frame = map_stable`에서 같은 동작을 본다.

이때 핵심은:

1. raw map 대비 stable map이 더 나빠지는지
2. `pending=1` 상태가 footprint lag 체감과 연결되는지

를 분리하는 것이다.

### 우선순위 3

TF는 정상이고 footprint overlay만 늦다면 costmap publish/tolerance를 조정한다.

후보:

1. global costmap `publish_frequency: 5.0 -> 10.0`
2. global costmap `transform_tolerance: 0.3 -> 0.15 ~ 0.2`
3. 필요 시 local costmap `publish_frequency: 10.0 -> 20.0`

중요:

1. tolerance를 너무 낮추면 TF timeout 경고가 생길 수 있으므로 한 번에 과하게 줄이지 않는다.

### 우선순위 4

raw map은 괜찮고 stable consumer만 나쁘면 `map_topic_stabilizer.py`를 손본다.

### 우선순위 5

footprint lag가 실제 TF latency로 확인될 때만 EKF/TF 쪽 A/B 실험으로 돌아간다.

후보:

1. `predict_to_current_time: false -> true` A/B 실험
2. 단, 이 값은 stop 직후 잔류 회전 적분을 다시 키울 수 있어 기본값으로 바로 확정하지 않는다.

즉 현재의 기본 원칙은:

1. local yaw는 유지
2. map ghosting은 해결된 것으로 보고 consumer lag를 먼저 분리
3. 필요할 때만 다시 EKF/IMU 또는 TF 예측으로 돌아간다

---

## 8. 최종 결론

이번 문서 시점의 최신 판단은 `run_20260318_152622` 와 `run_20260318_160012` 를 같이 봐야 한다.

이 실험으로 아래가 분명해졌다.

1. `imu0_twist_rejection_threshold=3.0`은 실제로 효과가 있었다.
2. 기존 `local yaw under-response`는 현재 설정에서 사실상 해소됐다.
3. pose와 TF는 계속 일치하므로 publish mismatch 문제는 아니다.
4. `map_tf_stabilizer`는 여전히 주원인이 아니다.
5. 최신 시각 검증 기준으로 raw `/rtabmap/map` 과 stable `/rtabmap/map_stable` 에서 map rotation / ghosting은 관찰되지 않았다.
6. `map_topic_stabilizer`의 `timeout / pending=1`은 그대로 남아 있다.
7. 대신 남은 체감 문제는 `footprint 또는 footprint처럼 보이는 consumer overlay가 TF보다 늦게 따라오는 것`이다.

따라서 지금부터의 작업 정의는 아래처럼 바뀐다.

1. 최종 목표: pure spin 중 체감 이상 현상 제거
2. 방금 해결 확인된 선행 원인: `local yaw under-response`
3. 현재 유지할 값: `imu0_twist_rejection_threshold=3.0`
4. 현재 상태 판정: map ghosting은 일단 해결된 것으로 본다
5. 현재 다음 목표: footprint/costmap consumer lag 분리
6. TF가 정상이면 costmap publish/tolerance와 stable relay 정책을 먼저 본다
7. TF 자체가 늦을 때만 EKF predict-to-current-time A/B로 돌아간다
8. 지금은 미루는 것: 추가 `Q/R` 튜닝, `yaw_rate_scale`, raw map source-level rewrite

한 문장으로 끝내면:

**이제 문제의 중심은 EKF local yaw나 raw map ghosting이 아니라, 회전 중 TF보다 늦게 따라오는 footprint/costmap consumer 경로를 정확히 가르는 것이다.**
