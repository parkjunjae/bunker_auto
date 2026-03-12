# plan_tfv1.md — 제자리 회전 시 맵 회전 문제 최신 재정리

## 0. 목적

이 문서는 제자리 회전 시 occupancy grid가 로봇 회전과 같이 돌아가거나 뒤틀려 보이는 문제를,
최신 10Hz 로그와 RTAB-Map 소스 기준으로 다시 정리한 문서다.

이 문서의 목표는 세 가지다.

1. 이미 약해진 가설과 아직 살아 있는 가설을 분리한다.
2. IMU/EKF 문제와 RTAB-Map global/map layer 문제를 분리한다.
3. 다음 수정 우선순위를 `TF spike 추적`이 아니라 `raw map redraw 경로 확인`으로 재정렬한다.

---

## 1. 현재 문제를 정확히 어떻게 정의할 것인가

현재 사용자가 해결하려는 문제는 단순히 “RViz가 보기 안 좋다”가 아니다.

정확한 정의는 다음과 같다.

1. 로봇이 제자리 회전을 한다.
2. `odom->base_link`는 그 회전을 따라간다.
3. 그런데 세계좌표 기준이어야 할 global map/occupancy grid가 로봇 회전에 같이 끌려간다.
4. 그 결과 footprint와 map의 상대관계가 깨지고, goal point의 물리적 의미도 달라진다.

즉 목표는 한 문장으로 정리된다.

**로봇이 회전할 때 로봇만 회전해야 하고, map과 occupancy grid는 세계좌표에 고정되어 있어야 한다.**

---

## 2. 최신 10Hz 로그 기준으로 확정된 사실

대상 런:

- [rtabmap_nav2.log](/home/atoz/ca_ws/logs/run_20260312_163745/rtabmap_nav2.log)
- [sensor_sync.log](/home/atoz/ca_ws/logs/run_20260312_163745/sensor_sync.log)

### 2.1 IMU 상태기계(P1)는 fast spin을 놓치지 않는다

다음 로그에서 `yaw_zeroing=OFF reason=hard_wz`가 반복적으로 확인된다.

- [sensor_sync.log:45](/home/atoz/ca_ws/logs/run_20260312_163745/sensor_sync.log:45)
- [sensor_sync.log:179](/home/atoz/ca_ws/logs/run_20260312_163745/sensor_sync.log:179)
- [sensor_sync.log:234](/home/atoz/ca_ws/logs/run_20260312_163745/sensor_sync.log:234)
- [sensor_sync.log:963](/home/atoz/ca_ws/logs/run_20260312_163745/sensor_sync.log:963)

의미:

1. fast pure spin 자체는 IMU가 감지한다.
2. 회전 구간을 “IMU가 몰라서 맵이 도는 것”으로 설명하는 건 현재 기준으로 맞지 않다.
3. 따라서 P1은 현재 주범 후보에서 내려야 한다.

### 2.2 `map_tf_stabilizer` 로그상 TF yaw spike는 보이지 않는다

최신 10Hz 로그에서는 아래 패턴이 장시간 반복된다.

- `raw_yaw=0.000`
- `stable_map_yaw=0.000`
- `score=0.000`
- `gain=1.000`

대표 로그:

- [rtabmap_nav2.log:25](/home/atoz/ca_ws/logs/run_20260312_163745/rtabmap_nav2.log:25)
- [rtabmap_nav2.log:31](/home/atoz/ca_ws/logs/run_20260312_163745/rtabmap_nav2.log:31)
- [rtabmap_nav2.log:100](/home/atoz/ca_ws/logs/run_20260312_163745/rtabmap_nav2.log:100)

현재 데이터가 의미하는 것:

1. 최소 현재 로그 해상도에서는 raw `map->odom` yaw jump가 보이지 않는다.
2. 최소 현재 로그 해상도에서는 filtered `map_stable->map` yaw jump도 보이지 않는다.
3. 따라서 severe twist의 1차 주범을 TF yaw spike로 두는 건 현재 기준으로 근거가 약하다.

주의:

이건 “절대 TF spike가 없다”는 뜻이 아니다.
다만 현재 확보한 `0.05s / 소수점 3자리` 로그 해상도에서 **유의미한 TF yaw spike는 확인되지 않았다**는 뜻이다.

### 2.3 RTAB-Map 파라미터는 여전히 보수적이다

- [rtabmap.launch.py:369](/home/atoz/ca_ws/src/rtabmap_ros/rtabmap_launch/launch/rtabmap.launch.py:369)
- [rtabmap.launch.py:370](/home/atoz/ca_ws/src/rtabmap_ros/rtabmap_launch/launch/rtabmap.launch.py:370)
- [rtabmap.launch.py:372](/home/atoz/ca_ws/src/rtabmap_ros/rtabmap_launch/launch/rtabmap.launch.py:372)

현재 값:

- `RGBD/LinearUpdate = 0.50`
- `RGBD/AngularUpdate = 6.28`
- `Rtabmap/DetectionRate = 5.0`

즉 지금 문제를 “노드가 너무 자주 생겨서 map이 돈다”로 단순하게 설명하는 것도 맞지 않는다.

---

## 3. 기존 가설 세 가지의 재평가

이전 문서들(plan_v14 포함)은 대략 아래 세 후보를 다뤘다.

1. raw `map->odom` TF spike
2. filtered `map_stable->map` TF spike
3. `/rtabmap/map` redraw / grid refresh

### 3.1 후보 1: raw `map->odom` TF spike

현재 판정:

**약화**

이유:

1. 최신 10Hz 로그에서 raw yaw 변화가 관측되지 않는다.
2. 현재 severe twist를 설명하기 위한 주가설로는 힘이 약하다.

### 3.2 후보 2: filtered `map_stable->map` TF spike

현재 판정:

**약화**

이유:

1. latest 10Hz stabilizer 로그에서 stable yaw 변화가 관측되지 않는다.
2. 현재 severe twist가 stabilizer 자체의 출력 흔들림이라고 보기 어렵다.

### 3.3 후보 3: `/rtabmap/map` redraw / occupancy grid refresh

현재 판정:

**아직 살아 있음**

이유:

1. `AngularUpdate=6.28`, `LinearUpdate=0.50`만으로 `/rtabmap/map` redraw 부재를 결론낼 수 없다.
2. RTAB-Map 내부에서 map publication은 keyframe threshold와 완전히 동일한 경로가 아니다.

관련 소스:

- [CoreWrapper.cpp:4632](/home/atoz/ca_ws/src/rtabmap_ros/rtabmap_slam/src/CoreWrapper.cpp:4632)
- [MapsManager.cpp:365](/home/atoz/ca_ws/src/rtabmap_ros/rtabmap_util/src/MapsManager.cpp:365)
- [MapsManager.cpp:649](/home/atoz/ca_ws/src/rtabmap_ros/rtabmap_util/src/MapsManager.cpp:649)
- [MapsManager.cpp:1310](/home/atoz/ca_ws/src/rtabmap_ros/rtabmap_util/src/MapsManager.cpp:1310)

핵심 해석:

1. `AngularUpdate`는 “노드 생성/키프레임 생성” 조건이다.
2. occupancy map publish는 `MapsManager::gridUpdated_` 및 publish 조건과 연결된다.
3. 따라서 “키프레임이 없으니 map redraw도 없다”는 결론은 소스상 성립하지 않는다.

즉 후보 3은 현재 **가장 유력한 축**이다.

---

## 4. 기존 문서들의 잘못된 결론

아래 결론은 현재 기준으로 수정해야 한다.

### 4.1 “plan_v14의 세 후보가 모두 반박되었다”

이건 틀렸다.

정확한 표현:

1. 후보 1, 2는 약해졌다.
2. 후보 3은 아직 살아 있다.

### 4.2 “stable map topic은 안 바뀌는 것을 hold하는 구조라 무의미하다”

이것도 틀렸다.

왜냐하면:

1. 만약 문제의 본체가 `/rtabmap/map` redraw라면
2. stable map topic은 바로 그 redraw가 consumer에 전파되는 경로를 끊는 용도이기 때문이다.

즉 stable map layer는 **반박되지 않았다.**

### 4.3 “근본 원인은 EKF heading 보정 경로 없음”

이것도 현재 우선순위로는 맞지 않는다.

과거 mixed motion/heading mismatch 분석에서는 의미가 있었지만,
최신 pure spin 10Hz 분석 기준으로는 우선순위가 더 낮다.

즉 지금은:

- EKF yaw 보강
보다
- **RTAB-Map raw global/map 결과가 pure spin에서 어떻게 보이는지**

가 먼저다.

---

## 5. 현재 가장 타당한 원인 체계

최신 데이터 기준으로 지금 가장 타당한 해석은 다음과 같다.

### 5.1 local 회전 추종은 대체로 정상이다

1. IMU는 fast spin을 감지한다.
2. `odom->base_link` 쪽은 현재 주범이 아니다.

### 5.2 문제는 global/map layer에 있다

1. TF yaw spike는 현재 로그 기준으로 약하다.
2. 반면 사용자는 분명히 “맵이 돈다”고 본다.
3. 이 불일치는 **occupancy map redraw / grid refresh / redraw 소비 경로**를 의심하게 만든다.

즉 지금 문제는:

**TF가 돌고 있다기보다 map raster가 다시 그려지면서 회전한 것처럼 보이는 문제**

일 가능성이 더 크다.

---

## 6. stable map layer의 현재 위치

stable map layer는 이제 다음처럼 재정의해야 한다.

### 6.1 이것은 단독 근본 해결책이 아니다

stable map layer는 raw RTAB-Map 계산 자체를 고치지 않는다.

즉 아래를 직접 고치진 못한다.

1. raw graph drift
2. raw global correction 오판
3. RTAB-Map 내부 occupancy update 판단

### 6.2 하지만 유효한 보완층이다

stable map layer는 아래를 막을 수 있다.

1. raw `/rtabmap/map` redraw가 Nav2/RViz/global consumer에 즉시 전파되는 것
2. pure spin 동안 global consumer가 map redraw에 흔들리는 것

즉:

- 근본 원인 수정은 아님
- 하지만 **증상 전파 경로 차단**에는 유효함

### 6.3 현재 이 아이디어는 폐기 대상이 아니다

기존 문서처럼 “stable map topic은 무의미하다”고 결론내리면 안 된다.

오히려 현재 데이터 기준으로는:

**candidate 3이 살아 있기 때문에 stable map layer의 필요성은 더 올라간 상태**다.

---

## 7. 수정된 해결 우선순위

### 1순위: `/rtabmap/map` redraw/refresh 경로 확인

확인 대상:

1. `/rtabmap/map`
2. `/rtabmap/map_updates`
3. `/rtabmap/mapGraph`
4. `/rtabmap/info`

목표:

pure spin 구간과 raw map redraw 시점을 같은 시간축에 놓고,
실제로 redraw가 그 시점에 생기는지 확인한다.

### 2순위: redraw가 맞으면 consumer-side 완화층 적용

즉:

- raw map: `/rtabmap/map`
- stable map: `/rtabmap/map_stable`

pure spin 동안:

1. hold-last-good
2. translation 재개 후 refresh

를 적용한다.

### 3순위: redraw가 아니라면 source-level TF/event로 복귀

즉 그때만 다시:

1. raw TF high-rate capture
2. filtered TF high-rate capture
3. source-level TF gating

쪽으로 돌아간다.

---

## 8. 지금 당장 하지 말아야 할 것

1. `AngularUpdate/LinearUpdate`를 주 해결축으로 다시 만지는 것
2. IMU state machine을 다시 주범으로 보는 것
3. ICP odom vyaw 추가를 현 문제의 1차 해결책으로 보는 것
4. stable map 아이디어를 이미 반박된 것으로 취급하는 것

---

## 9. 성공 기준

1. 제자리 회전 중 footprint는 정상 회전한다
2. 같은 구간에 occupancy grid가 사용자 눈에 같이 회전하지 않는다
3. goal point의 세계 기준 의미가 회전 전후 일관된다
4. fast pure spin 이후 global map이 jump하거나 뒤틀리지 않는다
5. 전진/후진/곡선 주행 성능은 유지된다

---

## 10. 현재 최종 결론

최신 10Hz 로그 기준으로:

1. raw `map->odom` TF spike 가설은 약해졌다
2. `map_stable->map` TF spike 가설도 약해졌다
3. `/rtabmap/map` redraw / occupancy refresh 가설은 아직 살아 있다

따라서 현재 주 해결축은:

**TF spike 추적보다 RTAB-Map raw map redraw 경로를 먼저 확인하고, 필요하면 stable map layer로 그 전파를 끊는 것**이다.

---

## 11. 1차 구현 반영 사항

현재 문서 기준으로 아래 1차 구현을 반영했다.

### 11.1 새 relay 노드 추가

- [map_topic_stabilizer.py](/home/atoz/ca_ws/src/rtabmap_ros/rtabmap_launch/scripts/map_topic_stabilizer.py)

역할:

1. raw `/rtabmap/map`를 입력으로 받는다.
2. consumer-facing `/rtabmap/map_stable`를 publish한다.
3. pure spin 동안에는 last-good map을 유지한다.
4. pure spin이 약해지고 translation evidence가 다시 생길 때만 raw map으로 refresh한다.

이 노드는 root cause를 직접 고치는 source-level fix가 아니라,
현재 가장 유력한 `raw map redraw -> consumer propagation` 경로를 끊기 위한
**consumer-side relay layer**다.

### 11.2 launch 연결

- [rtabmap_nav2.launch.py](/home/atoz/ca_ws/src/rtabmap_ros/rtabmap_launch/launch/rtabmap_nav2.launch.py)

새 노드를 launch에 추가했다.

입력:

1. `/rtabmap/map`
2. `/camera/camera/imu_fixed`
3. `/odometry/filtered`

출력:

1. `/rtabmap/map_stable`

중요:

- pure spin detector는 TF stabilizer와 같은 창을 보도록 파라미터를 맞췄다.
- 즉 TF layer와 map topic layer가 가능한 한 같은 pure spin 구간을 보게 했다.

### 11.3 Nav2 consumer 변경

- [nav2_rtabmap_params.yaml](/home/atoz/ca_ws/src/rtabmap_ros/rtabmap_launch/launch/config/nav2_rtabmap_params.yaml)
- [nav2_rtabmap_params_train.yaml](/home/atoz/ca_ws/src/rtabmap_ros/rtabmap_launch/launch/config/nav2_rtabmap_params_train.yaml)

변경:

1. global static layer `map_topic`을 `/rtabmap/map_stable`로 변경
2. `subscribe_to_updates`를 `false`로 변경

의미:

- 이번 1차 구현은 full OccupancyGrid만 relay한다.
- `map_updates` incremental path는 아직 다루지 않는다.
- 즉 raw redraw 전파를 우선 끊고, incremental update 최적화는 다음 단계로 미룬다.

### 11.4 RViz consumer 변경

- [rgbd.rviz](/home/atoz/ca_ws/src/rtabmap_ros/rtabmap_launch/launch/config/rgbd.rviz)

변경:

1. Map display topic을 `/rtabmap/map_stable`로 변경
2. update topic은 `/rtabmap/map_stable_updates`로 분리

주의:

- 이번 단계에서는 `/rtabmap/map_stable_updates`를 실제로 publish하지 않는다.
- 즉 RViz도 full stable map만 우선 보게 만든다.

### 11.5 install 반영

- [CMakeLists.txt](/home/atoz/ca_ws/src/rtabmap_ros/rtabmap_launch/CMakeLists.txt)

변경:

1. `scripts/map_topic_stabilizer.py`를 install 목록에 추가

---

## 12. 1차 구현의 의도와 한계

### 12.1 왜 일부러 범위를 좁혔는가

이번 단계는 다음 질문에 답하기 위한 최소 구현이다.

> “현재 severe twist의 상당 부분이 raw `/rtabmap/map` redraw가 consumer에 바로 전파되는 문제인가?”

그래서 일부러 아래는 하지 않았다.

1. raw RTAB-Map source 수정
2. `map_updates` incremental relay
3. graph/optimizer gating
4. TF 구조 재수정

즉 이번 구현은 **원인 분리용 보완층**이다.

### 12.2 이 구현이 기대하는 효과

1. pure spin 중 raw map redraw가 Nav2 global static layer에 바로 들어가지 않음
2. pure spin 중 RViz가 raw occupancy redraw를 그대로 보여주지 않음
3. 짧은 pure spin에서 global consumer 흔들림이 줄어듦

### 12.3 이 구현이 아직 못 하는 것

1. RTAB-Map 내부 raw graph drift 자체 수정
2. raw `map->odom` source-level correction acceptance 수정
3. `/rtabmap/map_updates` 기반 incremental redraw 억제
4. long pure spin 중 누적되는 raw global inconsistency 해결

즉 이 구현은 단독 근본 해결책이 아니다.

### 12.4 이번 단계의 성공 기준

다음 실차에서 아래가 줄어들면 이번 레이어는 유효하다.

1. 제자리 회전 중 RViz occupancy map 회전/고스팅 체감 감소
2. pure spin 동안 global costmap 흔들림 감소
3. 회전 종료 후 global map jump 빈도 감소

반대로 아래면 이 레이어만으로는 부족하다.

1. `/rtabmap/map_stable`로 바꿔도 severe twist가 그대로
2. 회전 중 TF는 안정적이나 global consumer 오동작 동일
3. pure spin 종료 후 여전히 raw source-level jump가 그대로 보임

그 경우 다음 단계는:

**consumer-side stable map layer가 아니라 raw map redraw source-level gating 또는 RTAB-Map publish path 분석**으로 넘어가야 한다.

---

## 13. `map_topic_stabilizer` 파라미터 설정 이유

이번 stable map relay 구현은 단순히 “토픽 하나 더 만들기”가 아니라,
pure spin 동안 raw `/rtabmap/map` redraw가 consumer에 바로 전파되지 않도록
hold-last-good / translation-based refresh 정책을 넣은 것이다.

따라서 파라미터는 다음 네 원칙에 맞춰 잡았다.

1. pure spin은 빨리 감지한다.
2. 일반 회전/곡선 주행은 hold 대상이 아니어야 한다.
3. hold는 pure spin에서만 유지한다.
4. 복귀는 회전 종료가 아니라 translation 재개 기준으로 한다.

### 13.1 입력/주기 파라미터

#### `tick_hz = 20.0`

의미:

- relay 판단 주기

이유:

1. local odom/EKF 계층과 비슷한 수준의 주기로 돌리기 위함
2. pure spin 진입/해제가 너무 늦지 않게 하기 위함
3. 과도한 CPU 부하나 로그량 증가는 피하기 위함

#### `input_map_topic = /rtabmap/map`
#### `output_map_topic = /rtabmap/map_stable`

의미:

- raw source와 consumer-facing source 분리

이유:

1. raw map redraw는 계속 관측하되
2. Nav2/RViz는 stable map만 소비하게 하기 위함

#### `stable_map_frame_id = map_stable`

의미:

- stable map의 `header.frame_id`

이유:

1. 현재 TF 체인이 `map_stable -> map -> odom -> base_link`이므로
2. stable map도 `map_stable` 기준으로 보여야 TF와 map 해석이 일치함

### 13.2 pure spin 검출 파라미터

이 값들은 [map_tf_stabilizer.py](/home/atoz/ca_ws/src/rtabmap_ros/rtabmap_launch/scripts/map_tf_stabilizer.py)와 같은 pure spin 창을 보도록 맞췄다.

즉 목표는:

- TF layer와 map layer가 서로 다른 spin 구간을 보는 상황을 피하는 것

#### `wz_filter_tau_sec = 0.06`

의미:

- IMU `wz` EMA 필터 시정수

이유:

1. pure spin 시작을 빨리 잡아야 함
2. 한두 샘플 노이즈로 즉시 흔들리면 안 됨
3. 그래서 짧은 시정수로 빠르게 반응하되 약간의 jitter는 제거

#### `speed_filter_tau_sec = 0.12`

의미:

- odom speed EMA 필터 시정수

이유:

1. speed는 IMU보다 jitter가 더 큰 편
2. pure spin 여부를 볼 때 translation speed가 샘플 단위로 흔들리면 hold 경계가 불안정해짐
3. 그래서 `wz`보다 느리게 필터링

#### `spin_wz_start = 0.08`

의미:

- pure spin 후보로 보기 시작하는 최소 `|wz|`

이유:

1. 이전 분석에서 실제 problematic spin이 `~0.10 rad/s` 근처에서도 나타남
2. 예전 `0.35` 같은 값으로는 pure spin을 놓쳤음
3. 그래서 실제 문제 구간 바로 아래로 내림

#### `spin_wz_full = 0.18`

의미:

- `wz`만 봐도 pure spin score가 거의 충분히 올라가는 상단 기준

이유:

1. `0.08`부터 감지는 시작하되
2. `0.18` 정도면 “이건 명확한 pure spin”으로 봄
3. 너무 낮으면 일반 회전도 pure spin으로 오인
4. 너무 높으면 fast spin을 또 늦게 잡음

#### `spin_speed_quiet = 0.05`

의미:

- pure spin으로 인정할 수 있는 최대 translation speed 근처 기준

이유:

1. pure spin은 translation이 거의 없어야 함
2. 하지만 odom 노이즈와 미끄러짐 때문에 완전 0을 요구하면 검출이 불안정해짐
3. 약간의 residual translation은 허용하되, 움직이며 회전하는 구간과는 분리하려는 값

### 13.3 score 구성 파라미터

#### `spin_duration_ref_sec = 0.80`

의미:

- “지속 pure spin”이라고 확신하기 위한 시간 스케일

이유:

1. joystick twitch와 지속 pure spin을 구분하려는 목적
2. 너무 짧으면 순간 회전에도 hold가 걸림
3. 너무 길면 pure spin 감지가 늦어짐

#### `spin_yaw_ref_rad = 0.20`

의미:

- 누적 yaw 기준

이유:

1. 시간만 보면 low-rate spin과 short twitch를 구분하기 어렵다
2. 실제 회전량도 같이 봐야 함
3. 약 `11.5도` 정도 누적되면 “pure spin이 실제로 진행 중”이라는 느린 증거로 충분하다고 봄

#### `spin_decay_sec = 0.80`

의미:

- spin evidence를 잊는 시간 상수

이유:

1. pure spin이 끝나면 score가 바로 0이 되면 경계에서 흔들림
2. 너무 오래 남으면 hold가 지나치게 길어짐
3. 따라서 evidence를 적당히 천천히 감쇠

#### `score_fast_weight = 0.75`
#### `score_slow_weight = 0.25`

의미:

- fast term / slow term 가중치

이유:

1. 이번 문제는 “감지를 못해서 늦다”가 더 컸음
2. 따라서 `wz + 저속 speed` 기반 fast term에 더 큰 비중을 둠
3. duration/누적 yaw는 보조 확신 성분만 담당

#### `score_rise_tau_sec = 0.05`
#### `score_fall_tau_sec = 0.50`

의미:

- pure spin score의 상승/하강 응답 시간

이유:

1. spin 시작은 빨리 감지해야 함
2. spin 종료는 조금 더 천천히 내려가야 hold 경계에서 흔들리지 않음

### 13.4 hold / refresh 정책 파라미터

이 값들이 stable map layer의 본체다.

#### `hold_enter_score = 0.55`

의미:

- hold-last-good map 진입 기준

이유:

1. pure spin 확신이 충분할 때만 hold를 걸어야 함
2. score가 조금 오른 것만으로 hold하면 일반 회전/곡선 주행도 stale map이 됨

#### `hold_exit_score = 0.20`

의미:

- hold 해제 기준

이유:

1. enter보다 충분히 낮아야 hysteresis가 생김
2. pure spin 경계에서 hold on/off 떨림을 막음

#### `translation_release_speed = 0.06`

의미:

- stable map refresh를 다시 허용하기 위한 translation evidence 기준

이유:

1. “회전이 끝났다”는 이유만으로 raw map을 바로 복귀시키면 안 됨
2. pure spin이 아니라 실제 이동이 다시 생겼을 때만 global map을 다시 믿도록 하기 위함

#### `translation_release_hold_sec = 0.30`

의미:

- translation evidence가 유지되어야 하는 최소 시간

이유:

1. 한두 샘플 speed 튐만으로 refresh를 열면 snap-back 위험이 큼
2. 실제 주행 재개인지 짧게 확인하기 위한 값

#### `refresh_settle_sec = 0.25`

의미:

- hold 해제 직후 raw map으로 바로 갈아타지 않기 위한 settle 시간

이유:

1. hold가 풀렸다고 즉시 raw map을 쓰면 종료 후 jump가 날 수 있음
2. 짧게 안정화한 뒤 refresh하도록 하기 위함

#### `max_hold_sec = 4.0`

의미:

- stable map hold 최대 시간

이유:

1. pure spin이 너무 길어지면 stale map을 영원히 유지할 수 없음
2. long pure spin에서는 timeout 경로도 열어둬야 함
3. 다만 이 값은 이후 실차에서 다시 조정될 가능성이 큼

#### `log_period_sec = 0.05`

의미:

- 현재 단계의 디버깅용 로그 주기

이유:

1. 이번 분석은 high-rate 관측이 필요함
2. pure spin 진입/유지/해제 타이밍을 촘촘히 보기 위한 값
3. 운영 기본값이라기보다 현재 진단 단계용 값

### 13.5 Nav2 static layer 변경 이유

이번 1차 구현에서 아래도 같이 바꿨다.

- `map_topic: /rtabmap/map_stable`
- `subscribe_to_updates: false`

이유:

1. 이번 단계는 full OccupancyGrid relay만 구현
2. `/rtabmap/map_updates` incremental path까지 같이 다루면 범위가 커짐
3. 우선은 “raw full map redraw 전파 차단”만 확인
4. incremental update relay는 다음 단계로 미룸

### 13.6 이번 파라미터 세트의 의미

한 문장으로 정리하면:

**pure spin은 빨리 잡고, 일반 회전은 hold하지 않으며, pure spin 동안에는 last-good map을 유지하고, 회전 종료가 아니라 translation 재개를 기준으로 raw map을 다시 수용하도록 만든 값들**이다.
