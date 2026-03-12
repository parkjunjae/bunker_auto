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
