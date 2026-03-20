# plan_obsv2.md

## 1. 목표

이 문서는 **로봇 앞에 실제 장애물이 있는데도 RL 기반 로컬 컨트롤러가 그 장애물을 가로질러 지나가려는 문제**를 현재 코드베이스 기준으로 다시 정리한 문서다.

이번 버전은 특히 아래 현실을 반영한다.

1. `inflation_radius`를 크게 키우면 안전해지기는 하지만, 실제로 지나갈 수 있는 통로까지 막혀 버린다.
2. 그래서 이번 문제는 **inflation 확대로 푸는 방식이 아니라**, `footprint_padding`, `lookahead_dist`는 최소 운영 보정으로만 두고,
3. **RL controller 내부에 “후보 명령 미래 시뮬레이션 + footprint polygon 충돌 검사”를 추가하는 방향**으로 해결해야 한다.

즉 이 문서의 핵심 질문은 이것이다.

- 왜 지금 controller가 장애물을 가로질러 보게 되는가?
- 왜 inflation을 더 키우는 것이 정답이 아닌가?
- 그렇다면 RL controller를 어떻게 바꿔야 실제 차체 기준으로 안전하게 판단하게 만들 수 있는가?


## 2. 현재 관찰된 현상

사용자가 올린 화면과 현재 설정을 기준으로 보면 증상은 아래처럼 정리된다.

1. costmap 상 전방 장애물은 분명 존재한다.
2. 그런데 controller가 장애물을 넓게 우회하기보다, 장애물 가장자리나 장애물 사이의 애매한 공간을 타고 지나가려는 명령을 만든다.
3. 이 현상은 단순한 RViz 표시 지연과 다르다.
4. 즉 문제는 “장애물이 안 보인다”가 아니라, **“보이는 장애물에 대해 차체 폭을 충분히 반영한 충돌 판단을 하지 못한다”** 쪽에 가깝다.


## 3. 현재 코드/설정에서 확인한 사실

아래는 실제 워크스페이스 코드와 현재 설정 파일을 읽고 확인한 내용이다.

### 3.1 현재 controller 운영 파라미터는 이미 일부 보정이 들어간 상태다

파일:
- [nav2_rtabmap_params.yaml](/home/atoz/ca_ws/src/rtabmap_ros/rtabmap_launch/launch/config/nav2_rtabmap_params.yaml)

현재 반영된 값:

```yaml
lookahead_dist: 0.7
cost_threshold: 220
```

local costmap:

```yaml
footprint_padding: 0.03
```

즉 현재는 이미 아래 방향의 완화가 일부 들어가 있다.

- `lookahead_dist`를 줄여 corner-cutting을 줄이려 함
- `footprint_padding`을 0에서 0.03으로 올려 차체 여유를 조금 더 보수적으로 보게 함
- `cost_threshold`도 253에서 220으로 낮춰 inflated cost를 좀 더 일찍 위험으로 보게 함

이건 의미 있는 조정이다. 하지만 이것만으로는 **구조적인 한계**를 해결하지 못한다.


### 3.2 footprint는 크지만, costmap inflation을 크게 키우는 방식은 부작용이 크다

현재 footprint:

```yaml
footprint: "[[0.62, 0.389], [0.62, -0.389], [-0.62, -0.389], [-0.62, 0.389]]"
```

즉 대략:

- 길이 약 `1.24 m`
- 폭 약 `0.778 m`

현재 inflation 설정:

```yaml
local_costmap:
  inflation_layer:
    cost_scaling_factor: 4.0
    inflation_radius: 0.07

global_costmap:
  inflation_layer:
    cost_scaling_factor: 1.5
    inflation_radius: 0.07
```

런타임 로그도 현재 inflation이 footprint 내접반경보다 작다고 직접 경고한다.

- [rtabmap_nav2.log](/home/atoz/ca_ws/logs/run_20260320_095405/rtabmap_nav2.log#L457)
- [rtabmap_nav2.log](/home/atoz/ca_ws/logs/run_20260320_095405/rtabmap_nav2.log#L470)

[사실]

- 현재 `0.07`은 footprint 기준으로 너무 작다.

[사실]

- 하지만 사용자가 직접 경험했듯, local/global inflation을
  - local `0.18 ~ 0.22`
  - global `0.22 ~ 0.30`
  수준으로 올리면 장애물이 지나치게 넓게 번져 보이고, 실제로는 통과 가능한 corridor까지 막히는 부작용이 발생한다.

[추론]

- 따라서 이번 문제는 “inflation이 작으니 무조건 크게 키우자”로 풀면 안 된다.
- inflation은 분명 배경 원인이지만, **이번 최종 해결축은 아니다.**


### 3.3 RL controller는 현재 중심점 raycast 기반 휴리스틱에 크게 의존한다

파일:
- [rl_local_controller.cpp](/home/atoz/ca_ws/src/rl_local_controller/src/rl_local_controller.cpp)
- [rl_local_controller.hpp](/home/atoz/ca_ws/src/rl_local_controller/include/rl_local_controller/rl_local_controller.hpp)

현재 구조를 보면:

1. `raycastDistance()`가 현재 pose의 중심에서 ray를 쏜다.
2. `computeSectorDistances()`가 `front`, `left`, `right`를 만든다.
3. 그 거리값들로 전진/회전/정지/탈출 명령을 만든다.

핵심 코드:

```cpp
const unsigned char cost = costmap->getCost(mx, my);
if (static_cast<int>(cost) >= cost_threshold_) {
  return dist;
}
```

즉 현재 장애물 판단의 중심은:

- 중심 기준 raycast
- 셀 cost threshold 비교
- 섹터별 최소 거리

이다.

[사실]

- 현재 구현에는 **후보 명령을 앞쪽으로 여러 스텝 시뮬레이션하고**
- **각 미래 pose에서 실제 footprint polygon이 costmap과 충돌하는지 검사하는 단계가 없다.**

이 점이 이번 문제의 핵심이다.


### 3.4 왜 padding과 lookahead만으로는 한계가 남는가

`footprint_padding`과 `lookahead_dist`는 둘 다 의미가 있다.

- `footprint_padding`
  - 차체 외곽을 costmap 상에서 조금 더 보수적으로 본다.
- `lookahead_dist`
  - path를 너무 멀리 보고 장애물 모서리를 잘라먹는 경향을 줄인다.

하지만 이 둘은 어디까지나 **행동을 덜 공격적으로 만드는 운영 튜닝**이다.

[추론]

- `padding`은 차체를 조금 더 크게 본다.
- `lookahead`는 경로 추종을 조금 더 조심스럽게 만든다.
- 그러나 둘 다 “이 명령을 0.8초 동안 실행했을 때 차체 polygon이 실제로 부딪히는가?”를 계산해 주지는 않는다.

즉:

- `padding/lookahead`는 “경향을 완화”
- `footprint collision check`는 “충돌 명령을 금지”

다.


## 4. 이번 문제의 핵심 해석

이번 문제는 아래처럼 보는 것이 가장 자연스럽다.

### 4.1 inflation은 배경 원인이지만, 주 해결 레버가 아니다

[사실]

- inflation이 너무 작으면 danger band가 충분히 퍼지지 않는다.

[사실]

- 그러나 inflation을 크게 키우면 corridor 전체가 막힌다.

[추론]

- 따라서 이번 문제는 costmap inflation 하나만으로는 “충돌 방지”와 “통과성 유지”를 동시에 만족시키기 어렵다.


### 4.2 본체 문제는 controller가 “차체” 대신 “중심선”을 더 강하게 보고 있다는 점이다

[사실]

- 현재 controller는 중심 raycast 기반이다.

[추론]

- 그래서 중심선은 지나갈 수 있어 보여도,
- 실제 차체 좌우 모서리는 못 지나가는 상황을 오판할 수 있다.

즉 이번 문제는 “장애물이 안 보이는가?”보다,
**“후보 명령이 실제 차체 외곽까지 고려되었는가?”**의 문제다.


### 4.3 이번 버전의 전략은 명확하다

1. `padding`과 `lookahead`는 최소 운영 보정으로 유지
2. inflation은 지나치게 넓히지 않음
3. 최종 충돌 판정은 RL controller 내부에서 미래 pose + footprint polygon 검사로 수행

즉 역할 분리는 아래처럼 가져간다.

- inflation: “장애물 가까이는 싫다”는 비용 힌트
- padding/lookahead: “행동을 덜 공격적으로 만드는 운영 튜닝”
- footprint collision check: “이 명령은 실제 차체 기준으로 충돌하므로 금지”


## 5. 현재 채택하는 1차 운영 보정

### 5.1 `footprint_padding`

현재 반영값:

```yaml
footprint_padding: 0.03
```

이 값을 둔 이유:

- 센서 marking 지연
- costmap 리샘플링 오차
- yaw 오차
- footprint 외곽과 실제 차체 외곽의 작은 불일치

를 조금 흡수하기 위해서다.

이건 corridor를 죽일 정도의 큰 inflation이 아니라,
**차체 가장자리 오차를 최소한으로 보수화하는 값**이다.


### 5.2 `lookahead_dist`

현재 반영값:

```yaml
lookahead_dist: 0.7
```

문서 기준 권장 탐색값:

- `0.7` 현재값 유지 후 실험
- 더 줄일 필요가 있으면 `0.6`

이 값을 줄이는 이유:

- 너무 먼 룩어헤드는 path를 “예쁘게 잇는 것”에 치우쳐
- 장애물 모서리를 깎아먹는 corner-cutting을 유도할 수 있기 때문이다.

즉 이 값은 **부드러운 곡선보다 실제 장애물 회피를 조금 더 우선**하게 만드는 조정이다.


### 5.3 왜 이것만으로는 끝이 아닌가

[사실]

- padding과 lookahead는 이미 일부 반영되었다.

[추론]

- 그런데도 여전히 “장애물을 가로질러 보려는” 상황이 남는다면,
- 이는 설정 튜닝의 부족보다 **controller 구조의 한계**를 더 강하게 시사한다.

따라서 다음 단계는 설정 미세조정이 아니라,
**controller 코드 수정**으로 넘어가는 것이 맞다.


## 6. 구조적으로 제대로 고치는 방법

### 6.1 목표

현재 RL controller는 후보 명령을 만들 때:

- “앞이 비어 보이는가?”
- “좌우 어느 쪽이 더 넓어 보이는가?”

를 본다.

이제는 여기에 아래 질문을 추가해야 한다.

- **“이 명령을 실제로 0.8초 정도 실행하면, 로봇 footprint polygon이 costmap과 충돌하는가?”**

이 질문을 넣는 것이 이번 구조 수정의 핵심이다.


### 6.2 어떻게 수정할 것인가

수정 방향은 아래 순서가 가장 자연스럽다.

#### 단계 1: 현재 휴리스틱은 유지한다

기존 raycast 로직은 완전히 버리지 않는다.

이유:

- 전방/좌우 여유를 빠르게 읽는 coarse heuristic으로는 여전히 유용하다.
- 이미 현재 controller의 전반적 행동 구조와 잘 맞는다.

즉 기존 역할은 그대로 둔다.

- raycast = 빠른 후보 생성기


#### 단계 2: 후보 명령을 만든 뒤 미래 pose를 forward simulate 한다

현재 pose `(x, y, yaw)`와 후보 명령 `(v, w)`가 있으면,
고정 시간 간격 `dt`로 몇 스텝 앞까지 자세를 적분한다.

예:

- `sim_horizon_sec = 0.8`
- `sim_dt = 0.1`

이면 총 8개 pose를 검사한다.

이 단계의 목적은:

- “지금 당장은 안 부딪혀 보여도”
- “이 명령을 0.8초만 유지하면 차체 모서리가 닿는 경우”

를 미리 차단하는 것이다.


#### 단계 3: 각 미래 pose마다 footprint polygon을 실제로 놓아 본다

이 단계에서는 중심점 하나가 아니라,
**현재 로봇 footprint polygon 전체**를 해당 미래 pose에 배치한다.

사용할 수 있는 자원:

- [footprint_collision_checker.hpp](/opt/ros/humble/include/nav2_costmap_2d/nav2_costmap_2d/footprint_collision_checker.hpp)
- [footprint.hpp](/opt/ros/humble/include/nav2_costmap_2d/nav2_costmap_2d/footprint.hpp)

즉:

1. padded footprint를 가져오고
2. 해당 `(x, y, yaw)`에 맞춰 footprint를 회전/이동시키고
3. costmap과 충돌하는지 검사한다

는 흐름으로 간다.


#### 단계 4: 충돌이 예측되면 후보 명령을 강등하거나 폐기한다

기존 controller는 명령을 만든 뒤 거의 바로 publish한다.

앞으로는 아래처럼 바꾼다.

1. 원래 후보 명령 `(v_des, w_des)` 검사
2. 충돌 예측이면
   - `v=0` 회전 위주 명령 검사
3. 그것도 충돌이면
   - 더 작은 각속도 / 반대 방향 회전 / 저속 후진 탈출 후보 검사
4. 그래도 충돌이면
   - 정지 또는 recovery로 넘김

즉 구조적으로는:

- “먼저 내보내고 부딪히지 않길 바란다”

가 아니라

- **“내보내기 전에 실제 차체로 한 번 걸러서 보낸다”**

로 바뀐다.


## 7. RL controller에서 구체적으로 어떤 코드를 추가할 것인가

### 7.1 헤더에 추가할 요소

파일:
- [rl_local_controller.hpp](/home/atoz/ca_ws/src/rl_local_controller/include/rl_local_controller/rl_local_controller.hpp)

추가할 핵심은 세 묶음이다.

1. 시뮬레이션 파라미터
   - `sim_horizon_sec_`
   - `sim_dt_`
   - `footprint_collision_cost_threshold_`

2. 보조 함수
   - `simulatePoseStep(...)`
   - `isPoseCollisionFree(...)`
   - `isCommandCollisionFree(...)`

3. 필요하면 fallback 후보 검사 함수
   - `selectCollisionFreeCommand(...)`


### 7.2 소스에서 추가할 핵심 로직

파일:
- [rl_local_controller.cpp](/home/atoz/ca_ws/src/rl_local_controller/src/rl_local_controller.cpp)

현재 `computeVelocityCommands()` 흐름은:

1. sector distance 계산
2. 목표 방향 계산
3. 후보 `v_des`, `w_des` 생성
4. PID 보정
5. publish

변경 후 흐름은:

1. sector distance 계산
2. 목표 방향 계산
3. 후보 `v_des`, `w_des` 생성
4. **후보 명령을 미래로 시뮬레이션**
5. **각 미래 pose에서 footprint collision 검사**
6. 통과한 명령만 PID 보정 후 publish

즉 insertion point는 **후보 속도를 다 만든 직후, PID 적용 직전**이 가장 적절하다.


### 7.3 왜 이 위치에 넣는가

[추론]

- 너무 앞단에 넣으면 아직 후보 명령이 없어서 검사할 대상이 없다.
- 너무 뒷단에 넣으면 PID가 조정한 속도와 실제 검사 속도가 달라질 수 있다.

그래서 가장 자연스러운 위치는:

- 원본 제어 명령이 결정된 뒤
- 저수준 추종용 PID에 넣기 직전

이다.


## 8. 이 방식의 이점

### 이점 1: 통로를 살릴 수 있다

이 방식의 가장 큰 장점은,
**지나갈 수 있는 통로와 지나가면 안 되는 틈을 inflation 하나로 구분하려 하지 않는다는 점**이다.

즉:

- inflation을 크게 키워 corridor 전체를 막지 않아도 되고
- 실제 차체가 닿는 후보 명령만 controller에서 걸러낼 수 있다

이건 사용자가 실제로 겪은 문제,

- inflation을 키우면 통로를 못 지나감
- inflation을 유지하면 장애물을 가로질러 감

을 동시에 다루는 가장 현실적인 방향이다.


### 이점 2: 중심선 착시를 줄인다

현재 문제의 본체는:

- 중심선은 비어 보이는데
- 좌우 모서리는 실제로 부딪히는 상황

을 controller가 충분히 못 잡는 데 있다.

footprint polygon 검사를 넣으면:

- 중심선이 아니라
- 실제 차체 외곽 전체

를 기준으로 판단하게 된다.

즉 “중심은 비었으니 가자” 착시를 크게 줄일 수 있다.


### 이점 3: padding과 lookahead의 역할이 더 명확해진다

지금은 padding과 lookahead가 사실상 “충돌 방지까지 떠안는” 상태에 가깝다.

하지만 footprint collision check가 들어가면:

- `padding`은 작은 운영 여유 흡수
- `lookahead`는 corner-cutting 완화
- collision check는 실제 충돌 금지

로 역할이 분리된다.

이렇게 되면 개별 파라미터 튜닝도 훨씬 안정적이다.


### 이점 4: 좁은 환경에서 튜닝 민감도가 줄어든다

폭이 큰 로봇이 좁은 공간을 다닐 때 inflation 중심 접근은 원래 매우 예민하다.

- 조금만 키워도 길이 없어 보이고
- 조금만 줄여도 지나치게 낙관적이 된다

footprint-aware gating이 들어가면,
inflation을 완벽한 “충돌 금지 장치”로 만들 필요가 없어진다.

즉 costmap 튜닝 부담이 줄고, 실제 행동 안전성은 오히려 올라간다.


### 이점 5: 현재 RL controller 구조를 버리지 않아도 된다

이 방법은 현재 controller를 갈아엎는 방식이 아니다.

현재 구조:

- raycast로 빠르게 상황 파악
- 기존 규칙 기반으로 후보 명령 생성

을 유지한 채,

- 최종 안전 게이트만 추가

하는 방식이다.

즉 구현 리스크 대비 효과가 크다.


## 9. 검증 계획

### Phase A: 운영 보정 유지

현재 값:

```yaml
footprint_padding: 0.03
lookahead_dist: 0.7
cost_threshold: 220
```

이 값으로 기본 행동을 확인한다.

판정:

- 모서리 잘라먹기가 얼마나 줄었는지
- 하지만 여전히 “못 지나가는 틈을 들어가려는지”


### Phase B: controller 코드 수정

1. 후보 명령 미래 시뮬레이션 추가
2. footprint polygon collision check 연결
3. 충돌 시 대체 명령/정지/회복 분기 추가

판정:

- 장애물 앞에서는 더 이상 중심선만 보고 전진하지 않아야 한다
- 통과 가능한 corridor는 계속 통과 가능해야 한다


### Phase C: 실주행 확인

다음 세 경우를 반드시 분리해서 본다.

1. 정면 장애물 접근
2. 장애물 모서리 스치기 상황
3. 실제로는 통과 가능한 좁은 통로

성공 기준:

- 못 지나가는 틈은 안 들어간다
- 지나갈 수 있는 통로는 과하게 막지 않는다
- corner-cutting이 유의미하게 줄어든다


## 10. 최종 결론

이번 문제는 단순히 inflation이 작아서만 생긴 문제가 아니다.

더 정확히는:

1. 현재 inflation은 footprint 기준으로 충분치 않지만
2. inflation을 크게 키우면 corridor 자체가 막히는 부작용이 있고
3. 현재 RL controller는 중심 raycast 위주의 휴리스틱으로 후보 명령을 만들며
4. 실제 footprint polygon 충돌 검사가 빠져 있어서
5. 장애물을 “가로질러도 된다”고 오판하는 구조다

따라서 이번 문서 기준의 올바른 해결 방향은 아래와 같다.

1. `footprint_padding`, `lookahead_dist`는 최소 운영 보정으로 유지
2. inflation은 과하게 키우지 않는다
3. **RL controller에 후보 명령 forward simulation + footprint polygon collision check를 넣어 실제 차체 기준으로 충돌 명령을 차단한다**

즉 이번 문제의 본체 해결은
**costmap을 더 크게 부풀리는 것**이 아니라,
**controller가 “중심점” 대신 “실제 차체”를 보도록 바꾸는 것**이다.


## 11. 이번 턴에서 실제 반영한 코드 수정

이번 턴에서는 위 계획 중 핵심 구조 수정이 실제 코드에 반영되었다.

수정 파일:

- [rl_local_controller.hpp](/home/atoz/ca_ws/src/rl_local_controller/include/rl_local_controller/rl_local_controller.hpp)
- [rl_local_controller.cpp](/home/atoz/ca_ws/src/rl_local_controller/src/rl_local_controller.cpp)
- [nav2_rtabmap_params.yaml](/home/atoz/ca_ws/src/rtabmap_ros/rtabmap_launch/launch/config/nav2_rtabmap_params.yaml)

### 11.1 헤더에 추가한 것

아래 helper와 파라미터를 추가했다.

1. `simulatePoseStep(...)`
   - 후보 명령을 짧은 시간 앞으로 적분하기 위한 함수
2. `isPoseCollisionFree(...)`
   - 특정 pose에 footprint polygon을 놓고 충돌 여부를 검사하는 함수
3. `isCommandCollisionFree(...)`
   - 후보 명령을 여러 스텝 앞까지 시뮬레이션하며 검사하는 함수
4. `selectCollisionFreeCommand(...)`
   - 원래 후보가 충돌이면 더 안전한 명령으로 강등하는 함수

추가 파라미터:

```yaml
enable_footprint_collision_gate: true
sim_horizon_sec: 0.8
sim_dt: 0.1
footprint_collision_cost_threshold: 253
```


### 11.2 소스에서 실제로 바뀐 제어 흐름

이전 흐름:

1. raycast 기반 `front / left / right` 계산
2. 후보 `v_des`, `w_des` 생성
3. PID 보정 후 publish

현재 흐름:

1. raycast 기반 `front / left / right` 계산
2. 후보 `v_des`, `w_des` 생성
3. **footprint collision gate 적용**
4. 통과한 명령만 PID 보정 후 publish

즉 지금은 기존 controller를 버린 것이 아니라,
**기존 후보 생성기 위에 2차 안전 게이트를 덧씌운 구조**다.


### 11.3 collision gate에서 실제로 하는 일

현재 구현은 아래 순서로 동작한다.

1. 원래 후보 명령이 collision-free인지 검사
2. 실패하면 제자리 회전 후보 검사
3. 그래도 실패하면 반대 방향 회전 후보 검사
4. 필요 시 저속 후진 + 회전 후보 검사
5. 모두 실패하면 정지

즉 이 구현의 본질은:

- “전진 명령을 일단 내보내고 부딪히지 않길 바란다”

가 아니라

- **“내보내기 전에 실제 차체로 한번 검증하고, 위험하면 더 안전한 명령으로 강등한다”**

는 점이다.


### 11.4 왜 PID 앞단에 넣었는가

[추론]

- 후보 명령이 결정되기 전에는 검사할 대상이 없다.
- 반대로 PID 이후에 검사하면, 실제 publish되는 속도와 검사 대상 속도가 어긋날 수 있다.

그래서 가장 자연스러운 위치는:

- 후보 `v_des`, `w_des`가 만들어진 직후
- PID 적용 직전

이다.


## 12. 이번 수정으로 기대하는 직접 효과

### 12.1 장애물 가장자리 가로지르기 감소

가장 직접적인 효과는 이것이다.

- 중심선은 비어 보여도
- 실제 차체 모서리가 닿는 명령

을 전방 시뮬레이션 단계에서 미리 차단하므로,
장애물 가장자리나 좁은 틈을 억지로 가로지르려는 행동이 줄어들어야 한다.


### 12.2 passable corridor 보존

이 수정은 inflation을 크게 키운 것이 아니다.

즉 corridor를 통째로 막지 않고도,
실제로 부딪히는 명령만 controller에서 걸러낼 수 있다.

이 점이 이번 수정의 가장 큰 실무적 장점이다.


### 12.3 tuning 민감도 완화

기존에는:

- inflation이 작으면 너무 낙관적
- inflation이 크면 통로가 막힘

이라는 양극단이 있었다.

이번 수정이 들어가면:

- costmap 파라미터는 “대략적인 비용 힌트”
- controller collision gate는 “실제 차체 안전 장치”

로 역할이 분리되므로, 파라미터 튜닝이 조금 덜 예민해질 가능성이 크다.


### 12.4 현재 파라미터와의 관계

현재 반영된 운영 보정:

```yaml
footprint_padding: 0.03
lookahead_dist: 0.7
cost_threshold: 220
```

이 값들은 여전히 의미가 있다.

- `footprint_padding`
  - 작은 오차와 센서 지연을 흡수
- `lookahead_dist`
  - corner-cutting 완화
- `cost_threshold`
  - inflated band를 너무 늦게 읽지 않게 보조

하지만 이제 이 값들은 “충돌 방지의 전부”가 아니라,
**collision gate를 돕는 운영 튜닝**으로 자리잡는다.


## 13. 수정 후 새로 확인된 문제

footprint collision gate를 넣은 뒤, 사용자가 새로 관찰한 증상은 아래와 같다.

1. 로봇이 goal 방향으로 전진하지 못한다.
2. 제자리에서 짧은 좌/우 회전만 반복한다.
3. 결과적으로 goal point 방향 진행이 거의 없거나 매우 적다.
4. Nav2는 결국 진행 실패로 판단한다.

최신 런타임 로그 근거:

- [rtabmap_nav2.log](/home/atoz/ca_ws/logs/run_20260320_131150/rtabmap_nav2.log#L2338)
- [rtabmap_nav2.log](/home/atoz/ca_ws/logs/run_20260320_131150/rtabmap_nav2.log#L3127)

[사실]

- `controller_server`는 두 번 `Failed to make progress`를 기록했다.

즉 지금 상태는 “충돌은 더 조심하려 하지만, 진행성(progress)을 잃은 상태”로 보는 것이 맞다.


## 14. 이 현상의 원인 해석

### 14.1 원인 1: collision gate가 너무 이른 단계에서 forward 명령을 차단할 가능성이 크다

현재 코드:

- [rl_local_controller.cpp](/home/atoz/ca_ws/src/rl_local_controller/src/rl_local_controller.cpp#L449)
- [rl_local_controller.cpp](/home/atoz/ca_ws/src/rl_local_controller/src/rl_local_controller.cpp#L602)
- [rl_local_controller.cpp](/home/atoz/ca_ws/src/rl_local_controller/src/rl_local_controller.cpp#L631)

[사실]

- gate는 매 주기마다 항상 동작한다.
- 후보 명령을 `0.8초` 앞까지 `0.1초` 간격으로 검사한다.
- footprint가 `253` 이상 cost와 닿으면 바로 충돌로 판정한다.
- `footprint_padding=0.03`이 적용된 padded footprint를 그대로 쓴다.

[추론]

- 이 조합은 “실제 접촉 직전”만 막는 것이 아니라,
- **조금만 보수적으로 닿아도 forward 명령을 빠르게 탈락시키는 구조**일 수 있다.
- 특히 좁은 환경, 장애물 가장자리, inflation band 근처에서는
  원래 후보 전진 명령이 거의 매번 탈락할 가능성이 높다.


### 14.2 원인 2: 현재 fallback 순서가 너무 빨리 “제자리 회전”으로 강등된다

현재 fallback 후보:

- 선호 방향 제자리 회전
- 반대 방향 제자리 회전
- 저속 후진 + 회전
- 최종 정지

관련 코드:
- [rl_local_controller.cpp](/home/atoz/ca_ws/src/rl_local_controller/src/rl_local_controller.cpp#L644)

[사실]

- 현재 구현에는 “감속된 전진 + 같은 조향” 같은 중간 후보가 없다.
- 즉 원래 forward 명령이 한번 충돌로 판정되면,
  거의 바로 회전 후보로 넘어간다.

[추론]

- 이 때문에 controller가 “조금 더 천천히 전진하며 빠져나갈” 기회를 잃고,
- **즉시 회전 위주 행동으로 떨어지기 쉽다.**


### 14.3 원인 3: turn direction에 대한 유지(hysteresis)가 부족해 좌우 짧은 회전이 번갈아 나올 수 있다

현재 코드:
- [rl_local_controller.cpp](/home/atoz/ca_ws/src/rl_local_controller/src/rl_local_controller.cpp#L455)

[사실]

- `preferred_turn_dir`는 매 주기마다 다시 계산된다.
- 계산 근거는 `lr_diff`, 현재 `w_des`, `heading_error`, `last_turn_dir_` 순이다.

[추론]

- 장애물 환경에서 좌우 여유 차이가 프레임마다 조금만 흔들려도,
- 또는 heading error와 avoidance term이 미세하게 바뀌어도,
- 선호 회전 방향이 짧게 좌/우로 바뀔 수 있다.
- 그러면 gate는 매 주기마다 다른 회전 후보를 “안전한 대체”로 선택하고,
- 사용자는 **제자리에서 짧게 좌우로 흔들리는 현상**으로 보게 된다.


### 14.4 원인 4: gate는 진행 가능성(progress)보다 충돌 회피를 우선하고 있다

[사실]

- 현재 gate는 “충돌하느냐”만 본다.
- “이 대체 명령이 실제로 goal 쪽 진행을 만들 수 있느냐”는 보지 않는다.

[추론]

- 그래서 collision-free이기만 하면,
- 전진성이 거의 없는 짧은 회전 명령도 계속 채택될 수 있다.
- 이 경우 controller는 안전하지만, 상위 Nav2 기준으로는 progress failure가 된다.


## 15. 해결 방향

이번 부작용의 본질은:

- collision gate가 틀렸다는 것이 아니라
- **gate가 너무 일찍, 너무 자주, 너무 회전 위주 fallback으로 들어간다**는 점이다.

따라서 다음 수정은 “gate 제거”가 아니라,
**gate를 진행성까지 고려하는 구조로 다듬는 것**이 맞다.


### 해결 1: gate를 항상 돌리지 말고 “장애물 근접 구간”에서만 강하게 켠다

[사실]

- 현재는 gate가 매 주기 무조건 돈다.

[추론]

- 자유공간이 충분한 구간까지 매번 엄격한 footprint simulation을 돌리면,
  필요 이상으로 forward 명령을 죽일 수 있다.

권장 수정:

1. `front`가 충분히 멀면 gate를 생략
2. 또는 `v_des <= 0`이고 이미 회전 중인 경우엔 gate를 완화
3. 또는 `front < stop_dist + margin` 구간에서만 gate를 강하게 적용

권장 파라미터 예:

- `collision_gate_activation_dist = stop_dist + 0.20`

즉 실제로 장애물이 가까운 구간에서만
“정밀한 차체 검증”을 켜는 방식으로 바꾸는 것이다.


### 해결 2: fallback에 “감속 전진 후보”를 먼저 넣는다

현재는:

- 원래 후보가 막히면 거의 바로 제자리 회전으로 간다.

이걸 아래처럼 바꾸는 것이 더 자연스럽다.

1. 원래 후보
2. `0.5 * v_des`, `w_des`
3. `0.25 * v_des`, `w_des`
4. `creep_speed`, 완화된 `w_des`
5. 그 다음에야 제자리 회전
6. 필요 시 후진

[추론]

- 지금 문제는 “약간만 완화하면 통과 가능한 상황”까지 너무 빨리 회전으로 꺾여 버리는 것이다.
- 먼저 **감속 전진 후보**를 넣으면, goal 쪽 진행성을 유지할 가능성이 커진다.


### 해결 3: 회전 fallback에 hysteresis/hold를 넣어 짧은 좌우 흔들림을 막는다

권장 수정:

1. 한번 회전 fallback 방향을 정하면
2. 최소 `0.4 ~ 0.8초`는 그 방향을 유지
3. 그 시간 동안은 미세한 `lr_diff` 변화로 반대 방향으로 안 바꿈

권장 파라미터 예:

- `collision_gate_turn_hold_sec = 0.6`

[추론]

- 지금의 좌우 흔들림은 “매 프레임 최적”을 다시 고르기 때문일 가능성이 높다.
- 회전 방향 유지 시간을 두면, controller가 짧게 좌/우를 번갈아 찍는 현상이 줄어든다.


### 해결 4: forward와 rotate에 같은 collision 기준을 쓰지 않는다

현재는:

- 전진 후보도
- 제자리 회전 후보도
- 동일한 `sim_horizon_sec`
- 동일한 `footprint_collision_cost_threshold`

를 쓴다.

[추론]

- 그런데 제자리 회전은 코너 sweep이 커서, 같은 기준을 그대로 쓰면 너무 쉽게 막힐 수 있다.
- 반대로 forward는 더 길게 보는 것이 맞을 수 있다.

권장 분리:

- forward 후보:
  - `horizon 0.8s`
  - `threshold 253`
- rotate 후보:
  - `horizon 0.3 ~ 0.4s`
  - 필요 시 더 엄격/완화된 별도 threshold 검토

즉 “명령 종류별로 다른 gate”가 필요하다.


### 해결 5: collision-free뿐 아니라 “progress-friendly” 후보를 우선한다

현재 구조는:

- 안전한 후보를 먼저 고르면 끝

이다.

이를 아래처럼 바꾸는 것이 더 낫다.

1. collision-free 후보를 여러 개 만든다
2. 그중
   - goal heading을 더 줄이는 후보
   - forward progress가 더 있는 후보
   - last_turn_dir와 일관된 후보
   를 우선 선택한다

[추론]

- 지금 문제는 “안전한데 제자리 회전만 하는 후보”가 너무 자주 선택된다는 점이다.
- 후보 선택 기준에 **진행성 점수(progress score)**를 넣으면 이 현상을 줄일 수 있다.


## 16. 우선순위

이번 증상에 대한 가장 좋은 수정 순서는 아래와 같다.

### 1순위: 감속 전진 후보 추가 [추론]

제자리 회전으로 바로 떨어지지 않게 하는 것이 가장 중요하다.

### 2순위: 회전 fallback hold/hysteresis 추가 [추론]

짧은 좌우 oscillation을 줄이는 데 직접적이다.

### 3순위: gate activation 조건 추가 [추론]

장애물이 충분히 멀면 gate를 덜 개입시키는 것이 좋다.

### 4순위: forward/rotate별 horizon 분리 [추론]

현재 단일 gate 설정을 motion type별로 나누는 단계다.


## 17. 최종 정리

현재 새로 생긴 문제는:

- “footprint collision gate가 있어서 안 간다”가 아니라
- **“footprint collision gate가 너무 이른 단계에서 forward를 죽이고, 회전 fallback을 너무 쉽게 고르며, 그 회전 방향도 충분히 유지되지 않아 진행성을 잃는다”**

로 정리하는 것이 맞다.

즉 다음 수정의 핵심은 아래 세 가지다.

1. gate를 항상 강하게 적용하지 말 것
2. 회전 전에 감속 전진 후보를 먼저 줄 것
3. 회전 fallback에는 방향 유지 시간을 둘 것

이 방향으로 가면,

- 장애물 가로지르기 문제는 유지해서 막고
- goal 쪽 진행성도 다시 회복할 수 있다.


## 18. 이번 턴에서 실제 반영한 2차 수정

이번 턴에서는 위 해결안 중 **1, 4, 5**를 실제 코드에 반영했다.

반영 파일:

- [rl_local_controller.hpp](/home/atoz/ca_ws/src/rl_local_controller/include/rl_local_controller/rl_local_controller.hpp)
- [rl_local_controller.cpp](/home/atoz/ca_ws/src/rl_local_controller/src/rl_local_controller.cpp)
- [nav2_rtabmap_params.yaml](/home/atoz/ca_ws/src/rtabmap_ros/rtabmap_launch/launch/config/nav2_rtabmap_params.yaml)

이번 턴에 일부러 **넣지 않은 것**도 있다.

- 해결 2의 “감속 전진 후보 확대”
- 해결 3의 “하드한 회전 방향 hold”

이 두 가지는 효과가 있을 수 있지만,

- 좁은 통로에서 다시 파고들 위험
- 경로가 실제로 반대 방향 회전을 요구할 때 대응이 늦어질 위험

이 있어 이번 턴에서는 제외했다.

즉 이번 수정은 “진행성 회복”을 노리되,
**불필요한 공격성 증가나 잘못된 방향 고집은 피하는 방향**으로 제한해서 넣었다.


### 18.1 반영 1: collision gate를 항상 돌리지 않고, 장애물 근접 구간에서만 강하게 켠다

현재 추가한 파라미터:

```yaml
collision_gate_front_activation_dist: 0.55
collision_gate_side_activation_dist: 0.50
```

관련 코드:
- [rl_local_controller.cpp](/home/atoz/ca_ws/src/rl_local_controller/src/rl_local_controller.cpp#L449)
- [rl_local_controller.cpp](/home/atoz/ca_ws/src/rl_local_controller/src/rl_local_controller.cpp#L636)

[사실]

- 이전 버전은 gate가 매 주기마다 항상 동작했다.
- 이번 버전은 `front`, `left`, `right`, 그리고 현재 `(v, w)`를 보고
  gate가 진짜 필요한 상황인지 먼저 판단한다.

구체적으로는:

1. 전방이 activation 거리보다 가까우면 gate on
2. 측면 최소 여유가 activation 거리보다 작으면 gate on
3. 자유공간에서는 gate를 덜 개입시킨다

[추론]

- 이렇게 해야 자유공간에서도 매 프레임 정밀 simulation을 돌리며
  forward 후보를 과하게 차단하는 문제를 줄일 수 있다.
- 즉 **필요한 곳에서는 정확하게 보고, 필요 없는 곳에서는 기존 RL 반응성을 살리는 구조**가 된다.


### 18.2 반영 4: forward와 rotate에 서로 다른 simulation window를 준다

현재 추가한 파라미터:

```yaml
forward_sim_horizon_sec: 0.8
forward_sim_dt: 0.1
rotate_sim_horizon_sec: 0.35
rotate_sim_dt: 0.1
```

관련 코드:
- [rl_local_controller.cpp](/home/atoz/ca_ws/src/rl_local_controller/src/rl_local_controller.cpp#L602)
- [rl_local_controller.cpp](/home/atoz/ca_ws/src/rl_local_controller/src/rl_local_controller.cpp#L658)

[사실]

- 전진/후진 명령은 실제 접촉까지 이어질 수 있어 더 긴 horizon이 필요하다.
- 반대로 제자리 회전은 같은 horizon을 그대로 쓰면 과보수적으로 변하고,
  짧은 좌우 oscillation만 늘 수 있다.

그래서 이번 버전은:

- 병진이 있는 명령은 더 길게 보고
- 순수 회전은 더 짧게 본다

는 구조로 바꿨다.

[추론]

- 이 변경은 “회전 후보가 너무 쉽게 막혀서 progress를 잃는 문제”와
  “회전 후보를 불필요하게 오래 검사해서 계산량이 늘어나는 문제”
  둘 다 줄일 수 있다.


### 18.3 반영 5: 안전한 후보들 중에서도 progress-friendly 후보를 고른다

관련 코드:
- [rl_local_controller.cpp](/home/atoz/ca_ws/src/rl_local_controller/src/rl_local_controller.cpp#L684)
- [rl_local_controller.cpp](/home/atoz/ca_ws/src/rl_local_controller/src/rl_local_controller.cpp#L721)

[사실]

- 이전 버전은 “첫 번째로 안전한 후보”를 바로 채택했다.
- 이번 버전은 안전한 후보를 모두 본 뒤,
  score를 계산해서 가장 나은 후보를 고른다.

현재 score의 성격은 다음과 같다.

1. 전진성이 있는 후보 가점
2. 후진 후보는 큰 감점
3. 선호 회전 방향과 일치하면 가점
4. 원래 명령과 너무 멀어지면 감점

즉 이번 버전은:

- collision-free이기만 하면 회전만 계속하는 후보

보다,

- **collision-free이면서도 원래 goal 추종 의도와 더 가까운 후보**

를 우선한다.

[추론]

- 이 점수화는 완전한 trajectory optimization은 아니지만,
  “안전하지만 진행성이 거의 없는 후보만 반복 선택되는 문제”를 줄이는 데 직접적이다.


## 19. 왜 1·4·5를 한 번에 넣는 것이 맞았는가

[추론]

이 셋은 서로 역할이 자연스럽게 이어진다.

1. 해결 1
   - gate가 너무 자주 켜지는 문제를 줄인다
2. 해결 4
   - 명령 종류에 따라 gate가 과보수적으로 동작하는 문제를 줄인다
3. 해결 5
   - gate를 통과한 후보들 중에서도 진행성이 더 있는 후보를 고른다

즉:

- 1만 넣으면 gate 빈도는 줄지만, 안전 후보 선택이 둔할 수 있고
- 4만 넣으면 horizon은 나뉘지만, 여전히 회전 위주 후보를 먼저 고를 수 있고
- 5만 넣으면 gate가 너무 자주 켜지는 근본 문제는 남는다

그래서 이 세 개는 실제로 **한 세트**에 가깝다.


## 20. 이번 수정의 실시간성 관점 평가

[사실]

- 기존 controller도 이미 다수의 raycast를 수행하고 있다.
- 이번 수정은 거기에 추가로 footprint collision simulation을 넣지만,
  이제 그 simulation은 “항상” 돌지 않고 근접 장애물 구간에서만 강하게 돈다.
- 또한 rotate는 forward보다 더 짧은 horizon으로 줄였다.

[추론]

- 즉 이번 수정은 이전 collision gate 버전보다
  **계산량을 줄이는 방향**으로 바뀌었다.
- 동시에 진행성 회복도 노리므로, 실시간성과 행동 품질 양쪽에서 더 균형이 좋다.

다만 이것도 최종 판단은 실제 런타임에서 확인해야 한다.

확인 포인트:

1. controller loop가 유지되는지
2. progress failure가 줄어드는지
3. 장애물 앞 좌우 짧은 oscillation이 줄어드는지


## 21. 다음 검증 포인트

이번 버전 테스트에서는 아래 세 가지를 꼭 같이 본다.

### 21.1 progress 회복 여부

- goal point 방향으로 실제 병진이 다시 생기는지
- `Failed to make progress` 빈도가 줄어드는지

### 21.2 안전성 유지 여부

- 예전처럼 장애물을 가로질러 들어가지는 않는지
- corner-cutting이 다시 커지지 않는지

### 21.3 oscillation 완화 여부

- 제자리에서 짧은 좌/우 회전 반복이 줄어드는지
- 회전 fallback이 한쪽 방향으로 좀 더 일관되게 유지되는지


## 22. 현재 문서 기준 최종 결론

현재 단계의 가장 타당한 해석은 아래와 같다.

1. 첫 번째 collision gate 버전은 안전성은 올렸지만 progress를 지나치게 죽였다.
2. 그 이유는 gate가 너무 자주 켜지고, 회전 후보를 너무 빨리 선택하며,
   안전 후보 중에서도 progress를 충분히 고려하지 않았기 때문이다.
3. 그래서 이번 턴에서는 1·4·5를 적용해
   - gate activation 조건 추가
   - forward/rotate horizon 분리
   - progress-friendly candidate scoring
   를 반영했다.

즉 이번 문서 기준 다음 버전의 목표는:

- **장애물 가로지르기는 계속 막으면서**
- **goal 방향 진행성은 다시 회복하는 것**

이다.
