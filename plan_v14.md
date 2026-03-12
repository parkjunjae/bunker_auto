# plan_v14

## 0. 문서 목적

이 문서는 현재 워크스페이스에서 발생하는 아래 문제를 대상으로 한다.

1. 로봇이 제자리회전할 때 `odom->base_link`는 어느 정도 회전을 따라가지만
2. 글로벌 맵 기준에서는 `map->odom`이 같이 돌아가며
3. RViz에서는 맵이 회전하거나, 심하면 jump/고스팅처럼 보인다

현재까지의 로그와 코드 수정 이력을 종합하면, 이 문제는 더 이상 `IMU yaw lock`만의 문제가 아니다.

현재 핵심 문제는:

**`pure spin` 구간에서 RTAB-Map 전역 yaw correction이 잘못되거나 과하게 반영되고, 그 결과가 `map->odom`에 그대로 드러나는 구조**

이다.

이 문서의 목표는 아래 세 가지다.

1. 현재 구현이 왜 실패했는지 정확히 정리
2. 지금부터 어떤 구조로 바꿔야 하는지 설계 수준에서 명확히 정리
3. 이후 코드 수정 순서를 혼선 없이 고정

---

## 1. 현재 증상 요약

현재 관측된 증상은 다음과 같이 나뉜다.

### 1.1 저속 제자리회전 / mixed motion

1. 천천히 도는 경우에는 footprint가 비교적 자연스럽게 움직인다.
2. 하지만 어떤 경우에는 맵이 살짝 돌아간다.
3. 될 때도 있고 안 될 때도 있어 일관성이 없다.

이 구간은 `P1`의 low-rate yaw unlock 문제와 일부 연결되어 있다.

### 1.2 빠른 제자리회전

1. 빠르게 제자리회전하면 맵이 같이 돈다.
2. 어떤 경우에는 약간의 지연 뒤에 갑자기 맵이 돌아간다.
3. 심한 경우에는 맵이 jump하듯이 뒤틀린다.

이 구간은 더 이상 `P1`이 아니라:

**RTAB-Map 전역 correction과 `map->odom` 반영 구조**

의 문제다.

### 1.3 핵심 판정

현재 가장 정확한 판정은 아래다.

1. `IMU state machine`은 fast spin 자체를 어느 정도 감지한다.
2. 그런데도 맵이 심하게 도는 것은 `map->odom` 계층이 잘못 동작한다는 뜻이다.
3. 즉 현재 주요 실패 지점은 `camera_imu_bias_corrector`보다 `map_tf` 계층이다.

---

## 2. 현재 코드 구조

### 2.1 IMU / EKF 계층

현재 yaw lock 관련 계층은 아래다.

1. `src/camera_imu_pipeline_cpp/src/camera_imu_bias_corrector.cpp`
2. `src/rtabmap_ros/rtabmap_launch/launch/sensor_sync.launch.py`
3. `src/robot_localization/params/ekf.yaml`

역할:

1. IMU bias 보정
2. `yaw_zeroing` 상태기계
3. `odom->base_link` yaw 입력 안정화

즉 이 계층은:

**로컬 odometry yaw를 어떻게 만들지**

를 담당한다.

### 2.2 RTAB-Map 계층

현재 RTAB-Map 핵심 설정은 아래다.

파일:

`src/rtabmap_ros/rtabmap_launch/launch/rtabmap.launch.py`

현재 실제 핵심값:

1. `RGBD/LinearUpdate = 0.50`
2. `RGBD/AngularUpdate = 6.28`
3. `Rtabmap/DetectionRate = 5.0`
4. `Reg/Strategy = 1`
5. `Icp/CorrespondenceRatio = 0.01`

의미:

1. graph/keyframe 생성은 보수적
2. 등록은 camera visual feature가 아니라 ICP 기반
3. 전역 correction source는 결국 RTAB-Map graph/optimizer 쪽

즉 이 계층은:

**전역 map correction을 계산하는 계층**

이다.

### 2.3 현재 B2 구현 계층

현재 추가된 코드:

1. `src/rtabmap_ros/rtabmap_launch/scripts/map_tf_stabilizer.py`
2. `src/rtabmap_ros/rtabmap_launch/launch/rtabmap_nav2.launch.py`

현재 구조:

1. RTAB-Map raw `map->odom` TF publish 끔
2. 대신 `/rtabmap/mapGraph.map_to_odom`를 raw source로 사용
3. `map_tf_stabilizer.py`가 filtered `map->odom`만 최종 TF로 publish

즉 현재 체인은:

`RTAB-Map graph snapshot -> stabilizer -> 최종 map->odom`

이다.

---

## 3. 현재 구조가 실패한 이유

### 3.1 IMU는 회전을 감지하지만 stabilizer는 pure spin을 못 잡는다

최신 로그에서 확인된 핵심 사실:

1. IMU 쪽은 fast spin에서 `hard_wz`가 뜬다.
2. 그런데 stabilizer는 같은 시점에 `score=0`, `gain=1`인 경우가 있다.

즉:

1. 로컬 yaw 계층은 회전을 인식했는데
2. `map_tf_stabilizer`는 pure spin 완화 모드로 들어가지 못한다

결과:

**전역 yaw correction authority가 전혀 줄지 않는다**

즉 B2가 존재하지만 실제로는 꺼져 있는 것과 비슷한 상황이 발생한다.

### 3.2 pure spin 검출 임계값이 실제 회전과 안 맞는다

현재 stabilizer는 pure spin 시작 임계값을 비교적 높게 둔다.

문제는:

1. 실제 fast spin이라도 IMU/EKF 기준 `wz`가 `0.10 rad/s` 안팎인 경우가 있다
2. 그런데 stabilizer의 pure spin 시작 기준은 그보다 높게 잡혀 있다

그 결과:

1. 사용자는 “분명 빠르게 돌았다”고 느끼지만
2. stabilizer는 pure spin으로 보지 않는다

즉 현재 B2는:

**실제 운용 속도 영역과 임계값이 불일치**

한 상태다.

### 3.3 `/rtabmap/mapGraph`는 B2 raw source로 부적절하다

이게 현재 구조의 가장 큰 약점이다.

문제는 세 가지다.

#### 3.3.1 저주기

사용자가 직접 측정한 `/rtabmap/mapGraph`는 약 `3.5Hz` 수준이다.

즉 raw correction source가:

1. 20Hz, 30Hz 같은 연속 신호가 아니라
2. 약 `0.3~0.6초` 간격의 이벤트성 갱신 신호

라는 뜻이다.

그런데 stabilizer는 20Hz로 돈다.

결과:

1. stabilizer는 대부분의 시간 동안 stale raw 값을 들고 있음
2. 새 graph가 오면 raw correction이 한 번에 점프
3. stabilizer는 그 이벤트를 따라가는 추종기처럼 동작

즉 현재 B2는:

**continuous correction stabilizer가 아니라 sparse graph event smoother**

가 되어 버렸다.

#### 3.3.2 raw TF와 시점이 다를 수 있음

`/rtabmap/mapGraph.map_to_odom`는 graph 메시지 필드다.

반면 RTAB-Map이 실제로 내보내는 raw `map->odom` TF는:

1. detection/update cadence
2. TF publish cadence

에 따라 더 자주 갱신될 수 있다.

즉 stabilizer는 현재:

1. RTAB-Map이 실제로 세계에 내보내는 raw TF
2. 와 정확히 같은 신호

를 보고 있다고 보장할 수 없다.

이건 output-layer stabilizer로서는 치명적이다.

왜냐하면:

**출력 안정화는 실제 출력 직전 신호를 봐야 하는데, 현재는 graph snapshot을 보고 있기 때문**

이다.

#### 3.3.3 startup 공백

현재 RTAB-Map raw TF publish는 꺼져 있다.

그러면:

1. stabilizer가 첫 `/rtabmap/mapGraph`를 받기 전까지
2. `map->odom` publisher가 없다

이 문제는 이미 실제로 한 번 timeout으로 드러났다.

즉 현재 구조는:

**startup robustness도 약하다**

는 뜻이다.

### 3.4 현재 구조는 sole TF authority 구조라 실패 시 피해가 크다

현재는:

1. RTAB-Map raw TF를 껐다
2. stabilizer만 최종 `map->odom`을 발행한다

즉 stabilizer가 잘못 동작하면:

1. fallback raw TF도 없고
2. system 전체 global frame이 바로 흔들린다

이건 운영 구조로 위험하다.

### 3.5 `AngularUpdate/LinearUpdate`는 주원인이 아니다

이 값들은 분명 graph update cadence에 영향을 준다.

하지만 현재 문제의 본질은:

1. 어떤 cadence로 keyframe을 만들지
보다
2. **전역 correction을 어떤 raw signal로 받아서, 어떤 계층에서, 어떤 authority로 반영하느냐**

이다.

즉 지금 구조에선:

1. `AngularUpdate`, `LinearUpdate`를 높여도
2. 낮춰도
3. 결국 B2 raw source와 TF authority 구조가 잘못돼 있으면
4. 제자리회전 시 맵 회전은 계속 남는다

---

## 4. 현재 문제의 진짜 본체

현재 문제의 본체를 한 문장으로 쓰면:

**pure spin 동안 RTAB-Map 전역 yaw correction이 애매하거나 과한데, 그 correction을 system global frame에 직접 반영하는 구조**

이다.

즉 현재 실패는:

1. IMU가 회전을 못 잡아서만이 아니라
2. global correction을 누가, 어떤 신호로, 어떤 frame에 반영하느냐

의 문제다.

다시 말해:

1. `P1`은 로컬 yaw unlock 문제
2. 지금 네가 보고 있는 심한 맵 회전은 global frame architecture 문제

이다.

---

## 5. 완전 해결에 필요한 설계 원칙

완전한 해결은 아래 원칙으로 가야 한다.

### 5.1 raw RTAB-Map global TF는 살려둔다

이유:

1. startup 공백 방지
2. raw global correction을 직접 관찰 가능
3. stabilizer 실패 시 fallback 확보

즉:

**raw source는 토픽 snapshot이 아니라 RTAB-Map이 실제 publish하는 TF여야 한다**

### 5.2 stabilizer는 raw TF를 직접 listen 해야 한다

즉 현재처럼:

`/rtabmap/mapGraph.map_to_odom`

를 raw input으로 쓰면 안 된다.

대신:

**RTAB-Map raw `map->odom` TF 자체**

를 input으로 써야 한다.

이유:

1. 주기/시점이 맞는다
2. graph snapshot 지연을 피할 수 있다
3. output-layer stabilizer로서 논리적으로 맞다

### 5.3 stabilizer는 같은 `map->odom`을 다시 내보내면 안 된다

raw RTAB-Map TF를 살린 상태에서 stabilizer가 같은 `map->odom`을 다시 내보내면
TF 충돌이 난다.

즉:

1. raw `map->odom`
2. filtered `map->odom`

를 동시에 둘 수 없다.

### 5.4 안정화는 global frame 위에 얹어야 한다

권장 체인은:

`map_stable -> map -> odom -> base_link`

즉:

1. RTAB-Map raw global frame은 `map`
2. stabilizer는 `map_stable -> map`

를 발행

이 구조의 장점:

1. raw TF 보존
2. TF 충돌 없음
3. startup 안전
4. filtered global frame과 raw global frame 동시 비교 가능

### 5.5 pure spin 감쇠는 `map->odom` 자체를 덮어쓰는 게 아니라 `map_stable -> map`에서 한다

즉 B2는:

1. raw global correction을 없애는 것이 아니라
2. 그 raw global frame 위에서
3. 사용자가 실제 global frame으로 쓰는 frame을 더 안정적으로 만드는 방향

이어야 한다.

---

## 6. 권장 구조

### 6.1 최종 권장 TF 체인

권장 체인:

1. RTAB-Map raw:
   - `map -> odom`
2. EKF:
   - `odom -> base_link`
3. stabilizer:
   - `map_stable -> map`

최종 system global chain:

`map_stable -> map -> odom -> base_link`

### 6.2 각 frame의 의미

#### `map`

1. RTAB-Map raw global frame
2. graph optimization과 ICP registration의 결과가 직접 반영되는 frame

#### `map_stable`

1. 사용자/네비게이션이 보는 안정화된 global frame
2. pure spin 동안 raw global yaw correction을 덜 믿도록 만든 frame

#### `odom`

1. EKF local frame
2. 짧은 시간 local motion continuity를 담당

즉:

1. raw SLAM 세계는 `map`
2. 운영 global 세계는 `map_stable`

로 분리한다.

### 6.3 Nav2 / RViz가 봐야 하는 frame

현재 global frame을 `map`으로 쓰고 있다면,
권장 구조에서는:

1. Nav2 global frame
2. RViz fixed frame
3. global costmap frame

을 `map_stable`로 바꿔야 한다.

이게 핵심이다.

왜냐하면:

**안정화된 global frame을 따로 만들었는데도 Nav2가 raw `map`을 계속 보면 아무 의미가 없기 때문**

이다.

---

## 7. 새 B2의 동작 철학

### 7.1 pure spin 검출 입력

pure spin 검출은 아래 입력을 쓴다.

1. IMU 또는 EKF 기반 `|wz|`
2. translation speed
3. pure spin 지속시간
4. pure spin 누적 yaw

중요:

1. 주 입력은 `wz`
2. translation speed는 pure spin 성격 확인용
3. 지속시간/누적 yaw는 보강용

즉:

**회전 검출은 IMU/odom motion 계층이 담당**

한다.

### 7.2 correction control 출력

출력은:

1. `pure_spin_score`
2. `global_correction_gain`

두 개면 충분하다.

즉:

1. pure spin이 강할수록 score 상승
2. score가 높을수록 `map_stable -> map` yaw correction gain 하락

### 7.3 hard freeze 금지

여기서도 원칙은 같다.

금지:

1. pure spin이면 correction 0으로 완전 차단
2. spin 종료 후 몰아서 보정

허용:

1. gain을 낮추되 0으로 닫지 않음
2. 매 프레임 현재 correction만 일부 반영
3. rate limit를 둠
4. recovery는 천천히

즉:

**`freeze -> accumulate -> repay` 금지**

### 7.4 backlog 없는 구조

핵심은:

1. 덜 반영한 correction을 debt로 따로 저장하지 않는다
2. 다음 프레임에는 다시 fresh raw TF를 읽는다
3. lag cap과 rate limit로 jump를 줄인다

즉:

**“과거에 못 넣은 correction을 나중에 갚는다”는 사고를 버려야 한다**

---

## 8. 단계별 전환 계획

### 단계 0. 현재 B2 구조는 장기 구조로 보지 않는다

현재 `mapGraph -> stabilizer -> map->odom` 구조는
운영 최종 구조가 아니라 폐기 예정 구조로 봐야 한다.

이유:

1. low-rate raw source
2. raw TF와 비동치 가능성
3. startup 공백
4. sole publisher risk

### 단계 1. raw RTAB-Map TF 복원

해야 할 일:

1. RTAB-Map `publish_tf_map` 다시 활성화
2. raw `map->odom` TF를 system에 복원

목적:

1. startup 안정화
2. 실제 raw global correction 관측
3. stabilizer 입력을 TF로 전환하기 위한 준비

### 단계 2. 기존 stabilizer의 역할 전환

기존 stabilizer는 더 이상:

`mapGraph -> map->odom`

구조로 쓰지 않는다.

새 역할:

1. raw `map->odom` TF listen
2. filtered `map_stable -> map` TF publish

즉 출력 frame을 바꿔야 한다.

### 단계 3. Nav2 / RViz frame 전환

해야 할 일:

1. global frame을 `map`에서 `map_stable`로 전환
2. RViz fixed frame도 `map_stable`
3. global costmap frame도 `map_stable`

주의:

1. local costmap의 `odom`은 유지
2. `odom->base_link` 체인은 그대로 유지

### 단계 4. pure spin score 재튜닝

이 단계에서 다시:

1. `spin_wz_start`
2. `spin_wz_full`
3. gain down/up
4. yaw rate limit
5. lag cap

를 실제 fast spin 로그에 맞춰 조정한다.

중요:

지금처럼 `wz=0.10`인데 score가 0으로 남는 상태는 다시 나오면 안 된다.

### 단계 5. 그래도 남으면 그때 registration 본체로 들어간다

즉 아래는 마지막 단계다.

1. `Reg/Strategy`
2. `Icp/CorrespondenceRatio`
3. `Icp/MaxCorrespondenceDistance`
4. pure spin 중 registration acceptance 조정

이건 지금보다 더 깊은 RTAB-Map 본체 단계이므로,
frame architecture를 먼저 바로잡은 뒤에만 들어간다.

---

## 9. 성공 기준

성공은 아래 조건을 동시에 만족해야 한다.

1. 저속 제자리회전에서 footprint와 global frame이 일관된다
2. fast pure spin에서 `map_stable` 기준 맵이 심하게 같이 돌지 않는다
3. 회전 중 jump가 없다
4. 회전 종료 직후 correction backlog jump가 없다
5. startup 때 `map->odom` 공백이 없다
6. raw `map`과 filtered `map_stable`를 비교했을 때 filtered 쪽이 분명히 더 안정적이다

즉 성공 기준은 단순히

“회전 중 한 번 안 돌았다”

가 아니라,

**fast pure spin / startup / spin 종료 후 recovery까지 모두 안정적인 것**

이다.

---

## 10. 실패 기준

아래 중 하나라도 계속 보이면 아직 실패다.

1. fast pure spin에서 `map_stable`도 같이 크게 돈다
2. spin 종료 후 `map_stable`이 뒤늦게 크게 jump한다
3. stabilizer score가 실제 회전을 또 못 잡는다
4. raw TF는 정상인데 stable frame이 더 불안정하다
5. Nav2 global frame 전환 후 오히려 planner/global costmap이 흔들린다

---

## 11. 최종 결론

현재 문제를 가장 정확히 요약하면:

1. IMU 상태기계만의 문제가 아니다
2. `AngularUpdate`, `LinearUpdate` 주 해결축도 아니다
3. 현재 `mapGraph` 기반 `map_tf_stabilizer` 구조가 운영용으로 잘못 잡혀 있다

즉 지금 필요한 것은:

**IMU를 더 만지는 것보다, raw RTAB-Map global TF와 안정화된 global frame을 분리하는 구조로 frame architecture를 다시 잡는 것**

이다.

최종 권장 방향은 아래 한 문장으로 정리된다.

**`mapGraph -> filtered map->odom` 구조를 버리고,  
RTAB-Map raw `map->odom` TF를 기준으로 `map_stable -> map` 안정화 계층을 추가해 Nav2/RViz는 `map_stable`을 global frame으로 보게 바꾼다.**

---

## 12. 2026-03-12 구현 반영 내용

이 문서의 설계를 실제 코드에 반영한 현재 상태는 아래와 같다.

### 12.1 stabilizer 입력/출력 구조 변경

파일:

1. `src/rtabmap_ros/rtabmap_launch/scripts/map_tf_stabilizer.py`

현재 구현은 더 이상 `/rtabmap/mapGraph`를 raw source로 사용하지 않는다.

새 구조:

1. 입력
   - RTAB-Map raw TF `map -> odom`를 `tf2_ros.Buffer`로 직접 lookup
   - `/camera/camera/imu_fixed`의 `angular_velocity.z`
   - `/odometry/filtered`의 선속도

2. 출력
   - `map_stable -> map` TF

즉 실제 글로벌 체인은:

`map_stable -> map -> odom -> base_link`

가 된다.

### 12.2 RTAB-Map raw TF 복원

파일:

1. `src/rtabmap_ros/rtabmap_launch/launch/rtabmap_nav2.launch.py`

변경:

1. `publish_tf_map = true`

의미:

1. RTAB-Map raw `map -> odom` TF를 다시 system에 복원
2. stabilizer는 raw TF를 직접 input으로 사용
3. startup 시 raw TF가 먼저 존재하므로 기존 `mapGraph` 기반 startup 공백 문제가 줄어든다

### 12.3 Nav2 / RViz global frame 전환

파일:

1. `src/rtabmap_ros/rtabmap_launch/launch/config/nav2_rtabmap_params.yaml`
2. `src/rtabmap_ros/rtabmap_launch/launch/config/nav2_rtabmap_params_train.yaml`
3. `src/rtabmap_ros/rtabmap_launch/launch/config/rgbd.rviz`

변경:

1. Nav2 global frame `map -> map_stable`
2. RViz Fixed Frame `map -> map_stable`

유지:

1. local costmap global frame은 여전히 `odom`
2. `odom -> base_link`는 EKF가 계속 담당
3. RTAB-Map occupancy map 토픽 자체는 여전히 `map` frame 기준

즉 system은 raw map과 stable map을 분리해서 운용하게 된다.

### 12.4 stabilizer 핵심 로직

새 stabilizer는 다음 블록으로 동작한다.

1. raw TF lookup
2. IMU wz + odom speed 기반 pure spin score 계산
3. score로 correction authority(`gain`) 계산
4. raw TF 증분을 gain만큼만 filtered global correction에 반영
5. `map_stable -> map` 출력 변화율 제한
6. pure spin 종료 후 `map_stable -> map`을 천천히 identity로 복귀

핵심 원칙:

1. hard freeze 금지
2. correction debt 저장 금지
3. stale graph message 추종 금지
4. live raw TF 기반으로만 동작

### 12.5 현재 구현 파라미터 의도

현재 launch 반영값은 아래 철학을 따른다.

1. `spin_wz_start = 0.08`
   - 기존 `0.35`는 실제 fast spin(`~0.10 rad/s`)을 못 잡았기 때문에 대폭 낮춤

2. `spin_wz_full = 0.18`
   - 실제 pure spin score를 빠르게 1.0 근처로 올릴 상한

3. `gain_min = 0.20`
   - pure spin 중에도 global correction을 0으로 닫지 않음
   - 종료 후 backlog jump를 막기 위한 최소 bleed-through

4. `out_yaw_rate_limit_spin = 0.20`
   - fast spin 중 `map_stable -> map` yaw가 step처럼 바뀌는 것을 직접 제한

5. `recovery_tau_quiet_sec = 1.20`
   - pure spin이 끝난 뒤 stable frame을 즉시 raw map으로 붙이지 않고
   - 천천히 복귀시켜 종료 직후 jump를 줄임

### 12.6 이번 구현이 해결하려는 것

1. fast pure spin 중 raw global yaw correction이 system global frame을 바로 흔드는 현상
2. `/rtabmap/mapGraph` 저주기 입력 때문에 stabilizer가 stale correction을 추종하던 문제
3. stabilizer가 sole `map->odom` publisher가 되어 구조적으로 취약했던 문제

### 12.7 이번 구현이 아직 보장하지 않는 것

아래는 아직 실차 검증이 필요하다.

1. fast pure spin에서 `map_stable` 기준 맵 회전이 충분히 줄어드는지
2. spin 종료 후 `map_stable -> map` recovery가 너무 느리거나 빠르지 않은지
3. Nav2 global frame을 `map_stable`로 바꾼 뒤 planner/global costmap이 안정적인지
4. RTAB-Map raw TF 자체가 pure spin에서 너무 크게 틀어질 때 stable layer가 충분히 흡수하는지

즉 현재 상태는:

**구조적 방향은 plan_v14에 맞게 반영되었고, 다음 단계는 실차 검증과 파라미터 미세조정이다.**

### 12.8 2026-03-12 최신 실차 로그 재분석: B2는 "꺼져 있는 상태"가 아니었다

최신 런:

1. `logs/run_20260312_144159/rtabmap_nav2.log`
2. `logs/run_20260312_144159/sensor_sync.log`

이 런은 plan_v14 구조 반영 후, 사용자가 다시 제자리회전을 수행했을 때의 로그다.
사용자 체감 증상은 여전히 다음과 같다.

1. 제자리회전 시 맵이 심하게 틀어진다
2. footprint보다 맵이 같이 돌거나, 점프/고스팅처럼 보인다
3. 기존 가설대로라면 B2 stabilizer가 pure spin을 감지하지 못했거나, gain이 내려가지 않아야 한다

하지만 최신 로그를 보면 이 가설은 더 이상 그대로 유지할 수 없다.

#### 12.8.1 IMU 상태기계는 빠른 회전을 정상적으로 감지했다

`sensor_sync.log` 기준:

1. `yaw_zeroing=OFF reason=hard_wz raw_wz=-0.10388`
2. `yaw_zeroing=OFF reason=hard_wz raw_wz=0.10901`
3. `yaw_zeroing=OFF reason=hard_wz raw_wz=-0.10241`

즉 IMU 상태기계(P1)는 빠른 제자리회전을 이미 "회전 중"으로 보고 있다.
따라서 지금 남은 severe map twist를

1. `yaw_lock`이 회전을 막아서 생긴 문제
2. `yaw_zeroing`이 계속 ON이라서 생긴 문제

로 설명하는 것은 맞지 않다.

#### 12.8.2 `map_tf_stabilizer`도 이번에는 pure spin을 실제로 감지했다

`rtabmap_nav2.log` 기준:

1. stabilizer 프로세스는 정상 기동했다
   - `process started`
2. 첫 raw TF도 정상 수신했다
   - `first raw map->odom TF received, publishing map_stable->map`
3. pure spin 구간에서 score/gain이 실제로 크게 반응했다

대표 로그:

1. `imu_wz=-0.317 speed=0.002 score=0.932 gain=0.261`
2. `imu_wz=-0.355 speed=0.002 score=0.974 gain=0.221`
3. `imu_wz=0.192 speed=0.003 score=0.952 gain=0.246`
4. `imu_wz=-0.387 speed=0.002 score=0.938 gain=0.265`

즉 이번 런에서 B2의 pure-spin detector는 이미 동작했다.
따라서 현재 severe twist를

1. `spin_wz_start`가 너무 높아서 stabilizer가 전혀 개입하지 못했다
2. gain이 항상 1.0으로 남았다

로 설명하는 것도 이제 맞지 않는다.

#### 12.8.3 그런데도 로그상 `raw_yaw`와 `stable_map_yaw`는 거의 0에 머물렀다

위 same log lines를 보면, pure spin score는 높게 올라가고 gain도 낮아졌지만:

1. `raw_yaw=0.000`
2. `stable_map_yaw=0.000`

가 대부분의 1초 주기 로그에서 유지된다.

이 관측은 매우 중요하다.

의미:

1. 현재 사용자가 본 severe twist는 적어도 "1Hz로 샘플링된 yaw 로그" 기준으로는 설명되지 않는다
2. 즉 `map_stable->map` yaw가 수 초 동안 크게 누적되어 돌아간 패턴은 로그에 안 보인다
3. 따라서 남은 문제는 단순히
   - B2 detection 실패
   - gain 미동작
   - `map_stable->map`의 지속적인 yaw drift
   로 볼 수 없다

#### 12.8.4 현재 시점의 가장 중요한 판정

최신 로그가 말해주는 것은 다음이다.

1. P1은 fast spin을 정상 감지한다
2. B2 detector도 fast spin을 정상 감지한다
3. 그런데 사용자는 여전히 severe map twist를 본다
4. 반면 1Hz stabilizer 로그에서는 `raw_yaw`, `stable_map_yaw`가 거의 0으로 보인다

즉 현재 남은 문제는:

**"B2가 아예 작동하지 않는다"가 아니라, 사용자가 보는 severe twist가 현재 로그 해상도에서 포착되지 않는 더 짧은 TF spike / step correction / map update 이벤트일 가능성이 높다**

로 재정의해야 한다.

#### 12.8.5 왜 1Hz 로그로는 severe twist를 놓칠 수 있는가

현재 stabilizer 로그는 `log_period_sec = 1.0`이다.

이 뜻은:

1. 1초에 한 번만 score/gain/raw_yaw/stable_map_yaw를 찍는다
2. 그 사이 100ms~300ms 동안 `map->odom` 또는 `map_stable->map`이 급변했다가 돌아오면
3. 사용자는 RViz에서 큰 회전/점프로 보지만
4. 로그에는 "직전 1초 전 상태와 거의 비슷함"으로만 남을 수 있다

즉 지금 로그가 보여주는 "yaw가 0.000 근처"라는 사실은

1. twist가 없었다

가 아니라

2. **현재 로그 샘플링 주기로는 그 twist를 포착하지 못했다**

일 가능성을 충분히 포함한다.

#### 12.8.6 현재 severe twist의 남은 유력 원인 후보

최신 로그를 반영하면 후보는 아래처럼 좁혀진다.

1. **짧은 시간의 raw `map->odom` TF spike**
   - RTAB-Map raw TF가 pure spin 중 짧게 크게 튀고, 1Hz 로그엔 안 잡혔을 가능성

2. **짧은 시간의 `map_stable->map` spike**
   - B2 output도 순간적으로 크게 움직였지만, 샘플 간격 사이에서 지나간 가능성

3. **RTAB-Map `/rtabmap/map` 또는 graph update의 시각적 재작성**
   - global TF의 장기 drift가 아니라, map update/graph optimization이 discrete하게 들어오며 RViz에서 severe twist/gosting처럼 보였을 가능성

현재 자료만으로는 이 셋 중 어느 하나를 단정할 수 없다.
하지만 분명한 것은:

**"지금 문제는 더 이상 순수한 P1 실패도, 단순한 B2 threshold 실패도 아니다."**

#### 12.8.7 plan_v14의 다음 조사 우선순위 변경

이 최신 로그를 반영하면, 다음 단계의 우선순위는 다음처럼 바뀐다.

1. `spin_wz_start`, `gain_min` 같은 threshold를 더 손대는 것보다
2. **high-rate TF/event capture로 실제 spike를 잡는 것**이 먼저다

즉 다음 단계는 파라미터 미세조정보다 계측이다.

필수 계측 항목:

1. high-rate raw TF `map -> odom`
2. high-rate filtered TF `map_stable -> map`
3. same window의 IMU `wz`
4. same window의 `/odometry/filtered`
5. 가능하면 `/rtabmap/map` update 시점 또는 `/rtabmap/mapGraph` update 시점

#### 12.8.8 현재 문서 기준 최종 수정된 판정

최신 런 기준 판정은 아래와 같다.

1. `P1`: fast spin 감지는 정상 동작
2. `B2 detector`: fast spin 감지는 정상 동작
3. 남은 severe twist: **짧은 TF spike / map update event를 1Hz 로그가 놓치는 상태**

즉 지금 바로 해야 할 일은:

1. P1 재튜닝 아님
2. `AngularUpdate/LinearUpdate` 재튜닝 아님
3. B2 threshold 감으로 추가 조정 아님
4. **TF/event 계측 해상도 증가 후, spike가 raw TF인지 filtered TF인지 map update인지 먼저 분리**

이렇게 정리하는 것이 현재 로그와 가장 일치한다.

## 13. TF Layer만으로는 완전 해결되지 않는 이유

최신 증상과 로그를 같이 보면, 현재 문제를 `map->odom` TF layer만의 문제로 보는 것은 부족하다.

현재까지 확인된 사실:

1. `P1` IMU 상태기계는 fast spin을 감지한다
2. `B2` pure-spin detector도 fast spin을 감지한다
3. 그런데 사용자는 여전히 "맵 자체가 심하게 돈다", "고스팅처럼 보인다", "점프한다"고 본다

이 조합은 다음 사실을 강하게 시사한다.

**사용자가 보는 severe twist는 TF 체인만의 문제가 아니라, `/rtabmap/map` 자체가 pure spin 동안 다시 그려지거나 graph optimization 결과가 map 토픽에 시각적으로 반영되는 문제까지 포함한다.**

즉 지금 남은 문제는 아래 두 층이 함께 있다.

1. **TF layer**
   - `map->odom`, `map_stable->map` 계열 전역 보정

2. **Map topic layer**
   - `/rtabmap/map` occupancy grid 또는 graph 재작성 결과

따라서 완전 해결은 더 이상 TF layer만으로는 닫히지 않는다.

## 14. 최종 해결 방향: stable TF + stable map topic 동시 도입

### 14.1 핵심 원칙

pure spin 동안에는 RTAB-Map raw 결과를 시스템 global consumer가 직접 보지 않게 해야 한다.

즉 소비 계층은 아래 둘을 모두 raw가 아닌 stable 쪽을 보아야 한다.

1. global TF
2. global map topic

한 문장으로 쓰면:

**pure spin은 "전역 지도 갱신 구간"이 아니라 "로컬 odom만 믿고, 전역 보정과 전역 지도 갱신은 완화/보류하는 구간"으로 취급해야 한다.**

### 14.2 권장 최종 구조

현재 raw 체인:

1. `map -> odom` : RTAB-Map raw global TF
2. `/rtabmap/map` : RTAB-Map raw occupancy grid

권장 stable 체인:

1. `map_stable -> map` : pure spin 동안 attenuation/recovery를 거친 stable global TF
2. `/rtabmap/map_stable` : pure spin 동안 마지막 stable map을 유지하는 stable map topic

최종 consumer 체인:

1. `map_stable -> map -> odom -> base_link`
2. Nav2 global costmap / planner / RViz는 `map_stable`과 `/rtabmap/map_stable`을 사용

즉 raw RTAB-Map은 내부적으로 계속 계산하지만, 시스템의 global consumer는 stable projection만 보게 만든다.

### 14.3 왜 stable map topic이 필요한가

TF만 안정화하면 충분하다는 가정이 틀릴 수 있는 이유는 다음과 같다.

1. RTAB-Map은 pure spin 중에도 graph optimization을 수행할 수 있다
2. 그 결과 `/rtabmap/map`이 discrete하게 다시 그려질 수 있다
3. RViz에서는 이것이 TF jump 없이도 "맵이 회전한다", "맵이 뒤틀린다", "고스팅이 생긴다"로 보일 수 있다
4. Nav2 global costmap도 raw map update를 그대로 받으면 global planning 기준이 흔들릴 수 있다

즉:

1. `map_stable -> map`만 있어도
2. `/rtabmap/map` raw topic을 계속 그대로 소비하면
3. 사용자는 여전히 "맵이 돌아간다"고 느낄 수 있다

따라서:

**stable TF와 stable map topic은 같이 가야 한다.**

## 15. stable map topic의 동작 철학

stable map topic의 목적은 raw map을 영원히 숨기는 것이 아니다.

목적:

1. pure spin 중 raw map이 흔들릴 때 global consumer를 보호
2. pure spin이 끝나고 translation이 다시 생긴 뒤 안정된 시점에서만 raw map 변경을 따라가게 함

즉 철학은:

1. pure spin 중: stable map은 마지막 good map 유지
2. pure spin 종료 후: raw map을 다시 천천히 혹은 조건부로 수용

### 15.1 pure spin 동안

조건:

1. IMU `|wz|` 큼
2. 선속도 작음
3. pure spin score 높음

이 구간에서 stable map layer는:

1. raw `/rtabmap/map`를 계속 읽을 수는 있음
2. 하지만 consumer로는 마지막 stable map을 유지
3. raw map update를 바로 publish하지 않음

즉 pure spin 동안에는 map topic 소비층에도 attenuation 대신 **hold-last-good** 정책을 적용한다.

주의:

이 "hold"는 TF layer의 hard freeze와는 다르다.

1. TF layer는 완전 freeze 금지
2. map topic layer는 discrete topic이므로 **last-good hold**가 더 자연스럽다

즉 TF와 map topic은 같은 pure spin이라도 제어 방식이 다를 수 있다.

### 15.2 pure spin 종료 후

pure spin 종료 직후도 주의 구간이다.

즉시 raw map으로 돌아가면:

1. pure spin 동안 누적된 graph/map 변화가 한 번에 보일 수 있고
2. 사용자는 "회전 끝나고 map이 갑자기 뒤집힌다"고 느낄 수 있다

그래서 종료 후에는 다음이 필요하다.

1. translation 재개 확인
2. 일정 quiet window 확인
3. 그 뒤 stable map을 raw map으로 갱신

즉 map topic layer는:

1. pure spin 중: hold
2. pure spin 종료 직후: immediate snap-back 금지
3. translation 재개 후 안정 구간에서만 refresh

이 원칙을 따른다.

## 16. TF layer와 map topic layer의 역할 분리

이 둘은 비슷해 보여도 다르다.

### 16.1 TF layer (`map_stable -> map`)

역할:

1. global frame yaw correction authority 완화
2. step-like `map->odom` correction 완화
3. pure spin 중 global frame jump 억제

수단:

1. score
2. gain
3. rate limit
4. lag cap
5. slow recovery

### 16.2 map topic layer (`/rtabmap/map_stable`)

역할:

1. raw occupancy/grid redraw를 pure spin 동안 소비자에게 숨김
2. graph rewrite가 시각적 twist/gosting처럼 보이는 문제 완화
3. pure spin 종료 후 안정된 시점에만 map refresh 허용

수단:

1. hold-last-good map
2. 조건부 refresh
3. spin 종료 후 delayed refresh

즉:

**TF layer는 continuous attenuation, map topic layer는 discrete hold/refresh가 맞다.**

## 17. 현재 남은 증상에 대한 최종 해석

사용자가 지금 보는 severe twist는 다음 조합일 가능성이 가장 높다.

1. raw RTAB-Map global correction 또는 map redraw 이벤트가 pure spin 중 발생
2. TF layer만으로는 raw map redraw를 가리지 못함
3. 결과적으로 RViz/Nav2는 stable TF 위에서도 raw map 변화 때문에 "맵이 돈다"고 보게 됨

즉 현재 남은 문제는:

**"stable TF는 일부 도입되었지만, raw map topic을 그대로 소비하고 있어 visual/global map distortion path가 아직 열려 있다"**

로 재정의하는 것이 맞다.

## 18. 구현 우선순위 재정의

이 문서 기준 다음 구현 우선순위는 아래와 같다.

### 18.1 유지할 것

1. `P1` IMU 상태기계
2. raw RTAB-Map TF 복원
3. `map_stable -> map` stable TF layer

### 18.2 새로 추가할 것

1. stable occupancy map relay layer
2. raw `/rtabmap/map` -> filtered `/rtabmap/map_stable`
3. pure spin 중 hold-last-good map 정책
4. 종료 후 조건부 refresh 정책

### 18.3 그 다음에만 볼 것

1. RTAB-Map registration 본체
2. `Reg/Strategy`
3. `Icp/*`
4. `AngularUpdate/LinearUpdate`

즉 현재는:

**등록 본체를 더 만지기보다, 먼저 consumer-facing stable map layer를 완성하는 것이 맞다.**

## 19. 성공 기준의 수정

최종 성공은 아래를 동시에 만족해야 한다.

1. fast pure spin 중 footprint는 정상 회전
2. `map_stable` 기준 global frame은 jump 없이 유지
3. `/rtabmap/map_stable`도 same spin 동안 크게 회전/뒤틀림/고스팅처럼 보이지 않음
4. pure spin 종료 후 map refresh가 즉시 snap-back 되지 않음
5. translation 재개 후에만 global map과 global frame이 안정적으로 다시 정렬

즉 성공 기준은 이제:

1. TF 안정화만이 아니라
2. **global map topic 안정화까지 포함**

한다.

## 20. 현재 문서의 최종 결론

지금 제자리회전 시 맵이 심하게 도는 문제를 완전히 해결하려면:

1. pure spin 동안 raw RTAB-Map 결과를 system global consumer가 직접 보지 않게 해야 하고
2. 이를 위해 `map_stable -> map` stable TF layer와
3. `/rtabmap/map_stable` stable map topic layer를 같이 도입해야 한다

즉 최종 구조는 다음 한 문장으로 정리된다.

**pure spin 동안은 "RTAB-Map raw global TF"와 "RTAB-Map raw map topic"을 system이 직접 소비하지 않고, stable TF와 stable map만 소비하게 만들어 로컬 odom 기반 회전을 우선시하고, pure spin 종료 후 translation 재개 시점에만 전역 보정과 지도 갱신을 다시 천천히 허용한다.**

## 21. stable occupancy map relay layer 설계

이 섹션은 TF layer와 별개로, occupancy map topic 자체를 안정화하기 위한 relay layer를 정의한다.

목표:

1. RTAB-Map raw `/rtabmap/map`를 그대로 global consumer가 보지 않게 한다
2. pure spin 동안 raw map redraw / graph rewrite / occupancy 재작성으로 인한 시각적 회전, 뒤틀림, 고스팅을 차단한다
3. stable map topic만 Nav2 global costmap과 RViz가 소비하게 만든다

핵심 정의:

1. raw map topic
   - `/rtabmap/map`
   - RTAB-Map이 직접 publish하는 occupancy grid

2. stable map topic
   - `/rtabmap/map_stable`
   - relay layer가 publish하는 filtered occupancy grid

3. relay layer
   - raw map을 입력으로 받고
   - pure spin 상태에 따라
     - 그대로 전달하거나
     - hold-last-good 하거나
     - 종료 후 조건부 refresh를 수행하는 중간 계층

이 relay layer는 TF stabilizer와 역할이 다르다.

1. TF stabilizer
   - continuous transform attenuation

2. map relay
   - discrete occupancy message selection / hold / refresh

즉 설계 철학 자체가 다르다.

---

## 22. raw `/rtabmap/map` -> filtered `/rtabmap/map_stable`

### 22.1 입력/출력 정의

입력:

1. raw occupancy grid `/rtabmap/map`
2. pure spin 상태 또는 pure spin score
3. 필요 시 `/odometry/filtered`
4. 필요 시 IMU `wz`

출력:

1. filtered occupancy grid `/rtabmap/map_stable`

frame 규칙:

1. raw `/rtabmap/map`의 `header.frame_id`는 계속 `map`
2. stable `/rtabmap/map_stable`의 `header.frame_id`는 `map_stable`

이 규칙이 중요한 이유:

1. 현재 global TF 체인은 `map_stable -> map -> odom -> base_link`
2. raw map은 raw global frame(`map`)에 묶여 있어야 한다
3. stable map은 stable global frame(`map_stable`)에 묶여 있어야 한다

즉 stable map relay는 **메시지 자체의 frame_id를 stable global frame으로 바꿔 publish**해야 한다.

단순히 topic 이름만 바꾸고 `frame_id`를 `map` 그대로 두면 구조가 틀어진다.

### 22.2 relay layer가 해야 할 일

relay layer는 raw map을 항상 그대로 복사하는 단순 republisher가 아니다.

해야 할 일:

1. 최신 raw map을 수신한다
2. 현재 pure spin 상태를 본다
3. pure spin이 아니면 raw map을 stable map으로 갱신한다
4. pure spin이면 stable map publish를 hold-last-good 모드로 바꾼다
5. pure spin 종료 후에는 즉시 raw map으로 snap-back 하지 않고 refresh 조건이 만족될 때만 갱신한다

즉 relay layer는 occupancy map에 대해

1. pass-through
2. hold
3. delayed refresh

세 가지 동작을 갖는다.

### 22.3 왜 raw map을 직접 쓰면 안 되나

현재 문제의 핵심은 pure spin 동안 raw `/rtabmap/map`가 다시 그려질 수 있다는 점이다.

그 결과:

1. TF가 안정돼도 raw map redraw만으로 화면에서 회전/고스팅처럼 보일 수 있다
2. global costmap이 raw map 변화를 그대로 받아 navigation 기준이 출렁일 수 있다

따라서:

**stable TF를 도입했으면, stable map topic도 반드시 같이 도입해야 한다.**

TF와 map topic의 소비 계층을 반쪽만 stable로 만들면 남은 반쪽에서 동일 문제가 반복된다.

---

## 23. pure spin 중 hold-last-good map 정책

### 23.1 정책의 의미

hold-last-good map 정책은 pure spin 동안

1. raw `/rtabmap/map`는 계속 들어오더라도
2. consumer에게는 마지막으로 "정상"으로 판단된 stable map만 계속 내보내는 정책이다

즉:

1. raw map을 버리는 것이 아니다
2. raw map을 지금 당장 소비자에게 보여주지 않는 것이다

중요:

이건 TF layer의 hard freeze와 다르다.

1. TF layer는 완전 freeze하면 correction debt가 생기므로 continuous attenuation이 필요하다
2. occupancy map topic은 message 단위이므로 **hold-last-good**이 더 자연스럽다

즉 map layer에서는 pure spin 동안 hold가 오히려 정석에 가깝다.

### 23.2 "last-good"의 정의

last-good map은 단순히 마지막으로 받은 map이 아니다.

권장 정의:

1. pure spin이 아닌 상태에서 수신된 map
2. 또는 pure spin score가 충분히 낮고 translation이 안정적으로 존재하는 구간에서 수신된 map

즉:

1. pure spin 도중에 들어온 raw map은 기본적으로 last-good 후보가 아니다
2. pure spin 종료 직후 아직 불안정한 구간의 raw map도 즉시 last-good으로 승격하면 안 된다

### 23.3 hold 진입 조건

hold 진입은 pure spin score를 기준으로 한다.

즉:

1. IMU `wz`가 일정 수준 이상
2. 선속도는 낮음
3. 그 상태가 짧게라도 지속

이면:

1. TF layer는 gain attenuation으로 들어가고
2. map layer는 hold mode로 들어간다

권장 원칙:

1. TF layer보다 map layer의 hold 진입은 조금 더 보수적이어도 된다
2. 이유는 map topic은 discrete message라, 잘못 hold해도 map이 잠깐 stale해질 뿐이고
3. 반대로 너무 쉽게 raw map을 통과시키면 사용자는 즉시 시각적 twist를 본다

즉 map layer는 TF layer보다 약간 더 보수적인 spin gating이 허용된다.

### 23.4 hold 동안의 동작

hold mode에서 relay는 다음처럼 동작한다.

1. raw `/rtabmap/map` 수신은 계속한다
2. 하지만 stable `/rtabmap/map_stable` publish 내용은 마지막 last-good map을 유지한다
3. 필요하면 keepalive 성격으로 동일한 stable map을 재발행할 수 있다
4. 또는 latched/transient_local QoS로 마지막 stable map만 유지할 수도 있다

핵심:

1. hold mode에서 raw map 변화를 소비자에게 전달하지 않는다
2. pure spin 동안 global map의 시각적 기준을 고정한다

### 23.5 hold 정책의 장점

1. RViz에서 맵이 같이 도는 듯한 현상을 직접 줄일 수 있다
2. global costmap이 pure spin 동안 raw map redraw로 흔들리는 것을 막을 수 있다
3. graph rewrite가 순간적으로 들어와도 사용자에게 바로 노출되지 않는다

### 23.6 hold 정책의 리스크

1. pure spin 중 실제로 환경 변화가 있어도 즉시 반영되지 않는다
2. spin이 길어지면 stable map이 오래 stale할 수 있다

하지만 현재 문제 우선순위는:

1. pure spin 동안 severe twist 제거
2. 그 다음 map freshness

이므로, 현재 단계에선 이 trade-off가 타당하다.

---

## 24. 종료 후 조건부 refresh 정책

### 24.1 왜 "즉시 refresh"가 안 되나

pure spin이 끝났다고 raw map을 바로 stable map으로 덮어쓰면 다음 문제가 생긴다.

1. pure spin 동안 RTAB-Map 내부에 누적된 graph/map 변화가 한 번에 사용자에게 보인다
2. 결과적으로 "회전 끝나고 나서 map이 갑자기 맞춰지며 뒤집힌다"는 새 증상이 생긴다

즉 stable map layer에도 **immediate snap-back 금지**가 필요하다.

### 24.2 refresh의 전제조건

stable map을 raw map으로 다시 따라가게 만들기 위한 전제조건은 다음이 좋다.

1. pure spin score가 충분히 낮다
2. translation speed가 다시 생겼다
3. 또는 translation 누적량이 최소 기준 이상이다
4. 짧은 quiet/settling window가 지났다

즉:

1. 단순히 `wz`가 0에 가까워졌다고 바로 refresh하지 않는다
2. **실제 이동이 다시 생겼을 때** refresh를 허용한다

이게 중요한 이유:

pure spin 종료 직후는 여전히 graph/map이 unstable할 수 있기 때문이다.

### 24.3 권장 refresh 단계

조건부 refresh는 3단계로 보는 게 좋다.

#### 단계 A. pure spin 종료 감지

조건:

1. score 하강
2. `wz` 감소
3. hold 해제 후보 진입

이 단계에서는 아직 raw map을 반영하지 않는다.

#### 단계 B. translation 재개 확인

조건:

1. `|vx|` 또는 translation speed가 최소 기준 이상
2. 일정 시간 유지

이 단계에서:

1. raw map을 다시 신뢰할 수 있는 후보로 보기 시작한다

#### 단계 C. stable map refresh

조건:

1. above 조건 충족
2. raw map이 실제로 갱신됨

이 단계에서:

1. stable map을 새 raw map으로 교체
2. 이후에는 다시 pass-through 모드로 복귀

즉:

1. spin 중: hold
2. spin 끝: waiting
3. translation 재개 후: refresh

으로 보는 것이 가장 안전하다.

### 24.4 refresh를 부드럽게 해야 하나

occupancy grid는 transform처럼 continuous interpolation이 자연스럽지 않다.

따라서 map topic layer는 보통:

1. continuous blend보다
2. **조건부 discrete refresh**

가 더 현실적이다.

즉 stable map은:

1. hold-last-good 유지
2. refresh 조건 충족 시 새 raw map으로 교체

가 기본이고, TF layer에서 하듯 gain/EMA를 map 메시지 자체에 적용하는 것은 일반적으로 복잡도만 높이고 효과는 낮을 수 있다.

### 24.5 refresh 정책의 장점

1. pure spin 종료 직후 map jump를 줄인다
2. translation이 다시 생긴 뒤 안정된 raw map만 소비자에게 보여준다
3. Nav2 global costmap 기준이 회전 종료 직후 급변하는 것을 막을 수 있다

### 24.6 refresh 정책의 리스크

1. refresh가 너무 늦으면 stable map이 오래 stale하다
2. refresh가 너무 빠르면 pure spin 종료 직후 jump가 다시 나온다

따라서 refresh는:

1. `wz` 기준만이 아니라
2. translation 재개 기준을 반드시 함께 써야 한다

이 원칙이 중요하다.

---

## 25. 현재 문서 기준 stable map layer의 최종 정의

지금 이 문서 기준으로 stable map layer는 아래처럼 정의한다.

1. raw source:
   - `/rtabmap/map`

2. stable output:
   - `/rtabmap/map_stable`

3. frame:
   - raw map은 `map`
   - stable map은 `map_stable`

4. pure spin 중:
   - stable map은 last-good hold

5. pure spin 종료 직후:
   - 즉시 raw refresh 금지

6. translation 재개 후:
   - 조건부 refresh 허용

7. global consumer:
   - Nav2 global costmap, planner, RViz는 stable map을 사용

즉 최종 구조는:

1. **TF는 `map_stable -> map -> odom -> base_link`**
2. **Map은 `/rtabmap/map_stable`를 소비**

이 두 축이 함께 있어야 pure spin 동안 map이 심하게 도는 문제를 구조적으로 닫을 수 있다.

## 26. stable map layer의 한계와 전제조건

위 설계는 "raw map을 바로 보여주지 않는다"는 점에서 분명한 완화 효과가 있지만,
그 자체만으로 완전 해결이라고 보면 안 된다.

핵심 한계는 다음 한 문장으로 정리된다.

**소비자에게 raw map을 안 보여줘도, RTAB-Map 내부 전역 pose/graph 자체가 크게 틀어지면 pure spin 종료 후 결국 시스템이 그 오차를 다시 맞춰야 하므로, stable map은 단독 해법이 아니라 pure spin 구간을 잠시 버티기 위한 운영 계층이다.**

즉 stable map layer는:

1. "문제를 숨기는 장치"가 아니라
2. pure spin 동안 global consumer를 보호하는 장치

이지만,

3. RTAB-Map 내부 raw 결과가 장시간 크게 틀어지는 것까지 영구적으로 무효화하진 못한다

이 점을 명확히 해야 한다.

### 26.1 stable map이 단독 해법이 아닌 이유

stable map이 하는 일:

1. pure spin 중 raw `/rtabmap/map` redraw를 consumer에 즉시 노출하지 않음
2. RViz와 Nav2 global layer가 visual/global map distortion을 바로 받지 않게 함

하지만 stable map이 하지 못하는 일:

1. RTAB-Map 내부 graph optimization 자체를 멈추지 않음
2. raw `map->odom` 오차 누적 자체를 없애지 않음
3. pure spin이 끝난 뒤 raw 결과가 계속 틀어진 상태라면 그 차이를 영원히 덮어두지 못함

즉 stable map만 추가해도 아래 상황은 여전히 위험하다.

1. pure spin이 길게 지속됨
2. 그동안 raw RTAB-Map 전역 pose가 크게 틀어짐
3. pure spin 종료 후 translation이 재개됨
4. 시스템이 결국 raw global result를 다시 수용해야 함
5. 그 과정에서 map/TF jump가 다시 나타날 수 있음

따라서 stable map은:

1. pure spin 중 immediate disturbance를 줄이는 수단
2. pure spin 종료 후 안전하게 복귀할 시간을 버는 수단

이지,

3. 내부 raw global drift를 완전히 무효화하는 수단

은 아니다.

### 26.2 이 전략이 성립하는 전제조건

이 전략은 다음 전제조건에서만 성립한다.

1. IMU `wz`가 pure spin 구간을 reasonably 잘 감지한다
2. `odom->base_link`는 pure spin 동안 local 회전을 충분히 잘 따라간다
3. pure spin은 짧고 제한된 특수 구간이다
4. pure spin 동안 global map을 매 순간 갱신하지 않아도 주행이 크게 나빠지지 않는다
5. pure spin 종료 후 translation이 다시 생기면 시스템이 안정된 raw global result를 회복할 수 있다

즉 이 전략의 본질은:

**pure spin 동안에는 global frame/map보다 local odom을 우선하고, translation이 다시 생긴 뒤에만 global 결과를 복권한다**

이다.

### 26.3 "소비자에게 안 보이면 끝"이 아닌 이유

현재 문서 구조를 오해하면 다음처럼 보일 수 있다.

1. raw TF/raw map은 뒤에서 계속 틀어져도
2. stable TF/stable map만 보여주면 주행이 된다

이건 정확하지 않다.

왜냐하면:

1. Nav2 global planner는 결국 global frame/global map을 다시 써야 한다
2. pure spin이 끝난 뒤에는 raw global result와 stable result를 다시 정렬해야 한다
3. 내부 raw 오차가 너무 크면 stable 계층만으로는 그 복귀를 부드럽게 만들기 어렵다

즉 stable layer는 "영구 마스킹"이 아니라 **일시적인 보호 계층**이다.

### 26.4 이 전략이 유효한 구간

다음 구간에서는 유효하다.

1. 짧은 제자리회전
2. 짧은 high-rate pure spin
3. 거의 translation 없는 회전 테스트
4. local odom이 안정적으로 회전을 따라가는 구간

이 구간에서는:

1. global map을 잠깐 hold해도 정보 손실이 크지 않고
2. severe twist/gosting을 줄이는 이득이 더 크다

### 26.5 이 전략이 위험해지는 구간

다음 구간에서는 stable map 단독 전략이 위험해진다.

1. pure spin이 길게 지속됨
2. pure spin 도중 raw RTAB-Map graph drift가 커짐
3. pure spin 도중 환경 변화가 큼
4. pure spin 이후 즉시 정확한 global map refresh가 필요한 상황

이때는:

1. stable map이 오래 stale해질 수 있고
2. 종료 후 refresh 시 jump가 커질 수 있고
3. 결국 registration 본체 또는 raw global authority 자체를 더 줄여야 할 수 있다

즉 long pure spin은 stable map만으로는 충분하지 않을 수 있다.

## 27. pure spin 구간의 운영 철학 재정의

이 문서 기준으로 pure spin은 "일반 주행 상태"가 아니라 **특수 운영 모드**로 본다.

그 철학은 다음과 같다.

### 27.1 pure spin 동안

1. local odom을 우선 신뢰
2. raw global TF authority는 attenuation
3. raw global map redraw는 hold-last-good
4. global consumer는 stable TF/stable map만 사용

즉 pure spin 동안은:

**"전역 결과를 적극적으로 업데이트하는 모드"가 아니라 "로컬 회전만 안전하게 통과시키는 모드"**

이다.

### 27.2 pure spin 종료 직후

1. raw global result를 즉시 full restore하지 않음
2. translation 재개를 먼저 확인
3. quiet/settling window를 둠
4. 그 뒤 stable TF/stable map을 천천히 raw 결과와 재정렬

즉:

**종료 직후는 recovery 모드**로 봐야 한다.

### 27.3 pure spin이 짧아야 하는 이유

이 전략이 가장 잘 맞는 전제는 pure spin이 "짧은 특수구간"이라는 점이다.

짧은 pure spin이면:

1. global result를 잠깐 hold해도 시스템 전체 일관성이 유지되기 쉽다
2. local odom만으로도 충분히 회전 구간을 버틸 수 있다
3. 종료 후 translation 재개 시 global result를 다시 붙이기 쉽다

반대로 long pure spin이면:

1. raw global drift가 내부적으로 커질 수 있고
2. stable layer가 오래 stale해지고
3. recovery cost가 커진다

즉 이 문서의 stable strategy는:

**short pure spin에는 실용적이고, long pure spin에는 추가 보호가 더 필요하다**

로 이해해야 한다.

## 28. 현재 문서 기준 최종 보완 결론

stable TF와 stable map은 필요하다.

하지만 그 의미는:

1. pure spin 동안 system global consumer를 보호하는 계층
2. local odom 기반 회전을 우선시키는 계층
3. 종료 후 안정된 구간에서 global 결과를 다시 복귀시키는 계층

이지,

4. RTAB-Map 내부 raw global drift를 영구적으로 무시하는 해법

은 아니다.

즉 현재 구조의 정확한 목적은 다음 한 문장으로 정리된다.

**stable TF + stable map 전략은 pure spin 동안 global 결과를 잠시 덜 믿고 local odom 기반으로 구간을 통과시키는 운영 전략이며, 이것이 성립하려면 pure spin은 짧고, IMU/odom은 회전 이벤트 검출과 로컬 회전을 충분히 잘 표현해야 한다.**
