# 자율주행 안정화 계획서 (v12 업데이트 - 최신 로그 + RViz 사진 반영)

## 롤백 메모 (2026-03-10, P1+P2+P3 동시 적용 후 회귀)

실차 재검증 결과, `P1 2차 튜닝 + P2 + P3`를 한 번에 적용한 상태는
`P1 1차 튜닝만 적용했을 때`보다 명확히 나빠졌다.

새 증상:

1. 회전을 시작하자마자 map이 같이 돌기 시작함
2. 후반부 lumped correction이 아니라, 초반부터 전역 지도가 비틀리며 누적됨
3. 사용자가 체감한 품질은 `P1 1차 튜닝 단독`보다 악화됨

이 결과는 기존 가정 중 일부를 수정하게 만든다.

### 새 결론

1. `P1 2차 튜닝(late unlock 완화용 slow_turn)`은 현 시점 실차에서는 이득보다 부작용이 컸다.
   느린 회전을 너무 일찍 또는 너무 자주 해제 후보로 만들면서,
   아직 안정되지 않은 odom yaw를 RTAB-Map이 더 이른 시점부터 그래프 제약으로 사용하게 만들 가능성이 높다.

2. `P2 (RGBD/AngularUpdate=1.57)`는 현재 조건에서 너무 이르다.
   회전 중 노드를 조금 더 만드는 수준을 의도했지만,
   실제로는 회전 초반부터 잘못된 회전/정합 제약이 그래프에 들어가
   "늦은 1회 보정" 문제가 "초반부터 map이 같이 도는 문제"로 바뀌었다.

3. `P3 (ICP CorrespondenceRatio 완화)`도 같은 방향으로 작동했을 가능성이 높다.
   정합 품질을 올리려던 의도와 달리,
   mixed motion 구간의 불안정한 등록 결과를 더 자주 받아들여
   전역 그래프 오염 시점을 앞당겼을 가능성이 있다.

### 현재 판단

지금 필요한 것은 추가 미세튜닝이 아니라 **실험 설계 리셋**이다.

즉:

1. `P1`은 **1차 상태기계 튜닝만 유지**
2. `P1 2차 slow_turn 보정`은 롤백
3. `P2`는 롤백 (`RGBD/AngularUpdate=6.28` 복귀)
4. `P3`는 롤백 (`Icp/CorrespondenceRatio=0.01` 복귀)

이 상태를 다시 baseline으로 잡고,
360도 회전 실차에서 증상이 다시 "후반부에만 크게 보정"으로 돌아오는지 먼저 확인해야 한다.

만약 롤백 후:

1. 다시 "거의 끝에서만 map이 돈다"
   - 주원인은 여전히 `P1` 쪽이며, `yaw_lock` 단독 재튜닝이 맞다.

2. map 회전이 크게 줄어든다
   - 이번 회귀의 주범은 `P2/P3`였다고 판단할 수 있다.

이 문서는 기존 [plan.md](/home/atoz/ca_ws/plan.md)의 최신 보완판이다.
이번 업데이트는 다음 3가지를 함께 반영한다.

1. 이전 로그 `run_20260310_112653`
2. 최신 로그 `run_20260310_135537`
3. 사용자 제공 RViz 스크린샷 추가 시퀀스

핵심은 예전처럼 "`startup IMU blackout`이 주원인"으로만 볼 수 없다는 점이다.
최신 런에서는 startup은 상당히 개선되었고,
이제 남은 핵심은 **운동 중 `yaw_zeroing` 오판**에서 한 단계 더 구체화되어
**`yaw_lock`의 late unlock + RTAB-Map의 늦은 전역 보정**으로 좁혀졌다.

## 목적

현재 해결 대상은 아래 2가지다.

1. 회전/직진/후진 테스트 중 로봇이 도는 대신 RViz에서 맵이 같이 도는 것처럼 보이는 문제
2. 주행 시작 후 또는 회전 후 짧은 이동에서 `tf`와 전역 지도가 틀어지는 문제

## 이번 테스트 조건

사용자 설명 기준 테스트는 다음과 같다.

1. 제자리 회전을 360도 수행
2. 회전 중간중간 짧은 직진/후진을 섞음
3. 이 과정에서 RViz에서 맵이 같이 회전하는 현상을 확인

이 테스트 패턴은 현재 RTAB-Map 설정과 매우 안 좋게 맞물린다.
특히 `AngularUpdate=6.28`, `LinearUpdate=0.50` 조합은
"긴 회전 + 짧은 이동 반복"에서 노드 생성 간격을 지나치게 띄울 수 있다.

## 현재 결론 요약

최신 근거를 종합한 현재 결론은 다음과 같다.

1. `publish_during_calib=false` 자체는 현재 주원인이 아니다.
   최신 런에서는 IMU bias calibration이 약 6.3초에 끝난다.
   따라서 지금은 `true`로 바꿀 이유보다 `false`를 유지할 이유가 더 크다.

2. 이전 런 `run_20260310_112653`에서는
   **운동 중 `yaw_zeroing` 채터링**이 핵심 문제였다.
   startup 직후가 아니라, 보정 완료 후 약 70초 뒤 실제 회전/이동 테스트 구간에서
   `yaw_zeroing`이 다수 ON/OFF를 반복했다.

3. 최신 런 `run_20260310_135537`에서는
   상태기계 적용 후 채터링은 **229건 -> 3건**으로 크게 줄었다.
   하지만 대신 **느린 360도 회전 동안 `yaw_lock`이 너무 오래 유지되다가,
   거의 한 바퀴를 다 돈 시점에 한 번 늦게 풀리는 패턴**이 나타났다.

4. 최신 OFF는 `raw_wz=0.00277`일 때도 `reason=xy_hold`로 발생했다.
   즉 최신 문제는 "회전을 너무 자주 OFF/ON 한다"가 아니라,
   **실제 느린 회전을 `wz`로 감지하지 못하고, body 흔들림/가감속 쪽 단서로 뒤늦게 해제한다**는 것이다.

5. RViz 스크린샷은 단순히 `base_link`만 흔들린 것이 아니라,
   전역 점유격자 자체가 재정렬되거나 회전한 것처럼 보이는 상황을 보여준다.
   즉 사용자가 본 "맵이 돈다"는 체감은 단순 시각 오해가 아니라
   **전역 지도 보정 또는 `map->odom` 계열 보정이 실제로 개입했다는 강한 증거**다.

6. 최신 상태에서 map 회전의 1차 원인 체인은 다음과 같다.

```text
느린 360도 회전 중 yaw_lock이 너무 오래 유지됨
  -> odom->base_link yaw가 실제 회전을 충분히 따라가지 못함
  -> RTAB-Map이 외부 odom 초기추정을 불안정하게 받음
  -> 360도 회전 + 짧은 직진/후진 패턴에서 회전 오차가 누적됨
  -> 거의 한 바퀴를 다 돈 시점에 늦은 unlock 또는 늦은 graph correction이 한 번 크게 발생
  -> RTAB-Map이 누적된 불일치를 한 번에 전역 보정으로 흡수
  -> RViz에서 "거의 다 돌았을 때 맵이 한 번에 돈다"로 관측
```

## 사용자 제공 RViz 사진 분석

추가 사진 시퀀스를 기준으로 보면,
초반에는 지도가 비교적 유지되다가,
회전이 많이 진행된 뒤 특정 시점에서 전역 구조가 한 번에 돌아간다.

즉 이번 현상은 "계속 조금씩 도는 map"이라기보다
**회전 누적 후 특정 시점에서 한 번 크게 재정렬되는 map**에 더 가깝다.

사진 시퀀스를 기준으로 보면,
검은 점유영역의 큰 구조물 방향이 화면 격자 기준으로 유의미하게 달라진다.
이는 단순히 로봇 footprint나 `base_link` 축만 회전한 그림이 아니다.

이 현상은 최소한 아래 둘 중 하나다.

1. `map->odom` 보정이 크게 들어감
2. RTAB-Map이 전역 occupancy를 다시 정렬하면서 결과 지도가 회전한 것처럼 보임

둘 중 어느 쪽이든 공통 결론은 같다.

- 문제의 위치는 로컬 footprint 표시가 아니라 **전역 지도/전역 정합 단계**에 있다.
- 따라서 "footprint만 정상화"해서는 해결되지 않는다.
- `odom->base_link` yaw 안정화와 RTAB-Map 노드 생성 간격 완화가 같이 필요하다.
- 특히 최신 증상은 "late unlock 후 lumped correction" 가능성이 높다.

## 최신 로그 증거

기준 로그:

- 이전 분석:
  [sensor_sync.log](/home/atoz/ca_ws/logs/run_20260310_112653/sensor_sync.log)
  [rtabmap_nav2.log](/home/atoz/ca_ws/logs/run_20260310_112653/rtabmap_nav2.log)
- 최신 분석:
  [sensor_sync.log](/home/atoz/ca_ws/logs/run_20260310_135537/sensor_sync.log)
  [rtabmap_nav2.log](/home/atoz/ca_ws/logs/run_20260310_135537/rtabmap_nav2.log)

### 1. startup IMU blackout은 크게 완화됨

최신 런 `run_20260310_112653` 기준:

- bias 보정 시작: [sensor_sync.log](/home/atoz/ca_ws/logs/run_20260310_112653/sensor_sync.log:26)
- bias 보정 완료: [sensor_sync.log](/home/atoz/ca_ws/logs/run_20260310_112653/sensor_sync.log:31)
- 첫 IMU 입력 수신: [sensor_sync.log](/home/atoz/ca_ws/logs/run_20260310_112653/sensor_sync.log:33)

실측:

```text
보정 시간: 약 6.321초
보정 완료 -> 첫 IMU 수신: 약 0.001초
```

따라서 이전 `run_20260310_094804`의 80초 지연은
현재 최신 상태에서는 더 이상 주원인으로 보기 어렵다.

즉 현재 판단은 다음과 같다.

- `publish_during_calib=false`는 유지하는 편이 맞다.
- 보정 전 raw IMU를 굳이 EKF에 넣을 필요가 없다.
- startup 문제의 우선순위는 내려간다.

### 2. 대신 운동 중 yaw_zeroing 채터링이 다시 강하게 나타남

최신 런에서 `yaw_zeroing` 이벤트는 총 229건이다.

- `ON=115`
- `OFF=114`

첫 `ON`:

- [sensor_sync.log](/home/atoz/ca_ws/logs/run_20260310_112653/sensor_sync.log:32)

첫 `OFF`:

- [sensor_sync.log](/home/atoz/ca_ws/logs/run_20260310_112653/sensor_sync.log:40)

시간 해석:

```text
첫 ON:  1773109627.4899
첫 OFF: 1773109698.2225
차이:   약 70.73초
```

즉,

- startup 직후에는 비교적 안정적이었다.
- 실제 회전/이동 테스트가 시작된 뒤 채터링이 집중적으로 발생했다.

이건 매우 중요하다.
지금 문제는 더 이상 "부팅 직후 IMU 미출력"이 아니라
**실제 운동 중 정지 판정 로직이 회전/미세이동을 제대로 분리하지 못하는 것**이다.

### 3. OFF가 발생한 값들이 너무 작음

최신 로그에는 아래와 같이 매우 작은 `wz`에서도 `OFF`가 나온다.

- `wz=-0.00632`: [sensor_sync.log](/home/atoz/ca_ws/logs/run_20260310_112653/sensor_sync.log:52)
- `wz=0.00414`: [sensor_sync.log](/home/atoz/ca_ws/logs/run_20260310_112653/sensor_sync.log:60)
- `wz=0.00061`: [sensor_sync.log](/home/atoz/ca_ws/logs/run_20260310_112653/sensor_sync.log:114)
- `wz=0.00237`: [sensor_sync.log](/home/atoz/ca_ws/logs/run_20260310_112653/sensor_sync.log:206)

이 값들은 "실제 의미 있는 회전이므로 OFF"라고 보기 어렵다.
즉 `yaw_zero_threshold`만의 문제가 아니라,
현재 정지 판정 로직 전체가 너무 예민하다는 뜻이다.

특히 다음이 의심된다.

1. `wx/wy` 조건이 회전 중 진동에 과민함
2. `|a|-g` 조건이 짧은 전후진 가감속에 과민함
3. 히스테리시스와 최소 유지시간이 없어 샘플 단위로 즉시 반전됨

결론:

- `yaw_zero_threshold`만 올려서는 해결이 불완전하다.
- **운동 중 OFF 진입 기준과 정지 시 ON 복귀 기준을 분리**해야 한다.

### 4-1. 최신 런에서는 채터링이 거의 사라졌지만, 늦은 unlock이 보임

최신 런 `run_20260310_135537` 기준:

- bias 보정 시작: [sensor_sync.log](/home/atoz/ca_ws/logs/run_20260310_135537/sensor_sync.log:25)
- bias 보정 완료: [sensor_sync.log](/home/atoz/ca_ws/logs/run_20260310_135537/sensor_sync.log:30)
- 첫 IMU 입력 수신: [sensor_sync.log](/home/atoz/ca_ws/logs/run_20260310_135537/sensor_sync.log:31)
- 첫 LOCK 진입: [sensor_sync.log](/home/atoz/ca_ws/logs/run_20260310_135537/sensor_sync.log:32)
- 유일한 LOCK 해제: [sensor_sync.log](/home/atoz/ca_ws/logs/run_20260310_135537/sensor_sync.log:39)
- 다시 LOCK 진입: [sensor_sync.log](/home/atoz/ca_ws/logs/run_20260310_135537/sensor_sync.log:40)

정량 해석:

```text
전체 상태 전환: 3건
ON  -> OFF : 약 82.03초 후
OFF -> ON  : 약 12.23초 후
```

이전 런의 229건 채터링과 비교하면,
상태기계는 **채터링 억제 자체는 성공**했다고 볼 수 있다.

하지만 새로운 문제가 드러난다.

- LOCK 해제가 한 번밖에 없고,
- 그마저도 `reason=xy_hold`
- `raw_wz=0.00277`, `wz_f=0.01047`

즉 이 해제는 "yaw가 명확히 커져서 풀린 것"이 아니라,
**회전 막바지의 body 흔들림 또는 mixed motion 흔적이 xy gyro 조건을 넘기면서 뒤늦게 풀린 것**에 가깝다.

### 4-2. 사진과 최신 로그를 같이 보면 "late unlock -> lumped correction" 패턴이 가장 유력

사용자 사진 설명:

1. 360도 회전 과정 중
2. 거의 다 돌아갈 때쯤
3. map이 한 번 크게 회전함

최신 로그:

1. `yaw_lock`은 대부분의 시간 동안 유지
2. 거의 끝 무렵으로 볼 수 있는 늦은 시점에 단 한 번 `OFF`
3. 해제 사유는 `wz`가 아니라 `xy_hold`

이 둘을 합치면 가장 설득력 있는 해석은 다음과 같다.

```text
회전 초중반:
  yaw_lock이 유지되어 odom yaw가 실제 회전을 충분히 반영하지 못함

회전 누적:
  RTAB-Map 내부에서는 scan/odom 불일치가 점차 커짐

회전 막바지:
  yaw_lock이 뒤늦게 풀리거나,
  그 시점 전후로 graph optimization이 누적 오차를 한 번에 반영

관측 결과:
  map이 회전 후반부 특정 시점에 한 번 크게 도는 것처럼 보임
```

이건 기존의 "계속 채터링해서 map이 계속 흔들린다"와는 다르다.
지금 최신 증상은 **"채터링은 줄었지만, unlock이 너무 늦어서 correction이 덩어리로 발생한다"**에 더 가깝다.

### 4-3. 최신 사진 + 최신 로그를 합치면 map 회전의 위치가 더 선명해짐

이번 테스트는 "360도 회전 + 중간중간 짧은 이동"이다.
이 패턴에서 최신 로그는 운동 중 `yaw_zeroing` 채터링을 보여주고,
RViz 사진은 전역 지도 재정렬을 보여준다.

이 둘을 합치면 가장 설득력 있는 해석은 아래다.

1. 실제 운동 중 `odom->base_link` yaw 입력이 매끄럽지 못하거나, 늦게 해제됨
2. RTAB-Map은 외부 odom을 초기값으로 사용
3. 그런데 현재 `AngularUpdate=6.28`, `LinearUpdate=0.50`라서
   짧은 이동이 누적될 때까지 노드가 늦게 생긴다
4. 노드가 뒤늦게 생길 때 이미 회전과 이동이 많이 섞인 상태라
   등록 링크가 커진다
5. 특히 회전이 거의 끝난 시점에 한 번 크게 보정이 들어오면
   사진처럼 map이 후반부에 갑자기 같이 도는 현상이 나타난다

### 5. RTAB-Map 로그에는 직접 품질지표가 부족함

최신 [rtabmap_nav2.log](/home/atoz/ca_ws/logs/run_20260310_135537/rtabmap_nav2.log:1)은
startup 메시지는 보이지만,
노드 추가/등록 residual/loop closure 품질을 직접 읽을 정도로 자세하지 않다.

현재 로그에서 직접 보이는 것은 startup transform 경고 정도다.

- [rtabmap_nav2.log](/home/atoz/ca_ws/logs/run_20260310_135537/rtabmap_nav2.log:25)

따라서 아래는 **강한 추론**이다.

- "RTAB-Map이 큰 링크에서 map 보정을 먹는다"는 가설은
  파라미터, 테스트 패턴, 스크린샷, `yaw_zeroing` 로그를 함께 보면 매우 강하다.
- 하지만 ICP fitness / residual / node creation 시점 로그가 없으므로
  최종 확정에는 추가 로깅이 필요하다.

## 확정된 원인과 강한 추론 구분

### 확정된 것

1. startup IMU blackout은 최신 런 기준 약 6.3초 수준으로 줄었다.
2. 이전 런에서는 운동 중 `yaw_zeroing` 채터링이 대량 발생했다.
3. 최신 런에서는 상태기계 적용 후 전환 수가 3건으로 줄었다.
4. 하지만 최신 런의 유일한 OFF는 `raw_wz`가 아니라 `xy_hold` 기반 늦은 해제다.
5. 사용자 사진상 전역 지도 정렬이 실제로 흔들린다.

### 강한 추론

1. 현재 mixed motion 테스트 패턴에서 `AngularUpdate=6.28`, `LinearUpdate=0.50`가
   과도하게 큰 등록 링크를 만든다.
2. 최신 런에서는 `yaw_lock`이 느린 회전 동안 너무 오래 유지되어
   RTAB-Map에 늦은 correction을 유도했을 가능성이 높다.
3. 그 큰 링크와 늦은 unlock이 graph optimization으로 전달되어
   사용자가 보는 "회전 막바지 map 회전"을 만든다.
4. `Icp/CorrespondenceRatio=0.01`의 느슨한 설정이 이 문제를 더 키우고 있다.

## 수정 우선순위 (최신 기준 재정렬)

### P0. `publish_during_calib=false` 유지, startup 개선 상태 고정

현재 판단:

- `publish_during_calib`는 **계속 `false` 유지**
- `stationary_threshold`는 지금처럼 완화된 상태를 유지

이유:

- 보정 시간이 이미 6초대면 실전에서 감당 가능하다.
- 굳이 보정 전 raw IMU를 EKF에 넣어 초기 오차 리스크를 늘릴 이유가 없다.
- startup은 이제 "해결된 편"으로 보고, 다음 문제로 넘어가는 것이 맞다.

### P1. yaw_zeroing 상태기계는 유지하되, "late unlock"을 막도록 2차 튜닝

현재 최우선 수정 포인트다.

필수 방향:

1. `enter`와 `exit` threshold를 분리
2. 최소 유지시간(dwell time / debounce) 추가
3. 단발성 `accel` 또는 `wx/wy` 스파이크로 바로 OFF하지 않게 변경
4. 회전/짧은 이동 중에는 OFF가 연속 유지되도록 변경
5. 정지 상태가 일정 시간 지속됐을 때만 ON으로 복귀
6. **느린 지속 회전이 `wz` 기준에서 누락되지 않도록 slow-turn escape를 추가 검토**

중요 판단:

- 지금은 `cmd_vel` 의존 설계로 가는 것이 아니다.
- 사용자는 리모컨 테스트도 수행하므로,
  **관측 기반(measured motion 기반) 정지 판정**이 우선이다.

최신 기준 추천:

1. 채터링 억제를 위해 상태기계는 유지
2. 다음 조정 포인트는 `exit_wz` / `hard_exit_wz`의 실회전 민감도 검증
3. 필요 시 `|wz|` 누적 적분 또는 작은 누적 yaw angle로 unlock하는 slow-turn escape 추가
4. 목표는 "회전 초중반부터 적절히 OFF, 후반부에 한 번 늦게 OFF 금지"

### P2. RTAB-Map 노드 생성 간격을 mixed motion 테스트에 맞게 줄임

현재 사용자 테스트 패턴에서는 이 항목 우선순위가 높다.

1차 권장:

1. `RGBD/AngularUpdate = 6.28 -> 1.0 rad`
2. `RGBD/LinearUpdate = 0.50`은 유지하되,
   동일 테스트에서 map 회전이 계속되면 `0.30~0.35`로 2차 인하

이유:

- 제자리 회전만 할 때보다,
  "회전 + 짧은 전후진"에서는 `LinearUpdate=0.50`가 더 늦게 노드를 만들 수 있다.
- 그 결과 노드 간 변환량이 회전/이동 혼합 상태로 커진다.
- 이 상황이 사진처럼 전역 지도 재정렬을 유도할 가능성이 높다.
- 특히 최신처럼 unlock이 늦으면 RTAB-Map correction이 더 "덩어리"처럼 들어올 수 있다.

### P3. ICP 등록 조건을 더 엄격하게 조정

이 단계는 P1/P2 뒤에 간다.

방향:

1. `Icp/CorrespondenceRatio=0.01` 상향 검토
2. 1차 후보는 `0.05`
3. 필요 시 `0.1`

의미:

- 품질이 낮은 매칭을 그래프에 덜 강하게 반영하게 만든다.
- 다만 앞단 yaw 관측이 흔들리면 ICP만 조여도 한계가 있다.

### P4. EKF 주파수는 올리지 않음

현재 판단은 유지한다.

- `20Hz`도 안정적으로 소화 못 하는 로그가 있었으므로
  지금 `50Hz`는 해법이 아니다.
- 더 빠른 EKF보다 더 안정적인 yaw 입력이 먼저다.

### P5. 추가 계측을 붙여 원인 확정도를 올림

현재 사진과 로그만으로 방향성은 충분히 잡혔지만,
RTAB-Map 내부 등록 품질은 직접 수치가 부족하다.

따라서 다음 계측이 필요하다.

1. mixed motion 테스트 동안 `tf2_echo odom base_link`
2. mixed motion 테스트 동안 `tf2_echo map odom`
3. `/odometry/filtered`의 `angular.z`
4. `yaw_zeroing` 상태 로그
5. 가능하면 RTAB-Map 노드 추가/등록 품질 로그
6. 가능하면 회전 시작/중반/후반을 영상 또는 타임스탬프로 구간화

목표:

- "yaw가 먼저 흔들렸는지"
- "그 직후 map 보정이 들어갔는지"
- "노드 추가 시점이 짧은 이동 뒤늦게 발생하는지"

를 같은 시간축에서 맞추는 것이다.

## 검증 기준 (이번 테스트 패턴 기준)

### 단계 1. startup 검증

성공 기준:

- launch 후 10초 이내 `IMU bias calibrated`
- 바로 이어서 `/camera/camera/imu_fixed` 입력 시작
- `publish_during_calib=false` 유지

현재 최신 런은 이 조건을 이미 거의 만족한다.

### 단계 2. mixed motion 중 yaw_zeroing 검증

테스트:

1. 제자리 회전
2. 짧은 직진/후진
3. 다시 회전

성공 기준:

- 실제 회전/이동 중에는 `OFF`가 유지되어야 함
- 지금처럼 ON/OFF가 수십~수백 번 튀면 실패

### 단계 3. `odom->base_link` yaw 추종 검증

성공 기준:

- 회전 중 yaw가 실제 회전을 부드럽게 따라감
- 정지 직후에만 천천히 안정화
- 이동 중 0 강제 주입 느낌이 없어야 함

### 단계 4. 전역 지도 재정렬 검증

현재와 같은 "360도 회전 + 짧은 이동" 테스트에서:

- 전역 occupancy가 격자 기준으로 크게 돌아가거나 재배치되지 않아야 함
- 사용자가 보는 "맵이 같이 돈다" 현상이 사라져야 함

## 최종 적용 순서

1. `P0`: 현재 startup 상태 유지 (`publish_during_calib=false`)
2. `P1`: `yaw_zeroing` 상태기계의 late unlock 방지 튜닝
3. `P2`: `AngularUpdate` 우선 조정, 필요 시 `LinearUpdate` 추가 조정
4. `P3`: ICP 등록 강건화
5. `P5`: 추가 계측으로 원인 확정
6. `P4`: EKF 주파수는 마지막에만 재평가

## v12 추가 업데이트 (2026-03-10, 롤백 후 재판정)

이번 섹션은 `P1 2차 + P2 + P3`를 실제로 적용했다가 회귀가 발생했고,
이후 `P1 1차만 유지 / P2 / P3 롤백` 상태에서 다시 실차를 본 결과를 반영한다.

기준 로그:

- [sensor_sync.log](/home/atoz/ca_ws/logs/run_20260310_150558/sensor_sync.log)
- [rtabmap_nav2.log](/home/atoz/ca_ws/logs/run_20260310_150558/rtabmap_nav2.log)

## 최신 판정

현재 상태는 **부분 해결**이다.

이 판정을 더 정확하게 쓰면:

- **대형 회귀는 제거됨**
- **잔여 전역 yaw 보정은 남아 있음**

즉, 아래 두 문장이 동시에 참이다.

1. 예전처럼 회전 시작 직후 map이 같이 크게 돌며 전역 지도가 무너지는 수준은 아니다.
2. 하지만 후속 움직임에서 사용자가 "맵이 살짝 왼쪽으로 돈 느낌"을 다시 관측했다.

따라서 현재 상태를 `해결`이라고 부르면 과장이고,
`재현 대기`라고 부르면 이미 관측된 잔여 증상을 무시하는 셈이다.

가장 정확한 분류는 **부분 해결**이다.

## 최신 로그가 말하는 것

### 1. yaw lock 이벤트는 아직 남아 있다

최신 런 `run_20260310_150558`에서 확인된 주요 이벤트:

1. bias 보정 완료:
   [sensor_sync.log:31](/home/atoz/ca_ws/logs/run_20260310_150558/sensor_sync.log:31)

2. 첫 `yaw_zeroing=ON reason=steady_hold`:
   [sensor_sync.log:33](/home/atoz/ca_ws/logs/run_20260310_150558/sensor_sync.log:33)

3. mixed motion 중 `yaw_zeroing=OFF reason=xy_hold`:
   [sensor_sync.log:40](/home/atoz/ca_ws/logs/run_20260310_150558/sensor_sync.log:40)

4. 추가 움직임에서 `yaw_zeroing=OFF reason=hard_wz`:
   [sensor_sync.log:42](/home/atoz/ca_ws/logs/run_20260310_150558/sensor_sync.log:42)

5. 다시 `yaw_zeroing=ON reason=steady_hold`:
   [sensor_sync.log:43](/home/atoz/ca_ws/logs/run_20260310_150558/sensor_sync.log:43)

이 뜻은 다음과 같다.

1. 강한 실제 회전은 `hard_wz`가 정상 감지한다.
2. 하지만 mixed motion에서 `xy_hold` 기반 해제가 여전히 존재한다.
3. 따라서 상태기계가 완전히 틀린 것은 아니지만,
   mixed motion에 특화된 잔여 unlock 경로가 아직 살아 있다.

### 2. 그런데 대형 map 회전은 줄었다

이건 `P1`이 완전히 해결돼서가 아니다.

핵심은 `P2/P3` 롤백으로
**오차가 전역 그래프에 초반부터 증폭되는 경로가 다시 약해졌기 때문**이다.

현재 baseline:

- `RGBD/LinearUpdate = 0.50`: [rtabmap.launch.py:369](/home/atoz/ca_ws/src/rtabmap_ros/rtabmap_launch/launch/rtabmap.launch.py:369)
- `RGBD/AngularUpdate = 6.28`: [rtabmap.launch.py:370](/home/atoz/ca_ws/src/rtabmap_ros/rtabmap_launch/launch/rtabmap.launch.py:370)
- `Icp/CorrespondenceRatio = 0.01`: [rtabmap.launch.py:390](/home/atoz/ca_ws/src/rtabmap_ros/rtabmap_launch/launch/rtabmap.launch.py:390)

이 조합은 회전 중 노드 생성과 ICP 개입을 줄인다.
그 결과:

1. 회전 초반부터 잘못된 제약이 연속적으로 graph에 들어가는 경로가 줄어든다.
2. 예전처럼 회전 시작하자마자 map이 같이 크게 도는 회귀가 억제된다.
3. 대신 residual issue는 더 작은 전역 보정으로 남는다.

즉 현재 안정성은
**근본 해결보다 증폭 경로 차단 효과**에 가깝다.

## 왜 "이번에는 괜찮다가, 한 번 더 움직이면 살짝 돈다"처럼 보이는가

이건 단순 운이 아니다.
하지만 완전한 재현성 확보도 아니다.

더 정확한 표현은:

**조건부 재현 상태**다.

map 회전이 눈에 띄게 보이려면 보통 아래 조건이 함께 맞아야 한다.

1. `yaw_lock` 또는 EKF yaw에서 실제 회전과 약간의 불일치가 생긴다.
2. 그 시점에 RTAB-Map이 새 노드를 만들거나 등록/최적화를 수행한다.
3. 그 결과가 전역 그래프에 반영된다.

현재는 `2, 3`이 이전보다 약해졌기 때문에
`1`이 남아 있어도 항상 대형 map 회전으로 드러나지 않는다.

하지만 `1`이 완전히 사라진 건 아니기 때문에,
사용자가 본 것처럼 후속 움직임에서 **작은 map yaw 보정**은 다시 나타날 수 있다.

## 지금까지의 실험으로 폐기된 가설

다음 가설들은 실차 기준으로 더 이상 우선 해법으로 보지 않는다.

### 폐기 1. slow_turn을 넣으면 late unlock이 줄고 map도 안정해질 것이다

실제 결과:

1. mixed motion에서 더 이른 unlock 후보가 생겼다.
2. 그 상태에서 P2/P3까지 같이 들어가면
   회전 초반부터 graph가 오염되기 쉬워졌다.
3. 결과적으로 map 조기 회전 회귀가 더 심해졌다.

결론:

- `slow_turn`은 현 시스템에서 우선 해법이 아니다.

### 폐기 2. AngularUpdate를 줄여 회전 중 노드를 조금 더 만들면 correction이 더 잘 분산될 것이다

실제 결과:

1. 분산보다 먼저 "잘못된 제약의 조기 입력"이 더 크게 작용했다.
2. 회전 시작 직후부터 map이 같이 도는 회귀가 발생했다.

결론:

- 지금 단계에서 `AngularUpdate` 재인하는 것은 금지한다.

### 폐기 3. CorrespondenceRatio를 올리면 ICP가 더 안정적이므로 map도 같이 좋아질 것이다

실제 결과:

1. 현 시스템에서는 "좋은 정합만 남긴다"보다
2. "불안정한 상태에서도 그래프가 더 자주 반응한다"는 부작용이 더 빨리 나타났다.

결론:

- 현재 `P3`는 해결 카드가 아니라 회귀 카드다.

## 현재 baseline과 그 의미

현재 유지해야 하는 baseline은 다음과 같다.

### Baseline-A. startup/IMU

- `publish_during_calib = false`
- `stationary_threshold = 0.01`
- `P1 1차 상태기계 유지`

의미:

1. startup blackout은 더 이상 주원인이 아니다.
2. 보정 전 raw IMU를 굳이 EKF에 흘릴 필요도 없다.
3. 현재는 startup보다 mixed motion residual이 문제다.

### Baseline-B. RTAB-Map/ICP

- `RGBD/AngularUpdate = 6.28`
- `RGBD/LinearUpdate = 0.50`
- `Icp/CorrespondenceRatio = 0.01`

의미:

1. map이 완벽히 고정되는 최적값이라서가 아니다.
2. 현재까지 확인된 조합 중 **대형 회귀를 가장 잘 막는 안전선**이다.

즉 지금 이 값은 "완성 파라미터"가 아니라
**회귀를 막기 위한 잠정 freeze baseline**이다.

## 남은 핵심 문제를 어떻게 좁혀야 하는가

이제 남은 문제는 막연히 "yaw 문제"가 아니다.

최신 로그 기준으로 residual issue를 더 좁히면:

**mixed motion 중 `xy_hold` 기반 unlock과, 그 직후의 잔여 map yaw correction**  

즉 다음 조사/수정의 중심축은 `xy_hold`다.

### 왜 `xy_hold`가 핵심인가

최신 런에서 남은 unlock 유형은 사실상 두 가지다.

1. `xy_hold`
2. `hard_wz`

여기서:

- `hard_wz`는 강한 실제 회전이므로 정상 동작일 가능성이 높다.
- 반대로 `xy_hold`는
  roll/pitch 흔들림, mixed motion, 짧은 가감속이 unlock으로 반영되는 경로다.

따라서 residual 문제를 줄이려면
`wz`보다 먼저 `xy` 기반 보조 해제를 의심하는 것이 논리적으로 맞다.

## 최신 기준 해결 계획

이제부터의 해결 계획은 예전 순서와 다르다.

핵심은:

1. **현재 baseline을 고정**
2. **P1만 더 좁혀서 다룸**
3. **P2/P3는 건드리지 않음**

이다.

### Step 1. baseline 유지

다음은 당분간 수정 금지다.

1. `RGBD/AngularUpdate`
2. `Icp/CorrespondenceRatio`
3. `slow_turn`류 late-unlock 완화 로직

이유:

- 이 셋은 이미 실차에서 "좋아질 줄 알고 만졌는데 더 나빠진" 축이다.
- 다시 동시에 또는 섣불리 재투입하면 현재의 부분 해결 상태를 잃는다.

### Step 2. `xy_hold`와 map 미세 회전의 시간 상관관계 확보

다음 수정 전에 반드시 확인해야 할 것:

1. `yaw_zeroing=OFF reason=xy_hold`
2. `/odometry/filtered.twist.twist.angular.z`
3. `tf2_echo odom base_link`
4. `tf2_echo map odom`
5. 사용자가 본 "맵이 살짝 왼쪽으로 돈 느낌" 시점

목적:

- `xy_hold`가 실제 필요한 해제인지
- 아니면 전역 보정을 유발하는 불필요한 해제인지

를 분리하기 위함이다.

이 상관관계 없이 다음 파라미터를 건드리면,
또다시 "개선 시도 -> 회귀"를 반복하게 된다.

### Step 3. 상관관계가 맞을 때만, P1의 `xy` 축만 단일 조정

이 단계는 로그 상관관계가 확인된 뒤에만 허용한다.

가능한 조정은 둘뿐이다.

#### 후보 A. `yaw_lock_exit_xy`만 소폭 상향

의도:

- 차체 흔들림만으로 unlock되는 빈도를 줄인다.

원칙:

1. 단일 파라미터만 수정
2. 큰 스텝 금지
3. 예: `0.05 -> 0.06` 정도의 1단계만 허용

#### 후보 B. `xy_hold`의 dwell time만 `wz_hold`와 분리

의도:

- `hard_wz`와 `wz_hold`는 그대로 두고
- `xy_hold`만 조금 더 신중하게 만든다.

이 접근이 의미 있는 이유:

1. 실제 yaw 기반 회전 추종성은 덜 건드린다.
2. mixed motion의 body shake만 조금 더 잘 무시할 수 있다.

단, 이 역시 `hard_wz`와 `xy_hold`의 시간 상관관계가 먼저 확인된 뒤에만 가능하다.

## 지금 기준에서 하지 말아야 할 것

### 금지 1. P2/P3 재도입

현재 문서에서 가장 강한 금지사항이다.

다음이 해결되기 전에는 하지 않는다.

1. `xy_hold` residual issue의 시간 상관관계 확인
2. P1 단독 상태에서 residual issue 감소 확인

그 전까지 `AngularUpdate`나 ICP를 다시 건드리면,
남아 있는 odom/yaw 잔차가 map graph에 더 빨리 올라가
대형 회귀가 다시 생길 가능성이 높다.

### 금지 2. 여러 축 동시 수정

이미 한 번 실패했다.

앞으로는:

1. 한 번에 하나의 축만 수정
2. 한 번 수정하면 같은 mixed motion 실차로 재검증
3. 결과가 좋지 않으면 즉시 baseline 복귀

이 원칙을 지켜야 한다.

## 최신 기준 성공/실패 정의

### 성공

1. 회전 시작 직후 map이 같이 크게 돌지 않는다.
2. 후속 움직임에서도 map yaw shift가 체감되지 않는다.
3. `xy_hold`가 드물거나, 발생해도 map 체감 회전과 겹치지 않는다.

### 실패

1. 사용자가 다시 "맵이 살짝 왼쪽으로 돈 느낌"을 반복 관측한다.
2. 그 시점이 `xy_hold`와 반복적으로 겹친다.
3. `P2/P3`를 건드리지 않았는데도 map shift가 누적된다.

이 경우 다음 수정은
`P1` 내부의 `xy` 보조 해제 경로 하나만 다루는 방향으로 제한해야 한다.

## 실무 결론

현재 단계에서 가장 중요한 판단은 다음 한 줄이다.

**지금은 "더 많은 튜닝"의 단계가 아니라, 현재 baseline을 보존하면서 residual issue를 `xy_hold` 하나로 좁히는 단계다.**

즉:

1. 지금 상태를 완전 해결로 착각하면 안 된다.
2. 하지만 baseline을 잃을 정도로 다시 크게 건드려서도 안 된다.
3. 다음 수정은 `P1` 안에서도 `xy` 보조 해제 축 하나만 허용된다.

## v12 추가 업데이트 (2026-03-10, 원인 통합 재정의)

이전까지는 아래 두 표현을 따로 썼다.

1. "`yaw_lock`이 정지 판정을 너무 넓게 잡는다"
2. "`저속 yaw` 해제가 늦고 `xy_hold`로 뒤늦게 풀린다"

하지만 최신 baseline과 실차 관측을 다시 합치면,
이 둘은 사실상 **같은 원인의 두 표현**으로 보는 것이 맞다.

정확한 통합 원인은 다음과 같다.

### 통합 원인

**mixed motion 구간의 작은 지속 yaw가 `moving yaw`로 충분히 승격되지 못하고,  
그 결과 `yaw_lock`이 실제보다 오래 유지되다가,  
마지막에는 `wz`가 아니라 `xy_hold` 경로로 뒤늦게 탈출한다.**

즉,

- "정지 판정이 넓다"는 말은 결과적으로 맞지만,
- 더 기술적으로는
  **저속 yaw escape 경로가 부족해서 quasi-stationary 상태에 너무 오래 머문다**가 핵심이다.

## 왜 두 현상이 같은 원인인가

이 둘을 따로 보면 혼동이 생긴다.
실제로는 아래 한 줄로 연결된다.

```text
작은 실제 yaw 발생
-> wz 기반 해제 조건을 못 넘김
-> yaw_lock 유지
-> EKF yaw 반영 지연
-> 차체 흔들림/roll-pitch 성분 누적
-> xy_hold로 뒤늦게 unlock
-> 그 시점 전후로 map 미세 yaw correction 체감
```

즉:

1. `정지 판정이 넓다`
   - 실제로는 정지 아님
   - 그런데 lock 상태를 계속 유지함

2. `저속 yaw 해제가 늦다`
   - 실제로는 작은 회전이 시작됐음
   - 그런데 `wz`가 escape를 만들지 못함

3. `xy_hold가 residual issue의 핵심이다`
   - 정상적인 yaw 해제가 아니라
   - 흔들림 기반 보조 해제로 나중에 빠져나오기 때문

결론:

세 문장은 서로 다른 원인이 아니라,
**같은 상태기계 구멍(state hole)을 서로 다른 관점으로 설명한 것**이다.

## 이 문제의 이름을 다시 정의

이제부터 문서에서는 이 잔여 문제를 다음 이름으로 다룬다.

### `quasi-moving yaw hole`

의미:

1. 로봇은 완전 정지가 아니다.
2. 하지만 회전도 강하지 않다.
3. 현재 상태기계는 이 중간 구간을 `moving`으로 충분히 빨리 올리지 못한다.
4. 그래서 `yaw_lock`이 길게 유지되고,
5. 끝에는 `xy_hold`가 주된 탈출 경로가 된다.

이 이름을 쓰는 이유는,
이 문제를 단순히 "`threshold` 문제"로 잘못 축소하지 않기 위해서다.

지금 문제는 단일 threshold가 아니라:

1. `wz exit sensitivity`
2. `xy` 보조 해제 구조
3. hold 시간 분리 부재
4. mixed motion 구간의 상태 전이 정의

가 함께 만든 구조 문제다.

## 최신 기준 해결 목표

이제 목표는 "yaw lock을 더 약하게 만들기"가 아니다.

정확한 목표는 다음과 같다.

### 목표

**저속 yaw가 발생했을 때, `xy_hold`가 아니라 `yaw evidence`만으로 더 이르게, 더 정상적으로 unlock되게 만들 것**

즉:

1. 강한 회전은 여전히 `hard_wz`로 즉시 해제
2. 완전 정지는 여전히 `steady_hold`로 안정적으로 lock
3. 그 중간의 mixed motion 저속 yaw는
   `xy 흔들림`이 아니라 `yaw evidence`로 판단

이게 핵심이다.

## 최신 기준 해결 계획

이 계획은 반드시 **현재 baseline 유지**를 전제로 한다.

### Step 0. baseline 동결

동결 항목:

1. `RGBD/AngularUpdate = 6.28`
2. `RGBD/LinearUpdate = 0.50`
3. `Icp/CorrespondenceRatio = 0.01`
4. `publish_during_calib = false`
5. `P1 1차 상태기계 유지`

이 단계는 선택이 아니라 필수다.
이 baseline을 흔들면 residual issue 분석보다 대형 회귀가 먼저 재발할 위험이 크다.

### Step 1. `xy_hold`가 정말 false unlock인지 시간축으로 확정

다음 항목을 같은 시간축에서 맞춘다.

1. `yaw_zeroing=OFF reason=xy_hold`
2. `/odometry/filtered.twist.twist.angular.z`
3. `tf2_echo odom base_link`
4. `tf2_echo map odom`
5. 사용자가 본 "맵이 살짝 왼쪽으로 돈 느낌" 시점

판정 기준:

1. `xy_hold` 직후 `angular.z`가 실제 의미 있는 회전으로 이어지지 않으면
   - false unlock 가능성이 크다.

2. `xy_hold` 직후 `map->odom` 또는 전역 occupancy가 미세하게 다시 정렬되면
   - `xy_hold`가 residual map shift의 직접 트리거일 가능성이 크다.

### Step 2. 다음 수정은 `xy` 축 하나만 허용

상관관계가 확인되면 다음 수정은 하나의 축만 허용한다.

#### 후보 A. `xy_hold` 전용 hold 시간 분리

현재 구조에서는 `wz_hold`와 `xy_hold`가 같은 `exit_hold_sec`를 공유한다.
이 구조는 residual issue를 좁히는 데 불리하다.

목표:

1. `hard_wz`는 그대로 즉시 해제
2. `wz_hold`는 현재 responsiveness 유지
3. `xy_hold`만 더 긴 hold를 사용

효과 기대:

1. 실제 회전 추종은 유지
2. 차체 흔들림 기반 false unlock만 감소

이게 현재 가장 보수적이고 논리적인 1순위 수정안이다.

#### 후보 B. `yaw_lock_exit_xy`만 소폭 상향

조건:

- 후보 A보다 구현은 단순하지만,
- `xy` 감도를 직접 바꾸므로 보정량을 아주 작게 제한해야 한다.

원칙:

1. 단일 스텝만 허용
2. 예: `0.05 -> 0.06`
3. 동시에 다른 파라미터 수정 금지

의미:

- 작은 차체 흔들림으로 인한 unlock 빈도를 줄인다.

주의:

- 너무 크게 올리면 실제 mixed motion에서 필요한 해제까지 늦어질 수 있다.

### Step 3. `wz` 축은 지금 당장 공격적으로 건드리지 않는다

이게 중요한 정책이다.

왜냐하면:

1. 강한 회전은 이미 `hard_wz`로 정상 분리되고 있다.
2. 문제는 `wz`가 전혀 없어서가 아니라,
   작은 yaw가 `moving`으로 충분히 빨리 승격되지 않는 것이다.
3. 이때 `wz` 임계값을 무작정 낮추면
   다시 불필요한 조기 unlock이나 chatter가 재발할 수 있다.

따라서 현 단계에서는:

1. `hard_wz` 수정 금지
2. `exit_wz` 대폭 수정 금지
3. 먼저 `xy` 보조 경로만 정리

가 맞다.

### Step 4. `slow_turn`은 현재 해법에서 제외

이 항목은 문서에 명시적으로 제외 정책으로 남긴다.

이유:

1. 개념상으로는 저속 yaw 문제를 직접 겨냥하는 듯 보였지만
2. 실차에서는 조기 graph 오염과 결합되며 대형 회귀를 만들었다
3. 현재 baseline에서는 `slow_turn`보다 `xy_hold` 제어가 더 안전하고 더 직접적이다

따라서:

- `slow_turn`은 현재 해결계획에서 제외한다.

## 최종 문제정의와 해결전략

이제 이 잔여 문제는 아래처럼 한 문장으로 정리한다.

### 최종 문제정의

**현재 residual map yaw shift는 `정지 판정이 넓어서` 생긴다기보다,  
mixed motion의 작은 실제 yaw가 `moving`으로 제때 승격되지 못하고,  
결국 `xy_hold`라는 보조 해제 경로로 늦게 풀리기 때문에 발생한다.**

### 최종 해결전략

1. baseline 유지
2. `xy_hold`와 map shift의 시간 상관관계 확정
3. `xy` 보조 해제 경로만 단일 축 미세조정
4. `P2/P3`는 건드리지 않음
5. `slow_turn`은 제외

## 실행 우선순위

1. 현재 baseline 그대로 실차 로그/체감 상관관계 수집
2. `xy_hold`가 residual map shift와 반복적으로 맞물리는지 확인
3. 맞물리면 `xy_hold` 전용 hold 분리 또는 `exit_xy` 소폭 상향 중 하나만 선택
4. 같은 mixed motion 시나리오로 재검증
5. 결과가 나쁘면 즉시 baseline 복귀

이 순서를 지키지 않으면,
현재의 `부분 해결` 상태를 잃고
다시 초반 대형 map 회전 회귀로 돌아갈 가능성이 높다.

## v12 추가 업데이트 (2026-03-10, 후보 A 적용)

현재 문서의 잔여 문제 정의에 따라,
다음 수정은 `xy` 보조 해제 경로 하나만 건드리는 것이 맞다고 판단했다.

이번 적용에서는 **후보 A**를 선택한다.

### 선택 이유

1. `hard_wz`는 강한 실제 회전을 정상적으로 분리하고 있다.
2. `wz_hold`는 실제 yaw 회전 추종성과 직접 연결되어 있어 건드리면 부작용이 크다.
3. residual issue는 mixed motion 중 `xy_hold`가 늦은 보조 탈출 경로로 작동하는 데 가장 강하게 연결돼 있다.
4. 따라서 threshold 자체를 바꾸기보다, `xy_hold`의 시간 조건만 분리하는 것이 가장 보수적이고 안전하다.

### 실제 적용 내용

기존:

1. `wz_hold`와 `xy_hold`가 같은 `exit_hold_sec`를 공유
2. 두 보조 해제가 모두 약 `30ms` 기준으로 unlock

변경:

1. `wz_hold`는 기존 유지
   - `yaw_lock_exit_hold_sec = 0.03`

2. `xy_hold`만 별도 hold 사용
   - `yaw_lock_exit_xy_hold_sec = 0.08`

의미:

1. 실제 yaw 회전 시작은 기존 responsiveness 유지
2. body shake / roll-pitch 진동 / 짧은 mixed motion 흔들림은
   `xy_hold`가 바로 unlock하지 않도록 더 오래 확인

### 이번 수정의 범위 제한

중요:

1. `exit_xy` threshold는 바꾸지 않음
2. `exit_wz`, `hard_wz`는 바꾸지 않음
3. `P2/P3`는 여전히 건드리지 않음
4. `slow_turn`은 다시 넣지 않음

즉 이번 수정은 철저히
**"`xy_hold`의 시간 조건만 분리"**
에 한정된다.

### 기대 효과

성공하면:

1. `wz_hold` 기반 실제 회전 응답은 유지
2. `xy_hold` 기반 false unlock만 감소
3. mixed motion 후 사용자가 느끼는 미세 map yaw shift가 줄어듦

실패하면:

1. 여전히 `xy_hold`가 map shift와 겹침
2. 또는 반대로 필요한 unlock까지 늦어짐

이 경우에만 다음 후보인
`yaw_lock_exit_xy` 소폭 상향을 검토한다.

## v12 추가 업데이트 (2026-03-11, 후보 A의 한계와 low-rate yaw unlock 계획)

후보 A를 적용한 뒤 다시 정리하면,
이 수정은 **`xy_hold` false unlock 억제책**이지
현재 남아 있는 근본 구조 문제를 직접 해결하는 방법은 아니다.

즉, 아래 두 문장은 동시에 참이다.

1. 후보 A는 `xy_hold`가 너무 쉽게 열리는 문제를 줄이는 데는 의미가 있다.
2. 하지만 **느린 지속 회전이 `wz` 경로로 unlock되지 않는 구조**는 그대로 남아 있다.

따라서 현재 문서의 해결 목표는 더 정확하게 다시 써야 한다.

### 현재 남은 근본 문제

현재 baseline의 핵심 한계는 다음과 같다.

1. 강한 회전은 `hard_wz`로 풀린다.
2. 보통 회전은 `wz_hold`로 풀릴 수 있다.
3. 하지만 `0.01 ~ 0.04 rad/s` 수준의 **저속 지속 yaw**는
   `hard_wz`에도 걸리지 않고,
   `wz_hold`에도 잘 도달하지 못한다.
4. 결국 이 구간은 `xy_hold`를 사실상의 주 탈출 경로로 쓰게 된다.

이 구조 때문에,
현재 residual issue는 아래처럼 설명된다.

```text
작은 실제 yaw 발생
-> abs(wz_f)가 exit_wz까지 도달하지 못함
-> yaw_lock 유지
-> yaw는 moving으로 충분히 승격되지 못함
-> mixed motion 중 body shake / roll-pitch 성분이 누적
-> xy_hold가 뒤늦게 unlock
-> 그 시점 전후로 map 미세 yaw correction 체감
```

즉 현재 문제의 본질은
**`xy_hold`가 나쁘다**가 아니라,
**저속 yaw를 yaw 근거만으로 moving으로 승격시키는 전용 경로가 없다**는 점이다.

### 문제 이름 재정의

이 잔여 구조 문제는 다음 이름으로 관리한다.

**`low-rate yaw unlock hole`**

의미:

1. 로봇은 완전 정지가 아니다.
2. 실제로는 조금씩 회전하고 있다.
3. 하지만 현재 상태기계는 그 회전을 충분히 이르게 `moving yaw`로 인정하지 못한다.
4. 그래서 `xy_hold` 같은 보조 경로가 주 unlock 경로처럼 동작하게 된다.

### 후보 A의 정확한 위치

후보 A는 폐기할 수정은 아니다.
다만 역할을 정확히 내려야 한다.

현재 위치:

1. 후보 A = `xy_hold` false unlock 억제
2. 역할 = 보조 해제 경로 안정화
3. 한계 = **low-rate yaw unlock hole 자체는 해결하지 못함**

즉 후보 A는
`주요 해법`이 아니라 `보조 경로 정리용 조치`로 봐야 한다.

## 새 해결 목표

문서의 핵심 목표를 아래처럼 다시 명시한다.

### 목표

**저속 yaw가 발생했을 때, `xy_hold`가 아니라 `yaw evidence`만으로 더 이르게, 더 정상적으로 unlock되게 만들 것**

이 문장은 단순 문구가 아니라,
이제부터 P1의 성공/실패를 가르는 기준으로 사용한다.

## 새 해결 전략

현재부터의 P1은 `xy_hold`만 조정하는 단계에서 끝나면 안 된다.
다음 단계는 반드시 **저속 yaw 전용 unlock 경로**를 설계하는 것이다.

핵심 원칙은 아래와 같다.

1. `hard_wz`는 유지
   - 강한 회전 즉시 해제는 절대 손상시키지 않는다.

2. `wz_hold`는 유지
   - 보통 회전 응답성은 보존한다.

3. `low-rate yaw evidence` 경로를 새로 둔다
   - 작은 지속 yaw를 `yaw` 근거만으로 해제한다.

4. `xy_hold`는 마지막 fallback으로 격하한다
   - 더 이상 주 unlock 경로가 되어서는 안 된다.

## 권장 아키텍처

이제 해제 우선순위는 아래처럼 재구성하는 것이 목표다.

```text
1. hard_wz         : 강한 회전 즉시 해제
2. wz_hold         : 보통 회전 해제
3. low_yaw_evidence: 저속 지속 회전 해제   <- 새로 필요
4. xy_hold         : 마지막 보조 탈출 경로
```

현재 구조의 문제는 3번이 비어 있다는 점이다.
따라서 4번이 과도하게 중요해진다.

## 새 경로 설계 방향

새 경로는 예전 `slow_turn`의 단순 복구가 아니다.
이전 `slow_turn`은 실차에서 조기 graph 오염을 만들었기 때문에,
같은 설계를 그대로 다시 넣으면 안 된다.

새 경로는 아래 조건을 만족해야 한다.

1. 작은 `wz`라도 무조건 적분하지 않는다.
2. 부호 일관성이 있을 때만 누적한다.
3. 필요하면 `vx` 또는 실제 이동 상태를 함께 본다.
4. `xy` 흔들림이 큰 경우는 오히려 후보에서 제외할 수 있어야 한다.
5. 일정 시간 또는 누적 yaw evidence가 기준을 넘을 때만 unlock한다.

즉:

- 단순 적분이 아니라
- **조건부 yaw evidence 누적**이어야 한다.

## 구체적 후보 설계

### 후보 C. `low_wz_hold`

개념:

1. `|wz_f|`가 아주 작더라도
2. `low_wz_enter < |wz_f| < exit_wz` 범위에서
3. 부호가 일정하게 유지되고
4. 일정 시간 이상 지속되면
5. `moving yaw`로 승격해 unlock

의미:

- `wz_hold`까지는 못 가는 느린 회전도
  `yaw 자체`로 탈출시킨다.

장점:

1. 구현이 비교적 단순하다.
2. 기존 상태기계를 크게 깨지 않는다.
3. `xy_hold` 의존도를 줄일 수 있다.

주의:

1. 너무 짧은 hold를 쓰면 과거 `slow_turn`과 비슷한 조기 해제 회귀가 날 수 있다.
2. 따라서 sign coherence와 최소 지속시간이 반드시 필요하다.

### 후보 D. `low_yaw_evidence + vx gate`

개념:

1. `|wz_f|`가 작아도 일정 기준 이상이면
2. 그 값이 같은 부호로 누적될 때만 evidence를 쌓는다.
3. 가능하면 `|vx| > vx_min` 같은 선속도 조건을 추가한다.
4. 누적 yaw evidence가 기준을 넘으면 unlock한다.

의미:

- 정지 중 작은 bias/noise와
- 주행 중 실제 heading correction을 더 잘 구분한다.

장점:

1. mixed motion에 가장 직접적으로 맞는다.
2. "직진 중 미세 heading correction" 문제를 구조적으로 다룰 수 있다.

주의:

1. 구현 복잡도가 후보 C보다 높다.
2. 잘못 잡으면 다시 조기 unlock 또는 graph 조기 오염이 생길 수 있다.

## 우선순위 제안

안전성과 효과를 같이 보면 순서는 아래가 맞다.

1. 후보 A 적용 상태로 baseline 유지
2. 그 위에 후보 C(`low_wz_hold`)를 1차로 검토
3. 후보 C가 부족하면 후보 D(`low_yaw_evidence + vx gate`)로 확장

즉:

- `xy_hold` 정리만으로 끝내지 않고
- 그 다음은 반드시 **yaw 기반 저속 unlock 경로**로 넘어가야 한다.

## 현재 기준 검증 포인트

새 low-rate yaw 경로를 넣기 전과 후에 반드시 비교할 항목:

1. `reason=xy_hold` 발생 빈도
2. `reason=wz_hold` 또는 새 `reason=low_wz_hold`/`low_yaw_evidence` 발생 빈도
3. mixed motion 중 `/odometry/filtered.twist.twist.angular.z`
4. `tf2_echo odom base_link`의 yaw 추종성
5. 사용자가 느끼는 map 미세 좌/우 회전 체감

성공 기준:

1. 저속 mixed motion에서 unlock이 `xy_hold`보다 `yaw evidence`로 먼저 발생
2. map 미세 yaw shift가 줄어듦
3. 회전 시작 직후 대형 map 회귀는 재발하지 않음

실패 기준:

1. 새 경로가 너무 공격적으로 동작해 조기 unlock이 생김
2. 또는 여전히 `xy_hold`가 주 탈출 경로로 남음
3. 그 결과 map 미세 회전이 유지되거나 다시 커짐

## 최종 정리

현재 해결계획은 두 층으로 나뉜다.

1. 후보 A:
   - `xy_hold` false unlock 억제
   - 보조 경로 안정화

2. 다음 필수 단계:
   - **저속 지속 yaw를 `yaw evidence`만으로 unlock시키는 전용 경로 추가**

따라서 지금 문서 기준의 핵심 결론은 다음 한 줄이다.

**후보 A는 필요하지만 충분하지 않다.  
현재 더 근본적인 해결은 `low-rate yaw unlock hole`을 메우는 yaw-evidence 경로를 설계하는 것이다.**

## v12 추가 업데이트 (2026-03-11, yaw evidence primary 적용)

현재 코드/계획은 후보 A 단독에서 한 단계 더 진행했다.
즉, 이제 목표는 단순히 `xy_hold`를 늦추는 것이 아니라,
**unlock의 주근거를 `xy`에서 `yaw evidence`로 다시 돌려놓는 것**이다.

### 적용 원칙

상태는 늘리지 않는다.

유지되는 상태:

1. `LOCKED`
2. `UNLOCKED`

바뀌는 것은 `LOCKED -> UNLOCKED` 판단식이다.

기존 개념:

1. `hard_wz`
2. `wz_hold`
3. `xy_hold`

새 개념:

1. `hard_wz`
2. `yaw_evidence`
3. `xy_hold` fallback

즉:

- 강한 회전은 여전히 즉시 해제
- 보통/저속 회전은 `yaw evidence score`로 해제
- `xy_hold`는 마지막 보조 경로로 격하

### 왜 이렇게 바꾸는가

현재 residual issue의 본질은
저속 지속 회전이 `wz` 경로로 moving 판정을 받지 못한다는 점이다.

그래서 해결식은 다음이어야 한다.

```text
강한 yaw          -> hard_wz
보통/저속 실제 yaw -> yaw evidence
남는 예외          -> xy_hold fallback
```

이렇게 해야:

1. 직진 중 미세 heading correction
2. 저속 제자리회전
3. mixed motion 중 작은 실제 yaw

를 별도 상태 분기 없이 한 구조에서 다룰 수 있다.

### yaw evidence score 구성

현재 설계는 아래 세 항목으로 점수를 만든다.

1. `yaw magnitude evidence`
   - 지금 순간의 `|wz_f|`가 얼마나 회전다운가

2. `yaw persistence evidence`
   - 같은 방향의 yaw가 얼마나 계속 유지됐는가

3. `xy shake penalty`
   - roll/pitch 흔들림이 크면 yaw evidence 신뢰도를 감산

즉 식의 형태는 다음과 같다.

```text
score = yaw magnitude + yaw persistence - xy shake penalty
```

중요:

1. `xy`는 더 이상 주 unlock trigger가 아니다.
2. `xy`는 evidence를 깎는 패널티이거나, 최후 fallback일 뿐이다.
3. unlock의 주근거는 항상 `yaw`여야 한다.

### persistence가 왜 필요한가

저속 yaw는 순간 크기만 보면 약하다.
그래서 `|wz|` 한 샘플만으로는 노이즈와 구분이 안 된다.

따라서:

1. deadband보다 큰 작은 yaw가
2. 같은 부호로
3. 일정 시간 유지되면
4. 그것을 실제 회전으로 본다

이 로직이 필요하다.

이 persistence가 없으면
현재처럼 `0.01 ~ 0.04 rad/s` 구간이 또 hole로 남는다.

### relock 조건도 같이 바뀌어야 한다

unlock만 바꾸고 relock은 그대로 두면,
저속 yaw를 evidence로 어렵게 풀어도
곧바로 다시 `LOCK`으로 들어갈 수 있다.

그래서 현재 계획은 다음 원칙을 따른다.

1. `LOCK` 진입은 기존 정지 조건을 유지
2. 단, `yaw evidence score`가 충분히 낮아질 때만 재진입 허용

즉:

- 저속 yaw가 남아 있으면
- 다시 `steady_hold`로 들어가지 못하게 막는다

이 항목은 매우 중요하다.
이걸 빼면 저속 회전 중 `unlock -> relock -> unlock` 진동이 다시 생길 수 있다.

### 현재 파라미터 의도

현재 구현/계획에서 핵심 파라미터는 아래다.

1. `yaw_lock_evidence_wz_low`
   - 저속 yaw deadband
   - 이 아래는 evidence를 거의 쌓지 않는다

2. `yaw_lock_evidence_wz_high`
   - magnitude evidence 상한
   - 이 값이면 현재 순간 yaw 크기만으로도 충분히 회전답다고 본다

3. `yaw_lock_evidence_yaw_ref_rad`
   - persistence evidence 기준
   - 같은 방향 yaw가 이 정도 누적되면 "실제 회전"으로 본다

4. `yaw_lock_evidence_decay_sec`
   - yaw가 사라졌을 때 evidence를 얼마나 빨리 지울지 결정

5. `yaw_lock_evidence_unlock_threshold`
   - 최종 score가 이 값을 넘으면 unlock 후보

6. `yaw_lock_evidence_unlock_hold_sec`
   - score가 기준 이상으로 짧게 유지돼야 실제 unlock

7. `yaw_lock_evidence_relock_threshold`
   - evidence가 충분히 줄어들기 전에는 다시 lock하지 않도록 하는 기준

### xy_hold의 현재 위치

`xy_hold`는 제거하지 않는다.
하지만 의미를 명확히 낮춘다.

현재 위치:

1. 주 unlock 경로: 아님
2. 정상적인 yaw 판단의 대체재: 아님
3. 최후 fallback: 맞음

즉 문서 기준으로 성공한 구조는
`xy_hold`가 자주 보이는 구조가 아니라,
`yaw_evidence`가 주 unlock reason으로 먼저 관측되는 구조다.

### 새로운 성공 기준

이제부터의 성공 기준은 더 명확하다.

1. mixed motion에서 unlock reason이 `xy_hold`보다 `yaw_evidence`로 먼저/더 자주 나온다
2. 저속 제자리회전과 저속 heading correction에서 map 미세 회전이 줄어든다
3. 회전 시작 직후 대형 map 회귀는 다시 생기지 않는다
4. relock은 완전 정지 후에만 안정적으로 일어난다

### 리스크

이 구조도 리스크는 있다.

1. persistence가 너무 빨리 쌓이면
   - 예전 `slow_turn`과 비슷한 조기 unlock 회귀 가능

2. xy penalty가 너무 약하면
   - 흔들림을 yaw로 오인할 수 있음

3. relock threshold가 너무 높으면
   - 저속 yaw가 남아 있는데도 다시 lock됨

4. relock threshold가 너무 낮으면
   - 정지 후 lock 복귀가 과도하게 늦어질 수 있음

따라서 앞으로의 튜닝은
`xy_hold` threshold 튜닝이 아니라,
**evidence 누적/감쇠와 relock 경계 튜닝**이 중심이 된다.

### 실행 순서 업데이트

이제 실제 우선순위는 아래처럼 바뀐다.

1. 현재 baseline + `yaw evidence primary` 상태로 실차 검증
2. unlock reason 분포 확인 (`hard_wz`, `yaw_evidence`, `xy_hold`)
3. residual map shift가 남으면
   - 먼저 evidence 누적/감쇠 파라미터 조정
4. 그 다음에만 relock threshold 조정
5. `P2/P3`는 계속 동결

즉, 현재 문서 기준 최종 방향은 다음 한 줄이다.

**이제 해결의 중심은 `xy_hold` 미세조정이 아니라,  
`저속 yaw도 yaw evidence만으로 moving 판정을 받게 만드는 구조`를 안정화하는 것이다.**

## v12 추가 업데이트 (2026-03-11, 고속 제자리회전 map jump 분리 진단)

최신 실차 관찰은 기존 문서를 한 단계 더 수정하게 만든다.

사용자 관찰:

1. 전진/후진은 상대적으로 괜찮다
2. 천천히 제자리회전은 될 때도 있고, map이 돌 때도 있다
3. 빠르게 제자리회전하면 map이 회전하고, 때로는 "점프"하듯 전역 지도가 한 번에 뒤바뀐다

이 관찰은 하나의 원인으로만 설명되지 않는다.

### 최신 로그가 말해주는 것

대상 런:

- `/home/atoz/ca_ws/logs/run_20260311_102047`

핵심 로그:

1. `yaw_zeroing=ON reason=steady_hold`
2. `yaw_zeroing=OFF reason=hard_wz raw_wz=-0.10385 wz_f=-0.05835`
3. 상당 시간 뒤 `yaw_zeroing=ON reason=steady_hold`

중요한 사실:

- 이 런에서는 빠른 회전 구간에서 `xy_hold`도, `yaw_evidence`도 보이지 않는다
- 빠른 회전은 이미 `hard_wz`로 즉시 unlock되었다
- 즉, **고속 제자리회전 중 map이 도는 현상은 더 이상 "yaw_lock이 회전을 막아서 생긴 현상"으로만 볼 수 없다**

이 점은 매우 중요하다.

기존 문서의 핵심 원인:

- `low-rate yaw unlock hole`
- `xy_hold` fallback 의존

이것은 여전히 저속 회전/mixed motion의 원인 후보로 유효하다.
하지만 **고속 제자리회전에서의 map 회전/jump는 별개의 잔여 문제**로 분리해서 봐야 한다.

### 새로운 분류: 잔여 문제는 두 계열이다

#### 계열 A: 저속 회전 / mixed motion residual

증상:

1. 미세한 map yaw shift
2. 직진/후진 중 heading correction에서 map이 살짝 도는 느낌
3. `xy_hold` 또는 늦은 unlock reason과 연결될 가능성

주 원인 축:

- `yaw_lock`의 저속 yaw 판정 구조

현재 대응 축:

- `yaw evidence primary`

#### 계열 B: 고속 제자리회전 map rotation / global jump

증상:

1. 빠른 제자리회전 시작 후 map이 함께 회전
2. 어느 순간 map이 "점프"하듯 크게 재정렬
3. footprint는 따라가는데 전역 지도 전체가 뒤집히는 것처럼 보임

주 원인 축:

- `yaw_lock`보다 **RTAB-Map의 전역 보정 계층**

즉:

- `odom->base_link`가 이미 회전을 따라가고 있어도
- `map->odom` 또는 전역 등록 결과가 한 번에 수정되면
- 사용자는 "맵이 돈다", "맵이 점프한다"고 보게 된다

### 계열 B의 현재 해석

현재 가장 타당한 해석은 다음과 같다.

1. 고속 제자리회전에서는 `hard_wz`로 `yaw_lock`이 풀린다
2. 따라서 IMU gate가 회전을 막는 구조는 아니다
3. 그런데 RTAB-Map은 여전히 외부 odom prior 위에서 전역 정합을 수행한다
4. 빠른 pure spin은 translation 정보가 거의 없고, orientation 변화만 크다
5. 이 상황은 scan-to-map 등록이 불안정해지기 쉬운 조건이다
6. 특히 장면 구조가 비대칭이 약하거나, Livox 패턴/누적 map 형상이 회전 구간에서 애매하면
7. 정합 결과가 어느 시점에 갑자기 다른 회전 해를 선택할 수 있다
8. 그러면 `map->odom` 계열 보정이 한 번에 바뀌고, 사용자는 global map jump로 본다

즉, 지금 보이는 "맵이 갑자기 뒤바뀐다"는 증상은
**저속 yaw hole의 연장선이라기보다, 고속 pure spin에서의 전역 등록/최적화 불안정**으로 보는 것이 더 맞다.

### 왜 이 현상이 간헐적인가

이 현상은 단순한 랜덤이 아니다.
하지만 한 번의 파라미터로 항상 재현되는 단순 deterministic bug도 아니다.

간헐적인 이유는 아래 조건이 동시에 맞아야 크게 보이기 때문이다.

1. 회전 속도
2. 회전 중 translation의 유무
3. 회전 당시의 scan 밀도/패턴
4. 주변 지형의 회전 대칭성/비대칭성
5. RTAB-Map이 그 순간 어떤 등록 결과를 수용했는지
6. 그 직전 그래프 상태와 누적 오차

즉 이것은 "운"이라기보다,
**특정 조건에서만 크게 드러나는 조건부 전역 정합 불안정**이다.

### 문서 관점에서의 중요한 수정

지금부터는 모든 map 회전 증상을 `yaw evidence` 문제로 설명하면 안 된다.

정확한 분리는 다음과 같다.

1. 저속 회전에서의 잔여 미세 회전:
   - `yaw evidence` 설계 축

2. 고속 제자리회전에서의 map rotation / jump:
   - RTAB-Map global correction 축

이 둘을 분리하지 않으면 튜닝이 계속 엇갈린다.

예를 들어:

- 고속 spin 문제를 잡겠다고 `yaw evidence`를 더 세게 만들면
  저속 쪽은 좋아질 수도 있지만, 고속 jump는 그대로 남을 수 있다

- 반대로 RTAB-Map 쪽을 급하게 건드리면
  저속 heading correction 문제와 섞여 다시 회귀를 부를 수 있다

### 현재 계획의 수정 방향

#### 1. `P1`의 역할 재정의

`P1`은 이제 전체 문제의 만능 해법이 아니다.

현재 `P1`이 책임지는 범위:

1. 저속 제자리회전
2. mixed motion
3. 직진/후진 중 미세 heading correction

즉, `P1`의 성공 기준은:

- "고속 spin map jump를 없애는 것"이 아니라
- "`xy_hold`가 주 탈출 경로가 되지 않게 하고, 저속 yaw를 정상적으로 unlock시키는 것"이다

#### 2. 고속 spin 문제는 별도 축으로 분리

새 축 이름:

- `Axis B: fast in-place spin global-correction instability`

이 축의 질문은 다음 하나다.

**고속 제자리회전에서 `yaw_lock`이 이미 OFF인데도 map이 돌고 jump한다면,  
실제 문제는 RTAB-Map의 어떤 전역 보정 경로인가?**

### Axis B의 우선 조사 항목

이 축은 소스 수정 전에 계측으로 확정해야 한다.

필수 계측:

1. `tf2_echo odom base_link`
2. `tf2_echo map odom`
3. `/odometry/filtered`의 `angular.z`
4. `yaw_zeroing` reason 로그
5. 가능하면 `/rtabmap/info`에서 `map_to_odom`, `Loop/MapToBase_yaw`
6. 가능하면 회전 시작/중반/후반 구간별 timestamp 정리

판정 기준:

1. 빠른 제자리회전 구간 내내 `yaw_zeroing=OFF`
2. `odom->base_link` yaw는 실제 회전을 따라감
3. 그런데 `map->odom`이 같은 시점에 변하면
4. 원인은 `yaw_lock`이 아니라 전역 보정 계층이다

### Axis B에서 고려할 해결 방향 (아직 미적용)

아직 코드는 수정하지 않는다.
다만 앞으로의 방향 후보는 분명히 정리해 둔다.

#### 후보 B1: high-spin 구간에서 map->odom correction 완화/동결

의미:

- `|vx|`는 작고 `|wz|`는 큰 pure spin에서
- 전역 보정을 바로 반영하지 않게 하는 방법

장점:

- 사용자가 보는 "map이 같이 돈다", "갑자기 jump한다"를 직접 겨냥한다

리스크:

- 진짜 필요한 전역 보정까지 늦출 수 있다

#### 후보 B2: high-spin 구간의 등록 결과 신뢰도 낮추기

의미:

- 고속 pure spin에서 나온 scan-to-map registration을
- 그래프에 바로 강하게 반영하지 않게 하는 방식

장점:

- jump의 직접 원인인 "갑작스런 전역 correction"을 줄일 수 있다

리스크:

- RTAB-Map 파라미터/노드 동작 이해가 더 필요하다

#### 후보 B3: pure spin 전용 graph update gating

의미:

- pure spin 구간에서는 전역 그래프 업데이트 조건을 더 보수적으로 둔다

장점:

- 회전 시작 즉시 global map이 따라도는 증상을 막기 쉽다

리스크:

- 이미 P2/P3에서 보았듯 잘못 건드리면 더 크게 망가질 수 있다

### 현재 문서 기준 최종 해석

지금 문제는 하나가 아니다.

1. `P1`이 해결하려는 문제:
   - 저속 yaw residual
   - mixed motion
   - `xy_hold` 의존

2. 새로 분리된 문제:
   - 빠른 제자리회전에서의 global map rotation / jump
   - 이쪽은 현재 로그상 `hard_wz` 이후에도 남으므로, `yaw_lock`만으로 설명되지 않는다

### 실행 순서 재정의

이제 우선순위는 아래처럼 분리한다.

1. `P1`은 low-rate yaw residual 안정화 축으로 유지
2. fast spin map jump는 `Axis B`로 별도 추적
3. `P1` 파라미터를 fast spin 증상만 보고 다시 크게 흔들지 않는다
4. 먼저 `Axis B`에서 `map->odom` 변화 여부를 확정한다
5. 그다음에만 RTAB-Map 쪽 제어 지점을 결정한다

### 현재 문서 기준 핵심 한 줄

**이제 남은 핵심은  
`저속 yaw unlock hole`과 `고속 pure spin global-correction jump`를 서로 다른 문제로 분리해서 다루는 것이다.**

## v12 추가 업데이트 (2026-03-11, 저속/짧은/고속 제자리회전 분리 테스트 결과 반영)

사용자가 분리해서 수행한 세 가지 제자리회전 테스트는
문제 구조를 한 단계 더 명확하게 만든다.

### 테스트 결과 요약

1. 저속으로 지속적인 제자리회전:
   - `reason=hard_wz`
   - map 회전 없음

2. 아주 짧은 회전:
   - `reason=yaw_evidence`
   - map 회전 없음

3. 고속으로 지속적인 제자리회전:
   - `reason=hard_wz`
   - map yaw 정렬이 무너지고 map이 회전
   - 심할 때는 jump처럼 보이는 전역 재정렬

이 결과는 기존 문서에서 암묵적으로 가정했던 한 가지를 뒤집는다.

기존 가정:

- `reason`이 다르면 결과도 달라질 것이다

새 결론:

- **`reason`만으로는 map 안정성을 설명할 수 없다**

즉:

- `hard_wz`라도 map이 안 돌 수 있고
- 같은 `hard_wz`라도 map이 크게 돌 수 있다
- `yaw_evidence` 자체가 map 회전의 직접 원인도 아니다

### 최신 로그와의 대응

짧은 회전 + `yaw_evidence` 예시:

- `/home/atoz/ca_ws/logs/run_20260311_104911/sensor_sync.log:43`
- `/home/atoz/ca_ws/logs/run_20260311_104911/sensor_sync.log:51`

여기서는:

1. `reason=yaw_evidence`
2. `raw_wz`는 약 `0.048 ~ 0.063 rad/s`
3. `xy_f`는 낮은 편

즉 `yaw_evidence` 경로 자체는 정상 동작한 것으로 볼 수 있다.
사용자 관찰처럼 짧은 회전에서 map이 안 도는 결과와도 모순되지 않는다.

지속 회전 + `hard_wz` 예시:

- `/home/atoz/ca_ws/logs/run_20260311_104911/sensor_sync.log:39`
- `/home/atoz/ca_ws/logs/run_20260311_104911/sensor_sync.log:41`
- `/home/atoz/ca_ws/logs/run_20260311_104911/sensor_sync.log:45`
- `/home/atoz/ca_ws/logs/run_20260311_104911/sensor_sync.log:47`
- `/home/atoz/ca_ws/logs/run_20260311_104911/sensor_sync.log:49`
- `/home/atoz/ca_ws/logs/run_20260311_102047/sensor_sync.log:40`

여기서는:

1. `reason=hard_wz`
2. `raw_wz`는 약 `0.10 rad/s` 부근
3. `yaw_lock`은 이미 OFF

즉 사용자가 본 고속 지속 회전 중 map 회전은
`yaw_lock`이 회전을 눌러서 생긴 현상이 아니라,
**UNLOCK된 상태에서 누적되는 pure spin이 RTAB-Map 전역 보정 쪽을 흔든 현상**으로 보는 것이 더 맞다.

### 이번 테스트가 보여준 가장 중요한 결론

현재 문제를 결정하는 1차 구분자는
`hard_wz`냐 `yaw_evidence`냐가 아니다.

더 중요한 구분자는 다음이다.

1. 회전이 짧은가 / 지속적인가
2. pure spin 상태가 얼마나 오래 유지되는가
3. 그동안 누적된 총 yaw가 얼마나 큰가
4. 그 시점에 RTAB-Map이 어떤 전역 보정을 수용하는가

즉, 현재 global map rotation/jump의 직접 조건은
**unlock reason**보다 **지속적인 pure spin 노출량(시간 + 누적 yaw)** 쪽에 더 가깝다.

### 문서 해석의 추가 수정

이제 `Axis B`는 단순히 "고속 회전 문제"라고만 쓰면 부족하다.

더 정확한 이름:

- `Axis B: sustained pure-spin global-correction instability`

의미:

- 단순히 회전 속도가 빠른 것만이 아니라
- **회전이 계속 유지되면서 큰 누적 yaw가 쌓이는 pure spin 구간에서**
- RTAB-Map 전역 보정이 불안정해지는 문제

즉 속도는 증폭 요인이지만,
지금 기준으로 더 본질적인 변수는
**지속시간과 누적 각도**다.

### 왜 저속 지속 회전은 괜찮을 수 있는가

사용자 관찰상 저속 지속 회전은 `hard_wz`가 걸려도 map이 안 돈다.

이건 다음 두 가능성을 시사한다.

1. 저속 지속 회전은 비슷한 `reason`이어도
   - scan-to-map 정합이 아직 유지 가능한 영역에 머문다

2. 고속 지속 회전은
   - 같은 `UNLOCK` 상태라도
   - 전역 정합이 한 번 다른 해를 선택할 가능성이 더 높아진다

즉,
- `UNLOCK` 자체가 문제는 아니다
- `UNLOCK된 pure spin이 얼마나 공격적으로, 얼마나 오래 누적되었는가`가 문제다

### 왜 짧은 회전은 괜찮은가

짧은 회전은 `yaw_evidence`가 걸려도 map이 안 돈다.

이 결과는 두 가지를 의미한다.

1. `yaw_evidence` 경로 자체가 map 회전을 만드는 원인은 아니다
2. 전역 jump는 아주 짧은 회전이나 작은 누적 yaw만으로는 잘 드러나지 않는다

즉 지금 남은 문제를
`yaw_evidence가 공격적이라서 map이 돈다`
로 해석하면 틀릴 가능성이 높다.

### 따라서 현재 설계의 책임 범위는 이렇게 정리된다

#### `P1`이 책임지는 범위

1. 저속 residual yaw
2. mixed motion
3. `xy_hold` 의존 제거
4. 짧은 회전에서 정상 unlock

#### `Axis B`가 책임지는 범위

1. 지속적 pure spin
2. 큰 누적 yaw
3. map yaw 정렬 붕괴
4. global map rotation / jump

이 둘을 섞어서 튜닝하면 안 된다.

### 앞으로의 분석 질문도 바뀐다

이제 `Axis B`에서 핵심 질문은 다음이다.

1. 고속 지속 회전에서 map이 틀어질 때,
   그 직전까지 `yaw_zeroing`은 이미 OFF였는가?

2. map이 틀어지기 시작하는 시점은
   - 특정 `wz` 값 때문인가?
   - 아니면 일정 시간 이상의 pure spin 이후인가?
   - 또는 누적 yaw가 특정 각도(예: 반 바퀴, 한 바퀴)를 넘은 뒤인가?

3. 문제 시점에 실제로 변하는 것은
   - `odom->base_link`인가?
   - `map->odom`인가?

이 질문에 대한 답이 잡혀야,
고속 spin 문제를 `P1`이 아니라 RTAB-Map/TF lock 축으로 옮겨서 풀 수 있다.

### 새 실행 기준

이제부터 fast spin 테스트는 아래 기준으로 봐야 한다.

1. `reason=hard_wz`가 떴는지 여부만 보지 않는다
2. `hard_wz` 이후 **얼마나 오래 OFF 상태가 유지됐는지** 본다
3. 그 동안의 **누적 yaw**를 같이 본다
4. map 회전/jump가 발생한 시점과
   - OFF 진입 시점
   - 누적 yaw
   - `map->odom` 변화
   를 연결해서 본다

### 실질적인 계획 수정

현재 문서 기준으로 더 이상 올바르지 않은 가정:

- "`hard_wz`가 뜨면 fast spin 문제도 사실상 해결된다"

현재 문서 기준으로 맞는 가정:

- `hard_wz`는 단지 unlock을 보장할 뿐이다
- fast spin map jump는 그 이후에 벌어지는 전역 보정 문제다

즉 다음 단계는:

1. `P1`을 fast spin 증상만 보고 다시 흔들지 않는다
2. `Axis B`의 핵심 변수를 "회전 속도" 단독이 아니라
   **지속시간 + 누적 yaw + map->odom 변동**으로 재정의한다
3. 이후 RTAB-Map 제어는 이 축에서만 판단한다

### 현재 문서 기준의 새 핵심 한 줄

**세 테스트를 분리해서 보면,  
현재 global map rotation/jump의 핵심 변수는 unlock reason이 아니라  
`지속적인 pure spin 동안 누적된 yaw와 그 구간에서 개입하는 전역 보정`이다.**

## v12 추가 업데이트 (2026-03-11, Axis B 해결전략 구체화)

현재 가장 가능성이 높은 fast spin 문제의 가설은 아래 3단 구조다.

1. 주원인:
   - `RTAB-Map의 pure spin 구간 전역 등록/최적화 불안정`

2. 증폭기:
   - `sync / queue / TF wait`에 의한 입력 지연

3. 사용자에게 보이는 최종 형태:
   - `map->odom step correction`

이 3개는 각각 따로 놀지 않는다.
실제 시스템에서는 하나의 체인으로 연결된다.

### 1. 전체 원인 체인

고속 제자리회전이 시작되면:

1. `odom->base_link`는 EKF가 먼저 회전을 따라간다
2. `yaw_zeroing`은 `hard_wz`로 이미 OFF가 된다
3. 즉 로봇 local pose는 회전 중으로 정상 인식된다
4. 하지만 RTAB-Map은 그 시점의 RGB-D/scan 입력을 바로 "전역 보정"으로 쓰지 않고,
   내부 sync, TF, queue, detection cycle을 거쳐 등록을 시도한다
5. pure spin 구간은 translation 정보가 거의 없고 yaw만 크게 바뀌므로,
   scan-to-map 등록이 애매해지거나, 같은 장소의 다른 회전 해 중 하나를 선택하기 쉬워진다
6. 그 결과가 즉시 조금씩 반영되지 않고,
   어느 시점에 "이 등록 결과를 수용하겠다"는 내부 trigger가 충족되면
7. `map->odom`이 이벤트성으로 바뀐다
8. 사용자는 이것을 "약 1초 후 갑자기 맵이 회전한다", "맵이 점프한다"고 본다

즉 현재 현상은 다음처럼 읽는 것이 가장 일관된다.

```text
pure spin
-> 전역 등록이 애매해짐
-> 입력/TF 지연으로 반영 시점이 늦어짐
-> map->odom이 step처럼 갱신됨
-> 사용자에게는 "맵이 갑자기 돈다"로 보임
```

### 2. 왜 이 가설이 현재 증상과 가장 잘 맞는가

사용자 관찰:

1. 전진/후진은 상대적으로 괜찮다
2. 저속 제자리회전은 될 때도 있고 안 될 때도 있다
3. 고속 제자리회전은 약간의 지연 뒤 map이 돌거나 jump한다

이 관찰은 아래와 잘 맞는다.

1. 전진/후진은 translation 정보가 있어 scan-to-map 정합이 더 안정적이다
2. 저속 회전은 pure spin 노출량이 적어 전역 정합 불안정이 덜 터질 수 있다
3. 고속 + 지속 pure spin은 orientation 변화는 큰데 translation 증거는 부족하다
4. 이런 조건에서 잘못된 등록 결과가 한 번 수용되면,
   `map->odom` step correction으로 크게 보인다

즉 문제의 핵심은 단순히 "빨리 도느냐"보다

1. pure spin인가
2. 얼마나 오래 지속됐는가
3. 그동안 누적 yaw가 얼마나 쌓였는가
4. 그 시점에 RTAB-Map이 어떤 등록을 받아들였는가

이다.

### 3. 이 가설이 맞다면 잘못된 해결 방식

현재 문제를 잘못 다루는 대표적 방식은 아래다.

1. `P1`만 더 세게 튜닝해서 fast spin 문제까지 잡으려는 것
   - `yaw_lock`은 이미 `hard_wz`로 풀리고 있을 수 있다
   - 즉 unlock 자체는 해결돼 있는데, 뒤에서 map이 다시 틀어지는 것이다

2. `hard_wz` threshold만 더 올리거나 내리는 것
   - 회전 감지 시점은 바뀔 수 있어도
   - pure spin 동안 발생하는 전역 등록 불안정은 그대로 남을 수 있다

3. `yaw_evidence`를 더 공격적으로 키우는 것
   - 짧은 회전/저속 회전에는 영향을 줄 수 있어도
   - fast spin jump를 직접 줄인다고 보장되지 않는다

즉 Axis B 문제는
`yaw unlock 로직`이 아니라
`UNLOCK 이후 전역 보정을 언제/어떻게 반영하느냐`를 다뤄야 한다.

### 4. Axis B 해결의 기본 원칙

해결 원칙은 다음 세 줄로 요약된다.

1. pure spin 자체를 막지 않는다
2. pure spin 중의 전역 보정 반영을 더 보수적으로 만든다
3. pure spin이 끝난 뒤 안정된 입력이 들어왔을 때 전역 보정을 다시 허용한다

즉 fast spin 문제의 핵심은
"회전을 못 하게 하는 것"이 아니라
**"회전 중 전역 correction이 성급하게 들어오지 않게 하는 것"**이다.

### 5. 구체적 해결전략

해결전략은 크게 4단계로 나뉜다.

#### B0. 계측으로 `step correction` 확정

코드 수정 전에 먼저 확정해야 하는 것:

1. fast spin 동안 `yaw_zeroing=OFF`가 유지되는지
2. 같은 시점에 `odom->base_link` yaw는 실제 회전을 따라가는지
3. map이 갑자기 돌 때 `map->odom`이 step처럼 변하는지

이 세 가지가 동시에 확인되면,
문제는 `yaw_lock`이 아니라 `map->odom` 반영 계층으로 확정된다.

즉 B0의 목표는
**"사용자가 본 map jump가 실제로 map->odom correction jump인지"를 시간축으로 증명하는 것**이다.

#### B1. 지연 증폭기 축소

현재 설정상 지연을 키울 수 있는 요소:

1. `Rtabmap/DetectionRate = 5.0`
2. `topic_queue_size = 10`
3. `sync_queue_size = 10`
4. `approx_sync_max_interval = 0.05`
5. `wait_for_transform = 2.0`

이 값들은 각각 단독으로 map을 돌리지는 않는다.
하지만 "입력이 처리되는 시점"을 늦출 수 있다.

즉 B1의 목적은:

**잘못되거나 애매한 등록 결과가 늦게 한 번에 반영되는 현상을 줄이는 것**

이다.

중요:

- 이것은 "근본 원인 제거"가 아니라
- "증폭기 감쇠"다

따라서 B1만으로 완전 해결은 기대하지 않는다.
하지만 사용자가 느끼는
"약 1초 뒤 갑자기 돈다"는 step 체감을 줄일 수 있다.

#### B2. pure spin 구간의 전역 보정 gating

이것이 Axis B의 본체다.

아이디어:

1. `|vx|`는 매우 작고
2. `|wz|`는 충분히 크며
3. 그 상태가 일정 시간 지속되는 pure spin window에서는
4. RTAB-Map의 전역 보정 결과를 바로 `map->odom`에 반영하지 않거나
5. 반영 강도를 완화한다

의미:

- local odom 회전은 그대로 유지
- global map correction만 잠시 보수적으로 만든다

기대 효과:

- 회전 시작 후 map이 같이 따라도는 느낌 감소
- 누적된 애매한 회전 정합이 한 번에 jump하는 현상 감소

핵심 트레이드오프:

- pure spin 중 필요한 global correction까지 늦출 수 있다
- 따라서 "항상 차단"이 아니라 "조건부 완화"가 맞다

즉 B2의 핵심 설계 철학은:

**fast spin을 막는 것이 아니라, fast spin 동안 global correction authority를 낮추는 것**

이다.

#### B3. pure spin 등록 결과 신뢰도 자체를 낮추기

이 단계는 B2보다 더 안쪽이다.

의미:

- pure spin에서 얻은 scan-to-map 등록 결과를
  "그대로 믿지 않고"
  "애매한 구간의 제약"으로 취급하는 방향

즉:

1. pure spin window에서는 등록 결과를 reject하거나
2. weight를 낮추거나
3. graph에 바로 강하게 넣지 않게 하는 방식

이 단계가 필요한 이유:

- B2가 "반영 시점/반영 강도" 문제라면
- B3는 "애초에 들어오는 등록 결과의 신뢰도" 문제를 다룬다

리스크:

- RTAB-Map 파라미터/동작 이해 없이 건드리면
  P2/P3 때처럼 더 크게 회귀할 수 있다

따라서 B3는
반드시 B0, B1 관찰 뒤에 들어가야 한다.

### 6. 해결의 우선순위

현재 기준의 우선순위는 아래처럼 정리한다.

1. `P1`은 low-rate yaw / mixed motion 전용으로 유지
2. fast spin 문제는 `Axis B` 전용으로 분리
3. Axis B는 먼저 `map->odom step correction` 여부를 확정
4. 그다음 지연 증폭기(B1) 축소
5. 그래도 남으면 pure spin correction gating(B2)
6. 마지막으로 registration trust control(B3)

즉 해결 순서는

```text
확정
-> 지연 줄이기
-> pure spin 중 전역 보정 완화
-> 등록 신뢰도 제어
```

다.

### 7. 왜 이 순서가 맞는가

이 순서가 중요한 이유는
문제를 가장 바깥층부터 안쪽층으로 좁혀가기 때문이다.

1. `map->odom` step correction이 실제인지 먼저 확인
   - 안 그러면 `yaw_lock`을 계속 잘못 의심하게 된다

2. 지연을 먼저 줄이면
   - "왜 늦게 보였는지"를 줄일 수 있다
   - 그리고 이후의 gating 효과도 더 해석하기 쉬워진다

3. 그다음 pure spin 동안 correction 자체를 제어
   - 실제 현상을 직접 겨냥하는 단계

4. 마지막으로 registration trust를 건드림
   - 가장 효과가 클 수도 있지만
   - 가장 회귀 리스크도 크다

### 8. 성공 기준

Axis B가 해결 방향으로 가고 있다고 판단하려면 다음이 보여야 한다.

1. fast spin 중 `yaw_zeroing=OFF`는 유지된다
2. `odom->base_link`는 계속 정상 회전한다
3. map 회전 시작 시점과 `map->odom` step correction 시점이 일치한다
4. 이후 조치 후에는 fast spin 중 map yaw 붕괴가 줄어든다
5. "약 1초 뒤 갑자기 도는" 현상이 줄어든다
6. pure spin 종료 후에는 map이 더 안정적으로 재정렬된다

### 9. 실패 기준

아래 중 하나가 나오면 현재 가설을 수정해야 한다.

1. fast spin 동안 `yaw_zeroing`이 실제로 ON/OFF를 반복한다
2. map이 돌 때 `map->odom`은 거의 안 변하고 `odom->base_link`가 틀어진다
3. 지연을 줄여도 jump 시점이 그대로다
4. pure spin이 아닌 짧은 회전에서도 같은 jump가 반복된다

즉 현재 가설은 강하지만,
반드시 시간축 계측으로 검증되어야 한다.

### 10. 현재 문서 기준의 최종 해결 문장

**Axis B의 해결은  
`pure spin을 막는 것`이 아니라  
`pure spin 동안 애매한 전역 등록 결과가 지연 후 step correction으로 map->odom에 반영되는 구조를 끊는 것`이다.**

## v12 추가 업데이트 (2026-03-11, 현재 실행 순서 확정)

사용자 판단과 현재 측정 결과를 종합하면,
Axis B의 실제 실행 순서는 아래처럼 고정하는 것이 맞다.

### 현재 확정된 실행 순서

1. `RTAB-Map 증폭기 파라미터(B1)`를 먼저 조정한다
2. 그 상태로 fast spin 실차 검증을 다시 한다
3. 그래도 map 회전/jump가 남으면
   - 그때만 `pure spin 전역 등록 불안정(B2/B3)` 축으로 넘어간다

즉 현재 단계는

```text
P1 유지
-> B1 먼저 적용
-> 재검증
-> 잔존 시에만 B2/B3
```

이다.

### 왜 이 순서로 고정하는가

현재까지의 판단은 아래와 같다.

1. `yaw_lock`은 fast spin에서 이미 `hard_wz`로 OFF가 된다
2. 따라서 fast spin 문제를 `P1`로 계속 해결하려는 것은 비효율적이다
3. pure spin 전역 등록 불안정은 실제 주원인 후보이지만,
   바로 건드리면 P2/P3 때처럼 회귀 리스크가 크다
4. 반면 queue/sync/wait/detection 계열은
   사용자가 느끼는 "1초 뒤 갑자기 도는" 현상과 직접 연결되는 증폭기다

즉 현재는
**주원인을 바로 수술하기 전에, 먼저 증폭기를 줄여 현상이 얼마나 감소하는지 보는 것이 가장 안전한 순서**다.

### 현재 문서 기준의 역할 분담

#### 유지

1. `P1`
   - low-rate yaw residual
   - mixed motion
   - `xy_hold` 의존 제거

#### 지금 먼저 적용

2. `B1`
   - RTAB-Map 증폭기 파라미터 조정
   - 목표: delayed step correction 체감 감소

#### 아직 보류

3. `B2`
   - pure spin 구간 global correction gating

4. `B3`
   - pure spin registration trust control

즉 현재 문서 기준으로
`B2/B3`는 "다음 후보"이지, "지금 당장 같이 적용할 항목"이 아니다.

### B1을 먼저 보는 이유를 더 구체적으로 쓰면

현재 증폭기 성격이 강한 파라미터는 아래 계열이다.

1. `topic_queue_size`
2. `sync_queue_size`
3. `approx_sync_max_interval`
4. `wait_for_transform`
5. `Rtabmap/DetectionRate`

이 값들은 공통적으로:

1. 입력을 더 오래 들고 있거나
2. 더 느슨하게 동기화하거나
3. TF를 더 오래 기다리거나
4. correction을 더 늦게 적용하게 만든다

즉 이 값들을 먼저 줄이는 것은
주원인을 바꾸는 것이 아니라
**주원인이 사용자에게 크게 보이는 방식(step-like delayed correction)을 먼저 약화시키는 작업**이다.

### B1 이후의 판정 기준

B1을 먼저 적용한 뒤 다시 fast spin을 봤을 때,
결과는 두 갈래로 나뉜다.

#### 결과 A: 현상이 뚜렷하게 줄어든다

의미:

1. 주원인이 완전히 없어진 것은 아닐 수 있다
2. 하지만 사용자 체감 대부분이 증폭기에서 커졌다는 뜻이다
3. 이 경우 B1 미세조정만으로도 실용 수준까지 갈 가능성이 있다

즉:

- B2/B3는 바로 들어가지 않고
- B1 범위 안에서 더 안전하게 마감할 수 있다

#### 결과 B: 여전히 map 회전/jump가 강하게 남는다

의미:

1. delayed correction만의 문제가 아니다
2. pure spin 전역 등록 불안정 자체가 더 본질적이라는 뜻이다

즉 이 경우에만
다음 축으로 넘어간다.

```text
B1로도 부족
-> B2 검토
-> 그래도 부족하면 B3
```

### B2/B3를 지금 바로 하지 않는 이유

지금 이 축을 바로 건드리면 안 되는 이유는 명확하다.

1. pure spin correction gating(B2)은 효과가 크지만,
   잘못 만들면 또 다른 조건 분기/회귀를 만든다

2. registration trust control(B3)은 더 강한 수단이지만,
   실제로는 SLAM 등록 본체를 건드리는 것이어서 회귀 리스크가 가장 크다

3. 이미 과거 P2/P3에서
   "좋아질 것 같아 여러 축을 같이 건드렸다가 더 심해진" 경험이 있다

즉 현재는
**원인 분리 없는 본체 수정은 금지**가 맞다.

### 현재 문서 기준 운영 원칙

1. `P1`은 fast spin 증상만 보고 다시 흔들지 않는다
2. `B1`을 먼저 적용하고 실차에서 delayed jump가 얼마나 줄어드는지 본다
3. `B1` 이후에도 fast spin jump가 구조적으로 남아 있을 때만 `B2`로 간다
4. `B3`는 마지막 수단으로만 남긴다

### 현재 문서 기준의 최종 실행 문장

**지금은 `주원인 후보`를 바로 고치는 단계가 아니라,  
`RTAB-Map 증폭기 파라미터를 먼저 줄여 delayed global correction을 약화시키고,  
그래도 남는 잔여 현상만 pure spin 전역 등록 불안정 축으로 넘기는 단계`다.**

## v12 추가 업데이트 (2026-03-11, B2 구현 구조 결정)

### 1. B1 이후 판정

실차에서 RTAB-Map 증폭기 파라미터(B1)를 먼저 줄였음에도
fast pure spin에서 `map->odom` yaw jump 체감이 본질적으로 줄지 않았다.

이 결과의 의미는 명확하다.

1. `queue/sync/wait/detection`은 여전히 증폭기이지만,
   fast spin 문제의 1차 원인은 아니다.
2. 이제 문제의 중심은
   **pure spin 중 RTAB-Map이 계산한 전역 correction을 어떻게 바깥으로 반영하느냐**
   로 이동했다.
3. 따라서 다음 단계는 `B2`이며,
   이 단계의 목표는
   **RTAB-Map 본체 등록 결과를 즉시 크게 내보내지 못하게 하는 출력 계층 제어**
   다.

### 2. B2를 넣을 위치

이번 구현 방향에서 B2는 IMU/yaw-lock 계층이 아니라
`map->odom` 출력 계층에 넣는다.

구체적으로는:

1. RTAB-Map 원본 `publish_tf_map`은 비활성화한다.
2. 대신 `/rtabmap/mapGraph`의 `map_to_odom`를 원본 correction 입력으로 사용한다.
3. 외부 stabilizer 노드가
   `/odometry/filtered`의 `vx`, `wz`와
   `/rtabmap/mapGraph.map_to_odom`를 함께 받아
   **filtered `map->odom` TF를 최종 publish**한다.

이 구조를 선택한 이유:

1. RTAB-Map 본체를 직접 수정하지 않아도 된다.
2. TF 중복 발행을 피할 수 있다.
3. `map->odom` 반영 강도만 따로 제어할 수 있어
   `P1`과 `Axis B`를 계속 분리할 수 있다.

### 3. B2의 핵심 철학

B2는
`pure spin이면 correction을 완전히 막는다`
가 아니다.

그렇게 하면:

1. 회전 중에는 조용해 보여도
2. 회전 종료 후 미반영 correction debt가 한 번에 살아나고
3. 결국 더 큰 jump가 발생할 수 있다.

따라서 구현 철학은 아래처럼 고정한다.

1. hard freeze 금지
2. correction debt 저장 금지
3. 매 주기 current raw correction만 일부 반영
4. pure spin일수록 correction gain을 연속적으로 낮춤
5. gain recovery는 빠른 복귀가 아니라 완만한 복귀로 설계
6. 최종 yaw correction 자체에는 rate limit를 둔다

즉 B2는
`freeze -> accumulate -> repay`
구조가 아니라,

`detect fast -> attenuate softly -> apply continuously -> cap lag -> recover slowly`

구조로 구현해야 한다.

### 4. pure_spin_score의 입력

B2는 조건문 분기를 여러 개 늘리는 방식으로 구현하지 않는다.
대신 pure spin 정도를 나타내는 연속 score 하나를 사용한다.

입력은 최소한 아래 네 개로 고정한다.

1. `|wz|`
2. translation speed (`sqrt(vx^2 + vy^2)` 또는 `|vx|`)
3. pure spin 지속시간
4. 최근 누적 yaw

각 입력의 역할:

1. `|wz|`
   - 회전 시작 직후를 빠르게 감지하는 fast term
2. translation speed
   - pure spin과 일반 곡선/주행 회전을 구분
3. pure spin 지속시간
   - 짧은 twitch와 지속 pure spin 구분
4. 최근 누적 yaw
   - 회전량이 충분히 쌓였는지 반영

여기서 중요한 원칙:

1. `|wz|`, `|vx|`는 score의 빠른 반응 성분
2. 지속시간, 누적 yaw는 score의 느린 확신 성분

즉 duration/누적 yaw만으로 pure spin을 열면 늦고,
`|wz|`만으로 pure spin을 열면 과민해진다.

### 5. gain 설계 원칙

`pure_spin_score`가 커질수록
`global_correction_gain`은 낮아진다.

하지만 gain은 이분법(`0 or 1`)이 아니라 연속값이어야 한다.

예시 해석:

1. score 낮음 -> gain `1.0`
2. score 중간 -> gain `0.5 ~ 0.7`
3. score 높음 -> gain `0.2 ~ 0.3`

중요한 점:

1. gain은 빠르게 낮아져야 한다
   - fast spin 초반 correction 개입 억제
2. gain은 천천히 회복되어야 한다
   - spin 종료 직후 jump 방지
3. gain 최소값은 0보다 커야 한다
   - correction debt 누적 방지

즉 B2에서 gain은
`빠르게 줄고, 느리게 회복하며, 완전히 닫히지 않는`
형태여야 한다.

### 6. apply 단계 원칙

filtered `map->odom` publish는
매 주기 아래 원칙으로 진행한다.

1. RTAB-Map raw correction을 읽는다.
2. raw와 현재 filtered 출력의 차이를 계산한다.
3. pure spin score에 따라 gain을 곱한다.
4. 최종 yaw/x/y step에는 rate limit를 건다.
5. 이번 주기에 허용된 양만 반영한다.

여기서 가장 중요한 원칙은:

**이번 프레임에 덜 반영한 correction을 debt로 저장하지 않는다**

는 점이다.

즉:

1. `raw - applied` 차이를 따로 backlog queue로 관리하지 않는다.
2. 다음 주기에는 다시 fresh raw correction만 읽고 판단한다.
3. 대신 raw/applied 차이 자체에는 별도 lag cap을 둔다.

이 lag cap의 목적:

1. pure spin 동안 raw correction이 너무 멀리 달아나도
2. 종료 후 그 전체 차이를 한 번에 갚지 않게 한다.

즉 B2의 핵심은
`correction을 막는 것`이 아니라
`과도한 raw correction 차이를 debt로 키우지 않는 것`
이다.

### 7. 이번 구현에서 선택한 구조

이번 코드 반영에서는 아래 구조를 기본형으로 채택한다.

1. RTAB-Map raw source:
   - `/rtabmap/mapGraph.map_to_odom`
2. motion source:
   - `/odometry/filtered`
3. stabilizer output:
   - filtered `map->odom` TF
4. pure spin 판단:
   - fast term (`|wz|`, low translation)
   - slow term (spin duration, accumulated yaw)
5. correction control:
   - score -> gain
   - gain down fast / up slow
   - yaw/x/y correction rate limit
   - lag cap

이 방식의 장점:

1. RTAB-Map 본체 무수정
2. TF 중복 발행 제거
3. hard lock보다 부드러움
4. `P1`과 논리적으로 분리 가능

### 8. 이번 단계가 해결하려는 것 / 아직 해결하지 못하는 것

이번 B2 구현이 직접 해결하려는 것:

1. fast pure spin 중 map이 갑자기 따라도는 현상
2. 약 1초 뒤 step처럼 map이 회전하는 체감
3. spin 종료 직후 correction jump

이번 단계만으로 아직 보장하지 못하는 것:

1. pure spin 등록 결과 자체가 완전히 안정되는 것
2. 환경 geometry가 회전에 매우 애매할 때 raw correction 자체가 틀리는 것
3. low-rate yaw / mixed motion residual 문제

즉 이번 단계는
`등록 본체를 고치는 단계`가 아니라
`등록 결과의 외부 반영을 안전하게 제어하는 단계`
다.

### 9. 이번 단계의 성공 기준

성공으로 볼 기준은 아래처럼 둔다.

1. fast pure spin 시작 직후 map이 즉시 같이 돌지 않는다
2. fast pure spin 중 map yaw가 따라 돌더라도 jump가 아니라 완만하다
3. spin 종료 후 map이 뒤늦게 크게 튀지 않는다
4. filtered `map->odom` yaw step가 raw보다 눈에 띄게 완화된다
5. `odom->base_link` 회전 응답은 유지된다

반대로 아래가 보이면 실패다.

1. 회전 중은 괜찮지만 spin 종료 후 더 큰 jump가 생긴다
2. map이 덜 움직이는 대신 종료 후 correction이 몰린다
3. 일반 곡선 주행까지 pure spin처럼 억제된다
4. local rotation 응답이 visibly 둔해진다

### 10. 실행 우선순위 재고정

이제 Axis B 실행 순서는 아래처럼 고정한다.

1. `P1` 유지
2. `B1`은 이미 선행 실험 완료
3. 현재 단계는 `B2`
4. `B2` 이후에도 fast pure spin jump가 구조적으로 남을 때만 `B3`

즉 현재 문서 기준에서 다음 코드 변경의 목적은:

**RTAB-Map raw `map_to_odom`을 그대로 TF로 내보내지 않고,  
pure spin 동안만 correction authority를 연속적으로 줄이는 외부 stabilizer 계층을 넣는 것**이다.

### 11. 이번에 실제로 추가된 소스 코드와 구조

이번 B2 단계에서는 `RTAB-Map 본체 내부 알고리즘`을 직접 바꾸지 않고,
`map->odom` 출력 계층을 외부에서 감싸는 구조로 구현했다.

추가/수정된 핵심 파일은 아래 3개다.

1. `src/rtabmap_ros/rtabmap_launch/scripts/map_tf_stabilizer.py`
2. `src/rtabmap_ros/rtabmap_launch/launch/rtabmap_nav2.launch.py`
3. `src/rtabmap_ros/rtabmap_launch/package.xml`

핵심 철학은:

1. RTAB-Map은 raw global correction을 계속 계산한다.
2. 하지만 raw `map->odom`을 더 이상 RTAB-Map이 직접 TF로 publish 하지 않는다.
3. 대신 외부 `map_tf_stabilizer.py`가 raw correction을 읽는다.
4. pure spin이면 correction authority를 낮춘 filtered `map->odom`만 최종 TF로 publish 한다.

즉 구조를 한 줄로 쓰면:

`RTAB-Map raw map_to_odom -> stabilizer filtering -> 최종 map->odom TF`

### 12. launch 구조가 어떻게 바뀌었는가

`rtabmap_nav2.launch.py`에서 바뀐 핵심은 두 가지다.

1. RTAB-Map raw TF publish 비활성화
2. `map_tf_stabilizer.py` 노드 추가

이 변경의 의미는 명확하다.

기존:

1. RTAB-Map이 직접 `map->odom` TF 발행

현재:

1. RTAB-Map은 `/rtabmap/mapGraph.map_to_odom`만 계산
2. `map_tf_stabilizer.py`가 그 값을 받아 최종 `map->odom` TF 발행

이 구조가 필요한 이유:

1. RTAB-Map raw TF와 stabilizer TF가 동시에 `map->odom`을 발행하면 TF 충돌이 난다.
2. B2는 `map->odom`을 직접 제어하는 단계이므로 publisher는 반드시 하나여야 한다.

즉 이번 구조의 필수 전제는:

**`map->odom` publisher는 RTAB-Map이 아니라 stabilizer 하나만 존재해야 한다**

### 13. `map_tf_stabilizer.py` 내부 구조

이 스크립트는 크게 6개 블록으로 나뉜다.

1. 입력 구독
2. raw correction 저장
3. pure spin 상태 추정
4. score -> gain 계산
5. filtered correction 적용
6. TF publish / 로그 출력

#### 13.1 입력 구독

입력은 두 개뿐이다.

1. `/rtabmap/mapGraph`
2. `/odometry/filtered`

역할은 분명하다.

1. `/rtabmap/mapGraph`
   - RTAB-Map이 계산한 raw `map_to_odom` source
2. `/odometry/filtered`
   - pure spin 판단용 `speed`, `wz`

즉 stabilizer는:

- global correction 자체는 RTAB-Map에서 받아오고
- motion state는 EKF odom에서 받아온다

#### 13.2 raw correction 저장

`_map_graph_cb()`에서는 `msg.map_to_odom`를 읽어:

1. `raw_x`
2. `raw_y`
3. `raw_yaw`

로 저장한다.

그리고 **첫 raw correction을 받았을 때만** 다음을 수행한다.

1. `out_x = raw_x`
2. `out_y = raw_y`
3. `out_yaw = raw_yaw`
4. `have_raw = True`

이 초기화의 목적은:

1. 시작 시 artificial catch-up 방지
2. 처음부터 `raw`와 `filtered` 차이를 크게 만들지 않기 위함

즉 첫 raw correction을 받기 전까지는 stabilizer가 TF를 발행하지 않는다.

이 점은 startup에서 매우 중요하다.

#### 13.3 pure spin 상태 추정

`_update_spin_state()`의 입력은 아래 4개 축이다.

1. 현재 `|wz|`
2. 현재 translation speed
3. pure spin 지속시간
4. pure spin 누적 yaw

여기서 fast term과 slow term을 나눴다.

fast term:

1. `|wz|`가 클수록 증가
2. speed가 작을수록 증가

즉 pure spin 시작 직후 빠르게 반응하기 위한 항이다.

slow term:

1. pure spin 지속시간
2. pure spin 누적 yaw

즉 pure spin이 계속 유지될수록 확신을 높이는 항이다.

최종적으로:

1. `spin_score`는 EMA로 부드럽게 갱신된다.
2. 점수 상승은 빠르게, 하강은 느리게 반응하도록 구성했다.

이 구조를 쓴 이유:

1. duration/누적 yaw만 보면 반응이 늦다.
2. `|wz|`만 보면 과민하다.
3. 둘을 합쳐야 fast pure spin 초반과 sustained pure spin 후반을 동시에 다룰 수 있다.

#### 13.4 score -> gain 계산

여기서 중요한 설계 원칙은 세 가지다.

1. gain은 0으로 닫지 않는다.
2. gain은 pure spin 시작 시 빠르게 감소한다.
3. gain은 pure spin 종료 후 천천히 회복한다.

즉:

1. pure spin 중 correction authority는 약해진다.
2. 하지만 correction을 완전히 막지는 않는다.

이렇게 한 이유는 명확하다.

1. 완전 차단하면 correction debt/backlog가 쌓인다.
2. spin 종료 후 그 debt가 한 번에 살아나면 map jump가 다시 난다.

즉 B2는 hard freeze가 아니라 **soft attenuation**이다.

#### 13.5 filtered correction 적용

`_update_filtered_tf()`는 B2의 본체다.

처리 순서는 아래와 같다.

1. raw-applied 차이 계산
   - `raw_dx`
   - `raw_dy`
   - `raw_dyaw`
2. pure spin score에 따라 lag cap 계산
3. raw-applied 차이를 lag cap으로 제한
4. gain을 곱해 target step 계산
5. yaw/x/y rate limit를 다시 적용
6. `out_x`, `out_y`, `out_yaw` 갱신

이 구현에서 가장 중요한 원칙은:

**덜 반영한 correction을 debt로 저장하지 않는 것**

즉:

1. 이번 프레임에서 덜 반영한 correction을 backlog queue로 관리하지 않는다.
2. 다음 프레임에서도 다시 fresh raw correction만 읽는다.
3. 대신 raw-applied 차이 자체에 lag cap을 둔다.

이 설계가 필요한 이유:

1. pure spin 중 correction을 너무 많이 억제하면
2. 종료 후 그 차이가 한 번에 복귀하며 jump가 날 수 있다.

그래서:

1. 완전 차단 대신 최소 gain 유지
2. lag cap으로 raw-applied 차이 상한 제한
3. rate limit로 최종 TF step 억제

이 세 장치를 같이 넣었다.

#### 13.6 TF publish와 로그

`_publish_filtered_tf()`는 filtered 결과만 `/tf`로 내보낸다.

즉 현재 구조에서 실제 Nav2/rviz가 보는 `map->odom`은:

1. RTAB-Map raw TF가 아니라
2. stabilizer가 계산한 filtered TF

`_log_state()`는 주기적으로 아래를 출력한다.

1. `wz`
2. `speed`
3. `score`
4. `gain`
5. `spin_duration`
6. `spin_yaw_accum`
7. `yaw_err`
8. `pos_err`

이 로그의 목적은:

1. pure spin score가 실제 fast spin에서 올라가는지
2. gain이 제때 줄어드는지
3. raw-applied 차이가 lag cap 안에서 유지되는지

를 실차에서 바로 검증하기 위함이다.

### 14. 이번 구현이 startup에서 주의해야 하는 점

이번 구조는 TF chain 관점에서 startup 주의점이 분명하다.

1. RTAB-Map raw TF publish는 꺼져 있다.
2. stabilizer는 첫 `/rtabmap/mapGraph`를 받기 전까지 TF를 publish 하지 않는다.
3. 따라서 startup 초기에 잠깐 `map->odom` 공백이 생길 수 있다.

즉 `/rtabmap/mapGraph` 값이 보인다고 해서 자동으로 TF가 생기는 것이 아니다.

왜냐하면:

1. `/rtabmap/mapGraph`는 메시지 토픽이다.
2. `map_to_odom`은 그 메시지 안의 field일 뿐이다.
3. TF tree에 `map->odom`이 생기려면 누군가 `/tf`로 브로드캐스트해야 한다.

현재 구조에서는 그 역할이 오직 `map_tf_stabilizer.py`다.

즉:

1. `/rtabmap/mapGraph`는 raw source
2. `map_tf_stabilizer.py`는 TF publisher

이 둘을 혼동하면 안 된다.

### 15. `use_sim_time` 관리 구조

이번에 stabilizer가 한 번 죽었던 원인도 문서에 남긴다.

원인:

1. launch에서 이미 `use_sim_time`가 노드 파라미터로 주입됨
2. stabilizer 코드가 이를 다시 `declare_parameter('use_sim_time', ...)` 하려고 함
3. 그 결과 `ParameterAlreadyDeclaredException`
4. stabilizer 즉시 종료
5. RTAB-Map raw TF는 꺼져 있으므로 `map->odom` publisher가 0개가 됨

즉 `use_sim_time`는 현재:

1. 노드 내부가 아니라
2. `rtabmap_nav2.launch.py`가 관리하는 공통 launch 파라미터

로 보는 것이 맞다.

이번 수정에서 stabilizer는:

1. `use_sim_time`를 직접 declare하지 않고
2. launch가 주입한 값을 그대로 사용하게 바뀌었다.

### 16. 이번 구현이 해결하려는 것 / 일부러 하지 않은 것

이번 B2 코드가 직접 해결하려는 것은 아래 세 가지다.

1. fast pure spin 중 `map->odom` yaw correction이 step처럼 갑자기 들어오는 현상
2. pure spin 중 map이 footprint를 따라 급하게 도는 현상
3. pure spin 종료 직후 correction jump

반대로 이번 구현이 일부러 하지 않은 것은 아래다.

1. RTAB-Map 내부 registration/optimizer 알고리즘 자체 수정
2. `P1` yaw evidence 상태기계 재수정
3. ICP acceptance 규칙 변경
4. RTAB-Map raw correction을 hard freeze

즉 이번 단계는:

**본체 알고리즘을 다시 흔드는 단계가 아니라,  
이미 계산된 raw global correction의 외부 반영을 더 안전하게 만드는 단계**

로 이해해야 한다.

### 17. 이번 구조를 읽을 때 반드시 기억할 점

이번 B2 구조의 핵심은 아래 5문장으로 요약된다.

1. RTAB-Map은 raw `map_to_odom`을 계속 계산한다.
2. RTAB-Map은 raw `map->odom` TF를 직접 publish 하지 않는다.
3. stabilizer가 raw correction과 EKF odom을 함께 읽는다.
4. pure spin일수록 correction gain을 낮춘 filtered `map->odom`만 publish 한다.
5. correction debt를 저장하지 않고, lag cap + rate limit로 jump를 줄인다.

즉 구현 철학은:

**`freeze -> accumulate -> repay`가 아니라  
`detect fast -> attenuate softly -> apply continuously -> cap lag -> recover slowly`**

이다.
