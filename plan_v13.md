# 자율주행 안정화 계획서 (v13 - AngularUpdate/LinearUpdate 심층 분석)

## 1. 배경: 사용자가 보고한 현상

골 포인트를 찍고 로봇이 제자리에서 빠르게 회전하면:

1. map이 로봇과 함께 크게 회전한다
2. 회전이 끝나면 처음과 다른 방향으로 map이 교정된다
3. 때로는 "점프"하듯 전역 지도가 한 번에 뒤바뀐다

이 현상은 `map_tf_stabilizer.py`(B2)를 적용한 이후에도 여전히 발생한다.

## 2. 질문: `RGBD/AngularUpdate`와 `RGBD/LinearUpdate`가 원인인가?

누군가 제안한 값:

```
RGBD/AngularUpdate: 0.05    # 약 2.8도
RGBD/LinearUpdate:  0.1     # 10cm
```

현재 값:

```
RGBD/AngularUpdate: 6.28    # 360도 = 사실상 비활성화
RGBD/LinearUpdate:  0.50    # 50cm
```

### 2-1. 이 값들이 하는 일

RTAB-Map은 **키프레임(노드) 생성 조건**으로 이 두 값을 사용한다.

```
새 노드 생성 조건: (직전 노드 이후 이동 ≥ LinearUpdate) OR (직전 노드 이후 회전 ≥ AngularUpdate)
```

새 노드가 생기면:
1. 직전 노드와의 ICP/visual 등록 제약이 그래프에 추가됨
2. loop closure 검사가 실행됨
3. 그래프 최적화가 실행됨
4. `map→odom` correction이 업데이트됨
5. mapGraph 토픽으로 결과가 발행됨

### 2-2. 현재 값(6.28 / 0.50)에서 제자리 회전 시 일어나는 일

```
[제자리 회전 시작]
  AngularUpdate=6.28 → 360도 돌아야 angular 조건 충족
  LinearUpdate=0.50  → 제자리 회전은 translation 거의 없음

  결과: 회전 중 새 노드가 거의 생기지 않음
  → 그래프 최적화 안 돌아감
  → map→odom correction 업데이트 없음
  → stabilizer도 새 raw 값을 못 받음

[회전 끝나고 직진 시작]
  50cm 이동 후 첫 노드 생성
  → 이 시점에 회전 전 누적된 odom yaw 오차가 드러남
  → 그래프 최적화가 한 번에 큰 correction을 계산
  → map→odom이 step-like하게 변경
  → 사용자: "맵이 갑자기 뒤바뀐다"
```

**현재 값의 문제**: 회전 중 correction이 없어서, 회전 후 첫 노드에서 누적된 오차가 한 번에 터진다.

### 2-3. 제안된 값(0.05 / 0.1)에서 제자리 회전 시 일어나는 일

```
[제자리 회전 시작]
  AngularUpdate=0.05rad = 2.8도
  360도 회전 시: 360 / 2.8 ≈ 128개 노드 생성

  DetectionRate=5Hz → 최대 초당 5개 노드 처리 가능
  빠른 회전(~2초에 360도) → 128개 / 2초 = 64 노드/초 생성 요구

  결과: 처리 능력(5Hz) 대비 13배 과부하
```

**128개 노드 각각에서 일어나는 일:**

1. 직전 노드(2.8도 전)와 ICP 등록
   - 2.8도 차이의 스캔 매칭 → overlap은 높지만
   - Livox MID360의 non-repetitive 패턴 + 짧은 시간 간격
   - → 매칭 품질이 불안정할 수 있음

2. 그래프 최적화 실행
   - 128개 노드 × 최적화 = 지속적인 map→odom 변경
   - 각 최적화마다 작은 correction → 누적 시 "map이 따라 돈다"

3. loop closure 검사
   - 회전 중 같은 위치에서 다른 각도로 찍은 이미지
   - visual BoW 매칭이 false positive를 만들기 쉬운 조건
   - false loop → 그래프에 잘못된 제약 → map 왜곡

**plan_v12에서 이미 실험한 결과 (AngularUpdate=1.57):**

```
P2 실험: AngularUpdate=6.28 → 1.57 (90도)
결과: 회전 초반부터 map이 같이 도는 회귀 발생

원인 분석:
1. 분산보다 먼저 "잘못된 제약의 조기 입력"이 더 크게 작용
2. 회전 시작 직후부터 map이 같이 도는 회귀 발생
→ 결론: AngularUpdate 재인하 금지 판정
```

**1.57에서도 나빠졌는데, 0.05는 그보다 31배 더 공격적이다.**

### 2-4. AngularUpdate=0.05가 만드는 구체적 문제들

#### 문제 1: 처리 큐 폭주 (Queue Flooding)

```
DetectionRate = 5Hz (초당 최대 5개 노드 처리)
회전 속도 1.0 rad/s 기준:
  1초당 생성 노드 = 1.0 / 0.05 = 20개
  처리 가능 = 5개
  대기열 = 15개/초 누적

→ 3초 회전이면 45개가 큐에 쌓임
→ 회전 끝나고 9초 동안 밀린 노드 처리
→ 이 9초 동안 stale 스캔 기반의 최적화가 계속 돌아감
→ map→odom이 회전 끝난 뒤에도 계속 변함
```

#### 문제 2: 제자리 회전 ICP 등록 불안정

```
제자리 회전에서의 ICP 특성:
- translation = 0 (또는 매우 작음)
- rotation만 존재
- PointToPlane ICP는 법선벡터 방향의 displacement를 최소화
- 순수 회전에서는 대응점 매칭이 ambiguous해지기 쉬움

특히 Livox MID360:
- non-repetitive 스캔 패턴
- 같은 각도에서도 다른 점군 분포
- 빠른 회전 중에는 motion distortion이 남아있을 수 있음
  (deskewing은 odom 기반이므로, odom 자체에 오차가 있으면 잔류)

노드가 128개 = 128번의 ICP 등록
→ 그 중 일부는 잘못된 상대 포즈 제약
→ 그래프 최적화가 이 잘못된 제약을 반영
→ map이 틀어짐
```

#### 문제 3: False Loop Closure 위험 증가

```
제자리 회전 중 카메라가 같은 위치에서 360도를 바라봄
같은 물체를 다른 각도에서 촬영 → visual BoW가 유사하게 판단

AngularUpdate=0.05 → 128개 노드 각각에서 loop closure 검사
→ false loop가 하나라도 들어가면 그래프 전체가 왜곡

현재 보호장치:
- Rtabmap/LoopThr = 0.50 (높은 임계)
- Vis/MinInliers = 50
- RGBD/ProximityBySpace = false

하지만 128번 검사하면 한두 번은 통과할 확률이 올라감
```

#### 문제 4: 연산 부하

```
Jetson 플랫폼 기준:
- ICP 등록 1회: ~50-100ms
- 그래프 최적화 1회: ~10-50ms (노드 수에 따라 증가)
- 128개 노드 처리: 최소 ~8초

이 시간 동안:
- RTAB-Map 메인 스레드가 바쁨
- 다른 센서 데이터 처리 지연
- 전체 시스템 반응성 저하
```

### 2-5. LinearUpdate=0.1의 영향

```
현재: 50cm마다 노드 → 1m 직진 시 2개 노드
제안: 10cm마다 노드 → 1m 직진 시 10개 노드 (5배)

직진 주행에서:
- 10cm 간격은 ICP 등록이 잘 되는 조건 (높은 overlap)
- 노드가 많으면 loop closure 기회도 많아짐
- 하지만 DetectionRate=5Hz 제한에 걸림
  vx=0.5 m/s 기준: 초당 5개 노드 생성 vs 처리 가능 5개 → 한계
  vx=1.0 m/s 기준: 초당 10개 노드 생성 vs 처리 가능 5개 → 큐 폭주

직진 자체는 덜 위험하지만, 연산 부하가 불필요하게 높아짐
```

### 2-6. 결론: 0.05 / 0.1 조합은 현재 시스템에서 사용 불가

| 항목 | AngularUpdate=0.05 | 판정 |
|------|-------------------|------|
| 회전 중 노드 수 | 128개/360도 | **과다** |
| DetectionRate 대비 | 13배 초과 | **큐 폭주** |
| ICP 등록 품질 | 제자리 회전에서 불안정 | **위험** |
| False loop 확률 | 128번 검사 | **증가** |
| 연산 부하 | ~8초 처리 시간 | **과부하** |
| plan_v12 실험 결과 | 1.57에서도 악화 | **0.05는 더 심각** |

**이 값을 적용하면 현재보다 확실히 나빠진다.**

## 3. 그러면 AngularUpdate는 어떤 값이 맞는가?

### 3-1. 왜 6.28(현재)도 완벽하지 않은가

```
제자리 회전 → 노드 0개 → correction 0회 → 회전 후 첫 직진에서 한 번에 correction
```

이것이 "회전 끝나고 갑자기 map이 뒤바뀌는" 현상의 구조적 원인이다.

### 3-2. 딜레마

```
AngularUpdate 작게 → 회전 중 노드 많이 → 잘못된 제약 유입 → map 초반부터 돌아감
AngularUpdate 크게 → 회전 중 노드 없음 → correction 지연 → map 나중에 한 번에 점프
```

**이것은 AngularUpdate 값 자체로는 해결할 수 없는 구조적 딜레마다.**

### 3-3. 왜 AngularUpdate로 해결할 수 없는가

근본 원인은 **제자리 회전 중 ICP/visual 등록 자체가 불안정**하다는 것이다.

```
제자리 회전의 특성:
1. translation ≈ 0 → ICP의 position constraint가 약함
2. 순수 rotation → Livox scan overlap이 환경 대칭성에 의존
3. 빠른 회전 → scan간 시간 차이가 크고 motion distortion 잔류 가능
4. 같은 위치에서 다른 각도 → loop closure가 혼동하기 쉬움
```

이 조건에서:
- 노드를 많이 만들면 → 불안정한 등록 결과가 그래프에 들어감
- 노드를 안 만들면 → correction이 지연되어 한 번에 터짐

**따라서 현재 AngularUpdate=6.28(비활성화)이 "가장 덜 나쁜 선택"이다.**

### 3-4. 현재 baseline 유지 판정

```
RGBD/AngularUpdate = 6.28  → 유지
RGBD/LinearUpdate  = 0.50  → 유지
```

이 값을 바꾸는 것은 **근본 해결이 아니라 문제의 형태만 바꾸는 것**이다.

## 4. 진짜 해결 방향: AngularUpdate가 아닌 곳에서 풀어야 한다

### 4-1. 문제의 본질 재정의

```
사용자가 보는 현상:
  "빠른 제자리 회전 → map이 크게 회전 → 다른 방향으로 교정"

이 현상의 원인 체인:

[1] 빠른 회전 중 odom→base_link yaw에 오차 누적
    └─ IMU vyaw 양자화/bias + EKF 적분 → 회전 후 수도 yaw 오차

[2] 회전 후 첫 노드 생성 시 RTAB-Map이 불일치 감지
    └─ odom 기반 예상 위치 vs 실제 스캔 위치 차이

[3] 그래프 최적화가 큰 map→odom correction 계산
    └─ 누적된 오차 전체를 한 번에 보정하려 함

[4] map→odom TF가 step-like하게 변경
    └─ stabilizer가 완화하더라도 raw 자체가 크면 한계
```

### 4-2. 각 단계별 해결 가능성

#### 단계 [1]: odom→base_link yaw 오차 줄이기

현재 상태:
- IMU vyaw만 사용 (imu0_config: vyaw=true)
- ICP odom은 비활성화됨 (`icp_odometry: false`)
- 휠 오돔 vyaw는 슬립으로 비활성화

**핵심 약점: 빠른 회전 중 yaw 추정의 유일한 소스가 카메라 IMU뿐이다.**

카메라 IMU (RealSense D455)의 한계:
```
- LSB = 0.00107 rad/s (양자화 노이즈)
- 200Hz 출력
- consumer-grade MEMS → bias instability 존재
- 빠른 회전에서는 scale factor error도 영향
```

가능한 개선:
```
A. ICP odometry 재활성화 (F2F)
   - 회전 중에도 scan 기반 vyaw를 EKF에 공급
   - 단, 제자리 회전에서 ICP 자체도 불안정할 수 있음
   - 이전에 비활성화한 이유를 먼저 확인해야 함

B. 외장 IMU 추가 (산업용 9축)
   - 하드웨어 변경이 필요하지만 가장 확실한 해결
   - 빠른 회전에서의 yaw 정확도가 근본적으로 올라감

C. IMU scale factor 보정
   - 빠른 회전에서 IMU 출력이 실제 각속도보다 작거나 큰 경우 보정
   - calibration 절차 필요
```

#### 단계 [2]: RTAB-Map이 감지하는 불일치 줄이기

이것은 [1]이 해결되면 자동으로 줄어든다.
odom yaw가 정확하면 → 첫 노드의 예상 위치가 실제와 가까움 → correction이 작아짐.

#### 단계 [3]: 큰 correction 자체를 완화하기

현재 `map_tf_stabilizer.py`가 하는 역할:
```
- pure spin 감지 → gain 낮춤 → rate limit
- 하지만 회전이 끝난 후 첫 노드에서 오는 correction은
  spin이 아닌 직진 구간이므로 gain이 이미 올라와 있음
- 따라서 회전 후 첫 correction은 stabilizer를 거의 통과함
```

**개선 방향: stabilizer의 gain 회복을 더 느리게**

현재:
```python
'gain_up_tau_sec': 1.00  # spin 종료 후 1초 tau로 gain 회복
```

spin이 끝나고 50cm 직진하는 데 걸리는 시간 ≈ 1~3초.
이 시간 안에 gain이 상당히 회복되므로, 첫 correction이 크게 반영된다.

```
개선안: gain_up_tau_sec를 2.0~3.0으로 늘림
→ spin 종료 후 첫 correction이 오는 시점에 gain이 아직 낮음
→ 큰 correction이 완화됨
```

#### 단계 [4]: TF 변경의 체감 줄이기

stabilizer의 rate limit이 이 역할을 한다.
현재 설정이 충분한지는 실차 테스트로 확인해야 한다.

### 4-3. 실행 가능한 즉시 조치

코드 변경 없이 파라미터만 바꿔서 테스트할 수 있는 것들:

#### 조치 1: stabilizer gain 회복 지연 강화

```python
# 현재
'gain_up_tau_sec': 1.00

# 변경
'gain_up_tau_sec': 3.00  # spin 종료 후 더 천천히 gain 회복
```

**효과**: 회전 끝나고 첫 직진에서 오는 큰 correction이 더 많이 깎임

**리스크**: 정상 주행 중 correction 반영도 느려질 수 있음.
하지만 `gain_min=0.20`이 있으므로 완전히 막히지는 않음.

#### 조치 2: stabilizer spin decay 지연

```python
# 현재
'spin_decay_sec': 0.80

# 변경
'spin_decay_sec': 2.00  # spin 감지 상태가 더 오래 유지
```

**효과**: spin 종료 후에도 한동안 spin score가 높게 유지됨
→ gain이 낮은 상태가 더 오래 지속
→ 회전 후 첫 correction이 더 많이 완화됨

#### 조치 3: 회전 후 첫 correction의 lag cap 강화

```python
# 현재
'yaw_lag_cap_normal': 0.80  # 일반 상태에서 raw-applied yaw 차이 최대 0.8rad

# 변경
'yaw_lag_cap_normal': 0.40  # 일반 상태에서도 차이를 더 작게 유지
```

**효과**: 회전 후 첫 correction이 크더라도 lag cap이 차이를 제한
→ step jump 크기 자체가 줄어듬

### 4-4. 중기 조치: ICP odometry 재활성화 검토

현재 `icp_odometry: false`인 이유를 확인해야 한다.

만약 이전에 다른 문제 때문에 비활성화한 것이라면,
EKF에 ICP vyaw를 추가 입력으로 공급하면:

```
[현재]
  EKF vyaw 입력 = IMU만
  → 빠른 회전에서 IMU 오차 누적

[ICP 재활성화 시]
  EKF vyaw 입력 = IMU + ICP(icp_odom_cov_scale 경유)
  → 두 소스가 서로 보완
  → 빠른 회전에서도 yaw 추정 안정성 향상
```

단, 제자리 회전에서 ICP 자체가 불안정할 수 있으므로,
`icp_odom_cov_scale.py`의 회전 모드 공분산 조정이 중요하다.

## 5. startup identity TF 적용

별도로 논의한 startup TF 공백 문제 해결:

```python
# map_tf_stabilizer.py의 _publish_filtered_tf에서
# have_raw=False일 때도 identity TF를 발행
# RTAB-Map도 시작 시 mapToOdom=identity이므로 의미 일치
```

이것은 map 회전 문제와 직접 관련은 없지만,
startup 시 Nav2가 TF를 찾지 못하는 공백을 해결한다.

## 6. 최종 정리

### 6-1. AngularUpdate/LinearUpdate에 대한 최종 판정

| 값 | 결과 | 판정 |
|----|------|------|
| 0.05 / 0.1 (제안) | 128노드/360도, 큐 폭주, false loop, 연산 과부하 | **절대 불가** |
| 1.57 (plan_v12 실험) | 회전 초반부터 map이 같이 돔 | **이미 실패 확인** |
| 1.0 (plan_v12 제안) | 1.57과 유사한 문제 예상 | **위험** |
| 6.28 (현재) | 회전 후 지연된 step correction | **가장 덜 나쁨** |

**현재 값 유지. AngularUpdate로는 이 문제를 해결할 수 없다.**

### 6-2. 실제 해결 축

```
우선순위 1: stabilizer 파라미터 강화 (gain_up_tau, spin_decay, lag_cap)
           → 코드 변경 없이 파라미터만 바꿔서 즉시 테스트 가능
           → 회전 후 step correction 체감을 줄이는 데 직접적

우선순위 2: ICP odometry 재활성화 검토
           → odom yaw 정확도 자체를 올리는 근본 방향
           → 하지만 이전 비활성화 이유 확인 필요

우선순위 3: 외장 IMU (하드웨어)
           → 가장 확실하지만 시간/비용 필요
```

### 6-3. 즉시 적용 가능한 변경 목록

1. **startup identity TF** (map_tf_stabilizer.py)
   - have_raw=False일 때 identity TF 발행
   - 위험도: 없음

2. **stabilizer 파라미터 조정** (rtabmap_nav2.launch.py)
   ```
   gain_up_tau_sec:    1.00 → 3.00
   spin_decay_sec:     0.80 → 2.00
   yaw_lag_cap_normal: 0.80 → 0.40
   ```
   - 위험도: 낮음 (정상 주행 correction이 약간 느려질 수 있으나, gain_min=0.20 존재)

3. **AngularUpdate/LinearUpdate**: 변경하지 않음 (6.28 / 0.50 유지)
