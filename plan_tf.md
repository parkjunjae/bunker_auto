# plan_tf.md — 제자리 회전 시 맵 회전 문제 근본 원인 분석 및 해결 계획

## 0. 문서 목적

이 문서는 제자리 회전 시 occupancy grid가 조금씩 회전하여 goal point가 틀어지는 문제의
근본 원인을 정리하고, 해결 방향을 고정한다.

---

## 1. 현재 증상

1. footprint(로봇 아이콘)은 정상적으로 회전한다 → odom→base_link는 정상
2. 회전할 때마다 occupancy grid 내용이 조금씩 회전한다
3. 처음 찍은 goal point와 회전 후의 goal point가 달라진다
4. 누적되면 맵이 눈에 띄게 틀어진다

---

## 2. 10Hz 고속 로그 분석 결과 (run_20260312_163745)

### 2.1 map→odom TF: 완전 안정

```
전체 1709개 샘플 (약 10Hz)
raw_yaw: 모든 샘플에서 0.000 또는 -0.000
stable_map_yaw: 모든 샘플에서 0.000 또는 -0.000
|raw_yaw| > 0.001인 샘플: 0건
```

### 2.2 stabilizer: 정상 동작하지만 할 일이 없음

```
score: spin 중 0.97까지 상승 (pure spin 정상 감지)
gain: spin 중 0.22까지 하락 (authority 정상 감쇠)
→ 하지만 raw_yaw=0.000이므로 감쇠할 대상 자체가 없음
```

### 2.3 IMU 상태기계: 정상 동작

```
yaw_zeroing=OFF reason=hard_wz  (회전 감지 정상)
yaw_zeroing=ON  reason=steady_hold (정지 감지 정상)
```

### 2.4 RTAB-Map: spin 중 키프레임 없음

```
RGBD/AngularUpdate = 6.28 → 거의 360° 회전해야 키프레임
→ 제자리 회전 중 graph optimization 미실행
→ /rtabmap/map 재발행 없음
```

---

## 3. plan_v14 가설의 검증 결과

plan_v14 §12.8.6이 제시한 3가지 후보:

| 후보 | 가설 | 10Hz 실측 결과 |
|------|------|---------------|
| raw map→odom TF spike | 짧은 시간 크게 튀었다가 복귀 | **반박됨**: 1709샘플 전부 0.000 |
| map_stable→map spike | stabilizer 출력이 순간 spike | **반박됨**: 1709샘플 전부 0.000 |
| /rtabmap/map redraw | spin 중 grid 재작성 | **비현실적**: AngularUpdate=6.28 → 키프레임 없음 |

**결론: plan_v14의 핵심 가설 3개 모두 10Hz 데이터로 반박됨.**

---

## 4. stable map topic이 효과 없는 이유

plan_v14 §14~25는 `/rtabmap/map_stable` stable map topic 도입을 제안했다.

이것이 효과 없는 이유:

1. grid 회전은 **spin 중이 아니라 전진 중 키프레임 생성 시** 발생
2. spin 중: 키프레임 없음 → grid 변화 없음 → hold해도 의미 없음
3. 전진 중: 키프레임 생성 → graph optimization → grid 재빌드 → **여기서 회전 발생**
4. stable map은 spin 중 hold → 전진 중 refresh → refresh 시점에 회전된 grid를 받음

**결론: stable map topic은 안 바뀌는 것을 hold하는 구조이므로 이 문제에 효과 없음.**

---

## 5. 근본 원인: EKF heading에 보정 경로 없음

### 5.1 현재 EKF yaw 입력 구조

```
IMU vyaw ──→ EKF ──→ odom→base_link yaw
               ↑
          다른 yaw 입력 없음
          RTAB-Map도 map→odom으로 보정 안 함
          (보정은 graph 전체에 분산 → 현재 노드 변화 극소)
```

| yaw 소스 | 상태 | 비고 |
|----------|------|------|
| 휠 odom vyaw | OFF | 트랙 슬립 → 제자리 회전 시 완전히 틀린 값 |
| ICP odom vyaw | OFF | odom1 미설정 |
| IMU vyaw | ON (유일한 입력) | yaw_zeroing 전환 손실 + scale error |
| map→odom 보정 | 사실상 없음 | graph 분산 보정 → raw_yaw=0.000 |

### 5.2 heading 오차의 두 가지 원인

#### 원인 A: yaw_zeroing 전환 지연 (회전당 고정 손실)

```
t=120.525: Nav2 회전 명령 시작
t=120.806: yaw_zeroing=OFF (hard_wz, raw_wz=0.109)
           ↑ 0.28초 동안 wz=0 출력 → 실제 회전이 EKF에 전달 안 됨
```

yaw_zeroing=ON인 동안 로봇은 이미 물리적으로 회전 시작하지만,
`yaw_lock_hard_exit_wz=0.10` 임계값을 넘기 전까지 wz=0이 출력된다.

손실량 추정:
- 로봇 각속도: 0 → 0.10 rad/s (선형 가속 가정)
- 평균 wz ≈ 0.05 rad/s
- 누락 회전 ≈ 0.05 × 0.28 ≈ **0.014 rad (0.8°/회전)**

#### 원인 B: IMU 자이로 scale factor error (회전량에 비례)

RealSense D455 BMI055 자이로 사양:
- Scale factor tolerance: ±1~3% (typical)

45° 회전 시 scale error 1%: 0.45° 오차
45° 회전 시 scale error 3%: 1.35° 오차

이 오차는 net rotation에 비례해 누적된다.

#### 원인 A+B 합산

회전당 총 heading 손실 추정: **1~2°**

### 5.3 heading 오차가 grid 회전으로 이어지는 메커니즘

```
[1] 로봇 제자리 회전
    → yaw_zeroing 전환 손실 + scale error
    → EKF heading이 실제보다 1~2° 부족

[2] 회전 후 전진
    → LinearUpdate=0.50 → 0.5m마다 키프레임 생성
    → 새 스캔이 살짝 틀어진 heading으로 투영됨

[3] RTAB-Map ICP 등록
    → 새 스캔 vs 기존 맵 비교
    → heading 불일치 감지
    → graph constraint 추가

[4] Graph optimization
    → Optimizer/Robust=true (Huber cost)
    → 전체 노드 포즈를 조금씩 회전 조정
    → 최신 노드 보정량은 극소 → raw_yaw ≈ 0.000
    → 하지만 전체 노드 누적 효과로 grid 내용이 회전

[5] Occupancy grid 재빌드
    → 조정된 노드 포즈 기반으로 grid 재구성
    → grid 내용이 조금 회전
    → goal point의 물리적 대응 위치가 틀어짐
```

### 5.4 왜 raw_yaw=0.000인데 grid는 도는가

Graph optimization은 heading 보정을 **모든 노드에 분산**시킨다.

예시: 노드 20개, 총 heading 오차 2°
- 노드당 조정: 2° / 20 = 0.1° = 0.0017 rad
- 최신 노드 조정(=map→odom 변화): 0.0017 rad → 로그에 0.002로 표시될 수 있지만
  실제로는 Huber cost로 최신 노드 조정을 더 억제 → 0.000으로 보임
- 하지만 grid는 20개 노드 전체로 재빌드
  → 먼 곳일수록 누적 회전 효과가 크게 보임

---

## 6. 해결 방안

### 6.1 방안 1: yaw_zeroing 전환 속도 개선 (즉시 적용 가능)

**파일**: `src/camera_imu_pipeline_cpp/src/camera_imu_bias_corrector.cpp`
또는 launch 파라미터

**변경**: `yaw_lock_hard_exit_wz` 0.10 → 0.05

**효과**:
- 전환 시간: 0.28초 → 약 0.14초 (절반)
- 누락 회전: 0.8° → 약 0.2° (4배 감소)
- 로봇이 0.05 rad/s만 넘으면 즉시 wz 통과

**리스크**:
- IMU 노이즈가 0.05 rad/s를 넘어 yaw_zeroing이 조기 해제될 가능성
- 현재 정지 중 IMU wz 노이즈: ±0.003 rad/s → 0.05 임계값 대비 충분히 작음
- soft exit (evidence 기반)이 이미 존재하므로 false positive은 제한적

**판정**: 리스크 낮음, 효과 높음, 1줄 수정.

### 6.2 방안 2: ICP odom vyaw를 EKF에 추가 (근본 해결)

**파일**: `src/robot_localization/params/ekf.yaml`

**변경**:
```yaml
odom1: /icp_odom_filtered
odom1_queue_size: 10
odom1_differential: false
odom1_relative: false
odom1_config: [
    false, false, false,   # x, y, z
    false, false, false,   # roll, pitch, yaw
    false, false, false,   # vx, vy, vz
    false, false, true,    # vroll, vpitch, vyaw ← ICP vyaw만 활성화
    false, false, false    # ax, ay, az
]
```

**효과**:
- ICP PointToPlane이 스캔 기하학에서 직접 heading 변화 측정
- IMU scale error와 yaw_zeroing 손실을 실시간 보정
- heading 오차 누적 자체를 차단 → grid 회전 근본 해결

**전제조건**:
- `icp_odom_cov_scale.py` 적용 완료 (SCALE_STOP=1000, SCALE_ROT=1, SCALE_FWD=10)
- ICP는 PointToPlane + VoxelSize=0.15 + MaxCorrespondenceDistance=0.3 (이미 적용됨)

**이전 ICP 문제가 재발하지 않는 이유**:
| 이전 | 현재 |
|------|------|
| PointToPoint ICP | PointToPlane ICP (회전 추정 정확도 향상) |
| cov_scale 미적용 | 상태별 공분산 스케일링 적용 완료 |
| 정지 시 ICP 노이즈 → 드리프트 | SCALE_STOP=1000 → EKF가 무시 |
| 회전 시 과도한 신뢰 | SCALE_ROT=1, SCALE_FWD=10으로 상태별 제어 |

**판정**: 근본 해결. 검증 필요.

### 6.3 방안 3: 둘 다 적용 (권장)

1. `yaw_lock_hard_exit_wz` 낮추기 → heading 손실 최소화
2. ICP odom vyaw 추가 → 남은 오차 실시간 보정

이 조합이 가장 안전한 이유:
- 방안 1만으로는 scale error는 해결 안 됨
- 방안 2만으로는 yaw_zeroing 전환 구간의 순간 불일치가 남음
- 둘 다 적용하면 heading 오차 원인 두 가지를 모두 커버

---

## 7. 적용 순서

### 단계 1: yaw_lock_hard_exit_wz 낮추기

1. sensor_sync.launch.py에서 `yaw_lock_hard_exit_wz: 0.05` 설정
2. 빌드 후 테스트
3. 확인 사항:
   - 정지 중 yaw_zeroing=ON 유지되는지 (false positive 없는지)
   - 회전 시작 시 yaw_zeroing=OFF 전환 시간이 줄었는지
   - goal point 틀어짐이 줄었는지

### 단계 2: ICP odom vyaw 활성화

1. ekf.yaml에 odom1 추가
2. icp_odom_cov_scale.py 동작 확인
3. 빌드 후 테스트
4. 확인 사항:
   - 정지 중 odom→base_link yaw 드리프트 없는지 (SCALE_STOP=1000)
   - 회전 중 heading 추적 정확도 향상 확인
   - 전진 중 고스팅/맵 왜곡 없는지
   - 10회 이상 연속 네비게이션 후 grid 회전량 비교

### 단계 3: 검증

1. 10Hz stabilizer 로그로 raw_yaw 추이 확인
2. 10회+ 네비게이션 후 goal point 일관성 확인
3. 기존 대비 grid 회전량 정량 비교

---

## 8. 성공 기준

1. 10회 연속 네비게이션 후 goal point 위치가 체감상 동일
2. occupancy grid가 회전하지 않음 (또는 눈에 띄지 않는 수준)
3. 정지 중 odom→base_link yaw 드리프트 없음
4. 회전 중 footprint 정상 추적 유지
5. 전진 중 고스팅/맵 왜곡 없음

---

## 9. 실패 시 다음 단계

위 방안 적용 후에도 grid 회전이 계속되면:

1. IMU scale factor 실측 (known angle 회전 후 EKF yaw 변화량 비교)
2. RTAB-Map graph optimization 빈도/강도 조정
3. Reg/Strategy, Icp/* 파라미터 재검토
4. AngularUpdate를 낮춰 spin 중 키프레임 생성 → heading 실시간 보정 (부작용 주의)

---

## 10. 현재 유지할 것

1. IMU bias corrector 상태기계 (P1): 정상 동작 확인됨
2. map_tf_stabilizer: 정상 동작 확인됨 (map_stable→map TF 안정화)
3. RTAB-Map raw TF 복원 (publish_tf_map=true)
4. Nav2/RViz global frame = map_stable
5. icp_odom_cov_scale.py (상태별 공분산 스케일링)

---

## 11. 폐기할 것

1. plan_v14의 stable map topic 구상 (§14~25): 효과 없음 확인
2. plan_v14의 TF spike 가설 (§12.8.6): 10Hz 데이터로 반박됨
3. AngularUpdate/LinearUpdate 재튜닝 우선순위: 근본 원인이 아님
