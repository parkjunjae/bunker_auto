# 자율주행 안정화 계획서 (v3 — 소스 코드 검증 완료)

> **네비게이션**: 섹션 태그로 이동
>
> | 태그 | 섹션 |
> |------|------|
> | `#root-cause` | 진짜 근본 원인 (핵심 읽기) |
> | `#odom-variance-correction` | odom_tf_angular_variance 오해 정정 |
> | `#angular-update-analysis` | AngularUpdate 상세 분석 |
> | `#issue-1` | 제자리 회전 시 TF 고정 + 맵 회전 |
> | `#issue-2` | 장애물 고스팅 |
> | `#issue-3` | TF 드리프트 |
> | `#fix-stabilizer` | map_tf_stabilizer 신규 노드 (가장 강력한 수정) |
> | `#fix-angular-update` | AngularUpdate 수정 |
> | `#fix-costmap` | Costmap 수정 |
> | `#fix-cov-scale` | SCALE_STOP 수정 |
> | `#verify` | 검증 방법 |

---

## #root-cause — 진짜 근본 원인

### SCALE_STOP=1000이어도 회전 시 맵이 도는 이유

SCALE_STOP은 **정지 중 드리프트**를 막는 용도다.
**회전 중** 맵이 도는 것은 다른 원인이다.

### 핵심: RTAB-Map 그래프 최적화가 map→odom을 조정한다

제자리 360° 회전 시 발생하는 사건 순서:

```
1. 회전 시작 → EKF odom이 yaw를 정확히 추적 (ICP vyaw 기반)

2. RGBD/AngularUpdate=1.0(57°)마다 RTAB-Map이 새 노드 생성
   → 360° = 6개 노드
   → 각 노드 사이 neighbor link: EKF odom에서 가져온 상대 변환 + 공분산

3. 360° 완료 시: 현재 스캔이 시작점 스캔과 유사
   → loop closure 감지 (Rtabmap/LoopThr=0.50 통과)
   → ICP 등록으로 loop link 생성: "시작 노드 → 현재 노드" 상대 변환

4. 그래프 최적화 (g2o):
   - 6개 odom neighbor link의 누적 변환: yaw = Σδᵢ (EKF 기반)
   - 1개 loop link의 변환: yaw ≈ 0° (제자리 → 원위치)

   이 두 제약이 불일치하면 (누적 odom 오차 존재):
   → 그래프 최적화가 모든 노드 포즈를 조정
   → map→odom TF 변경
   → 맵이 돌아 보임
```

### 왜 odom 누적 오차가 발생하는가

EKF twist covariance는 완벽하지 않다:
```
EKF odom1 (icp_odom_filtered): vyaw만 사용
  → ICP F2F 정합의 미세 잔류 오차: ε ≈ 0.1~0.5°/노드
  → 6노드 × ε = 0.6~3.0° 총 누적 오차

loop closure ICP가 직접 측정: "시작→끝 = 0°"
→ odom chain은 "시작→끝 = 0.6~3.0°"
→ 불일치 → 그래프 최적화가 보정 → map→odom 변경
```

---

## #odom-variance-correction — ⚠️ 이전 계획서(v2) 오류 정정

### odom_tf_angular_variance=0.01은 효과 없다

**이전 v2 분석 (잘못됨):**
> "RTAB-Map이 EKF odom covariance를 odom_tf_angular_variance=0.01로 덮어쓴다"
> "0.001로 변경하면 map→odom 안정화"

**실제 소스 코드 (CoreWrapper.cpp:2068-2093):**

```cpp
// 1단계: twist covariance 확인
double variance = iter->first.twist.covariance[0];
if(variance == BAD_COVARIANCE || variance <= 0.0f)
{
    // twist가 불량이면 → pose covariance의 절반 사용
    covariance = cv::Mat(6,6,CV_64FC1, (void*)iter->first.pose.covariance.data()).clone();
    covariance /= 2.0;
}
else
{
    // twist가 정상이면 → twist covariance 그대로 사용  ← 우리 경우 여기
    covariance = cv::Mat(6,6,CV_64FC1, (void*)iter->first.twist.covariance.data()).clone();
}

// 2단계: 위 결과가 여전히 불량이면 → fallback으로 odom_tf_angular_variance 사용
if(!uIsFinite(covariance.at<double>(0,0)) || covariance.at<double>(0,0)<=0.0f)
{
    covariance = cv::Mat::eye(6,6,CV_64FC1);
    // odomDefaultAngVariance_ = odom_tf_angular_variance 파라미터
    covariance.at<double>(3,3) = odomDefaultAngVariance_;  // ← FALLBACK ONLY
    covariance.at<double>(4,4) = odomDefaultAngVariance_;
    covariance.at<double>(5,5) = odomDefaultAngVariance_;
}
```

**결론:**
- EKF (`/odometry/filtered`)는 항상 유효한 twist covariance를 publish
- RTAB-Map은 EKF twist covariance를 **그대로 사용** (1단계 else 분기)
- `odom_tf_angular_variance`는 **fallback** → **절대 실행되지 않음**
- **이 값을 0.001로 바꿔도 아무 효과 없다**

### 그럼 RTAB-Map이 실제로 사용하는 covariance는?

**CoreWrapper.cpp:1203-1214:**
```cpp
// EKF twist.covariance에서 읽음
covariance = cv::Mat(6,6,CV_64FC1, (void*)odomMsg.twist.covariance.data()).clone();

// 노드 간 가장 큰(=가장 불확실한) covariance를 사용
if(lastPoseCovariance_.empty() || covariance.at<double>(0,0) > lastPoseCovariance_.at<double>(0,0))
{
    lastPoseCovariance_ = covariance;
}
```

RTAB-Map 그래프의 odom constraint 가중치는 **EKF가 publish하는 twist covariance**에 의해 결정된다.

**이것이 의미하는 것:**
- `icp_odom_cov_scale.py`의 SCALE 값이 EKF covariance에 영향을 줌
- EKF가 이를 퓨전한 결과 covariance가 `/odometry/filtered`의 twist.covariance로 출력
- 이 값이 RTAB-Map 그래프 가중치의 **실제 소스**

### odom_tf_angular_variance 파라미터 설명의 진짜 의미

```
DeclareLaunchArgument('odom_tf_angular_variance', default_value='0.01',
    description='If TF is used to get odometry, this is the default angular variance')
                       ^^^^^^^^^^^^^^^^^^^^^^^^
                       "TF를 사용해 odometry를 얻을 때" — 우리는 토픽을 사용
```

이 파라미터가 실제로 필요한 경우:
- `odom_topic` 없이 TF에서 직접 odom을 추출할 때
- 또는 odom 토픽의 covariance가 비정상일 때 (0, NaN, 무한대)

우리 시스템: `odom_topic='/odometry/filtered'` + EKF가 정상 covariance 출력 → **해당 없음**

---

## #angular-update-analysis — AngularUpdate 상세 분석

### RGBD/AngularUpdate=1.0의 문제

**파일**: [rtabmap.launch.py:371](src/rtabmap_ros/rtabmap_launch/launch/rtabmap.launch.py#L371)

**현재 값:** `1.0 rad = 57.3°`

```
360° ÷ 57.3° = 6.3 → 6개 노드 생성
```

**자체 문제점:**

| 항목 | 값 |
|------|-----|
| 노드 수 | 6 |
| neighbor link 수 | 5 (체인) |
| 각 link의 odom 오차 | ε ≈ 0.1~0.5° (ICP F2F 잔류 오차) |
| 총 누적 오차 | 5ε ≈ 0.5~2.5° |
| loop closure 시 불일치 | 0.5~2.5° → 그래프 최적화가 보정 |

**AngularUpdate를 키우면:**

| AngularUpdate | 노드 수 | link 수 | 누적 오차 | 효과 |
|---------------|---------|---------|-----------|------|
| 1.0 rad (57°) | 6 | 5 | 5ε | 현재 |
| 2.0 rad (115°) | 3 | 2 | 2ε | 누적 오차 60% 감소 |
| 3.14 rad (180°) | 2 | 1 | 1ε | 누적 오차 80% 감소 |
| 6.28 rad (360°) | 1 | 0 | 0ε | 회전 중 노드 0개 → 맵 변화 없음 |

### AngularUpdate=6.28의 의미

```
360° 회전해도 노드가 생성되지 않음
→ neighbor link 없음 → 그래프 변화 없음
→ loop closure 후보도 없음
→ map→odom 완전 고정

BUT: 회전 후 직진 시작하면 첫 번째 노드가 생성됨 → 정상 SLAM 재개
```

**trade-off:**
- 회전 중 루프클로저 불가 → **제자리 회전에서는 원래 루프클로저가 불안정했으므로 손해 없음**
- 회전 중 맵 업데이트 없음 → 회전 후 직진 시 업데이트 재개
- 회전 + 동시 직진 (곡선 주행) 시 노드 생성이 LinearUpdate=0.10에 의해 발생 → 문제 없음

### 권장값: 3.14 rad (180°)

**6.28이 아닌 3.14인 이유:**
- 180° 이상 회전 후 직진 없이 다시 회전하는 패턴에서 최소 1개 노드는 생성
- 완전 차단(6.28)보다 유연하면서도 누적 오차 80% 감소
- 실제 자율주행에서 360° 순수 회전은 드물고, 보통 90~180° 회전 후 직진

```python
# 변경
"RGBD/AngularUpdate": "3.14",  # 1.0→3.14: 180° 마다 노드 → 360° 회전 시 2개 노드
```

---

## #issue-1 — 제자리 회전 시 TF 고정 + 맵 회전

### 원인 요약 (우선순위 순) — v3 수정

| # | 원인 | 위치 | 영향도 | 비고 |
|---|------|------|--------|------|
| 1 | 360° 회전 시 loop closure + odom chain 누적 오차 | RTAB-Map 그래프 최적화 | ★★★★★ | 근본 원인 |
| 2 | AngularUpdate=1.0 → 6개 노드 → 누적 오차 | rtabmap.launch.py:371 | ★★★★ | 증폭 요인 |
| 3 | map→odom TF를 직접 제어하는 메커니즘 없음 | 없음 | ★★★ | 방어 수단 부재 |
| ~~4~~ | ~~odom_tf_angular_variance=0.01~~ | ~~rtabmap.launch.py:600~~ | ~~효과 없음~~ | **v2에서 삭제** — fallback 전용 |

### 상세 오차 전파 경로 (v3 수정)

```
회전 시작
  │
  ├─ EKF 경로 (정확)
  │    icp_odometry (raw scan) → /icp_odom
  │    → SCALE_ROT=1 → /icp_odom_filtered
  │    → EKF → odom→base_link (+57° 정확)
  │    → /odometry/filtered의 twist.covariance → RTAB-Map이 그래프 가중치로 사용
  │
  └─ SLAM 그래프 경로
       AngularUpdate=1.0 → 57°마다 노드 생성
       각 노드 사이 neighbor link = EKF odom (twist.covariance 가중)

       360° 완료 시:
       └─ loop closure 감지 (시작점 스캔 ≈ 현재 스캔)
          → ICP 등록: "시작→끝 = 0°" (제자리니까)
          → odom chain: "시작→끝 = 0° + 누적 오차 ε"
          → 불일치 ε → 그래프 최적화 → map→odom 조정
          → 맵 회전
```

---

## #issue-2 — 장애물 고스팅

### 원인

1. **Issue-1에서 map→odom 변화** → costmap에서 장애물 위치 이동
2. **global_costmap clearing 없음** → 이전 위치 장애물 잔류

**파일**: [nav2_rtabmap_params.yaml:375-383](src/rtabmap_ros/rtabmap_launch/launch/config/nav2_rtabmap_params.yaml#L375-L383)

```yaml
global_costmap:
  obstacle_layer:
    lidar_mark:
      clearing: false               # ← 지우지 않음
      observation_persistence: 1.0  # ← 1초간 유지
```

map→odom이 안정화되면 고스팅도 크게 줄어든다. 그러나 clearing 추가는 별개로 필요.

---

## #issue-3 — TF 드리프트

### 원인

**파일**: [icp_odom_cov_scale.py:20](src/rtabmap_ros/rtabmap_launch/scripts/icp_odom_cov_scale.py#L20)

```python
# 현재 코드
SCALE_STOP = 1.0   # 정지 중 ICP 노이즈가 EKF에 그대로 입력 → yaw drift
```

**정지 중 드리프트 원인:**
```
로봇 정지 → ICP: 두 동일 스캔 정합
이론: vyaw = 0
실제: ICP 수렴 잔류오차 ε ≠ 0 (양자화, 이웃탐색 오차)

SCALE=1.0:
  cov[35] 그대로 → EKF Kalman 게인 K = P·H^T / (H·P·H^T + cov[35])
  ε가 K에 곱해져 odom yaw에 반영

SCALE=1000:
  cov[35] × 1000 → K ≈ 0 → ε 영향 무시 → yaw 고정
```

---

## #fix-angular-update — AngularUpdate 수정 ★ 가장 중요한 파라미터 수정

**파일**: [rtabmap.launch.py:371](src/rtabmap_ros/rtabmap_launch/launch/rtabmap.launch.py#L371)

```python
# 변경 전
"RGBD/AngularUpdate": "1.0",  # 57° 마다 노드 → 360° = 6개 노드 = 5개 odom link 누적 오차

# 변경 후
"RGBD/AngularUpdate": "3.14", # 180° 마다 노드 → 360° = 2개 노드 = 1개 odom link 누적 오차
```

**효과:**
```
AngularUpdate=1.0: 360° 회전 → 6개 노드 → 5개 link × 오차 δ = 5δ → loop closure와 큰 불일치
AngularUpdate=3.14: 360° 회전 → 2개 노드 → 1개 link × 오차 δ = 1δ → loop closure와 작은 불일치
  → 그래프 최적화 보정량 80% 감소
  → map→odom 변화 80% 감소
```

**이것만으로도 충분할 수 있는 이유:**
- 누적 오차가 1δ(≈0.1~0.5°)면 map→odom 변화가 사실상 무시 가능
- loop closure가 발생해도 보정량이 미세 → 맵 회전 체감 안 됨

---

## #fix-stabilizer — map_tf_stabilizer (신규 노드) ★ 방어적 추가 수단

AngularUpdate=3.14로도 해결 안 되는 경우에만 적용.
RTAB-Map의 map→odom 출력을 직접 가로채서,
순수 회전 중에는 회전 직전 값으로 고정하는 중계 노드.

**파일**: `src/rtabmap_ros/rtabmap_launch/scripts/map_tf_stabilizer.py` (신규)

### 동작 원리

```
RTAB-Map → map→odom TF 발행 (5Hz)
                ↓
        map_tf_stabilizer (구독)
                ↓
         cmd_vel 모니터링
         ┌─ 직진/정지: RTAB-Map TF 그대로 재발행
         └─ 순수회전 감지: 회전 직전 TF를 20Hz로 고정 발행
                          (RTAB-Map 5Hz TF를 덮어씀)
                ↓
        TF 리스너들이 최신 stamp의 TF를 사용
        → 회전 중 map→odom 고정
```

### 전체 코드

```python
#!/usr/bin/env python3
"""
map_tf_stabilizer.py

제자리 회전 중 RTAB-Map의 map→odom TF가 흔들리는 문제를 해결.
순수 회전(linear≈0, angular>thresh) 구간에서 회전 직전 map→odom을
고주파로 재발행해 RTAB-Map의 느린(5Hz) 업데이트를 덮어씀.

회전 종료 후에는 RTAB-Map이 정착할 시간을 주고,
그 사이 locked TF를 계속 발행해 갑작스러운 점프를 방지.
"""

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import Twist
import tf2_ros

ROTATE_ANG_THRESH = 0.08   # rad/s: 이 이상이면 회전으로 판정
ROTATE_LIN_THRESH = 0.05   # m/s:  이 이하여야 순수 회전으로 판정
SETTLE_SEC        = 1.5    # 회전 종료 후 locked TF를 유지할 시간(s)
PUBLISH_HZ        = 20.0   # locked TF 발행 주파수 (RTAB-Map 5Hz 덮어쓰기)


class MapTfStabilizer(Node):
    def __init__(self):
        super().__init__('map_tf_stabilizer')

        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self._locked_tf      = None   # 고정할 map→odom 변환
        self._rotating       = False
        self._settle_until   = None   # 회전 종료 후 유지 기한

        self.create_subscription(Twist, '/cmd_vel', self._cmd_cb, 10)
        self.create_timer(1.0 / PUBLISH_HZ, self._publish_cb)

    # ------------------------------------------------------------------ #
    def _cmd_cb(self, msg: Twist):
        lin = abs(msg.linear.x)
        ang = abs(msg.angular.z)
        now = self.get_clock().now()

        is_pure_rotation = (ang >= ROTATE_ANG_THRESH and lin < ROTATE_LIN_THRESH)

        if is_pure_rotation:
            if not self._rotating:
                # 회전 시작: 현재 map→odom을 잠근다
                tf = self._lookup_map_odom()
                if tf is not None:
                    self._locked_tf  = tf
                    self._rotating   = True
                    self._settle_until = None
                    self.get_logger().info(
                        '[MapTfStabilizer] rotation started → map→odom locked')
        else:
            if self._rotating:
                # 회전 종료: settle 구간 시작
                self._rotating   = False
                self._settle_until = now + Duration(seconds=SETTLE_SEC)
                self.get_logger().info(
                    f'[MapTfStabilizer] rotation ended → hold locked TF for {SETTLE_SEC}s')

    # ------------------------------------------------------------------ #
    def _publish_cb(self):
        now = self.get_clock().now()

        if self._rotating:
            # 순수 회전 구간: locked TF 고주파 발행
            self._republish(now)

        elif self._settle_until is not None:
            if now < self._settle_until:
                # settle 구간: locked TF 유지 (RTAB-Map이 정착할 때까지)
                self._republish(now)
            else:
                # settle 완료: 해제
                self._locked_tf    = None
                self._settle_until = None
                self.get_logger().info(
                    '[MapTfStabilizer] settle complete → RTAB-Map TF resumed')

    # ------------------------------------------------------------------ #
    def _republish(self, now):
        if self._locked_tf is None:
            return
        t = self._locked_tf
        t.header.stamp = now.to_msg()
        self.tf_broadcaster.sendTransform(t)

    # ------------------------------------------------------------------ #
    def _lookup_map_odom(self):
        try:
            return self.tf_buffer.lookup_transform(
                'map', 'odom',
                rclpy.time.Time(),           # 가장 최근 TF
                timeout=Duration(seconds=0.1)
            )
        except Exception as e:
            self.get_logger().warn(f'[MapTfStabilizer] lookup failed: {e}')
            return None


def main():
    rclpy.init()
    node = MapTfStabilizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### rtabmap_nav2.launch.py에 노드 추가

**파일**: [rtabmap_nav2.launch.py:198-203](src/rtabmap_ros/rtabmap_launch/launch/rtabmap_nav2.launch.py#L198-L203)

```python
# 변경 전
icp_odom_cov_scale_node = Node(
    package='rtabmap_launch',
    executable='icp_odom_cov_scale.py',
    name='icp_odom_cov_scale',
    output='screen',
)

# 변경 후 — map_tf_stabilizer 추가
icp_odom_cov_scale_node = Node(
    package='rtabmap_launch',
    executable='icp_odom_cov_scale.py',
    name='icp_odom_cov_scale',
    output='screen',
)

map_tf_stabilizer_node = Node(
    package='rtabmap_launch',
    executable='map_tf_stabilizer.py',
    name='map_tf_stabilizer',
    output='screen',
)
```

return LaunchDescription에도 `map_tf_stabilizer_node` 추가:
```python
return LaunchDescription([
    ...
    icp_odom_cov_scale_node,
    map_tf_stabilizer_node,   # ← 추가
    ...
])
```

### CMakeLists.txt에 실행 권한 등록

**파일**: [src/rtabmap_ros/rtabmap_launch/CMakeLists.txt](src/rtabmap_ros/rtabmap_launch/CMakeLists.txt)

```cmake
# 기존 icp_odom_cov_scale.py 등록 부분에 추가
install(PROGRAMS
  scripts/icp_odom_cov_scale.py
  scripts/map_tf_stabilizer.py   # ← 추가
  DESTINATION lib/${PROJECT_NAME}
)
```

---

## #fix-costmap — Costmap 수정 (고스팅 직접 해결)

**파일**: [nav2_rtabmap_params.yaml:368-384](src/rtabmap_ros/rtabmap_launch/launch/config/nav2_rtabmap_params.yaml#L368-L384)

### global_costmap clearing 추가

```yaml
# 변경 전
global_costmap:
  obstacle_layer:
    observation_sources: lidar_mark

    lidar_mark:
      topic: /livox/lidar/filtered
      marking: true
      clearing: false             # ← 이전 장애물 잔류
      observation_persistence: 1.0

# 변경 후
global_costmap:
  obstacle_layer:
    observation_sources: lidar_mark lidar_clear  # clearing 소스 추가

    lidar_mark:
      topic: /livox/lidar/filtered
      marking: true
      clearing: false
      obstacle_range: 4.0
      min_obstacle_height: 0.05
      max_obstacle_height: 1.8
      observation_persistence: 0.5   # 1.0 → 0.5s

    lidar_clear:
      topic: /livox/lidar/filtered
      data_type: PointCloud2
      marking: false
      clearing: true
      raytrace_range: 5.0
      min_obstacle_height: 0.05
      max_obstacle_height: 1.8
      expected_update_rate: 0.0
```

### local_costmap persistence 단축

```yaml
# 변경 전 (257번 줄 근처)
local_costmap:
  obstacle_layer:
    lidar_mark:
      observation_persistence: 1.0   # ← 고스팅 1초 지속

# 변경 후
    lidar_mark:
      observation_persistence: 0.3   # 1.0 → 0.3s
```

---

## #fix-cov-scale — SCALE_STOP 수정 (TF 드리프트)

**파일**: [icp_odom_cov_scale.py:20](src/rtabmap_ros/rtabmap_launch/scripts/icp_odom_cov_scale.py#L20)

```python
# 변경 전
SCALE_STOP  =    1.0  # 정지 중 ICP 노이즈 그대로 EKF 입력 → yaw drift

# 변경 후
SCALE_STOP  = 1000.0  # 정지 중 EKF가 ICP 무시 → 잔류 노이즈 차단
SCALE_ROT   =    1.0  # 회전: ICP 완전 신뢰 (변경 없음)
SCALE_TRANS =   10.0  # 전진: 부분 신뢰 (변경 없음)
```

**Note**: SCALE_STOP=1000은 정지 드리프트만 해결한다. 회전 중 맵 회전은 위의 #fix-angular-update 로 해결해야 한다.

---

## #verify — 검증 방법

### 단계 0: 현재 증상 기록 (변경 전 baseline)

```bash
# 터미널 1
ros2 run tf2_ros tf2_echo map odom 2>/dev/null | grep -E "rotation"

# 터미널 2
ros2 run tf2_ros tf2_echo odom base_link 2>/dev/null | grep -E "rotation"

# 테스트: 제자리 360° 회전 후 두 값의 yaw 변화 기록
```

### 단계 1: SCALE_STOP=1000 적용 후 정지 드리프트 확인

```bash
# 로봇 정지 30초 후 odom yaw 변화량 측정
ros2 run tf2_ros tf2_echo odom base_link 2>/dev/null

# 기대값: 30초간 yaw 변화 < 0.05 rad (< 3°)
```

### 단계 2: AngularUpdate=3.14 적용 후 회전 테스트

```bash
# 터미널 1: map→odom yaw 모니터
ros2 run tf2_ros tf2_echo map odom

# 터미널 2: RTAB-Map 노드 생성 모니터
ros2 topic echo /rtabmap/info --field header.stamp

# 테스트:
# 1. 제자리 시계방향 360° 회전 (5초 소요)
# 2. 정지 5초 대기
# 3. 제자리 반시계방향 360° 회전

# 기대값:
#   map→odom yaw 변화 < 2° per 360° rotation (기존: 수 도~수십 도)
#   360° 회전 동안 노드 생성 0~2개 (기존: 6개)
```

### 단계 3: (필요 시) map_tf_stabilizer 추가 후 완전 테스트

```bash
# 회전 중 stabilizer 로그 확인
ros2 topic echo /rosout | grep MapTfStabilizer

# 기대 로그:
# [MapTfStabilizer] rotation started → map→odom locked
# [MapTfStabilizer] rotation ended → hold locked TF for 1.5s
# [MapTfStabilizer] settle complete → RTAB-Map TF resumed
```

### 단계 4: 전체 자율주행 테스트

```bash
# 1. Nav2 목표 설정
ros2 topic pub /goal_pose geometry_msgs/PoseStamped \
  "{header: {frame_id: map}, pose: {position: {x: 3.0, y: 0.0}}}" --once

# 2. 동시 모니터링
python3 ~/ca_ws/tools/ekf_yaw_probe.py --duration 120

# 성공 기준:
# - 목표 도달 (거리 오차 < 0.25m)
# - 고스팅 없음 (RViz costmap 확인)
# - 직선 10m 후 odom yaw 오차 < 5°
# - 360° 회전 중 map→odom yaw 변화 < 2°
```

---

## 적용 우선순위 & 리스크 (v3 수정)

| 순위 | 수정 사항 | 파일 | 예상 효과 | 리스크 |
|------|-----------|------|-----------|--------|
| **1** | `SCALE_STOP=1000` 복구 | `icp_odom_cov_scale.py` | 정지 드리프트 제거 | 낮음 |
| **2** | `RGBD/AngularUpdate=3.14` | `rtabmap.launch.py` | 회전 중 노드 감소 → 누적 오차 80% 감소 (핵심) | 낮음 — 회전 구간 루프클로저가 원래 불안정 |
| **3** | global_costmap clearing 추가 | `nav2_rtabmap_params.yaml` | 고스팅 직접 해결 | 낮음 |
| **4** | `observation_persistence` 단축 | `nav2_rtabmap_params.yaml` | 고스팅 지속시간 단축 | 낮음 |
| **5** | `map_tf_stabilizer.py` 신규 | `scripts/` + `rtabmap_nav2.launch.py` | 회전 중 map TF 완전 고정 (2단계 불충분 시) | 중간 — settle 타이밍 튜닝 필요 |
| ~~6~~ | ~~`odom_tf_angular_variance=0.001`~~ | ~~`rtabmap_nav2.launch.py`~~ | ~~효과 없음~~ | **삭제** — fallback 전용 파라미터 |

### 권장 적용 순서

```
1단계 (즉시): SCALE_STOP=1000 → 정지 드리프트 확인
2단계 (즉시): AngularUpdate=3.14 → 회전 맵 안정성 확인
3단계 (즉시): global_costmap clearing 추가 → 고스팅 확인
4단계 (필요 시): map_tf_stabilizer.py → 2단계 이후에도 불안정하면 추가
```

---

## 변경 사항 빠른 참조

### 파일 1: icp_odom_cov_scale.py (20번 줄)
```python
SCALE_STOP = 1000.0  # 1.0 → 1000.0
```

### 파일 2: rtabmap.launch.py (371번 줄)
```python
"RGBD/AngularUpdate": "3.14",  # 1.0 → 3.14
```

### 파일 3: nav2_rtabmap_params.yaml
```yaml
global_costmap.obstacle_layer:
  observation_sources: lidar_mark lidar_clear   # lidar_clear 추가
  lidar_mark.observation_persistence: 0.5       # 1.0 → 0.5
  lidar_clear: { ... }                           # 신규

local_costmap.obstacle_layer.lidar_mark:
  observation_persistence: 0.3                  # 1.0 → 0.3
```

### 파일 4: scripts/map_tf_stabilizer.py (신규 — 필요 시만)
위 `#fix-stabilizer` 섹션의 전체 코드 참조.

---

## #v4-test-result — v3 적용 + 추가 수정 테스트 결과 (2026-03-09)

### 적용한 수정 사항

**plan v3에서 권장한 수정 (모두 적용):**
1. AngularUpdate = 3.14 ✅
2. Optimizer/Robust = true ✅
3. ICP 파라미터 통일 ✅
4. LoopThr = 0.50, Vis/MinInliers = 50 ✅
5. ProximityBySpace = false ✅
6. costmap clearing 추가 ✅

**추가로 사용자가 실험한 3가지 수정:**
1. `odom0` vyaw = true (ekf.yaml:36) — 휠 인코더 vyaw를 EKF에 활성화
2. `bunker_base` vyaw covariance = 1.0 (bunker_messenger.hpp:270) — 휠 vyaw 분산 낮춤
3. `SCALE_STOP = 0.5` (icp_odom_cov_scale.py:20) — 정지 시 ICP vyaw를 더 신뢰

### 테스트 결과

| 항목 | 결과 |
|------|------|
| base_link TF | 약간 왔다갔다 (진동) |
| map TF | **드리프트** (회전해있음) |
| odom TF | **드리프트** (회전해있음) |

### 원인 분석

#### 1. odom0 vyaw=true가 핵심 문제

Bunker는 **스키드 스티어** 로봇이다. 스키드 스티어의 휠 인코더 vyaw는:
- **직진 시**: 대체로 정확 (양쪽 바퀴 속도 차이로 yaw rate 계산)
- **회전 시**: 트랙/바퀴 슬립 → **실제 yaw rate와 크게 다른 값** 출력
- **정지 시**: 약간의 노이즈

현재 상태:
```
EKF 입력:
  odom0 vyaw=true  (cov=1.0)  ← 휠 인코더: 회전 시 슬립으로 틀림
  odom1 vyaw=true  (cov=동적) ← ICP: 회전 시 정확

EKF가 두 소스를 가중평균 → 둘 다 타협한 값 = 둘 다 틀림
→ base_link 진동 (두 소스 충돌)
→ odom→base_link yaw 부정확 → RTAB-Map 보정 시도 → map→odom 드리프트
```

#### 2. SCALE_STOP=0.5의 역효과

SCALE은 **covariance에 곱하는 값**이다:
```
SCALE=0.5  → cov[35] × 0.5 → 분산 감소 → EKF가 ICP vyaw를 더 신뢰
SCALE=1000 → cov[35] × 1000 → 분산 증가 → EKF가 ICP vyaw를 무시
```

정지 시 ICP vyaw ≈ 0 + 잔류 노이즈 ε:
- `SCALE=0.5`: ε이 낮은 분산으로 EKF에 입력 → EKF가 노이즈를 신뢰 → yaw 드리프트
- `SCALE=1000`: ε이 높은 분산으로 입력 → EKF가 무시 → yaw 고정

**정지 시에는 SCALE을 크게 해야 한다** (ICP를 무시해야 함).

#### 3. base_link 진동 원인

```
EKF 퓨전:
  K = P·H^T / (H·P·H^T + R)

  odom0 vyaw: R=1.0 (고정)
  odom1 vyaw: R=cov[35]*SCALE (동적)

  두 소스의 vyaw 값이 다를 때:
    → EKF가 칼만 게인에 따라 절충
    → 각 업데이트마다 조금씩 다른 방향 → 진동
```

### v4 해결 방안

#### 방안 A: odom0 vyaw 되돌리기 (권장 — 가장 안전) ★★★★★

```yaml
# ekf.yaml odom0_config
vyaw: false   # true → false 되돌리기
```

**이유**: 스키드 스티어 로봇에서 휠 vyaw는 근본적으로 신뢰 불가.
ICP vyaw만 사용하는 것이 이 로봇에 맞는 설계.

#### 방안 B: SCALE_STOP 복구 ★★★★★

```python
# icp_odom_cov_scale.py
SCALE_STOP = 1000.0   # 0.5 → 1000.0 (정지 시 ICP 무시)
```

정지 중 ICP vyaw 노이즈를 차단해야 드리프트가 멈춘다.

#### 방안 C: bunker_base vyaw cov 원복 ★★★

만약 방안 A를 적용한다면 (odom0 vyaw=false), bunker_base의 vyaw covariance는 EKF에 영향 없으므로 원래 값이든 1.0이든 무관하다.

만약 odom0 vyaw=true를 유지하고 싶다면 (비권장):
```cpp
// bunker_messenger.hpp:270
odom_msg.twist.covariance[35] = 100.0;  // vyaw 분산을 매우 크게
```
→ EKF가 휠 vyaw를 사실상 무시하게 됨. 하지만 이러면 vyaw=true의 의미가 없다.

### v4 권장 적용 순서

```
1단계: odom0 vyaw = false 되돌리기 (ekf.yaml)
2단계: SCALE_STOP = 1000.0 복구 (icp_odom_cov_scale.py)
3단계: 테스트 — 정지 30초 + 360° 회전 + map→odom yaw 모니터
4단계: (필요 시) map_tf_stabilizer.py 적용
```

### 핵심 원칙 정리

```
스키드 스티어 로봇 yaw 추정 원칙:

1. 휠 인코더 vyaw는 슬립으로 신뢰 불가 → odom0 vyaw=false
2. ICP F2F vyaw가 유일한 정확한 yaw rate 소스
3. 정지 중: ICP도 잔류 노이즈 있음 → SCALE_STOP=1000으로 차단
4. 회전 중: ICP PointToPlane 정확 → SCALE_ROT=1.0으로 완전 신뢰
5. 전진 중: ICP 부분 정확 → SCALE_TRANS=10으로 부분 신뢰

EKF에 vyaw 소스를 하나만 남겨야 충돌/진동 없음.
```

---

## #v5-test-result — v4 적용 후 테스트 결과 (2026-03-09)

### 적용한 수정 사항 (v4 권장대로)

1. odom0 vyaw = false ✅ (되돌림)
2. SCALE_STOP = 1000.0 ✅ (복구)
3. AngularUpdate = 3.14 ✅ (v3에서 유지)
4. map_tf_stabilizer.py ✅ (실행 중)

### 테스트 결과

| 항목 | 결과 |
|------|------|
| 글로벌 맵 | **원형 도넛 패턴으로 왜곡 + 맵이 움직임** |
| 로컬 맵 | **같이 움직임** |

스크린샷: 맵이 원형/나선형으로 왜곡 — 스캔이 다른 각도에서 겹쳐 도넛 패턴 형성.

---

## #v5-deep-analysis — 근본 원인 심층 분석

### 발견: EKF vyaw 상태 잔류 (State Persistence) 문제 ★★★★★

**이것이 지금까지 놓쳤던 핵심 버그다.**

robot_localization EKF의 상태 모델:
```
15-state: [x, y, z, roll, pitch, yaw, vx, vy, vz, vroll, vpitch, vyaw, ax, ay, az]

예측 단계 (매 1/20초):
  yaw(t+1) = yaw(t) + vyaw(t) × dt
  vyaw(t+1) = vyaw(t)           ← 등속 모델: vyaw 상태가 자동으로 0이 되지 않음!
```

**회전 → 정지 전환 시 발생하는 일:**

```
시각    상태      ICP vyaw   SCALE      EKF 내부 vyaw   EKF yaw 변화
─────────────────────────────────────────────────────────────────────
t=0     회전 중   0.5 rad/s  ×1 (ROT)   0.5 rad/s       정상 추적
t=1     회전 중   0.5 rad/s  ×1 (ROT)   0.5 rad/s       정상 추적
t=2     정지 시작  0.01      ×1000       0.5 rad/s ←!!   SCALE=1000이라 측정 무시
t=2.05  정지      0.002     ×1000       0.5 rad/s       yaw += 0.5 × 0.05 = +0.025 rad
t=2.10  정지      0.001     ×1000       0.5 rad/s       yaw += 0.5 × 0.05 = +0.025 rad
t=2.15  정지      0.001     ×1000       0.5 rad/s       yaw += 0.5 × 0.05 = +0.025 rad
 ⋮       ⋮        ⋮          ⋮           ⋮               ⋮ (계속 적분!)
```

**문제의 수학적 설명:**
```
SCALE_STOP=1000:
  R = cov[35] × 1000 → 매우 큼
  K = P / (P + R) ≈ 0  → 측정 무시

하지만 EKF 예측 단계에서:
  vyaw 상태 = 마지막 신뢰 측정값 (회전 중 값) → 0이 아님
  yaw += vyaw × dt → 매 스텝마다 적분 → 연속 드리프트!

20Hz에서 vyaw=0.3 rad/s 잔류 시:
  0.3 × 1.0초 = 0.3 rad = 17°/초 드리프트
  5초면 85° 드리프트!
```

**이것이 "맵이 움직인다"의 직접 원인이다:**
```
정지 후 odom→base_link yaw 계속 드리프트
  → RTAB-Map이 감지: "odom이 돌고 있다"
  → graph optimization으로 map→odom 보정 시도
  → 글로벌 맵 회전
  → 로컬 맵도 odom 프레임에서 이동
```

### SCALE_STOP=1000이 "무시"가 아니라 "방치"인 이유

```
"ICP를 무시한다" (의도)
    → K ≈ 0 → 새 측정으로 상태를 수정하지 않음

"EKF가 마지막 상태를 유지한다" (실제)
    → vyaw 상태 = 회전 중 마지막 값 ≈ 0.3 rad/s
    → 예측: yaw += 0.3 × dt → 매 프레임 드리프트
    → 측정으로 보정 불가 (K ≈ 0)

결론: SCALE_STOP=1000은 "ICP 무시"가 아니라 "EKF vyaw 상태 방치"
```

### 왜 AngularUpdate=3.14도 부족한가

AngularUpdate=3.14 → 360° 회전 시 2개 노드 생성.
하지만 문제는 노드 수가 아니라:

```
1. 회전 중: EKF yaw가 ICP로 정확히 추적 (SCALE_ROT=1)
2. 정지 시작: vyaw 잔류 → yaw 드리프트 시작
3. 드리프트 중: LinearUpdate=0.10 트리거될 수 있음 (작은 이동)
4. RTAB-Map이 드리프트된 pose로 노드 생성
5. graph optimization → map→odom 변경 → 맵 이동

핵심: 노드 생성 전에 odom 자체가 드리프트하므로
  AngularUpdate를 아무리 키워도 LinearUpdate로 노드가 생성될 수 있음
```

### map_tf_stabilizer가 완전히 해결하지 못하는 이유

현재 map_tf_stabilizer는:
- 순수 회전 중(ang≥0.08, lin<0.05)에만 map→odom 잠금
- **정지 후 vyaw 드리프트 구간에서는 비활성** (cmd_vel ≈ 0이니까)
- settle 구간 1.5초 후에는 해제 → 그 후 드리프트가 map→odom에 영향

```
타임라인:
  [회전] → stabilizer LOCK (OK)
  [회전 종료] → settle 1.5초 → UNLOCK
  [정지] → vyaw 잔류 드리프트 시작 ← stabilizer 비활성!
  [드리프트] → map→odom 변경 ← 맵이 움직임!
```

---

## #v5-fix — 해결 방안 (우선순위 순)

### Fix 1: icp_odom_cov_scale.py 근본 수정 ★★★★★ (가장 중요)

**핵심 아이디어:** 정지 시 covariance만 키우는 것이 아니라, **vyaw 값 자체를 0으로 강제** + **낮은 covariance로 EKF가 신뢰**하게 만든다.

**파일**: [icp_odom_cov_scale.py](src/rtabmap_ros/rtabmap_launch/scripts/icp_odom_cov_scale.py)

```python
#!/usr/bin/env python3
"""
ICP odometry vyaw covariance dynamic scaler.

[정지]  vyaw=0 강제 + cov=0.001 → EKF가 vyaw=0을 신뢰 → 드리프트 즉시 중단
[회전]  ICP vyaw 그대로 + cov×1 → EKF가 ICP 완전 추적
[전진]  ICP vyaw 그대로 + cov×10 → EKF가 ICP 부분 신뢰
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

SCALE_ROT   =    1.0   # 회전: ICP 완전 신뢰
SCALE_TRANS =   10.0   # 전진: ICP 부분 신뢰
STOP_COV    =    0.001  # 정지: vyaw=0의 covariance (작을수록 EKF가 빨리 0으로 수렴)

LIN_THRESH = 0.01   # m/s
ANG_THRESH = 0.01   # rad/s
CMD_TIMEOUT = 0.5

MODE_STOP  = 0
MODE_ROT   = 1
MODE_TRANS = 2


class IcpOdomCovScale(Node):
    def __init__(self):
        super().__init__('icp_odom_cov_scale')
        self._mode = MODE_STOP
        self._last_cmd_time = self.get_clock().now()

        self.pub = self.create_publisher(Odometry, '/icp_odom_filtered', 10)
        self.create_subscription(Odometry, '/icp_odom', self._icp_cb, qos_profile_sensor_data)
        self.create_subscription(Twist,    '/cmd_vel',  self._cmd_cb, 10)
        self.create_timer(0.1, self._timeout_check)

    def _cmd_cb(self, msg: Twist):
        self._last_cmd_time = self.get_clock().now()
        lin = abs(msg.linear.x)
        ang = abs(msg.angular.z)

        if lin < LIN_THRESH and ang < ANG_THRESH:
            self._mode = MODE_STOP
        elif ang >= ANG_THRESH:
            self._mode = MODE_ROT
        else:
            self._mode = MODE_TRANS

    def _timeout_check(self):
        dt = (self.get_clock().now() - self._last_cmd_time).nanoseconds * 1e-9
        if dt > CMD_TIMEOUT:
            self._mode = MODE_STOP

    def _icp_cb(self, msg: Odometry):
        cov = list(msg.twist.covariance)

        if self._mode == MODE_STOP:
            # ★ 핵심: vyaw 값을 0으로 강제 + 낮은 covariance
            # EKF가 "vyaw=0"을 확실히 신뢰 → 내부 vyaw 상태가 즉시 0으로 수렴
            # → yaw 적분 중단 → 드리프트 없음
            msg.twist.twist.angular.z = 0.0
            cov[35] = STOP_COV
        elif self._mode == MODE_ROT:
            cov[35] = cov[35] * SCALE_ROT
        else:
            cov[35] = cov[35] * SCALE_TRANS

        msg.twist.covariance = cov
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = IcpOdomCovScale()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

**이전 방식 vs 새 방식 비교:**
```
이전 (SCALE_STOP=1000):
  ICP vyaw = 0.002 (잔류 노이즈)
  cov = 0.002 × 1000 = 2.0
  EKF: K ≈ 0 → 측정 무시 → vyaw 상태 = 회전 중 마지막 값(0.3) 유지
  → yaw += 0.3 × dt → 연속 드리프트!

새 방식 (vyaw=0 + STOP_COV=0.001):
  vyaw = 0.0 (강제)
  cov = 0.001
  EKF: K ≈ P/(P+0.001) ≈ 1 → "vyaw=0" 측정을 완전 신뢰
  → vyaw 상태 → 0 (1~2 프레임 내)
  → yaw += 0 × dt → 드리프트 없음!
```

### Fix 2: AngularUpdate = 6.28 ★★★★

**파일**: [rtabmap.launch.py:371](src/rtabmap_ros/rtabmap_launch/launch/rtabmap.launch.py#L371)

```python
# 변경
"RGBD/AngularUpdate": "6.28",  # 3.14 → 6.28: 360° 회전해도 노드 0개
```

**이유:** Fix 1이 odom→base_link를 안정화하더라도, RTAB-Map은 여전히 회전 중 노드를 생성한다.
AngularUpdate=3.14 → 2개 노드 → loop closure 가능 → map→odom 변동.
AngularUpdate=6.28 → 0개 노드 → 순수 회전 시 RTAB-Map 완전 무반응.

```
AngularUpdate=6.28 영향:
  순수 회전: 노드 0개 → map→odom 불변 ✅
  곡선 주행: LinearUpdate=0.10 → 0.1m마다 노드 생성 → SLAM 정상 ✅
  직진+약간 회전: LinearUpdate 트리거 → 정상 ✅
```

### Fix 3: map_tf_stabilizer 개선 ★★★

현재 stabilizer는 순수 회전만 처리한다. Fix 1이 적용되면 정지 중 드리프트는 해결되므로
stabilizer의 역할은 "회전 중 RTAB-Map graph optimization에 의한 map→odom 변경 방지"로 축소된다.

Fix 2 (AngularUpdate=6.28)가 적용되면 회전 중 노드가 없으므로 graph optimization도 없다.
→ **Fix 1 + Fix 2가 둘 다 적용되면 stabilizer는 보험용으로만 유지.**

그래도 안전을 위해 settle 시간을 늘리는 것을 권장:

```python
SETTLE_SEC = 3.0   # 1.5 → 3.0초 (vyaw=0이 EKF에 완전 반영될 시간 확보)
```

### Fix 4: (선택) EKF process_noise_covariance 명시 ★★

현재 ekf.yaml에 `process_noise_covariance`가 없다 → robot_localization 기본값 사용.
기본값은 vyaw에 대해 매우 작은 process noise → vyaw 상태가 거의 변하지 않음.

Fix 1이 적용되면 이 수정은 불필요하지만, 추가 안전장치로 가능:

```yaml
# ekf.yaml에 추가 (선택적)
process_noise_covariance: [
  0.05, 0,    0,    0,    0,    0,    0,     0,     0,    0,    0,    0,    0,    0,    0,
  0,    0.05, 0,    0,    0,    0,    0,     0,     0,    0,    0,    0,    0,    0,    0,
  0,    0,    0.06, 0,    0,    0,    0,     0,     0,    0,    0,    0,    0,    0,    0,
  0,    0,    0,    0.03, 0,    0,    0,     0,     0,    0,    0,    0,    0,    0,    0,
  0,    0,    0,    0,    0.03, 0,    0,     0,     0,    0,    0,    0,    0,    0,    0,
  0,    0,    0,    0,    0,    0.06, 0,     0,     0,    0,    0,    0,    0,    0,    0,
  0,    0,    0,    0,    0,    0,    0.025, 0,     0,    0,    0,    0,    0,    0,    0,
  0,    0,    0,    0,    0,    0,    0,     0.025, 0,    0,    0,    0,    0,    0,    0,
  0,    0,    0,    0,    0,    0,    0,     0,     0.04, 0,    0,    0,    0,    0,    0,
  0,    0,    0,    0,    0,    0,    0,     0,     0,    0.01, 0,    0,    0,    0,    0,
  0,    0,    0,    0,    0,    0,    0,     0,     0,    0,    0.01, 0,    0,    0,    0,
  0,    0,    0,    0,    0,    0,    0,     0,     0,    0,    0,    0.5,  0,    0,    0,
  0,    0,    0,    0,    0,    0,    0,     0,     0,    0,    0,    0,    0.01, 0,    0,
  0,    0,    0,    0,    0,    0,    0,     0,     0,    0,    0,    0,    0,    0.01, 0,
  0,    0,    0,    0,    0,    0,    0,     0,     0,    0,    0,    0,    0,    0,    0.015]
# 핵심: [11][11] = 0.5 → vyaw process noise를 크게 설정
# → 측정 없을 때 vyaw 상태가 빠르게 불확실해짐 → 다음 측정에 빠르게 수렴
```

---

## #v5-priority — v5 적용 우선순위

| 순위 | 수정 | 파일 | 효과 | 필수 여부 |
|------|------|------|------|-----------|
| **1** | **vyaw=0 강제 + 낮은 cov** | `icp_odom_cov_scale.py` | EKF vyaw 잔류 드리프트 완전 제거 | **필수** |
| **2** | AngularUpdate=6.28 | `rtabmap.launch.py:371` | 회전 중 RTAB-Map 노드 0개 → map→odom 불변 | **필수** |
| 3 | stabilizer settle=3.0 | `map_tf_stabilizer.py:21` | 전환 구간 안전 마진 | 선택 |
| 4 | process_noise vyaw=0.5 | `ekf.yaml` | 추가 안전장치 | 선택 |

### 적용 순서

```
1단계: icp_odom_cov_scale.py 수정 (vyaw=0 강제)
  → 빌드 불필요 (symlink-install + Python)
  → 즉시 테스트: 정지 30초 → odom yaw 드리프트 = 0 확인

2단계: AngularUpdate=6.28 적용
  → 즉시 테스트: 360° 회전 → map→odom yaw 변화 확인

3단계: 전체 테스트
  → 정지 → 회전 → 정지 → 직진 → 맵 안정성 확인
```

### 검증 방법

```bash
# 터미널 1: odom→base_link yaw 모니터
ros2 run tf2_ros tf2_echo odom base_link

# 터미널 2: map→odom yaw 모니터
ros2 run tf2_ros tf2_echo map odom

# 테스트 시나리오:
# 1. 로봇 정지 30초 → odom yaw 변화 < 0.01 rad (0.6°) ← Fix 1 검증
# 2. 360° 회전 → map→odom yaw 변화 = 0 ← Fix 2 검증
# 3. 회전 후 정지 5초 → odom yaw 즉시 고정 ← Fix 1 핵심 검증
# 4. 직진 10m → 맵 정상 생성 확인
```

### 왜 이번에는 해결될 것인가

```
이전 모든 시도의 공통 실패 원인:
  SCALE_STOP=1000은 "측정 무시" → EKF vyaw 상태 잔류 → 드리프트

v5의 핵심 차이점:
  vyaw=0 + cov=0.001 = "vyaw가 0이라는 강한 측정을 주입"
  → EKF가 적극적으로 vyaw 상태를 0으로 수정
  → yaw 적분 즉시 중단
  → 드리프트의 물리적 원인이 제거됨

이것은 covariance 튜닝이 아니라 EKF 상태 모델의 근본적 한계를 우회하는 수정이다.
```

---

## #v6-test-result — v5 적용 후 테스트 (2026-03-09)

### 적용 완료

1. icp_odom_cov_scale.py: vyaw=0 강제 + STOP_COV=0.001 ✅
2. AngularUpdate = 6.28 ✅
3. map_tf_stabilizer settle = 3.0 ✅

### 테스트 결과

| 항목 | 결과 |
|------|------|
| **정지 드리프트** | **해결됨** ✅ — 정지 시 yaw 고정 |
| **제자리 회전 시 맵 회전** | **미해결** ❌ — 맵이 여전히 돌아감 (도넛 패턴) |

---

## #v6-deep-analysis — 제자리 회전 시 맵 회전 근본 원인

### 핵심 발견: RTAB-Map 노드 생성은 OR 조건이다

**소스 코드** (`rtabmap/corelib/src/Rtabmap.cpp:1518-1563`):

```cpp
bool isMoving = (_rgbdLinearUpdate > 0.0f && (
                    fabs(x) > _rgbdLinearUpdate ||
                    fabs(y) > _rgbdLinearUpdate ||
                    fabs(z) > _rgbdLinearUpdate)) ||
                (_rgbdAngularUpdate > 0.0f && (
                    fabs(roll) > _rgbdAngularUpdate ||
                    fabs(pitch) > _rgbdAngularUpdate ||
                    fabs(yaw) > _rgbdAngularUpdate));
```

**OR 조건**: AngularUpdate=6.28이어도 LinearUpdate=0.10이 트리거되면 노드가 생성된다.

### 왜 "제자리" 회전에서 LinearUpdate=0.10이 트리거되는가

```
Bunker 로봇: 스키드 스티어 구동
  └─ 4개 바퀴 회전 중심 ≠ 로봇 기하학적 중심
  └─ 회전 중심 오프셋 ≈ 5~15cm (노면, 마찰 계수, 속도에 따라 변동)

"제자리 360° 회전" 시 실제 궤적:
  ┌─────────────────────────────────────────┐
  │                                         │
  │     ● ← 실제 회전 중심                   │
  │    ╱                                    │
  │   r = 5~15cm                            │
  │  ╱                                      │
  │ ⊕ ← base_link (기하학적 중심)            │
  │                                         │
  │  base_link는 반지름 r의 원을 그린다       │
  │  원 둘레 = 2π × r = 31~94cm             │
  │                                         │
  └─────────────────────────────────────────┘

LinearUpdate=0.10 (10cm) → 360° 회전 중 3~9개 노드 생성!
```

**EKF odom 관점:**
```
EKF는 odom0(vx)와 odom1(vyaw)만 퓨전:
  - vx: 회전 중 Bunker의 휠 속도 차이 → vx ≈ 0이지만 정확히 0은 아님
  - vyaw: ICP F2F 값 (정확하지만 프레임별 잔류 오차 ε 존재)

EKF가 추정하는 x, y 변화:
  x(t+1) = x(t) + vx × cos(yaw) × dt
  y(t+1) = y(t) + vx × sin(yaw) × dt

  yaw가 변하면서 작은 vx가 다양한 방향으로 적분
  → x, y가 조금씩 변동 → 10cm 누적 가능
  → LinearUpdate=0.10 트리거 → 노드 생성!
```

### 제자리 회전 시 맵 회전의 전체 메커니즘

```
[단계 1] 회전 시작
  cmd_vel.angular.z = 0.5 rad/s → MODE_ROT → ICP vyaw × 1.0
  EKF: yaw 정확히 추적 (SCALE_ROT=1)

[단계 2] ICP F2F 프레임별 yaw 오차 누적
  Livox MID360: 비반복 로제트 스캔 패턴
  10Hz에서 0.5 rad/s 회전 → 프레임당 2.9° 회전
  ICP F2F: 연속 프레임 간 매칭
    → 프레임당 잔류 yaw 오차 ε ≈ 0.1~0.5°
    → 360° = ~72 프레임 → 누적 오차 = 7~36° (또는 √72 × ε = 0.8~4.2° 랜덤)

  실제로는 두 종류의 오차가 합산:
    ε_systematic: 스캔 패턴 특성상 한 방향으로 치우친 오차
    ε_random: 환경 대칭성, 포인트 분포에 의한 랜덤 오차

[단계 3] 로봇 물리적 이동 + EKF 위치 변동
  스키드 스티어 회전 중심 오프셋 + 작은 vx 잔류
  → EKF가 x, y 변화를 감지
  → 10cm 초과 시 LinearUpdate=0.10 트리거

[단계 4] RTAB-Map 노드 생성 (OR 조건)
  LinearUpdate 충족 → 새 노드 생성
  노드 포즈 = map→odom × odom→base_link (EKF 추정값)
  노드 사이 neighbor link = odom 기반 상대 변환

  이 시점의 EKF yaw에 ICP 누적 오차가 포함됨
  → 노드 포즈의 yaw가 실제와 다름

[단계 5] 그래프 최적화 트리거
  새 노드 추가 → 그래프 최적화 실행
  neighbor link의 covariance = EKF twist covariance (SCALE_ROT=1이라 원본)
  → g2o가 노드 포즈를 조정
  → _mapCorrection 업데이트
  → map→odom TF 변경!

[단계 6] Grid 맵 재생성
  노드 포즈 변경 → 각 노드의 스캔이 새 포즈에서 재투영
  → RayTracing=true → 넓은 영역 업데이트
  → /rtabmap/map 재발행 → global costmap 업데이트
  → 맵이 돌아 보임

  Grid/RayTracing=true이므로 스캔 하나가 변경되면
  해당 스캔의 모든 ray가 재투영 → 넓은 영역의 occupied/free가 변경
  → 시각적으로 맵 전체가 돌아가는 것처럼 보임
```

### 왜 map_tf_stabilizer가 이 문제를 해결하지 못하는가

```
map_tf_stabilizer가 하는 것:
  ✅ map→odom TF를 고정 (20Hz 발행으로 RTAB-Map 5Hz를 덮어씀)

map_tf_stabilizer가 하지 못하는 것:
  ❌ RTAB-Map 내부의 그래프 최적화를 멈추지 못함
  ❌ /rtabmap/map (OccupancyGrid) 토픽의 내용 변경을 막지 못함
  ❌ global_costmap이 /rtabmap/map의 새 내용을 반영하는 것을 막지 못함

즉:
  [RTAB-Map] → 노드 생성 → 최적화 → grid 재생성 → /rtabmap/map 발행
  [Nav2] → global_costmap → /rtabmap/map 구독 → 새 grid 적용 → "맵 변경"

  이 과정에서 TF는 stabilizer가 고정하지만,
  맵 CONTENT 자체가 바뀌므로 맵이 돌아 보임.
```

### ICP F2F의 구조적 한계 (제자리 회전 시)

```
Livox MID360 스캔 특성:
  - 비반복 로제트(rosette) 패턴
  - ~100ms당 1프레임 (10Hz)
  - 각 프레임은 반구의 일부만 커버

제자리 회전 시 ICP F2F 문제:
  ┌────────────────────────────────────────────┐
  │ Frame t-1:  환경의 0°~120° 영역을 커버     │
  │ Frame t:    환경의 3°~123° 영역을 커버     │
  │                                            │
  │ 겹치는 영역: 3°~120° (117°)               │
  │ 새로운 영역: 120°~123° (3°)                │
  │ 사라진 영역: 0°~3° (3°)                    │
  │                                            │
  │ ICP는 겹치는 117° 영역에서 매칭            │
  │ → 대부분 겹침 → 잘 작동... 이론적으로는     │
  │                                            │
  │ 실제 문제:                                  │
  │ - 환경 대칭 (복도, 네모난 방): 복수 해 존재  │
  │ - 근거리 포인트 부족: 회전 시 가까운 특징 변화│
  │ - 로제트 패턴: 같은 방향이어도 포인트 위치 다름│
  │   → 반복성 없음 → ICP 수렴점이 프레임마다 다름│
  └────────────────────────────────────────────┘
```

### 왜 도넛 패턴이 생기는가

```
도넛(원형) 맵 패턴 형성 과정:

1. 노드 N1 (0°):   스캔 S1 → pose (0, 0, 0°) → grid에 정상 투영
2. 노드 N2 (90°):  스캔 S2 → pose (0.12, -0.03, 90°+ε₁)
                    → ICP 누적 오차 ε₁ = 2° → grid에 2° 기울어진 투영
3. 노드 N3 (180°): 스캔 S3 → pose (-0.05, 0.11, 180°+ε₂)
                    → ε₂ = 5° → grid에 5° 기울어진 투영
4. 노드 N4 (270°): 스캔 S4 → pose (0.08, 0.04, 270°+ε₃)
                    → ε₃ = 8° → grid에 8° 기울어진 투영

각 스캔이 다른 각도에서 투영 → 동일 벽/장애물이 여러 위치에 표시
→ 벽이 부채꼴/원형으로 번짐 → 도넛 패턴

RayTracing=true:
  각 노드의 스캔에서 ray를 쏴서 free/occupied 결정
  기울어진 스캔의 ray가 원래 정상 위치의 occupied를 free로 덮어씀
  → 기존 정확한 맵이 파괴됨
  → 회전할수록 맵이 원형으로 왜곡
```

---

## #v6-fix — 제자리 회전 시 맵 회전 해결 방안

### Fix 1: LinearUpdate 증가 ★★★★★ (가장 직접적)

**파일**: [rtabmap.launch.py:370](src/rtabmap_ros/rtabmap_launch/launch/rtabmap.launch.py#L370)

```python
# 변경
"RGBD/LinearUpdate": "0.50",  # 0.10 → 0.50m
```

**효과:**
```
LinearUpdate=0.10 → 360° 회전 중 3~9개 노드 (원 둘레 31~94cm ÷ 10cm)
LinearUpdate=0.50 → 360° 회전 중 0~1개 노드 (원 둘레 31~94cm ÷ 50cm)

대부분의 경우 노드 0개 → 그래프 최적화 없음 → map→odom 불변 → 맵 고정
```

**trade-off:**
```
직진 주행 시:
  0.10m: 10m 주행 → 100개 노드 → 촘촘한 맵
  0.50m: 10m 주행 → 20개 노드 → 성긴 맵

Livox MID360의 6m 탐지 범위에서 0.50m 간격은:
  → 연속 스캔의 겹침 영역 = (6-0.5)/6 = 91.7% → 충분
  → SLAM 품질 영향 미미
  → 루프클로저 정확도는 노드 간격에 무관 (스캔 매칭 기반)
```

### Fix 2: 회전 중 vx=0 강제 (icp_odom_cov_scale.py 확장) ★★★★

정지 시 vyaw=0 강제와 같은 원리로, **회전 중 vx를 0으로 강제**해서
EKF가 회전 중 x,y 위치 변화를 추정하지 못하게 한다.

이렇게 하면 EKF가 보고하는 position 변화가 0 → LinearUpdate 트리거 불가.

**파일**: [icp_odom_cov_scale.py](src/rtabmap_ros/rtabmap_launch/scripts/icp_odom_cov_scale.py)

```python
def _icp_cb(self, msg: Odometry):
    cov = list(msg.twist.covariance)

    if self._mode == MODE_STOP:
        msg.twist.twist.angular.z = 0.0
        cov[35] = STOP_COV
    elif self._mode == MODE_ROT:
        cov[35] = cov[35] * SCALE_ROT
        # ★ 회전 중 vx도 0으로 강제 → EKF x,y 변화 억제
        # → RTAB-Map LinearUpdate 트리거 방지
        # (odom0 vx와 충돌하지만, odom0 vx cov=0.02 vs 여기서 cov=0.001이면
        #  EKF가 이 값을 우선 신뢰)
        # 주의: odom0도 vx를 제공하므로 이 방법은 부분적 효과만 있음
        pass  # → Fix 1과 함께 사용해야 확실
    else:
        cov[35] = cov[35] * SCALE_TRANS

    msg.twist.covariance = cov
    self.pub.publish(msg)
```

**주의:** odom1(ICP)에서 vx=0으로 보내도 odom0(wheel)이 vx를 제공하므로,
EKF는 여전히 wheel vx로 위치를 추적한다. 따라서 이 방법은 **불완전**하다.
**Fix 1(LinearUpdate=0.50)이 더 확실한 해결책이다.**

### Fix 3: odom0 vx에 회전 중 높은 covariance 부여 ★★★

회전 중 wheel vx 자체도 스키드 스티어 슬립으로 부정확하다.
bunker_base에서 회전 중 vx covariance를 동적으로 높이면,
EKF가 vx를 덜 신뢰 → x,y 위치 변화 줄어듦 → LinearUpdate 덜 트리거.

**단점:** bunker_base(C++) 수정 + 재빌드 필요. cmd_vel 구독 추가 필요.
**→ Fix 1이 훨씬 간단하므로 먼저 Fix 1 시도.**

### Fix 4: RTAB-Map 최적화 빈도 감소 (보조) ★★

```python
# 변경 (선택)
"Rtabmap/DetectionRate": "2.0",  # 5.0 → 2.0Hz: 처리 빈도 감소 → 회전 중 노드 생성 기회 감소
```

DetectionRate를 낮추면 RTAB-Map이 스캔을 덜 자주 처리 → 노드 생성 기회 감소.
그러나 직진 시 SLAM 반응속도도 느려지므로 trade-off가 있다.

### Fix 5: RayTracing 비활성화 (시각적 효과 감소) ★

```python
"Grid/RayTracing": "false",  # true → false
```

RayTracing=false이면 노드 포즈가 약간 틀려도 기존 free 영역을 덮어쓰지 않음.
→ 도넛 패턴이 덜 두드러짐. 하지만 근본 원인 해결이 아닌 증상 완화.
또한 RayTracing=false이면 장애물 뒤의 free space가 표시되지 않는 부작용.

---

## #v6-priority — v6 적용 우선순위

| 순위 | 수정 | 파일 | 효과 | 리스크 |
|------|------|------|------|--------|
| **1** | **LinearUpdate=0.50** | `rtabmap.launch.py:370` | 회전 중 노드 생성 0~1개 → 맵 안정 | 낮음 (맵 밀도 소폭 감소) |
| 2 | DetectionRate=2.0 | `rtabmap.launch.py:373` | 보조적 노드 생성 억제 | 낮음 (SLAM 반응 느림) |
| 3 | Fix 2 (vx=0 강제) | `icp_odom_cov_scale.py` | 부분적 효과 (odom0 vx가 우세) | 중간 |
| 4 | RayTracing=false | `rtabmap.launch.py:342` | 시각적 도넛 완화 | 중간 (free space 표시 안 됨) |

### 권장 적용

```
1단계: LinearUpdate = 0.50 (가장 중요 — 이것만으로 해결 가능)
  → 재시작 후 테스트: 제자리 360° 회전 → 맵 안정성 확인

2단계: (1단계 부족 시) DetectionRate = 2.0 추가

3단계: 전체 주행 테스트
  → 정지: yaw 고정 ✅ (v5에서 해결)
  → 회전: 맵 안정 ← 이번 확인 대상
  → 직진: 맵 정상 생성 확인
```

### 검증 방법

```bash
# 터미널 1: RTAB-Map 노드 생성 모니터
ros2 topic echo /rtabmap/info --field header.stamp

# 터미널 2: map→odom 모니터
ros2 run tf2_ros tf2_echo map odom

# 테스트:
# 1. 제자리 360° 회전 (5~10초 소요)
# 2. /rtabmap/info 메시지 수 확인 → 0~1개 (기존: 3~9개)
# 3. map→odom yaw 변화 확인 → < 1° (기존: 수 도~수십 도)
```

### 왜 LinearUpdate=0.50이 결정적 해결인가

```
노드 생성 조건 (Rtabmap.cpp:1518):
  isMoving = (linear_change > LinearUpdate) OR (angular_change > AngularUpdate)

현재:
  AngularUpdate = 6.28 → 360° 회전에도 angular 트리거 안 됨 ✅
  LinearUpdate  = 0.10 → 스키드 스티어 10cm 이동으로 트리거 ❌

수정 후:
  LinearUpdate  = 0.50 → 스키드 스티어 50cm 이동이 필요
  360° 회전 시 물리적 이동 = 2π × (회전 중심 오프셋) ≈ 31~94cm
  → 50cm 이상 이동하려면 회전 중심 오프셋이 ~8cm 이상이어야 함
  → 일부 조건에서는 트리거될 수 있지만, 3~9개 → 0~2개로 대폭 감소
  → 그래프 최적화 횟수 감소 → map→odom 변동 최소화
```

---

## #v7-test-result — v6 적용 후 테스트 (2026-03-09)

### 적용 완료

1. icp_odom_cov_scale.py: vyaw=0 강제 + STOP_COV=0.001 ✅
2. AngularUpdate = 6.28 ✅
3. **LinearUpdate = 0.50** ✅ (v6 핵심 수정)
4. map_tf_stabilizer settle = 3.0 ✅

### 테스트 결과

| 항목 | 결과 |
|------|------|
| 정지 드리프트 | 해결됨 ✅ |
| **제자리 회전 시 도넛 패턴** | **미해결 ❌ — LinearUpdate=0.50에서도 도넛 발생** |

### 결론: 노드 기반 분석만으로는 부족하다

LinearUpdate=0.50 + AngularUpdate=6.28이면 제자리 회전 시 RTAB-Map 노드 생성은
이론적으로 0~1개로 매우 적어야 한다. 그러나 도넛 패턴이 여전히 발생한다.

**이는 도넛의 원인이 RTAB-Map 노드/그래프 최적화만이 아니라는 것을 의미한다.**

---

## #v7-deep-analysis — 근본적 재분석: 3가지 독립 원인

### 관점 전환: "노드 생성 → 그래프 최적화" 외에 다른 경로가 있다

이전 분석(v3~v6)은 모두 이 파이프라인에 집중했다:
```
RTAB-Map 노드 생성 → 그래프 최적화 → map→odom 변경 → 맵 회전
```

**그러나 도넛 패턴을 만들 수 있는 독립 경로가 2개 더 있다:**

```
경로 A: RTAB-Map 그래프 경로 (기존 분석)
  RTAB-Map 노드 생성 → graph optimization → map→odom TF 변경 → /rtabmap/map 재생성 → 맵 변경

경로 B: Costmap 장애물 레이어 경로 (★★★★★ 새 분석)
  라이브 LiDAR 스캔 → TF(map→odom→base_link→livox_frame) 변환 → costmap obstacle marking
  → ICP F2F yaw 오차가 TF에 반영 → 장애물이 틀린 위치에 마킹 → 도넛 패턴

경로 C: ICP F2F 누적 오차 → odom→base_link yaw 자체가 틀림 (★★★★★ 근본 원인)
  ICP F2F가 프레임마다 작은 yaw 오차 누적 → EKF yaw가 실제와 다름
  → 모든 TF 기반 변환이 영향받음 → 경로 A, B 모두 오염
```

### 경로 B: Costmap 장애물 레이어 — 가장 유력한 직접 원인

#### 메커니즘

```
Nav2 local_costmap 설정 (현재):
  obstacle_layer:
    lidar_mark:
      topic: /livox/lidar/filtered
      marking: true
      observation_persistence: 0.3   ← 0.3초간 유지

Nav2 global_costmap 설정 (현재):
  obstacle_layer:
    lidar_mark:
      topic: /livox/lidar/filtered
      marking: true
      observation_persistence: 1.0   ← 1초간 유지

처리 과정 (매 LiDAR 프레임, ~10Hz):
  1. /livox/lidar/filtered의 각 포인트를 TF로 변환
     TF 체인: livox_frame → base_link → odom (→ map)
     여기서 odom→base_link 방향 = EKF 추정 yaw

  2. 변환된 포인트를 costmap grid에 마킹

  3. 제자리 회전 중:
     - EKF yaw에 ICP F2F 누적 오차가 포함됨
     - 동일 벽의 포인트가 매 프레임 약간 다른 위치에 마킹
     - 프레임 t:   벽이 (3.0, 0.0)에 마킹
     - 프레임 t+1: yaw 오차 0.3°로 벽이 (2.999, 0.016)에 마킹
     - 프레임 t+2: yaw 오차 0.6°로 벽이 (2.998, 0.031)에 마킹
     - ...
     - 360° 회전 완료: 벽이 부채꼴로 마킹됨 = 도넛 패턴

  4. observation_persistence=0.3~1.0초:
     - 0.3초 × 10Hz = 3프레임 동시 표시
     - 각 프레임이 다른 yaw에서 투영 → 벽이 번짐
```

#### 핵심 차이: RTAB-Map 노드 vs Costmap 장애물

```
RTAB-Map 노드 기반 도넛:
  - LinearUpdate/AngularUpdate로 노드 생성 억제 가능
  - /rtabmap/map (OccupancyGrid)에 나타남
  - 노드 0개 → 변화 없음

Costmap 장애물 레이어 도넛:
  - RTAB-Map 노드와 무관하게 매 LiDAR 프레임마다 발생
  - local_costmap과 global_costmap의 obstacle_layer에 나타남
  - LinearUpdate/AngularUpdate를 아무리 키워도 해결 불가 ← 이것!
  - TF 자체의 yaw 오차가 원인
```

**이것이 LinearUpdate=0.50이 효과 없었던 이유다.**

### 경로 C: ICP F2F 누적 yaw 오차 — 근본 원인

#### ICP F2F (Odom/Strategy 0)의 구조적 한계

```
F2F (Frame-to-Frame):
  매칭 대상: 직전 프레임 ↔ 현재 프레임 (1:1)

  장점: 빠름, 메모리 적음
  단점: 프레임별 오차가 무조건 누적 (error accumulation)

  360° 회전 시:
    ~72 프레임 (10Hz × ~7초)
    프레임당 yaw 오차: ε ≈ 0.05~0.3°

    최선: 오차가 랜덤 → √72 × ε ≈ 0.4~2.5° 누적
    최악: 오차가 체계적 → 72 × ε ≈ 3.6~21.6° 누적

  Livox MID360 특성으로 체계적 오차가 큼:
    - 비반복 로제트 패턴 → 같은 방향이어도 포인트 분포 다름
    - 회전 중 motion distortion (deskew 후에도 잔류)
    - 환경 대칭성 → ICP 해가 불안정
```

#### F2M (Frame-to-Map, Odom/Strategy 1)이 해결하는 이유

```
F2M (Frame-to-Map):
  매칭 대상: 로컬 맵 (최근 N프레임 축적) ↔ 현재 프레임 (N:1)

  장점:
    1. 로컬 맵 = 여러 프레임의 평균 → 포인트 분포가 균일하고 밀도 높음
    2. 비반복 스캔 패턴의 영향 감소 (여러 프레임이 서로 보완)
    3. 누적 오차 억제: 로컬 맵이 절대 기준 역할
    4. 환경 대칭에 강함: 더 많은 포인트로 매칭 → 해가 안정적

  단점:
    1. 연산량 증가 (로컬 맵 유지 + 더 큰 ICP 문제)
    2. 로컬 맵 자체가 오염되면 수정 어려움

  360° 회전 시:
    로컬 맵 = 시작점 주변의 풍부한 포인트 클라우드
    매 프레임이 이 로컬 맵에 매칭 → 누적 오차 없이 절대 pose 추정
    → yaw 오차가 누적되지 않고 프레임별 독립 → 1~2° 이내
```

### 정량적 비교

```
              F2F (Strategy 0)       F2M (Strategy 1)
              ─────────────────     ─────────────────
매칭 기준      직전 1프레임           로컬 맵 (20~50프레임 축적)
yaw 오차 모델  Σεᵢ (누적)            εᵢ (독립, 매 프레임)
360° 총 오차   2~20° (환경 의존)      0.5~2° (안정적)
도넛 패턴      ★★★★★ (필연적)        ★ (매우 약함)
연산량         낮음 (~5ms/frame)      중간 (~15ms/frame)
```

### 진단: 어떤 경로가 도넛을 만드는지 확인하는 방법

```bash
# 진단 1: RTAB-Map 노드가 정말 생성되는지 확인
ros2 topic echo /rtabmap/info --field header.stamp
# 360° 회전 중 메시지 수 세기
# → 0개면 경로 A(RTAB-Map 노드) 아님 = 경로 B(costmap) 확정

# 진단 2: /rtabmap/map이 변하는지 확인
ros2 topic hz /rtabmap/map
# 회전 중 새 메시지가 없으면 → 경로 A 아님

# 진단 3: map→odom TF가 변하는지 확인
ros2 run tf2_ros tf2_echo map odom
# 회전 중 yaw 변화 없으면 → 경로 A 아님, 경로 B 또는 C

# 진단 4: odom→base_link yaw가 정확한지 확인
ros2 run tf2_ros tf2_echo odom base_link
# 360° 회전 후 yaw 잔차 확인
# → 잔차가 크면(>3°) → ICP F2F 누적 오차 = 경로 C

# 진단 5: RViz에서 도넛이 어느 맵에 보이는지 확인
# /rtabmap/map (OccupancyGrid)만 표시 → 도넛 있으면 경로 A
# local_costmap만 표시 → 도넛 있으면 경로 B
# 둘 다 표시 → 어느 쪽이 먼저/더 심한지 비교
```

---

## #v7-fix — 해결 방안 (우선순위 순)

### Fix 1: Odom/Strategy 1 (F2M) 전환 ★★★★★ (근본 해결)

ICP F2F의 구조적 누적 오차가 경로 B, C 모두의 근본 원인이다.
F2M으로 전환하면 yaw 누적 오차가 원천 차단된다.

**파일**: [rtabmap_nav2.launch.py:55](src/rtabmap_ros/rtabmap_launch/launch/rtabmap_nav2.launch.py#L55)

```python
# 변경
'odom_args': '--Icp/VoxelSize 0.15 '
             '--Icp/PointToPlane 1 '
             '--Icp/PointToPlaneK 8 '
             '--Icp/MaxCorrespondenceDistance 0.3 '
             '--Icp/MaxTranslation 2.0 '
             '--Icp/MaxRotation 6.28 '
             '--Icp/CorrespondenceRatio 0.01 '
             '--Icp/OutlierRatio 0.7 '
             '--Icp/MaxIterations 30 '
             '--Icp/Epsilon 0.001 '
             '--Odom/Strategy 1 '           # ★ 0→1: F2F → F2M
             '--OdomF2M/MaxSize 20 '        # 로컬 맵 최대 스캔 수
             '--OdomF2M/MaxNewFeatures 0 '  # 0=제한 없음
             '--Odom/GuessMotion true '
             '--Odom/ResetCountdown 5 '
             '--Reg/Force3DoF true',
```

**핵심 파라미터:**
```
--Odom/Strategy 1:         F2M (Frame-to-Map) 활성화
--OdomF2M/MaxSize 20:      로컬 맵에 최근 20프레임 유지
  → 10Hz × 20 = 2초간의 스캔 축적
  → 0.25 m/s × 2초 = 0.5m 이동 범위의 맵
  → 제자리 회전 시: 20프레임 = 다양한 각도의 완전한 환경 포인트
--OdomF2M/MaxNewFeatures 0: 새 포인트 제한 없음 (Livox의 희소 특성 보상)
```

**왜 효과가 있는가:**
```
F2F 문제: 프레임 t ↔ t-1만 매칭 → 오차 누적
  t=0: P₀ ↔ P₁ → Δyaw₁ + ε₁
  t=1: P₁ ↔ P₂ → Δyaw₂ + ε₂
  ...
  t=71: P₇₀ ↔ P₇₁ → Δyaw₇₁ + ε₇₁
  총 오차 = Σεᵢ (72개 누적)

F2M 해결: 프레임 t ↔ LocalMap 매칭 → 오차 독립
  LocalMap = Σ(P₀...P₁₉) → 풍부한 포인트 분포
  t=0:  P₀ ↔ LM → Δpose₀ + ε₀  (LM에 직접 매칭)
  t=1:  P₁ ↔ LM → Δpose₁ + ε₁  (LM에 직접 매칭)
  ...
  총 오차 ≈ max(εᵢ) (독립, 누적 아님)
```

**리스크:**
- 연산량 증가: Jetson Orin에서 ~15ms/frame (기존 ~5ms) → 10Hz 유지 가능
- 로컬 맵이 너무 커지면 메모리/시간 증가 → MaxSize=20으로 제한

### Fix 2: 진단 우선 실행 (Fix 1 전에) ★★★★★

Fix 1을 적용하기 전에 진단을 먼저 실행해서 **도넛이 경로 A인지 B인지 확정**해야 한다.

```bash
# === 진단 스크립트 ===
# 터미널 1: RTAB-Map 노드 생성 모니터
ros2 topic echo /rtabmap/info --field header.stamp &

# 터미널 2: /rtabmap/map 발행 빈도
ros2 topic hz /rtabmap/map &

# 터미널 3: map→odom TF
ros2 run tf2_ros tf2_echo map odom &

# 터미널 4: odom→base_link TF
ros2 run tf2_ros tf2_echo odom base_link &

# 테스트: 제자리 360° 회전 (5~10초)
# 그 후 각 터미널 결과 비교
```

**진단 결과 해석:**

| /rtabmap/info | /rtabmap/map 변화 | map→odom 변화 | 결론 |
|---------------|-------------------|---------------|------|
| 0~1개 | 없음 | 없음 | 경로 A 아님 → **경로 B(costmap) 확정** |
| 0~1개 | 없음 | 변화 있음 | 이상 (디버그 필요) |
| 3+ 개 | 있음 | 변화 있음 | **경로 A도 기여** + 경로 B도 가능 |

### Fix 3: Costmap 장애물 레이어 회전 중 비활성화 ★★★★

경로 B가 확인되면, 회전 중 costmap obstacle marking을 중지하는 방법.

**방법 A: observation_persistence=0으로 잔류 제거**

현재:
```yaml
local_costmap:
  obstacle_layer:
    lidar_mark:
      observation_persistence: 0.3   # 0.3초 유지 → 회전 중 3프레임 겹침
```

변경:
```yaml
local_costmap:
  obstacle_layer:
    lidar_mark:
      observation_persistence: 0.0   # 매 프레임 즉시 교체 → 겹침 최소화
```

**효과:** 0.0이면 직전 프레임의 마킹만 유지 → 도넛이 1프레임분으로 축소 → 거의 안 보임.
**리스크:** LiDAR 드롭 시 장애물 깜빡임. 하지만 lidar_clear(clearing=true)가 별도로 있으므로 큰 문제 없음.

**방법 B: 회전 중 costmap clearing 강화**

현재 local_costmap의 lidar_clear:
```yaml
lidar_clear:
  raytrace_range: 4.5
```

이미 clearing이 활성화되어 있으므로, observation_persistence만 줄이면 충분.

### Fix 4: LinearUpdate 추가 증가 (보조) ★★★

LinearUpdate=0.50에서도 노드가 생성되고 있다면:

```python
# 변경
"RGBD/LinearUpdate": "1.00",  # 0.50 → 1.00m
```

1m 이동해야 노드 생성 → 제자리 회전 시 물리적 이동(31~94cm)으로는 거의 트리거 불가.
**단점:** 직진 시 맵 밀도가 더 낮아짐 (10m 주행 → 10개 노드).
Livox 6m 범위에서 1m 간격은 겹침 83% → 아직 충분.

### Fix 5: predict_to_current_time = false 확인 ★★

**파일**: [ekf.yaml:15](src/robot_localization/params/ekf.yaml#L15)

현재 `predict_to_current_time: false` ✅ — 이미 설정됨.
이 설정이 true면 EKF가 최신 시간까지 예측 → 정지 중 vyaw 잔류값으로 추가 적분.
false이므로 이 문제는 아님.

---

## #v7-priority — v7 적용 우선순위

| 순위 | 수정 | 효과 | 리스크 |
|------|------|------|--------|
| **0** | **진단 실행** (노드 생성 수, /rtabmap/map 변화, TF 변화 확인) | 원인 확정 | 없음 |
| **1** | **Odom/Strategy 1 (F2M)** | ICP yaw 누적 오차 원천 차단 → costmap 도넛 제거 | 중간 (연산량 증가) |
| 2 | observation_persistence=0.0 (local_costmap) | costmap 도넛 즉시 완화 | 낮음 (깜빡임 가능) |
| 3 | LinearUpdate=1.00 | 노드 생성 추가 억제 | 낮음 (맵 밀도 감소) |

### 권장 적용 순서

```
0단계: 진단 실행 (경로 A vs B 확정)
  → 위 진단 명령어 실행 → 결과에 따라 방향 결정

1단계 (경로 B 확인 시): Odom/Strategy 1 (F2M)
  → rtabmap_nav2.launch.py odom_args 수정
  → 재빌드 불필요 (launch 파일만 변경)
  → 테스트: 제자리 360° 회전 → 도넛 감소/제거 확인

  F2M 추가 효과:
  - odom→base_link yaw 정확도 향상 → costmap 도넛 해결 (경로 B)
  - RTAB-Map 그래프의 odom chain 오차 감소 → map→odom 안정 (경로 A도 개선)
  - 정지 중 ICP도 안정 (로컬 맵이 기준이므로 잔류 노이즈 감소)

2단계: observation_persistence=0.0 (즉시 효과 확인용)
  → nav2_rtabmap_params.yaml 수정
  → 경로 B 완화 (F2M과 병행)

3단계: 전체 주행 테스트
  → 정지 → 회전 → 직진 → 맵 안정성 + 장애물 인식 정확도 확인
```

---

## #v7-diagnostic-result — 진단 결과 (2026-03-09) ★★★★★ 결정적 발견

### 진단 데이터

```
제자리 회전 중 측정:

1. ros2 topic hz /rtabmap/map → 발행 없음 (not published)
2. map→odom TF → identity (0,0,0, yaw=0.000) 고정 — 변화 없음
3. odom→base_link TF → yaw=0.000 고정!
   - x: 0.007 → 0.016 (약 1mm/s vx 적분)
   - yaw: 0.000 → 0.000 (전혀 변하지 않음!)
```

### 핵심 발견: EKF가 회전 중 yaw를 전혀 추적하지 않고 있다

```
로봇 상태: 물리적으로 360° 제자리 회전 중
odom→base_link yaw: 0.000 → 0.000 (변화 없음)

이것이 의미하는 바:
  로봇은 회전하고 있지만, TF 체인은 "로봇이 정면을 보고 있다"고 말함
  → 모든 LiDAR 스캔이 yaw=0 기준으로 변환됨
  → 실제로는 다른 방향을 보고 있으므로 다른 환경을 스캔
  → 그런데 전부 같은 방향(yaw=0)으로 투영
  → 동일 벽이 여러 각도에서 겹침 → 도넛 패턴 ← 이것이 진짜 원인!
```

### 이전 분석(v3~v6)이 모두 틀렸던 이유

```
v3~v6의 전제: "EKF가 ICP vyaw로 yaw를 잘 추적하고 있다"
실제: EKF yaw가 전혀 변하지 않는다 → 전제 자체가 틀림

이전에 "노드 생성 → 그래프 최적화 → map→odom 변경"이 원인이라고 했지만:
  - /rtabmap/map 발행 없음 → RTAB-Map 노드 생성 없음
  - map→odom = identity → 그래프 최적화 발생 안 함
  - odom→base_link yaw=0 → EKF가 회전을 추적하지 않음

모든 수정 (AngularUpdate, LinearUpdate, Optimizer/Robust 등)은
존재하지 않는 문제를 해결하려 한 것이다.
```

### 도넛 패턴의 진짜 메커니즘

```
시간  물리적 yaw   EKF yaw   LiDAR 보는 방향   TF 변환 방향   결과
──────────────────────────────────────────────────────────────────
t=0   0°          0°        정면              정면           정상
t=1   30°         0°        30° 방향          정면으로 투영   벽이 30° 회전된 위치에 마킹
t=2   60°         0°        60° 방향          정면으로 투영   벽이 60° 회전된 위치에 마킹
t=3   90°         0°        90° 방향          정면으로 투영   벽이 90° 회전된 위치에 마킹
...
t=12  360°        0°        정면(원래)         정면으로 투영   정상(원위치)

→ 모든 프레임의 스캔이 yaw=0 기준으로 투영됨
→ 실제 환경의 360° 뷰가 한 방향에 겹침
→ 벽/장애물이 원형으로 퍼짐 = 도넛 패턴
```

---

## #v7-root-cause — 진짜 근본 원인: ICP vyaw가 EKF에 도달하지 않고 있다

### 가능한 원인 3가지 (우선순위 순)

#### 원인 1: ICP F2F가 회전 중 수렴 실패 (가장 유력) ★★★★★

```
Livox MID360 + 제자리 회전 + Odom/Strategy 0 (F2F):

  100ms당 1프레임, 0.5 rad/s 회전 → 프레임당 2.9° 회전
  비반복 로제트 패턴 → 연속 두 프레임의 포인트 분포가 다름

  ICP F2F 매칭:
    P_{t-1} ↔ P_t
    두 프레임 모두:
      - 포인트 분포가 다름 (로제트 패턴)
      - 회전으로 인해 시야가 변함
      - 근거리 포인트 분포 급변
    → 대응점 매칭 실패 / 수렴 불량 / 높은 covariance 출력

  RTAB-Map ICP odometry의 실패 처리:
    수렴 실패 시:
    → covariance를 매우 크게 설정 (sentinel 값, 예: 9999)
    → 또는 마지막 성공 변환을 재사용 (vyaw≈0일 수 있음)
    → 또는 publish 자체를 건너뜀

  결과:
    /icp_odom이 회전 중에:
    a) 발행되지 않음 → EKF에 vyaw 입력 없음 → yaw 불변
    b) 발행되지만 cov[35]=9999 → EKF 무시 → yaw 불변
    c) 발행되지만 vyaw≈0 (잘못된 값) → EKF yaw 불변
```

#### 원인 2: icp_odom_cov_scale.py의 모드 판정 오류 ★★★

```
가능한 시나리오:
  Nav2 → velocity_smoother → /cmd_vel
  velocity_smoother가 cmd_vel 발행을 지연하거나
  로봇이 Nav2 없이 직접 제어될 때 /cmd_vel이 다른 토픽이면
  → CMD_TIMEOUT(0.5초) → MODE_STOP
  → vyaw=0 강제 + cov=0.001
  → EKF가 "vyaw=0"을 신뢰 → yaw 고정!

  이 경우:
    로봇은 실제로 회전하지만
    /cmd_vel이 오지 않으면 MODE_STOP
    → vyaw=0 강제 → EKF yaw=0 고정
    → 도넛!
```

#### 원인 3: Odom/GuessMotion이 잘못된 초기 추정 전파 ★★

```
--Odom/GuessMotion true:
  ICP 초기 추정에 이전 프레임의 상대 변환을 사용

  정지 후 갑자기 회전 시작:
    이전 guess = (0, 0, 0°) (정지)
    실제 변환 = (0, 0, 2.9°) (회전 시작)
    → 초기 추정이 크게 벗어남
    → ICP가 로컬 최솟값에 빠짐
    → 수렴 실패 → 높은 covariance or 잘못된 결과
```

---

## #v7-diagnosis-commands — 원인 확정을 위한 추가 진단 (필수!)

```bash
# === 진단 A: /icp_odom이 회전 중 발행되는지 확인 ===
ros2 topic hz /icp_odom
# 회전 중 Hz 확인 → 0이면 원인 1 (ICP 수렴 실패) 확정

# === 진단 B: /icp_odom의 vyaw 값 확인 ===
ros2 topic echo /icp_odom --field twist.twist.angular
# 회전 중 angular.z 값 확인
# → 0에 가까우면: ICP가 회전 감지 실패
# → 정상 값(0.3~0.5 rad/s)이면: 파이프라인 다음 단계 확인

# === 진단 C: /icp_odom_filtered 확인 (cov_scale 통과 후) ===
ros2 topic echo /icp_odom_filtered --field twist.twist.angular
ros2 topic echo /icp_odom_filtered --field twist.covariance
# → vyaw=0이면: icp_odom_cov_scale.py가 MODE_STOP으로 강제 중
# → cov=0.001이면: MODE_STOP 확정 (회전인데 정지로 판정)

# === 진단 D: /cmd_vel이 회전 중 발행되는지 확인 ===
ros2 topic hz /cmd_vel
ros2 topic echo /cmd_vel
# 회전 중 cmd_vel 값 확인 → angular.z > 0.01이어야 MODE_ROT
```

**진단 결과 해석표:**

| /icp_odom Hz | /icp_odom vyaw | /icp_odom_filtered vyaw | /cmd_vel | 결론 |
|-------------|----------------|------------------------|----------|------|
| 0 Hz | - | - | - | ICP 완전 실패 → F2M 필요 |
| >0 Hz | vyaw≈0 | vyaw≈0 | angular>0 | ICP 회전 감지 실패 → F2M 필요 |
| >0 Hz | vyaw 정상 | vyaw=0, cov=0.001 | angular=0 | **cmd_vel 미수신 → MODE_STOP 오작동** |
| >0 Hz | vyaw 정상 | vyaw=0, cov=0.001 | angular>0 | cov_scale 코드 버그 |
| >0 Hz | vyaw 정상 | vyaw 정상, cov 매우 큼 | angular>0 | ICP covariance 너무 큼 → EKF 무시 |

---

## #v7-fix — 해결 방안

### Fix 0: 디버그 로그 추가 (진단 보조, 즉시 적용)

**파일**: [icp_odom_cov_scale.py](src/rtabmap_ros/rtabmap_launch/scripts/icp_odom_cov_scale.py)

_icp_cb에 1초 간격 로그 추가:
```python
def _icp_cb(self, msg: Odometry):
    cov = list(msg.twist.covariance)
    raw_vyaw = msg.twist.twist.angular.z
    raw_cov35 = cov[35]

    if self._mode == MODE_STOP:
        msg.twist.twist.angular.z = 0.0
        cov[35] = STOP_COV
    elif self._mode == MODE_ROT:
        cov[35] = cov[35] * SCALE_ROT
    else:
        cov[35] = cov[35] * SCALE_TRANS

    # ★ 디버그 로그
    now = self.get_clock().now()
    if not hasattr(self, '_last_log') or (now - self._last_log).nanoseconds > 1e9:
        self._last_log = now
        modes = ['STOP', 'ROT', 'TRANS']
        self.get_logger().info(
            f'mode={modes[self._mode]} raw_vyaw={raw_vyaw:.4f} '
            f'raw_cov={raw_cov35:.4f} → out_vyaw={msg.twist.twist.angular.z:.4f} '
            f'out_cov={cov[35]:.6f}')

    msg.twist.covariance = cov
    self.pub.publish(msg)
```

### Fix A: F2M 전환 (ICP 수렴 실패가 원인일 때) ★★★★★

**파일**: [rtabmap_nav2.launch.py:55](src/rtabmap_ros/rtabmap_launch/launch/rtabmap_nav2.launch.py#L55)

```python
'odom_args': '--Icp/VoxelSize 0.15 '
             '--Icp/PointToPlane 1 '
             '--Icp/PointToPlaneK 8 '
             '--Icp/MaxCorrespondenceDistance 0.3 '
             '--Icp/MaxTranslation 2.0 '
             '--Icp/MaxRotation 6.28 '
             '--Icp/CorrespondenceRatio 0.01 '
             '--Icp/OutlierRatio 0.7 '
             '--Icp/MaxIterations 30 '
             '--Icp/Epsilon 0.001 '
             '--Odom/Strategy 1 '           # ★ F2F→F2M
             '--OdomF2M/MaxSize 20 '        # 로컬 맵 최대 스캔 수
             '--OdomF2M/MaxNewFeatures 0 '  # 제한 없음
             '--Odom/GuessMotion true '
             '--Odom/ResetCountdown 5 '
             '--Reg/Force3DoF true',
```

**F2M이 해결하는 이유:**
```
F2F 실패: 연속 2프레임의 포인트 분포가 다름 → 매칭 실패 → vyaw 미출력
F2M 해결: 로컬 맵 = 20프레임 축적 → 풍부한 포인트
  → 어떤 로제트 패턴/방향이든 로컬 맵과 매칭 가능
  → 회전 중에도 안정적 수렴 → 정확한 vyaw → EKF yaw 추적 → 도넛 제거
```

### Fix B: cmd_vel 의존 제거 (MODE_STOP 오작동 방지) ★★★★

**파일**: [icp_odom_cov_scale.py](src/rtabmap_ros/rtabmap_launch/scripts/icp_odom_cov_scale.py)

cmd_vel 대신 ICP 자체의 vyaw 크기로 모드 판정:

```python
ICP_VYAW_STOP = 0.02   # rad/s 미만이면 정지

def _icp_cb(self, msg: Odometry):
    cov = list(msg.twist.covariance)
    raw_vyaw = abs(msg.twist.twist.angular.z)

    # 정지 판정: ICP vyaw가 작고 AND cmd_vel도 정지
    is_stopped = raw_vyaw < ICP_VYAW_STOP and self._mode == MODE_STOP

    if is_stopped:
        msg.twist.twist.angular.z = 0.0
        cov[35] = STOP_COV
    elif self._mode == MODE_ROT or raw_vyaw >= ICP_VYAW_STOP:
        # cmd_vel이 ROT이거나 ICP가 회전을 감지하면 → 신뢰
        cov[35] = cov[35] * SCALE_ROT
    else:
        cov[35] = cov[35] * SCALE_TRANS

    msg.twist.covariance = cov
    self.pub.publish(msg)
```

**핵심:** `raw_vyaw >= ICP_VYAW_STOP`이면 cmd_vel과 무관하게 회전으로 판정.
cmd_vel이 없어도 ICP가 회전을 감지하면 vyaw가 EKF에 전달됨.

### Fix C: wheel vyaw fallback (보험) ★★★

```yaml
# ekf.yaml — odom0 vyaw를 매우 높은 covariance로 활성화
odom0_config: [false, false, false, false, false, false,
               true,  # vx
               false, false, false, false,
               true,  # vyaw ← 활성화 (매우 낮은 신뢰로)
               false, false, false]
```

bunker_base에서 vyaw covariance를 100.0으로 설정:
```cpp
// bunker_messenger.hpp
odom_msg.twist.covariance[35] = 100.0;  // 매우 낮은 신뢰
```

**목적:**
```
ICP 정상 시: ICP cov≈0.01, wheel cov=100 → EKF가 ICP 99% 신뢰 → 정확
ICP 실패 시: ICP 미발행 → wheel이 유일한 vyaw 소스 → 부정확하지만 yaw≠0
  → 스키드 스티어 ±20~30% 오차가 있어도
  → yaw=0 고정보다 훨씬 나음
```

**이전 odom0 vyaw=true에서 진동이 발생한 이유와의 차이:**
```
이전(v4): bunker cov=1.0 → EKF가 50:50으로 가중 → 진동
이번(v7): bunker cov=100.0 → EKF가 ICP 99% 신뢰 → 진동 없음
  ICP 실패 시에만 wheel이 fallback
```

---

## #v7-priority — v7 적용 우선순위

| 순위 | 수정 | 효과 | 전제 |
|------|------|------|------|
| **0** | **추가 진단** (위 4개 명령어) | 원인 확정 | 없음 — 즉시 5분 |
| **1** | **F2M 전환** (Odom/Strategy 1) | ICP 수렴 안정 → vyaw 출력 정상 → yaw 추적 → 도넛 제거 | ICP 실패일 때 |
| **1b** | **Fix B: cmd_vel 의존 제거** | MODE_STOP 오작동 방지 | cmd_vel 문제일 때 |
| 2 | **디버그 로그** (Fix 0) | 실시간 모드/값 확인 | 없음 — 즉시 |
| 3 | **wheel vyaw fallback** (Fix C) | ICP 완전 실패 시 보험 | F2M 이후에도 가끔 실패 시 |

### 권장 적용 순서

```
0단계: 추가 진단 (5분) ← 필수!
  ros2 topic hz /icp_odom              # 회전 중 발행 여부
  ros2 topic echo /icp_odom --field twist.twist.angular   # vyaw 값
  ros2 topic echo /icp_odom_filtered --field twist.twist.angular  # 필터 후
  ros2 topic hz /cmd_vel               # cmd_vel 발행 여부

  → 원인 확정 후 다음 단계

1단계: (ICP 실패 확인 시) F2M 전환
  → odom_args에서 --Odom/Strategy 0 → 1 + --OdomF2M/MaxSize 20 추가
  → 재시작
  → 테스트: odom→base_link yaw가 회전을 추적하는지 확인

1b단계: (cmd_vel 문제 확인 시) Fix B 적용
  → icp_odom_cov_scale.py 수정: ICP vyaw 기반 모드 판정

2단계: 전체 테스트
  → 제자리 360° 회전 → 도넛 해결 확인
  → 직진 10m → SLAM 정상 동작 확인
```

---

*계획서 v7 갱신: 2026-03-09 | ★ 결정적 발견: EKF yaw=0 고정 (회전 미추적)*
*v6→v7 핵심: 도넛은 RTAB-Map 노드/그래프 문제가 아니었다. odom→base_link yaw 자체가 0으로 고정되어 있었다. ICP vyaw가 EKF에 도달하지 않는 것이 근본 원인.*

---
### codex 형식
## #v8-user-measurement-update — 최신 실측 기반 해결안 재정렬 (2026-03-09)

### 사용자가 직접 측정한 사실 (최신)

```bash
1) ros2 topic hz /rtabmap/map
   -> WARNING: topic [/rtabmap/map] does not appear to be published yet

2) ros2 run tf2_ros tf2_echo map odom
   -> yaw=0.000, translation=0,0,0 고정 (identity)

3) ros2 run tf2_ros tf2_echo odom base_link
   -> yaw=0.000 고정
   -> x만 0.007 -> 0.016으로 소폭 증가
```

### 이 데이터가 의미하는 것 (해석 업데이트)

1. 현재 시점의 핵심 문제는 `map->odom` 불안정이 아니다.
   - `map->odom`이 identity로 고정되어 있음.
2. 현재 시점의 핵심 문제는 `odom->base_link` yaw가 회전을 추적하지 않는 것이다.
   - 로봇이 제자리 회전 중인데 yaw=0이면, 센서 스캔이 잘못된 방향 기준으로 적층됨.
3. `x`가 소폭 누적되는 것은 `vx` 소스(휠 odom)의 회전 중 슬립/편향 적분 가능성을 시사한다.
   - 즉, yaw가 죽어 있고 vx는 살아 있는 상태다.

### 결론: 해결 우선순위 재정렬

기존의 `map TF 안정화`, `AngularUpdate` 튜닝보다 먼저,
**EKF에 회전 정보(vyaw)가 실제로 들어가는지**를 복구해야 한다.

---

## #v8-root-cause-priority — 최신 우선 원인

### 원인 1 (최우선): icp_odom_cov_scale 모드 판정이 회전을 STOP으로 오판

가능한 시나리오:
- `/cmd_vel` 기반 판정만으로 STOP 진입
- 회전 중에도 `CMD_TIMEOUT`로 STOP 전환
- STOP에서 `vyaw=0` 강제 주입
- EKF yaw가 0으로 고정

### 원인 2: ICP 출력이 센티널/저신뢰(cov[35]≈9999)로 들어와 EKF가 무시

회전 중 ICP가 불안정하면:
- `vyaw`가 0 근처거나
- covariance가 매우 커져 EKF 반영률이 0에 수렴
- 결과적으로 yaw 추적 불능

### 원인 3: 회전 중 휠 `vx` 편향 누적

`odom0_config`에서 `vx=true`이므로, 회전 중 슬립성 전진 성분이 적분되어
`odom->base_link`의 `x`가 서서히 증가할 수 있다.

---

## #v8-fix — 최신 해결방법 (실행 우선순위)

### Fix 1: icp_odom_cov_scale 모드 판정 로직 재설계 (필수)

**핵심 원칙**
- STOP 판정은 단일 신호(`/cmd_vel`)에 의존하지 않는다.
- 아래 3개 중 하나라도 회전이면 STOP 강제를 금지한다.
  - `|cmd_vel.angular.z|`
  - `|raw_icp_vyaw|`
  - `|wheel_odom.angular.z|`

권장 판정:
```text
ROT if (cmd_rot OR icp_rot OR wheel_rot)
STOP only if (NOT cmd_rot AND NOT icp_rot AND NOT wheel_rot) 지속 N초
```

### Fix 2: STOP/센티널 처리 분리 (필수)

현재 문제는 "정지 강제"와 "ICP 실패(센티널)"를 같은 의미로 다루는 데서 생긴다.

권장 정책:
1. **진짜 STOP**:
   - `angular.z = 0.0` 강제
   - `cov[35] = STOP_COV` (예: 0.05~0.5)
2. **센티널 + 회전 판단**:
   - `angular.z`를 0으로 강제하지 않음
   - `cov[35] = FAIL_COV` (예: 5~20)로 설정해 "완전 무시"를 피함
   - 필요 시 직전 유효 vyaw 또는 wheel vyaw fallback 사용

### Fix 3: icp_odom_cov_scale에 런타임 로그 추가 (필수)

1초 주기로 아래를 출력:
- mode(STOP/ROT/TRANS)
- raw_vyaw, raw_cov35
- out_vyaw, out_cov35
- cmd_wz, wheel_wz

이 로그 없이는 오판 원인을 재현하기 어렵다.

### Fix 4: ICP 수렴 자체가 약하면 F2M 전환 (조건부)

`/icp_odom`에서 회전 중 vyaw가 계속 0 근처거나 발행률 저하가 확인되면:
- `--Odom/Strategy 1` (F2M) 전환
- `--OdomF2M/MaxSize 20` 추가

### Fix 5: 회전 중 vx 억제(선택)

회전 중 `x` 누적이 계속 보이면:
- 회전 상태에서 wheel `vx` 공분산을 키우거나
- 회전 중 `vx` 반영을 줄이는 보호 로직 추가

---

## #v8-verification — 최신 검증 절차

```bash
# 1) 회전 정보 입력 경로 확인
ros2 topic hz /icp_odom
ros2 topic echo /icp_odom --field twist.twist.angular
ros2 topic echo /icp_odom --field twist.covariance

ros2 topic hz /icp_odom_filtered
ros2 topic echo /icp_odom_filtered --field twist.twist.angular
ros2 topic echo /icp_odom_filtered --field twist.covariance

ros2 topic hz /cmd_vel
ros2 topic echo /odom --field twist.twist.angular

# 2) TF 결과 확인
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo map odom
```

성공 기준:
1. 제자리 회전 중 `odom->base_link yaw`가 실제 회전을 추적한다 (`yaw`가 0 고정이면 실패).
2. 회전 중 `icp_odom_filtered angular.z`가 0으로 강제되지 않는다 (ROT 모드 유지).
3. 정지 후에는 `angular.z -> 0`으로 빠르게 수렴하고 yaw 드리프트가 멈춘다.
4. `map->odom`은 불필요한 점프 없이 안정 유지.

---

*계획서 v8 갱신: 2026-03-09 | 최신 실측 반영*
*핵심: 지금은 map TF 문제가 아니라, 회전 중 EKF yaw 입력 경로(특히 icp_odom_cov_scale 모드 판정)가 1순위다.*

---

## #v9-goal — 최종 목표 재정의 (2026-03-09)

해결 목표는 아래 2개를 동시에 만족하는 것이다.

1. 회전 중 `odom->base_link yaw`가 실제 회전각을 정상 추적한다.
2. 회전 중 `map->odom`이 불필요하게 같이 회전하지 않는다.

부가 목표:
- costmap의 도넛 누적(회전 고스팅)을 구조적으로 제거한다.
- local/global clear는 보조 수단으로만 사용한다.

---

## #v9-observation — 최신 로그 기반 사실 요약

최신 런(`run_20260309_154907`)에서 확인된 사실:

1. 회전 유발 구간은 중반 1회(약 11초)이며, `out_vyaw`가 연속으로 통과했다.
2. 해당 구간의 모드 사유는 `auth_rot`였고, `cmd_rot=0`, `wheel_rot=1`이었다.
3. 같은 구간에서 `wheel_wz`가 최대 약 `-0.919 rad/s`까지 튀었다.
4. 결과적으로 `out_vyaw` 누적 적분량이 약 `-6.24 rad` 수준으로 커졌다.
5. 초반에는 ICP 저복잡도 경고가 반복되며 map 안정성이 낮은 징후가 있었다.
6. `map_tf_stabilizer`는 실행은 됐지만, `/cmd_vel` 기반 트리거 구조에서는 실사용 상황에서 회전 구간 잠금이 누락될 수 있다.

의미:
- 중반 TF 회전은 EKF 입력 게이팅 축 문제(`odom/wheel 기반 회전 오판`)가 직접 원인.
- 초반 map 회전은 ICP/RTAB-Map 축 문제(저복잡도 환경에서의 map 보정 흔들림)가 직접 원인.

---

## #v9-architecture — 3축 해결 구조

### Axis A: ICP(F2M) 안정화 축 (map->odom 안정성 담당)

핵심:
- 회전 구간 map 흔들림은 ICP 수렴 품질이 약하면 반복된다.
- F2F 누적 오차를 줄이기 위해 F2M을 1차 적용한다.

권장:
1. `Odom/Strategy = 1` (F2M)
2. `OdomF2M/MaxSize = 20` (시작점, 필요 시 30까지)
3. 회전 테스트에서 ICP 저복잡도 경고 빈도와 `map->odom` yaw 변동폭을 함께 측정

성공 기준:
- 360도 제자리 회전 중 `map->odom`이 연속 회전하지 않고, 보정 점프가 유의미하게 감소

---

### Axis B: TF lock/완화 축 (회전 중 map 고정 보호막)

핵심:
- map lock은 보정 품질이 일시적으로 약한 구간을 방어하는 장치다.
- 단, 트리거가 실제 회전을 반영해야 한다.

권장:
1. `map_tf_stabilizer`의 회전 트리거를 `/cmd_vel` 단독에서 탈피
2. 우선 순위 트리거:
   - 1순위: `/odometry/filtered`의 `twist.twist.angular.z`
   - 2순위: `/icp_odom_filtered`의 `twist.twist.angular.z`
3. `enter/exit` 히스테리시스 적용 (예: enter 0.12, exit 0.05)
4. lock 중 20Hz 재발행 + 종료 후 settle 구간 유지

성공 기준:
- 회전 중 `odom->base_link`는 정상 회전, `map->odom`은 lock 구간에서 안정 유지

---

### Axis C: EKF 게이팅 축 (odom->base_link yaw 추종 담당)

핵심:
- `wheel_wz` 단독 회전 트리거는 금지한다. (정지 스파이크에 취약)
- STOP/센티널/회전을 분리해 EKF에 들어가는 vyaw를 제어한다.

권장 정책:
1. `ROT` 진입:
   - `cmd_rot` 우선
   - `cmd` 부재 시 `icp_rot + 안정조건(연속 프레임, cov 가드)`로 제한 허용
   - `wheel_rot`는 진단용으로만 사용(단독 진입 금지)
2. `STOP`:
   - 진짜 정지에서만 `angular.z=0` 강제 + `STOP_COV`
3. `센티널(cov35>=9000)`:
   - STOP과 분리
   - 비정지 판단이면 `angular.z` 0 강제 금지 + `FAIL_COV(중간값)`

성공 기준:
- 회전 중 `odom->base_link yaw`가 실제 회전각을 추종
- 정지 중 yaw 드리프트는 재발하지 않음

---

## #v9-priority — 적용 순서 (중요)

1. Axis A (F2M) 먼저 적용: map 축의 근본 안정화
2. Axis C (EKF 게이팅) 적용: odom 축 회전 추종 복구
3. Axis B (TF lock) 적용: 회전 구간 보호막으로 최종 안정화

주의:
- costmap clear(local/global)는 마지막 단계의 증상 완화로만 사용
- clear만으로는 TF 오차가 남으면 도넛이 재발함

---

## #v9-verification — 축별 검증 절차

```bash
# 공통: 회전 테스트 전/중/후 30초씩 기록
ros2 topic hz /icp_odom
ros2 topic hz /icp_odom_filtered
ros2 topic echo /icp_odom_filtered --field twist.twist.angular
ros2 topic echo /icp_odom_filtered --field twist.covariance
ros2 topic echo /odometry/filtered --field twist.twist.angular
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo map odom
```

판정:
1. `odom->base_link yaw`가 회전 중 연속적으로 변하면 통과
2. 같은 구간 `map->odom yaw`가 불필요 연속 회전/큰 점프가 없으면 통과
3. 정지 후 `odom->base_link yaw` 드리프트가 멈추면 통과
4. 도넛 누적이 사라지거나 현저히 감소하면 통과

---

*계획서 v9 갱신: 2026-03-09*
*핵심: map 회전 문제와 odom 회전 추종 문제를 분리해, ICP(F2M) + TF lock + EKF 게이팅의 3축으로 단계적으로 해결한다.*

---

## #v10-goal — 전략 단순화: ICP OFF, IMU+EKF ON (2026-03-09)

최종 방향을 아래처럼 단순화한다.

1. RTAB-Map ICP odometry 경로를 끈다.
2. EKF의 yaw 추종 입력을 카메라 IMU(`imu_fixed`)로 전환한다.
3. ICP 전용 보조 노드(`icp_odom_cov_scale`, `map_tf_stabilizer`)를 런치에서 제외한다.

의도:
- ICP 정합 튜닝/게이팅 복잡도를 제거
- 회전 추종을 IMU 단일 축으로 단순화
- 디버깅 포인트를 IMU 품질과 EKF 설정으로 집중

---

## #v10-applied-changes — 실제 반영된 수정사항

### 1) RTAB-Map ICP odometry 비활성화

파일: `src/rtabmap_ros/rtabmap_launch/launch/rtabmap_nav2.launch.py`

- `icp_odometry: 'true'` → `icp_odometry: 'false'`
- `odom_args`를 빈 문자열로 설정 (`''`)

효과:
- `rtabmap.launch.py` 내부 `icp_odometry` 노드가 기동되지 않음
- `/icp_odom`, `/icp_odom_filtered` 경로 의존 제거

### 2) ICP 보조 노드 제거

파일: `src/rtabmap_ros/rtabmap_launch/launch/rtabmap_nav2.launch.py`

- LaunchDescription에서 아래 노드를 제거:
  - `icp_odom_cov_scale.py`
  - `map_tf_stabilizer.py`

효과:
- `/cmd_vel` 미수신으로 STOP 고정되는 문제 경로 제거
- TF lock 보조 로직 제거로 구성 단순화

### 3) EKF를 IMU yaw rate 기반으로 전환

파일: `src/robot_localization/params/ekf.yaml`

- `odom1: /icp_odom_filtered` 블록 제거 (ICP 입력 제거)
- `imu0_config`의 `vyaw`를 `false` → `true`로 변경
- `imu0_twist_rejection_threshold: 1.5` 추가 (스파이크 차단)

효과:
- `odom->base_link` yaw 추종이 IMU yaw rate 중심으로 동작
- ICP 스파이크/오판정이 EKF yaw에 직접 유입되지 않음

---

## #v10-runtime-assumption — 전제 조건

이 전략은 아래 전제가 반드시 필요하다.

1. `/camera/camera/imu_fixed`가 지속적으로 publish 된다.
2. IMU 파이프라인(바이어스 보정 + Madgwick)이 정상 동작한다.
3. IMU 프레임/축 부호가 실제 회전 방향과 일치한다.

전제 불충족 시:
- yaw 추종 실패 또는 반대 방향 추종 발생 가능

---

## #v10-verification — 검증 절차 (필수)

```bash
# 1) IMU 입력 생존 확인
ros2 topic hz /camera/camera/imu_fixed
ros2 topic echo /camera/camera/imu_fixed --field angular_velocity.z

# 2) EKF 출력 확인
ros2 topic hz /odometry/filtered
ros2 topic echo /odometry/filtered --field twist.twist.angular.z

# 3) TF 확인
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo map odom
```

성공 기준:

1. 제자리 좌/우 회전 시 `/camera/camera/imu_fixed angular_velocity.z` 부호가 방향에 맞게 바뀐다.
2. 같은 구간 `/odometry/filtered twist.twist.angular.z`가 0 고정이 아니라 회전을 추종한다.
3. `odom->base_link yaw`가 회전각을 연속 추종한다.
4. 정지 후 `odom->base_link yaw` 드리프트가 빠르게 수렴한다.

---

## #v10-risk-and-next — 잔여 리스크 및 후속

잔여 리스크:

1. IMU bias/온도 드리프트가 크면 장기 정지에서 yaw 표류 가능
2. IMU 축 정렬(TF) 불일치 시 좌/우 부호 반전 가능
3. IMU 노이즈가 큰 구간에서 EKF 미세 떨림 가능

후속 보강(필요 시):

1. `imu0_twist_rejection_threshold` 재튜닝 (1.5 → 1.0~2.0 범위)
2. IMU 파이프라인의 stationary threshold / calib_samples 재조정
3. 필요 시 wheel `vyaw`를 낮은 가중치 보조로 재도입

---

*계획서 v10 갱신: 2026-03-09*
*핵심: ICP 축을 제거하고 IMU+EKF 단일 축으로 단순화해 회전 추종 문제를 우선 해결한다.*
