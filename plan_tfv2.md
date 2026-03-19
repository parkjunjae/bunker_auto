# plan_tfv2.md - 제자리 회전 시 local yaw mismatch 문제 원인 재판정 및 해결 계획

## 0. 목적

이 문서는 `plan_tfv1.md`를 대체하는 2차 계획 문서다.

이번 버전의 핵심은 하나다.

이 문서는 더 이상 "yaw 과대회전"을 전제로 하지 않는다.
2026-03-18 기준 실차 관찰에 더해 `Phase A/A-2` 계측까지 반영하면,
지금 가장 강한 1차 이상 신호는
`raw map` 자체보다 **IMU 입력과 EKF 출력 사이의 local yaw mismatch**다.

즉 이제 우리가 풀어야 할 질문은 다음처럼 바뀐다.

1. `raw /rtabmap/map`이 실제로 도는가?
2. 아니면 local yaw stack 안에서 IMU와 EKF가 같은 회전을 다른 크기로 보고 있는가?
3. stable map relay의 경고와 공백은 1차 원인인가, 아니면 별도 2차 문제인가?

이 문서는 위 세 질문에 대해:

1. 실차 관찰
2. 실제 코드베이스 구조
3. 그 둘을 합친 원인 우선순위
4. 구현 순서

를 아주 명확하게 재정리한다.

추가로, 새 진단 로그 조회는 이전과 동일하게 **offset paging이 아니라 input/cursor 기반 paging**으로 설계한다.

---

## 1. 2026-03-18 실차 관찰 반영 재판정

이번 업데이트는 네가 직접 확인한 세 가지 관찰을 가장 중요한 근거로 사용한다.

### 1.1 모드 A 관찰

조건:

1. `Fixed Frame = map`
2. `Map topic = /rtabmap/map`
3. 첫 번째 사진 기준

관찰:

1. 맵 자체가 크게 드리프트하거나 고스팅되지는 않는다.
2. 그런데 TF 축 기준 로봇 heading이 실제 로봇보다 왼쪽으로 더 돌아가 있다.
3. 즉 "벽/맵이 크게 흔들린다"기보다 "로봇 축이 더 많이 회전한 것처럼 보인다"가 핵심이다.

이 관찰이 중요한 이유:

`Fixed Frame = map`이면, RViz는 `map`을 기준으로 세상을 고정해서 본다.
이 상태에서 occupancy 벽면이 비교적 안정적인데 로봇 축만 더 많이 돌아가 보인다면,
문제의 중심은 `map` raster보다 **`map` 아래쪽 체인인 `odom -> base_link` heading 추정**일 가능성이 높다.

쉽게 말하면:

- 맵이 도는 그림이 아니라
- 맵 위에서 로봇이 실제보다 더 돈 그림

에 가깝다.

이건 `raw occupancy redraw` 1순위 가설을 약하게 만든다.

### 1.2 모드 B 관찰

관찰:

1. `/rtabmap/grid_prob_map`
2. `/rtabmap/octomap_grid`

토픽밖에 보이지 않아 원래 의도했던 `graph / cloud` 비교 테스트를 수행하지 못했다.

이 관찰의 의미:

1. `raw graph/point cloud` 문제라고 결론내릴 수는 없다.
2. 반대로 `raw graph/point cloud`가 멀쩡하다고 결론내릴 수도 없다.
3. 즉 모드 B는 **증거 부재** 상태다.

따라서 현재 시점에서 모드 B는 원인 판정의 주 근거가 아니라,
나중에 3차 검증 대상으로 남겨 두는 것이 맞다.

### 1.3 모드 C 관찰

조건:

1. `Fixed Frame = map_stable`
2. 두 번째 사진 기준

관찰:

1. 전진/후진이 없으면 `/rtabmap/map` 또는 `/rtabmap/map_stable`이 바로 잘 그려지지 않는다.
2. RViz `Map` 디스플레이에 경고가 뜬다.
3. 제자리 회전 시 고스팅은 없지만, 그래도 실제 로봇보다 조금 더 돌아간다.

이 관찰의 의미는 둘로 나뉜다.

첫째, `map_stable` 경고와 빈 화면 문제는 **publish/QoS/갱신 타이밍 문제**일 가능성이 높다.

둘째, 고스팅이 없는데도 로봇이 조금 더 돌아간다면,
이 역시 `raw map redraw`보다 **heading 과대회전** 쪽 설명력이 더 높다.

즉 모드 C는 "stable layer가 문제의 본체다"를 보여주기보다,

1. stable layer 평가 자체가 아직 깔끔하지 않고
2. 그래도 남는 증상은 heading 과대회전 쪽과 더 잘 맞는다

를 보여준다.

### 1.4 이번 관찰로 바뀐 핵심 판단

여기까지는 어디까지나 **계측 전 임시 판단**이었다.

즉 이 시점의 문서는:

1. RViz 시각 관찰
2. 코드 구조

를 합쳐서

- "`odom -> base_link` yaw 과대회전이 1차 원인일 수 있다"

고 가설을 세운 상태였다.

하지만 이 가설은 정량 계측이 들어오면 언제든 뒤집혀야 한다.
그리고 2026-03-18 Phase A/A-2 결과는 실제로 그 역할을 했다.

### 1.5 첫 번째 Phase A/A-2 실측 런이 보여준 것

첫 번째 계측 런:

- `logs/run_20260318_105521/spin_yaw_probe.jsonl`
- `logs/run_20260318_105521/spin_yaw_probe.summary.json`
- `logs/run_20260318_105521/rtabmap_nav2.log`
- `logs/run_20260318_105521/sensor_sync.log`

에서 확인된 사실은 아래와 같다.

#### 1.5.1 `spin_yaw_probe`가 잡은 pure spin 세션은 2개다

세션 1:

1. duration `7.755005 s`
2. `imu_delta_yaw_raw_deg = +178.467`
3. `odom_delta_yaw_deg = +61.393`
4. `ratio_odom_over_imu_raw = 0.344001`

세션 2:

1. duration `7.849499 s`
2. `imu_delta_yaw_raw_deg = -179.100`
3. `odom_delta_yaw_deg = -78.967`
4. `ratio_odom_over_imu_raw = 0.440912`

즉 같은 pure spin 동안:

1. IMU 적분 yaw는 거의 `+/-180 deg`
2. `/odometry/filtered` pose yaw는 `+61 deg`, `-79 deg`

만 변했다.

이건 아주 강한 신호다.

**적어도 이번 런에서는 `IMU 입력`과 `EKF 출력`이 같은 회전을 전혀 같은 크기로 보지 않는다.**

#### 1.5.2 이 수치는 "과회전"보다 "yaw scale mismatch"를 더 강하게 가리킨다

이전 가설은:

- "`odometry/filtered`가 실제보다 더 많이 돈다"

였다.

그런데 이번 계측은 그 반대 그림을 보여준다.

정확히는:

1. IMU는 거의 `180 deg`
2. `/odometry/filtered`는 그중 `34% ~ 44%` 수준만 따라간다

이다.

이 사실 하나만으로도

- "`odom -> base_link` yaw 과대회전"

가 현재 1차 결론이 될 수는 없다.

이번 런이 증명한 것은:

**local yaw stack 내부에 큰 scale mismatch가 존재한다**

는 점이다.

다만 주의할 점도 있다.

이 probe는:

1. IMU와 EKF의 상대 관계는 강하게 증명하지만
2. "어느 쪽이 실제 로봇 회전에 더 가깝냐"는 외부 ground truth 없이 단정하지는 못한다

즉 현재 단계에서 확정할 수 있는 것은:

- "IMU와 EKF가 심하게 다르다"

이지,

- "무조건 IMU가 맞다" 또는 "무조건 EKF가 맞다"

는 아니다.

#### 1.5.3 같은 런의 `sensor_sync.log`는 IMU spin 감지가 살아 있음을 보여준다

`sensor_sync.log`:

1. `1773798999.455301782`에 `yaw_zeroing=OFF reason=hard_wz`
2. `1773799007.868433327`에 `yaw_zeroing=ON reason=steady_hold`
3. `1773799009.391175050`에 다시 `yaw_zeroing=OFF reason=hard_wz`
4. `1773799018.213687587`에 `yaw_zeroing=ON reason=steady_hold`

즉 두 세션 모두:

1. 회전 시작 시 IMU yaw lock은 정상 해제됐고
2. 회전 종료 후 다시 정상 잠겼다

는 뜻이다.

이건 중요한 소거 결과다.

이번 런에서는:

- "IMU가 spin 자체를 놓쳤다"

는 설명이 맞지 않는다.

#### 1.5.4 같은 런의 `rtabmap_nav2.log`는 map-level TF 회전을 거의 보여주지 않는다

첫 번째 세션 진입 구간과 두 번째 세션 진입 구간에서
`map_tf_stabilizer` 로그를 보면:

1. `raw_yaw=0.000`
2. `stable_map_yaw=0.000` 또는 `-0.000`

가 유지된다.

세션 종료 직전까지도 마찬가지다.

즉 이 런에서는 최소한:

- raw `map -> odom` yaw spike가 회전 주범이었다

는 그림이 보이지 않는다.

다시 말해,

**이번 계측 런에서는 map-level TF보다 local yaw mismatch가 훨씬 더 강한 이상 신호다.**

#### 1.5.5 `map_topic_stabilizer` secondary issue는 실제로 존재한다

같은 로그에서:

1. 두 spin 세션 모두 `hold_enter`가 발생한다.
2. 세션 종료 후 translation이 없어서 release가 바로 일어나지 않는다.
3. 두 번째 세션 후에는 `hold_exit reason=timeout`과 `pending=1`이 보인다.

즉 이전 문서에서 별도 문제로 본:

- stable map 경고 / pending refresh / no-translation 갱신 정체

는 실제로 존재한다.

하지만 이것도 여전히 2차 문제다.
이번 런의 가장 강한 숫자는 map 문제가 아니라 yaw mismatch 쪽에 있다.

### 1.6 최신 재실험 런은 `pose/TF 분리`까지 포함해 같은 결론을 더 강하게 만들었다

최신 런:

- `logs/run_20260318_131836/spin_yaw_probe.jsonl`
- `logs/run_20260318_131836/spin_yaw_probe.summary.json`
- `logs/run_20260318_131836/rtabmap_nav2.log`
- `logs/run_20260318_131836/sensor_sync.log`

이 런은 사용자가 명시한 절차,

1. 제자리 좌회전 `180 deg`
2. 잠시 정지
3. 제자리 우회전 `180 deg`

를 기준으로 수행한 최신 확인 런이다.

#### 1.6.1 최신 런의 숫자 자체는 더 강하다

세션 1, 좌회전:

1. `imu_delta_yaw_raw_deg = +176.305`
2. `odom_delta_yaw_deg = +45.862`
3. `tf_delta_yaw_deg = +45.862`
4. `ratio_odom_over_imu_raw = 0.260128`

세션 2, 우회전:

1. `imu_delta_yaw_raw_deg = -176.603`
2. `odom_delta_yaw_deg = -62.565`
3. `tf_delta_yaw_deg = -62.565`
4. `ratio_odom_over_imu_raw = 0.354271`

즉 최신 런에서는:

1. IMU 적분 yaw는 두 세션 모두 거의 `+/-176 deg`
2. `/odometry/filtered.pose`와 실제 TF `odom -> base_link`는 `+45.9 deg`, `-62.6 deg`
3. 그리고 pose와 TF는 서로 사실상 같은 값을 말한다

#### 1.6.2 이 런으로 `publish path` 가설은 크게 약해졌다

이번 probe는 `/odometry/filtered.pose`와 TF `odom -> base_link`를 동시에 기록했다.

결과:

1. 세션 1 `pose_minus_tf_delta_yaw_deg = -0.0`
2. 세션 2 `pose_minus_tf_delta_yaw_deg = 0.0`
3. 평균 pose-TF yaw 차는 `0.048 deg`, `0.002 deg` 수준이다
4. 평균 pose-TF 위치 차도 `7e-06 m`, `3e-06 m` 수준이다

이건 의미가 아주 분명하다.

**이번 최신 런에서는 `/odometry/filtered.pose`와 TF `odom -> base_link`가 서로 다른 경로를 말하고 있는 것이 아니다.**

즉:

- "EKF pose는 맞는데 TF publish가 틀렸다"
- "TF consumer 경로에서만 각도가 망가진다"

같은 설명은 이번 런 기준으로는 주원인이 아니다.

#### 1.6.3 최신 런은 local yaw mismatch가 `과대회전`이 아니라 `과소추종`임을 더 분명히 보여준다

세션 1 내부 sample을 보면 특히 패턴이 눈에 띈다.

1. 세션 중간 시점에 IMU 누적 yaw는 이미 약 `+155 deg`
2. 같은 시점의 odom/TF yaw는 약 `+1 deg`
3. 그 뒤 후반에 odom/TF yaw가 약 `48 deg` 수준 jump처럼 반영된다

세션 2도 형태는 덜 극단적이지만 같은 방향이다.

1. 중간 시점 IMU 누적 yaw는 약 `-170 deg`
2. 같은 시점 odom/TF yaw는 약 `-61 deg`

즉 현재 패턴은:

1. IMU가 계속 회전을 보고 있고
2. odom/TF가 그 회전을 실시간으로 충분히 따라가지 못하고
3. 특히 좌회전에서는 더 심한 지연/저응답이 보인다

에 가깝다.

이건 "`odom yaw 과대회전`"과는 반대 방향의 증거다.

#### 1.6.4 최신 런에서도 IMU unlock은 살아 있고 map-level TF는 조용하다

`sensor_sync.log`:

1. `1773807600.008305006`에 `yaw_zeroing=OFF reason=hard_wz`
2. `1773807607.856254715`에 `yaw_zeroing=ON reason=steady_hold`
3. `1773807608.610631470`에 `yaw_zeroing=OFF reason=yaw_evidence`
4. `1773807617.626530663`에 `yaw_zeroing=ON reason=steady_hold`

즉 회전 시작/종료에 맞춰 IMU yaw lock 상태기계는 여전히 정상 반응한다.

또 `rtabmap_nav2.log`에서는 세션 진입/종료 구간에도:

1. `map_tf_stabilizer raw_yaw=0.000`
2. `stable_map_yaw=0.000` 또는 `-0.000`

이 유지된다.

따라서 최신 런도:

1. map-level TF spike가 본체라는 설명보다
2. local yaw estimate 자체가 작게 나오는 설명

을 훨씬 더 강하게 지지한다.

#### 1.6.5 latest rerun이 바꾼 핵심 결론

이제 최신 런까지 포함하면 다음 세 가지는 거의 확정에 가깝다.

1. 1차 문제는 `map -> odom` spike가 아니라 local yaw stack이다
2. 그 local yaw stack 안에서도 `publish path`보다 EKF local yaw estimate 자체가 더 의심스럽다
3. 현재 가장 자연스러운 이름은 "`IMU 대비 EKF yaw under-response / under-fusion`"이다

즉 이제 다음 단계의 초점은:

- `yaw_rate_scale`을 먼저 넣는 것이 아니라
- `robot_localization`이 IMU `vyaw`를 왜 이렇게 작게 반영하는지 확인하는 것

으로 좁혀진다.

---

## 2. 실제 코드베이스에서 확인한 구조적 사실

아래 내용은 실제 소스를 읽고 정리한 것이다.

### 2.1 EKF는 현재 heading을 사실상 IMU z-rate 하나에 크게 의존한다

파일:

- `src/robot_localization/params/ekf.yaml`

핵심 설정:

1. `world_frame: odom`
2. `publish_tf: true`
3. 따라서 EKF가 `odom -> base_link` TF를 직접 publish한다.

그리고 센서 사용 방식이 중요하다.

`odom0_config`:

1. wheel odom의 `vx`만 사용한다.
2. wheel `yaw`는 꺼져 있다.
3. wheel `vyaw`도 꺼져 있다.

`imu0_config`:

1. IMU에서 `vyaw`만 사용한다.
2. IMU `yaw orientation`은 사용하지 않는다.

즉 현재 heading 체인은 사실상 이렇게 읽는 게 맞다.

1. 로컬 위치는 wheel `vx`로 조금 돕는다.
2. 하지만 회전 heading은 거의 IMU z축 각속도 적분이 주도한다.
3. 회전량을 되돌려 줄 두 번째 heading anchor가 거의 없다.

이 구조에서 중요한 결론은 더 일반적이다.

IMU z-rate와 EKF yaw 출력 사이에:

1. scale mismatch
2. timing mismatch
3. fusion attenuation
4. transform error

중 하나라도 있으면,
`odom -> base_link` yaw는 실제보다 더 크거나 더 작게 누적될 수 있다.

2026-03-18 Phase A 결과는 바로 이 지점,
즉 **IMU와 EKF 출력 사이의 큰 yaw scale mismatch**를 보여줬다.

### 2.2 IMU 보정 노드는 z-rate를 `base_link` 기준으로 돌린 뒤 그대로 EKF에 넘긴다

파일:

- `src/rtabmap_ros/rtabmap_launch/launch/sensor_sync.launch.py`
- `src/camera_imu_pipeline_cpp/src/camera_imu_bias_corrector.cpp`

확인된 사실:

1. `sensor_sync.launch.py`에서 `camera_imu_bias_corrector`의 `target_frame`은 `base_link`다.
2. `camera_imu_bias_corrector.cpp`는 raw IMU 각속도/가속도를 `target_frame`으로 회전시킨다.
3. bias가 준비되면 bias를 뺀 뒤 `angular_velocity.z`를 내보낸다.
4. yaw lock이 걸린 정지 구간에서는 `angular_velocity.z = 0.0`으로 clamp한다.
5. 하지만 실제 회전으로 판단되면 lock을 풀고 z-rate를 그대로 내보낸다.

즉 실제 회전 중에는 결국 EKF가 받는 값이:

- "base_link 기준으로 회전된 뒤 bias가 빠진 IMU z-rate"

이다.

이 경로의 함의는 명확하다.

1. z-rate bias가 조금 남아 있거나
2. 스케일이 조금 크거나
3. IMU 프레임에서 `base_link`로 회전할 때 roll/pitch 성분이 틀려서 x/y rate가 z로 섞이거나
4. 저속/가속 구간에서 bias 추적이나 unlock 타이밍이 완전히 맞지 않으면

그 오차는 결국 `odom -> base_link` yaw mismatch로 들어간다.

그리고 이번 실측에서는 그 mismatch가 "과대회전"이라기보다
"IMU 대비 EKF 저응답/과소추종" 형태로 나타났다.

### 2.3 Madgwick가 있어도 현재 설정에서는 절대 yaw anchor가 생기지 않는다

파일:

- `src/rtabmap_ros/rtabmap_launch/launch/sensor_sync.launch.py`

확인된 사실:

1. `imu_filter_madgwick`를 사용한다.
2. 그러나 `use_mag: False`다.
3. `zeta: 0.0`으로 gyro drift correction도 끈 상태다.

이 말은 중요하다.

`/camera/camera/imu_fixed`에 orientation이 있더라도,
현재 구조에서 그 yaw는 절대 북쪽 기준 yaw가 아니다.
즉 "Madgwick orientation을 EKF에 그냥 넣으면 heading anchor가 생긴다"는 식의 기대는 맞지 않다.

지금 구조에서 yaw는 여전히 gyro 기반 상대 회전에 가깝다.

그래서 현재 문제를 풀려면 먼저:

1. IMU와 EKF 중 어느 쪽이 실제 회전에 더 가까운지
2. mismatch가 IMU 과대인지, EKF 과소인지
3. 그 원인이 scale, bias, frame transform, fusion timing 중 무엇인지

를 봐야 한다.

### 2.4 `map_tf_stabilizer`와 `map_topic_stabilizer`는 `map` 위 레이어를 다룬다

파일:

- `src/rtabmap_ros/rtabmap_launch/scripts/map_tf_stabilizer.py`
- `src/rtabmap_ros/rtabmap_launch/scripts/map_topic_stabilizer.py`

`map_tf_stabilizer.py` 헤더에 명시된 체인:

1. raw TF: `map -> odom`
2. EKF local TF: `odom -> base_link`
3. filtered TF: `map_stable -> map`

즉 전체 체인은:

`map_stable -> map -> odom -> base_link`

이다.

이건 아주 중요하다.

만약 진짜 문제가 local yaw stack 내부 mismatch라면:

1. `map_tf_stabilizer`는 global frame을 부드럽게 보이게 할 수는 있다.
2. `map_topic_stabilizer`는 consumer가 안정된 occupancy를 보게 할 수는 있다.
3. 하지만 둘 다 **잘못된 `odom -> base_link` 자체를 고치지는 못한다.**

즉 stabilizer는 현재 1차 원인 해결층이 아니라,
최대해도 2차 완화층이다.

### 2.5 `/rtabmap/map_stable` 경고는 현재 코드상 별도 이유로도 충분히 설명된다

파일:

- `src/rtabmap_ros/rtabmap_launch/launch/rtabmap_nav2.launch.py`
- `src/rtabmap_ros/rtabmap_launch/scripts/map_topic_stabilizer.py`
- `src/rtabmap_ros/rtabmap_util/src/MapsManager.cpp`

확인된 사실:

1. launch에서 `latch = false`다.
2. RTAB-Map raw `/rtabmap/map` publisher는 이 경우 `VOLATILE` durability를 쓴다.
3. 그런데 현재 `map_topic_stabilizer.py`는 입력 `/rtabmap/map`를 `TRANSIENT_LOCAL`로 구독한다.

이 구조에서는 두 가지 문제가 생길 수 있다.

첫째, relay가 raw map을 아예 못 받을 수 있다.

둘째, late subscriber가 마지막 map을 자동으로 못 받을 수 있다.

즉 RViz에서 `/rtabmap/map_stable` 또는 `/rtabmap/map`가:

1. 전진/후진 전에는 안 보이고
2. 실제 새 publish가 들어와야만 다시 보이거나
3. 경고가 뜨는 현상

은 지금 QoS/publish 조건만으로도 충분히 설명된다.

그래서 모드 C 경고는 "local yaw mismatch의 본체"라기보다,
**stable map path가 아직 진단 불가능한 상태**라는 신호에 가깝다.

### 2.6 RTAB-Map redraw 가능성은 남아 있지만, 현재는 더 뒤로 밀린다

파일:

- `src/rtabmap_ros/rtabmap_util/src/MapsManager.cpp`

기존에 확인한 사실은 여전히 유효하다.

1. occupancy grid publish는 `AngularUpdate`와 완전히 같은 경로가 아니다.
2. `fullUpdateNeeded(filteredPoses)`, `update(filteredPoses)`, `gridUpdated_` 경로가 따로 있다.

따라서 "redraw 가능성" 자체는 아직 살아 있다.

하지만 이번 실차 관찰과 Phase A 계측을 합치면:

1. `map_tf_stabilizer`의 `raw_yaw`는 세션 중 `0.000` 근처에 머물렀고
2. local yaw stack에서는 IMU와 EKF가 큰 차이를 보였다

따라서 redraw는 이제:

1. "남아는 있는 가능성"
2. 그러나 "지금 가장 먼저 파야 할 대상은 아님"

으로 더 명확히 밀려난다.

---

## 3. 현재 가장 타당한 원인 판단

### 3.1 1차 원인 후보: IMU 입력과 EKF 출력 사이의 local yaw scale mismatch

Phase A/A-2 이후의 1차 결론은 바뀌었다.

지금 가장 타당한 설명은:

**"local yaw stack 내부에서 IMU와 EKF가 같은 회전을 전혀 같은 크기로 보지 않는다"**

이다.

수치로 보면:

1. 세션 1에서 IMU는 `+178.467 deg`, odom은 `+61.393 deg`
2. 세션 2에서 IMU는 `-179.100 deg`, odom은 `-78.967 deg`
3. `odom / imu` 비율은 `0.344`, `0.441`

즉 이 런에서는 local yaw가 "너무 많이 돈다"가 아니라
"어느 단계에서 크게 줄어들거나, 반대로 IMU 쪽이 크게 부풀려져 있다"가 더 정확하다.

이 결론의 장점은 두 가지다.

1. 실제 계측값과 직접 맞는다.
2. 아직 ground truth가 없는 상태에서도 과도한 단정을 피한다.

현재 문서가 확정적으로 말할 수 있는 것은:

- `odometry/filtered` yaw와 IMU yaw가 심하게 다르다

이지,

- 무조건 EKF가 과소다
- 무조건 IMU가 과대다

까지는 아니다.

하지만 적어도 한 가지는 분명하다.

**이번 런은 이전의 "odom yaw 과대회전" 가설을 지지하지 않는다.**

### 3.2 2차 원인 후보: raw map publish 부족 + stable relay QoS 문제

이건 별도의 2차 문제로 남고, 이번 런에서 실제로도 관측됐다.

설명 가능한 증상:

1. 전진/후진이 없으면 `/rtabmap/map` 또는 `/rtabmap/map_stable`가 잘 안 보임
2. RViz `Map` 경고 발생
3. stable map 실험 자체가 일관되게 재현되지 않음
4. `MapTopicStabilizer`가 spin 후 `pending=1`로 남고 translation 없이 refresh하지 못함

하지만 이 문제만으로는:

- `IMU vs EKF yaw mismatch`

를 설명하기 어렵다.

즉 이건 1차 원인이 아니라,
**진단을 방해하는 2차 문제**다.

### 3.3 3차 원인 후보: raw occupancy redraw 또는 graph/cloud artifact

이 후보를 완전히 버리지는 않는다.

다만 현재 증거 수준은 이렇다.

1. 소스상으로는 가능하다.
2. 하지만 이번 실측 런에서는 `raw_yaw`가 거의 0이었다.
3. 모드 B는 여전히 실제 테스트가 성립하지 않았다.

그래서 지금 redraw/graph를 먼저 파면,
주요 원인을 우회할 가능성이 높다.

### 3.4 한 줄 요약

지금 이 문제의 가장 유력한 본체는:

**"맵이 돈다" 자체보다 "IMU 입력과 EKF 출력이 pure spin에서 같은 yaw를 보지 않는다"** 이다.

---

## 4. 증상별 해석표

이 표는 앞으로 관찰 결과를 어떻게 해석할지 기준을 주기 위한 것이다.

### 4.1 해석 규칙

1. `Fixed Frame = map`에서 벽이 안정적이고 local yaw probe에서 `odom_delta_yaw / imu_delta_yaw`가 크게 벗어나면  
   1차 문제는 map redraw보다 local yaw stack mismatch 쪽이다.

2. 같은 런에서 `map_tf_stabilizer raw_yaw`가 거의 0이고 `stable_map_yaw`도 거의 0이면  
   map-level TF spike는 우선순위가 내려간다.

3. `Fixed Frame = map`에서 벽 자체가 같이 회전하거나 밀리면  
   `raw /rtabmap/map` redraw 또는 `map -> odom` 쪽을 더 의심한다.

4. `Fixed Frame = map_stable`에서만 경고가 뜨거나 map이 안 보이면  
   stable relay/QoS/publish 문제를 먼저 본다.

5. `map`에서는 멀쩡하고 Nav2 global costmap에서만 이상하면  
   consumer layer 또는 obstacle/static layer 조합 문제를 본다.

### 4.2 현재 네 관찰을 위 규칙에 대입한 결과

2026-03-18 기준으로는 관찰만이 아니라 계측 결과까지 같이 넣어야 한다.

현재 결과는 이렇게 읽는 것이 맞다.

1. local yaw stack 내부 mismatch는 **확인됨**
2. map-level TF spike는 **이번 런에서는 증거 약함**
3. stable map path secondary issue는 **확인됨**
4. redraw/graph는 **여전히 보류**

따라서 현재 우선순위는:

1. local yaw mismatch에서 어느 쪽이 실제와 다른지 결정
2. stable map path/QoS 문제 정리
3. redraw/graph 재평가

이다.

---

## 5. 해결 전략

이제 구현 순서도 다시 바뀌어야 한다.

중요한 점은:

1. `Phase A`와 `Phase A-2`는 이미 구현되었고
2. 그 결과가 이전 가설을 수정했다

는 것이다.

따라서 다음 단계는:

- "`yaw 과대회전`을 더 증명하는 단계"

가 아니라

- "`IMU와 EKF 중 어느 쪽이 실제와 다른지 가르는 단계`"

가 되어야 한다.

### 5.1 Phase A - 완료, 그리고 local yaw mismatch를 수치화했다

구현된 스크립트:

- `src/rtabmap_ros/rtabmap_launch/scripts/spin_yaw_probe.py`

이 노드는 실제로 아래를 기록했다.

1. `/odometry/filtered`를 구독한다.
2. `/camera/camera/imu_fixed`를 구독한다.
3. pure spin 세션을 자동 분리한다.
4. 세션별 `odom_delta_yaw`, `imu_delta_yaw_raw`, `ratio_odom_over_imu_raw`를 남긴다.
5. append-only JSONL과 summary JSON을 만든다.

이번 런의 결과는 이미 정리됐다.

1. 세션 1: `ratio_odom_over_imu_raw = 0.344`
2. 세션 2: `ratio_odom_over_imu_raw = 0.441`

즉 Phase A의 역할은 이미 끝났고,
지금은 그 결과를 해석하는 단계다.

### 5.2 Phase A-2 - 완료, cursor 기반 조회로 세션 증거를 읽었다

구현된 도구:

- `tools/map_event_query.py`

이 도구로 실제 확인한 핵심은 다음이다.

1. `after_seq`, `after_stamp_ns`, `trial_id`, `session_id` 기반으로 세션을 clean하게 잘라볼 수 있었다.
2. `spin_exit` summary만 봐도 local yaw mismatch가 바로 드러났다.
3. 사용자가 조회한 `after_seq 120` 구간은 단순 idle sample이었다.
   즉 A-2에서는 "원하는 session/event 종류를 정확히 고르는 것"이 중요하다.

이 도구는 이후에도 그대로 유지한다.

### 5.3 Phase B - 최신 런까지 반영하면 질문이 더 좁아진다

Phase A/A-2가 끝난 지금,
Phase B의 질문은 더 구체적이어야 한다.

현재 가장 중요한 질문은 세 가지다.

1. 실제 로봇 회전에 더 가까운 쪽이 IMU인가, EKF인가
2. `/odometry/filtered.pose`와 실제 `odom -> base_link` TF는 같은 yaw를 말하고 있는가
3. mismatch의 원인이 IMU 과대인지, EKF 과소인지, 둘 다인지

#### Phase B-1: ground truth를 꼭 붙인다

최신 런은 사용자가:

1. 좌 `180 deg`
2. 잠시 정지
3. 우 `180 deg`

순서로 수행했다고 명시했다.

이건 방향성을 잡는 데 충분히 강한 실험 정보다.
다만 앞으로 수정 근거로 쓰려면 여전히 아래 수준의 ground truth를 유지하는 것이 좋다.

1. 바닥 마킹 기준 `정확히 90도 / 180도` 회전
2. trial id에 `left_180`, `right_180`처럼 명시
3. 가능하면 육안이 아니라 기준선으로 실제 각도 확인

이게 있어야:

1. IMU가 맞는지
2. EKF가 맞는지

를 가를 수 있다.

#### Phase B-2: `/odometry/filtered` pose와 TF를 분리해서 본다

이 단계는 이미 구현했고, 최신 런에서 실제로 확인했다.

1. `/odometry/filtered.pose.pose.orientation`
2. 실제 TF `odom -> base_link`

최신 `run_20260318_131836` 결과:

1. 좌회전 `pose_minus_tf_delta_yaw_deg = -0.0`
2. 우회전 `pose_minus_tf_delta_yaw_deg = 0.0`
3. pose-TF 평균 yaw 차는 사실상 `0 deg`

즉 현재 결론은 이미 나왔다.

- 이번 문제는 publish/consumption path보다 **local yaw estimate 자체**를 먼저 봐야 한다.

#### Phase B-3: 분기 기준

최신 런까지 반영하면 분기표는 아래처럼 읽는 것이 맞다.

1. 실제 회전이 `180 deg`에 가깝고 IMU도 `180 deg`에 가깝다면  
   EKF yaw under-fusion / under-response를 먼저 본다.

2. 실제 회전이 `60 ~ 80 deg`에 가깝고 odom도 그 근처라면  
   IMU z-rate over-scale / transform / bias를 먼저 본다.

3. 실제 회전은 `180 deg`인데 IMU와 odom 둘 다 틀리면  
   IMU와 EKF를 둘 다 다시 봐야 한다.

4. pose와 TF가 서로 다르면  
   local estimate보다 publish/consumption path가 먼저다.

현재 latest rerun은 사용자가 명시한 회전 절차를 기준으로 보면:

1. IMU는 `+176.305 deg`, `-176.603 deg`로 거의 `180 deg`
2. odom/TF는 `+45.862 deg`, `-62.565 deg`
3. pose와 TF는 사실상 같다

따라서 **현재 가장 유력한 분기는 1번, 즉 EKF yaw under-response / under-fusion**이다.

#### Phase B-4: 코드 수정 우선순위

ground truth를 얻기 전에는 `yaw_rate_scale`을 바로 넣는 것이 아니다.

그보다 먼저:

1. `spin_yaw_probe.py`를 TF까지 기록하도록 확장하거나
2. 기존 `tools/ekf_yaw_probe.py`를 병행해 wheel/IMU/EKF 비교를 하고
3. `robot_localization` debug를 짧은 한 런에만 켜서 실제 fusion 경로를 확인해야 한다

현재 상태:

1. `spin_yaw_probe.py`는 이미 `odom -> base_link` TF yaw/xy와 `/odometry/filtered.pose` yaw/xy를 함께 기록하도록 확장했다.
2. 최신 `run_20260318_131836`에서 pose yaw, TF yaw, IMU 적분 yaw를 한 번에 비교했고, pose와 TF가 사실상 같음을 확인했다.
3. `robot_localization/launch/ekf.launch.py`와 `run_all.sh`는 이제 opt-in debug를 지원하며, debug 출력 파일을 같은 run 디렉터리에 남길 수 있다.
4. 따라서 다음 우선순위는 ground truth를 더 반복 확보하면서 `robot_localization` debug로 fusion 경로를 확인하는 것이다.

그 다음에만 아래 수정이 의미가 있다.

1. IMU가 실제보다 크다고 확인되면  
   `camera_imu_bias_corrector.cpp` 쪽 `yaw_rate_scale` 또는 frame/bias 보정

2. EKF가 IMU를 지나치게 작게 반영한다고 확인되면  
   `robot_localization` 설정, timestamp, queue, update-rate, debug 분석

3. pose와 TF가 다르면  
   EKF보다 publish 경로를 먼저 수정

### 5.4 Phase B에서 하지 말아야 할 것

이건 명확히 적어 둔다.

1. wheel `vyaw`를 바로 다시 켜지 않는다.  
   현재 주석대로 슬립 때문에 완전히 틀린 값을 만들 수 있다.

2. Madgwick yaw를 "절대 heading"처럼 믿고 바로 EKF에 넣지 않는다.  
   현재 `use_mag=false`라서 절대 anchor가 아니다.

3. 이번 런 결과를 보고도 계속 "`odom yaw 과대회전`이 본체"라고 고집하지 않는다.  
   숫자가 그 가설을 지지하지 않는다.

4. stabilizer를 더 세게 거는 것으로 본체 문제를 덮지 않는다.  
   local yaw mismatch가 틀리면 stabilizer는 본질 해결이 아니다.

### 5.5 Phase C - stable map path는 2차 문제로 정리한다

`map_topic_stabilizer.py`는 버릴 대상은 아니다.
다만 우선순위가 바뀌었다.

이제 이 레이어는:

1. 1차 원인 해결층이 아니라
2. 진단 보조층 + 소비자 보호층

으로 다뤄야 한다.

#### 필요한 수정

수정 파일:

- `src/rtabmap_ros/rtabmap_launch/scripts/map_topic_stabilizer.py`
- `src/rtabmap_ros/rtabmap_launch/launch/rtabmap_nav2.launch.py`

수정 내용:

1. input QoS를 raw `/rtabmap/map`와 맞춘다.
2. output QoS만 `TRANSIENT_LOCAL`로 유지한다.
3. hold/publish/refresh 이벤트를 JSONL로 남긴다.

#### QoS 분리 예시 코드

```python
input_qos = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)

output_qos = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)

self.map_pub = self.create_publisher(OccupancyGrid, self.output_map_topic, output_qos)
self.create_subscription(OccupancyGrid, self.input_map_topic, self._map_cb, input_qos)
```

#### 왜 이 단계가 여전히 필요한가

모드 C 경고는 지금 local yaw mismatch와 별개로,
실험을 흐리게 만들고 있다.

그래서 stable layer는 원인 본체는 아니더라도 반드시 정리해야 한다.

### 5.6 Phase C-2 - `map_publish_probe.py`는 이제 2차 검증 도구다

새 스크립트:

- `src/rtabmap_ros/rtabmap_launch/scripts/map_publish_probe.py`

역할:

1. raw `/rtabmap/map` publish 시각을 기록한다.
2. `origin_x`, `origin_y`, `origin_yaw`, `crc32`를 남긴다.
3. pure spin 동안 실제 map publish가 얼마나 드문지 확인한다.

#### 예시 코드 스니펫

```python
def build_map_event(seq, msg, spin_score, imu_wz, speed):
    stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
    return {
        "seq": seq,
        "stamp_ns": stamp_ns,
        "frame_id": msg.header.frame_id,
        "origin_x": msg.info.origin.position.x,
        "origin_y": msg.info.origin.position.y,
        "origin_yaw": yaw_from_quaternion(msg.info.origin.orientation),
        "width": msg.info.width,
        "height": msg.info.height,
        "resolution": msg.info.resolution,
        "crc32": f"{grid_crc32(msg):08x}",
        "spin_score": round(spin_score, 3),
        "imu_wz": round(imu_wz, 3),
        "speed": round(speed, 3),
    }
```

이 probe의 목적은 이제 더 명확하다.

1. stable map warning이 publish 부족 때문인지 확인
2. local yaw mismatch를 분리한 뒤에도 raw map redraw가 남는지 확인

즉 여전히 2차 확인용이다.

### 5.7 Phase D - local yaw mismatch를 분리한 뒤에도 증상이 남을 때만 redraw/graph로 복귀한다

여기서만 다시 예전 가설을 본다.

조건:

1. local yaw mismatch의 원인을 분리했다.
2. 그런데도 `Fixed Frame = map`에서 벽이 실제로 회전하거나 밀린다.
3. 또는 `crc32/origin` 변화가 pure spin 중 비정상적으로 나타난다.

그때만 다음을 본다.

1. `MapsManager.cpp` publish 경로
2. `raw /rtabmap/mapData`, `/rtabmap/mapGraph` 또는 대응 topic 노출 문제
3. consumer 측 static layer / obstacle layer 조합

즉 redraw/graph는 **조건부 재진입 항목**이다.

---

## 6. 실제 수정 순서 제안

이 순서는 지금 가장 안전하고 설명력이 높다.

### 6.1 1순위

1. 다음 pure spin 런은 `left_180`, `right_180`처럼 ground truth가 읽히게 다시 수행
2. `spin_exit` summary를 먼저 보고 IMU와 odom/TF 중 누가 실제에 가까운지 판단
3. 현재는 `spin_yaw_probe.py`에 TF 기록까지 이미 들어갔으므로, 다음은 `robot_localization` debug를 짧은 런에만 켜서 실제 fusion 경로를 확인
4. 필요 시 `tools/ekf_yaw_probe.py`를 병행해 wheel/IMU/EKF 비교를 보조

이 단계의 목표는:

`"IMU와 EKF가 왜 다르냐"`를 `실제 기준`으로 가르는 것

이다.

### 6.2 2순위

1. 최신 런처럼 IMU가 거의 `180 deg`인데 odom/TF가 `46 ~ 63 deg` 수준이면 `robot_localization` debug와 설정을 먼저 본다
2. 그 뒤에도 IMU가 실제보다 크다고 확인되면 `camera_imu_bias_corrector.cpp`와 `sensor_sync.launch.py` 쪽으로 간다
3. `/odometry/filtered.pose`와 `odom -> base_link` TF가 다를 때만 publish/consumption 경로를 먼저 본다

이 단계의 목표는:

local yaw mismatch가

- IMU 과대인지
- EKF 과소인지
- TF/publish 문제인지

를 분리하는 것이다.

### 6.3 3순위

1. `map_topic_stabilizer.py` input/output QoS 분리
2. `map_publish_probe.py` 추가
3. `map_stable` 경고 원인 분리

이 단계의 목표는:

stable map path를 정상 진단 가능한 상태로 만드는 것

이다.

### 6.4 4순위

1. raw redraw/graph 재평가
2. 필요 시 RViz debug profile 분리
3. 필요 시 Nav2 static-only debug profile 추가

이건 local yaw mismatch를 분리한 뒤에만 들어간다.

---

## 7. 변경 제안 파일 목록

### 필수

1. `src/rtabmap_ros/rtabmap_launch/scripts/spin_yaw_probe.py`
   - local yaw mismatch 정량화
   - 이미 구현됨

2. `tools/map_event_query.py`
   - input/cursor 기반 paging 지원
   - `after_seq`, `after_stamp_ns`, `trial_id` 조회
   - 이미 구현됨

3. `src/rtabmap_ros/rtabmap_launch/launch/rtabmap_nav2.launch.py`
   - `spin_yaw_probe.py` 연결
   - 이미 구현됨

4. `run_all.sh`
   - probe를 같은 run 디렉터리에 자동 기록
   - 이미 구현됨

5. `src/camera_imu_pipeline_cpp/src/camera_imu_bias_corrector.cpp`
   - ground truth가 IMU 과대를 가리킬 때만 수정
   - `yaw_rate_scale` 또는 bias/transform 진단용

6. `src/rtabmap_ros/rtabmap_launch/launch/sensor_sync.launch.py`
   - bias corrector 진단/보정 파라미터 노출

7. `src/robot_localization/params/ekf.yaml`
   - EKF under-fusion이 확인될 때만 재검토

8. `src/rtabmap_ros/rtabmap_launch/scripts/map_topic_stabilizer.py`
   - input/output QoS 분리
   - 이벤트 로깅

9. `src/rtabmap_ros/rtabmap_launch/scripts/map_publish_probe.py`
   - raw map publish 기록

### 권장

10. `tools/ekf_yaw_probe.py`
    - wheel/IMU/EKF 보조 비교용으로 병행 가능

11. `src/rtabmap_ros/rtabmap_launch/launch/config/rgbd_raw_map_only.rviz`
12. `src/rtabmap_ros/rtabmap_launch/launch/config/rgbd_stable_map_only.rviz`
13. `src/rtabmap_ros/rtabmap_launch/launch/config/nav2_rtabmap_params_debug_static_only.yaml`

### 아직 보류

14. `src/rtabmap_ros/rtabmap_util/src/MapsManager.cpp`
15. `src/rtabmap_ros/rtabmap_slam/src/CoreWrapper.cpp`

이 둘은 local yaw mismatch를 분리하고도 raw redraw가 남을 때만 들어간다.

---

## 8. 최종 결론

2026-03-18 기준, 네가 확인한 모드 A/B/C 관찰과 실제 코드베이스, 그리고
실제 `Phase A/A-2` 계측 결과, 최신 `run_20260318_131836` 재실험 결과를 같이 보면
가장 타당한 결론은 다음이다.

1. 현재 1차 이상 신호는 `raw occupancy redraw`보다 **IMU와 EKF 사이의 local yaw scale mismatch**다.
2. 최신 런의 숫자는 `IMU +176 / -176 deg` 대 `odom/TF +45.9 / -62.6 deg`로, local yaw가 `과대회전`이 아니라 **심한 과소추종**을 보였다.
3. 최신 런에서 `/odometry/filtered.pose`와 TF `odom -> base_link`는 사실상 같은 값을 말했다.
4. 즉 이번 문제는 publish path보다 EKF local yaw estimate 자체가 먼저다.
5. `sensor_sync.log` 기준 IMU spin unlock은 최신 런에서도 정상 동작했다.
6. `rtabmap_nav2.log` 기준 `map_tf_stabilizer raw_yaw`와 `stable_map_yaw`는 최신 런에서도 세션 중 거의 `0.000`이었다.
7. `map_topic_stabilizer`의 hold/pending/timeout 문제는 실제로 존재하지만, 여전히 2차 문제다.
8. 따라서 다음 단계의 정답은 "`맵이 도는가`를 더 먼저 파는 것"이 아니라,
   **`robot_localization`이 IMU `vyaw`를 왜 이렇게 작게 반영하는지 debug와 timing/fusion 분석으로 좁히는 것**이다.

한 문장으로 정리하면:

**지금 단계에서 가장 강한 사실은 "IMU와 EKF가 pure spin yaw를 전혀 같은 크기로 보지 않는다"를 넘어, "pose와 TF는 같은데 둘 다 IMU 회전을 크게 못 따라간다"는 점이며, 다음 단계는 `robot_localization` 쪽 under-fusion / under-response 원인을 확정하는 것이다.**
