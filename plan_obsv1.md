# plan_obsv1.md

## 1. 목표

이 문서는 사진에서 보인 **로봇 전방의 가짜 장애물(실제 환경에는 없는 obstacle blob)** 에 대해,

1. 현재 코드베이스 기준으로 가능한 원인을 정리하고
2. 원인 우선순위를 매기고
3. 실제로 안전하게 줄여나갈 해결 순서를 계획으로 남기기 위한 문서다.

이번 문서의 핵심 포인트는 다음과 같다.

- 이 현상은 지금까지의 TF/맵 회전 문제와는 성격이 다르다.
- 이미 앞선 검증에서 `/rtabmap/map`과 `/rtabmap/map_stable` 자체의 회전/고스팅은 주증상으로 보이지 않았다.
- 따라서 이번 전방 obstacle blob은 **맵 자체의 고정 구조물**이라기보다, **Nav2 obstacle layer가 센서 포인트를 장애물로 해석한 결과**일 가능성이 훨씬 높다.
- [사실 - 실험] 이후 A/B 테스트에서 `depth_mark`를 제외하자 전방의 깜빡이는 가짜 장애물이 사라졌다. 따라서 지금 시점에서 1차 원인은 “depth source가 local obstacle layer에 raw에 가깝게 들어가고 있는 구조”로 본다.
- 하지만 LiDAR만으로는 광택이 나는 가죽 제품, 일부 어두운 가구, 저반사 물체를 충분히 안정적으로 못 잡는 경우가 있어 카메라를 완전히 제거하는 방향은 맞지 않다.
- 따라서 최종 목표는 `depth_mark 제거`가 아니라, **`raw depth_mark 직접 사용`을 중단하고, navigation용으로 더 엄격하게 전처리된 depth source를 다시 도입하는 것**이다.


## 2. 관찰된 증상

사진에서 보이는 전방 obstacle은 다음 특성을 가진다.

- 로봇 footprint 바로 앞에 작고 진한 장애물 덩어리처럼 보인다.
- 실제 현장에는 그 위치에 물리적인 장애물이 없다.
- 고정 벽/기둥처럼 맵 전체 구조에 맞춰 길게 이어진 형태가 아니라, **로봇 근처에 붙는 짧은 local blob** 형태다.
- 이런 형태는 보통 다음 둘 중 하나에서 많이 나온다.
  - 근거리 depth point cloud의 오탐
  - LiDAR point cloud의 근거리 반사/자기 몸체/지면/희소 노이즈가 obstacle layer marking에 그대로 들어간 경우

이 관찰만으로도, 우선 의심 대상은 `/rtabmap/map` static map보다 **local obstacle marking 경로**다.

추가로, 이번 실험에서 아주 중요한 관찰이 하나 더 생겼다.

- [사실 - 실험] `depth_mark`를 local obstacle layer에서 제외하자, 문제가 된 전방 가짜 장애물이 **확실히 사라졌다.**

이 결과는 단순한 추정이 아니라, 원인 분리 실험으로 얻은 직접 증거다.  
즉 현재 문서에서 `depth_mark`는 더 이상 “가능성 높은 후보”가 아니라 **확인된 1차 원인 경로**다.


## 3. 코드베이스에서 확인한 사실

아래는 실제 워크스페이스 코드를 읽고 확인한 내용이다.

### 3.1 Local costmap은 LiDAR와 Depth를 동시에 장애물로 사용한다

`local_costmap`의 `obstacle_layer`는 세 개의 observation source를 사용한다.

- `lidar_mark`
- `lidar_clear`
- `depth_mark`

파일:
- `/home/atoz/ca_ws/src/rtabmap_ros/rtabmap_launch/launch/config/nav2_rtabmap_params.yaml`

핵심 설정:

```yaml
observation_sources: lidar_mark lidar_clear depth_mark
```

```yaml
lidar_mark:
  topic: /livox/lidar/filtered
  data_type: PointCloud2
  sensor_frame: livox_frame
  marking: true
  clearing: false
  obstacle_range: 3.0
  min_obstacle_height: 0.05
  max_obstacle_height: 1.5
  observation_persistence: 0.3
```

```yaml
lidar_clear:
  topic: /livox/lidar/filtered
  data_type: PointCloud2
  sensor_frame: livox_frame
  marking: false
  clearing: true
  raytrace_range: 4.5
  min_obstacle_height: 0.05
  max_obstacle_height: 1.5
```

```yaml
depth_mark:
  topic: /camera/camera/depth/color/points
  data_type: PointCloud2
  sensor_frame: camera_link
  marking: true
  clearing: false
  obstacle_range: 1.5
  min_obstacle_height: 0.05
  max_obstacle_height: 1.5
  observation_persistence: 0.5
```

정리:

- local obstacle은 LiDAR와 camera depth point cloud가 **같이** 만든다.
- depth는 marking만 하고 clearing는 하지 않는다.
- 따라서 depth에서 한번 잘못 찍힌 근거리 점은, `observation_persistence` 동안 local obstacle blob으로 남기 쉽다.


### 3.2 Local costmap은 LiDAR의 “raw obstacle용 filtered cloud”를 바로 marking한다

현재 local costmap의 LiDAR mark/clear 토픽은 둘 다 `/livox/lidar/filtered`다.

즉 local obstacle layer는 **temporal static/dynamic 필터를 거친 `/livox/lidar/static_filtered`가 아니라**, 그 전 단계 결과를 그대로 본다.

이건 중요하다.

- `/livox/lidar/filtered`는 이름은 filtered지만
- 실제 구현상 “navigation false positive를 강하게 억제하는 장애물 전용 필터”가 아니다.


### 3.3 `/livox/lidar/filtered`는 현재 Voxel + RadiusOutlierRemoval까지만 한다

실제 구현:
- `/home/atoz/ca_ws/src/livox_pointcloud_filter/src/livox_pointcloud_filter_node.cpp`

현재 필터 내용:

1. VoxelGrid 다운샘플
2. RadiusOutlierRemoval(ROR)

launch에서 주는 파라미터:
- `leaf_size = 0.05`
- `ror_radius = 0.20`
- `ror_min_neighbors = 2`

파일:
- `/home/atoz/ca_ws/src/rtabmap_ros/rtabmap_launch/launch/rtabmap_nav2.launch.py`

즉 현재 구조는:

- 다운샘플은 함
- 이웃 수가 아주 적은 점은 제거함
- 하지만 **self-filter(로봇 몸체 제거)** 는 없다
- **footprint/crop box 기반 제거**도 없다
- **전방 근거리 blind-zone 제거**도 없다
- **지면/반사 전용 분리**도 없다

`ror_min_neighbors = 2`는 특히 navigation false positive 억제 관점에서는 꽤 느슨하다.  
이 값은 sparse한 point를 살리기 쉽고, 그만큼 작은 근거리 노이즈 클러스터도 통과시키기 쉽다.


### 3.4 더 강한 temporal filter는 존재하지만 local marking에 직접 쓰지 않는다

실제 launch에서는 `dynamic_object_filter_node`가 떠서:

- 입력: `/livox/lidar/filtered`
- 출력: `/livox/lidar/static_filtered`

를 만든다.

파일:
- `/home/atoz/ca_ws/src/rtabmap_ros/rtabmap_launch/launch/rtabmap_nav2.launch.py`
- `/home/atoz/ca_ws/src/livox_pointcloud_filter/src/dynamic_object_filter_node.cpp`

이 필터는 다음 성질을 가진다.

- target frame(`odom`) 기준으로 보셀 누적
- `min_hits`
- `hit_window_sec`
- `min_static_sec`
- `max_stale_sec`
- `min_range`

즉 **잠깐 보였다 사라지는 점을 정적으로 인정하지 않도록** 설계되어 있다.

하지만 중요한 점:

- 이 `static_filtered`는 global obstacle의 clearing 쪽에는 일부 쓰이지만
- local obstacle layer의 mark source는 여전히 `/livox/lidar/filtered`다.

즉 **local 앞장애물 false positive를 줄여주는 강한 temporal filter 결과가 local marking에 직접 반영되지 않는다.**


### 3.5 Depth source는 근거리 오탐에 취약한 구조다

`depth_mark`는 현재 다음 성질을 가진다.

- 토픽: `/camera/camera/depth/color/points`
- marking만 함
- clearing는 없음
- obstacle_range `1.5m`
- min_obstacle_height `0.05m`
- observation_persistence `0.5s`

이 구조는 아래 문제를 만들기 쉽다.

- 카메라 근거리 depth hole/반사/엣지 노이즈
- 로봇 전방 바닥 경계나 물체 모서리의 점 몇 개
- pointcloud texture 매칭 불안정

이런 것들이 **짧은 obstacle blob**으로 앞에 생기고,
clearing 없이 persistence 때문에 잠깐 유지될 수 있다.

최신 로그에서도 Realsense 쪽 pointcloud 관련 경고가 있다.

- `/home/atoz/ca_ws/logs/run_20260319_132557/realsense.log`
- `No stream match for pointcloud chosen texture Process - Color`

이 경고만으로 “depth pointcloud가 고장났다”고 단정할 수는 없지만,  
현재 depth source를 **가짜 장애물 1순위 후보 중 하나**로 올려두기에는 충분한 런타임 신호다.


### 3.6 센서 마운트 위치가 둘 다 로봇 전방에 있다

`sensor_sync.launch.py` 기준 정적 TF:

- LiDAR: `base_link -> livox_frame`
  - x=`0.3`, y=`0.0`, z=`0.63`
- Camera: `base_link -> camera_link`
  - x=`0.3`, y=`0.0`, z=`0.55`

즉 두 센서 모두 로봇 중심보다 **전방 30cm**에 달려 있다.

이 의미는 크다.

- 자기 몸체 반사
- 전방 상판/범퍼 근처 에지
- 근거리 바닥 점
- 카메라 근거리 노이즈

가 생기면, 그것이 footprint 안이 아니라 **footprint 바로 앞**에 찍히기 쉽다.


### 3.7 footprint clearing가 있어도 “앞쪽” blob은 남을 수 있다

현재 costmap에는 `footprint_clearing_enabled: true`가 있다.
하지만 이건 로봇 footprint **내부**를 지우는 기능이다.

문제는 사진의 blob이 footprint 내부가 아니라 **전방 바깥쪽**에 붙어 있다는 점이다.

따라서 이런 blob은:

- 센서 기준으로는 아주 가까운 노이즈여도
- base_link 기준으로는 footprint 앞에 놓인 obstacle로 해석될 수 있고
- footprint clearing로 자동 제거되지 않을 수 있다.


## 4. 이번 현상의 가장 가능성 높은 원인 순위

아래 순위는 실제 코드/설정/로그를 합쳐서 매긴 것이다.

### 1순위: local costmap의 `depth_mark` 오탐 [확인됨]

이유:

- 사진의 장애물이 로봇 전방 근거리 blob 형태다.
- `depth_mark`는 정확히 근거리 보강용(`obstacle_range: 1.5`)으로 들어가 있다.
- marking만 하고 clearing를 하지 않는다.
- persistence가 `0.5s`라서 짧은 오탐도 눈에 띄게 남을 수 있다.
- Realsense pointcloud 관련 경고도 최신 로그에 존재한다.
- [사실 - 실험] `depth_mark`를 제거하자 해당 blob이 사라졌다.

판단:

- **가장 먼저 A/B 테스트해야 할 후보**가 아니라, 현재는 **직접 확인된 원인 경로**다.


### 2순위: local costmap의 `lidar_mark`가 `/livox/lidar/filtered` 노이즈를 그대로 mark

이유:

- local marking은 temporal static filter 결과가 아니라 raw filtered cloud를 본다.
- raw filtered cloud는 현재 voxel + 느슨한 ROR만 적용된다.
- `ror_min_neighbors = 2`는 navigation false positive 억제에는 약한 편이다.
- self-mask가 없어서 로봇 몸체 주변 반사/근거리 점을 남길 수 있다.

판단:

- 현재 전방 blob 자체는 `depth_mark`가 주범으로 확인되었으므로, 이 경로는 “같은 모양의 blob”의 1차 원인은 아니다.
- 하지만 depth를 줄인 뒤에 남을 수 있는 다른 false positive, 또는 카메라를 줄였을 때 다시 드러날 LiDAR blind spot 보완 문제 때문에 여전히 중요한 2차 경로다.


### 3순위: 센서 프레임/근거리 body reflection에 의한 로봇 전방 자기 장애물화

이유:

- LiDAR와 camera 모두 전방 0.3m에 있다.
- 센서 근처 반사가 base_link 기준 front cell로 투영되기 쉽다.
- 현재 local marking 경로에는 “로봇 자기 몸체/센서 마운트” 제거 로직이 없다.

판단:

- 특히 obstacle이 항상 비슷한 상대 위치(전방 중심 또는 약간 우/좌)에 생기면 강하게 의심


### 4순위: global/static map이 아니라 consumer/local costmap 해석 문제

이유:

- 앞선 실험에서 `/rtabmap/map`, `/rtabmap/map_stable` 자체 회전/고스팅은 주증상이 아니었다.
- 이번 blob은 로봇 전방에 붙는 local obstacle 모양이다.

판단:

- 이번 문서에서는 static map source보다 local obstacle source를 먼저 본다.


## 5. 왜 “로봇 앞”에 보이기 쉬운가

이 부분은 중요하다. 단순히 “노이즈다”로 끝내면 해결이 안 된다.

### 5.1 센서 원점이 전방으로 치우쳐 있다

센서 둘 다 `x=0.3`에 있다.  
즉 센서에서 본 근거리 노이즈는 base_link 기준으로 보면 이미 전방 쪽이다.

### 5.2 footprint clearing는 내부만 지운다

노이즈가 footprint 안이 아니라 **footprint 바로 앞 10~40cm**에 잡히면 clearing 대상이 아니다.

### 5.3 depth는 near-field 보강용으로 설정되어 있다

`depth_mark.obstacle_range=1.5m`는 근거리 물체를 적극적으로 넣겠다는 뜻이다.  
실제 장애물도 잘 잡지만, 근거리 오탐도 그만큼 costmap에 잘 들어간다.

### 5.4 LiDAR nav용 필터가 아직 보수적이지 않다

현재 LiDAR filter는 mapping 데이터 보존 쪽에 더 가깝다.

- `leaf_size=0.05`
- `ror_radius=0.20`
- `ror_min_neighbors=2`

즉 “장애물을 놓치지 않기”에는 유리하지만,  
“가짜 장애물을 강하게 지우기”에는 아직 부족하다.


## 6. 최신 로그에서 참고할 수 있는 신호

최신 런:
- `/home/atoz/ca_ws/logs/run_20260319_132557`

확인된 신호:

1. Realsense pointcloud 경고
- `No stream match for pointcloud chosen texture Process - Color`
- 파일: `/home/atoz/ca_ws/logs/run_20260319_132557/realsense.log`

2. Dynamic filter는 정상 실행 중
- `/livox/lidar/filtered -> /livox/lidar/static_filtered`
- voxel=`0.150`, min_hits=`4`, hit_window=`0.50s`, min_static=`0.50s`
- 파일: `/home/atoz/ca_ws/logs/run_20260319_132557/rtabmap_nav2.log`

3. Local/global inflation radius 경고
- 현재 inflation radius가 inscribed radius보다 작다는 경고가 있다.
- 이건 **가짜 장애물 생성 원인**이라기보다, costmap의 안전거리/표시 해석에 혼동을 줄 수 있는 별도 설정 문제다.

중요:

- 이번 문서에서는 inflation warning을 주원인으로 보지 않는다.
- 주원인은 여전히 **mark source 오탐**이다.


### 6.1 확인된 실험 결과 업데이트

- [사실 - 실험] local obstacle layer에서 `depth_mark`를 제거했을 때, 전방에 반복적으로 생겼다 없어졌다 하는 가짜 장애물이 사라졌다.
- 따라서 지금 시점에서 “깜빡이는 전방 blob”의 1차 원인은 `depth_mark`다.
- 이 결론은 이미지 해석이나 로그 추정이 아니라, **local obstacle source를 실제로 끄고 켰을 때 증상이 사라졌는지 본 결과**다.

이 업데이트 때문에 해결 전략도 바뀐다.

- 이전 전략: `depth`와 `lidar` 중 무엇이 주범인지 먼저 분리
- 현재 전략: `depth`가 주범으로 확인되었으니, **카메라를 완전히 버릴지 말지**가 아니라 **카메라를 어떤 형태로 다시 안전하게 넣을지**를 설계


### 6.2 외부 자료와 현재 현상 해석

이 섹션은 코드베이스 밖의 공식 자료와, 그 자료를 현재 로봇 상황에 연결한 해석을 정리한 것이다.

#### 이론 A: LiDAR가 특정 재질/근거리 조건에서 놓칠 수 있기 때문에 카메라 보강이 필요하다

- [출처 - Livox MID-360 공식 스펙] https://www.livoxtech.com/de/mid-360/specs
- 공식 스펙에는 다음 내용이 있다.
  - 반사율이 낮을수록 유효 검출 성능이 떨어진다.
  - `40 m @ 10% reflectivity`, `70 m @ 80% reflectivity`
  - 특히 `0.1~1 m` 구간에서는 **low reflectivity**, **polished**, **matte finish**, **thin and tiny** objects의 detection effect를 보장하지 않는다고 명시되어 있다.

- [추론]
  - “광택이 나는 가죽 제품”이나 “어두운 가구”가 항상 위 공식 문서의 예시와 완전히 동일하다고 단정할 수는 없다.
  - 하지만 실제 현장에서 LiDAR가 특정 가구를 놓친다는 관찰과, Livox가 **저반사/광택/특정 표면 조건에서 근거리 검출 보장을 하지 않는다**고 명시한 점은 잘 맞아떨어진다.
  - 따라서 “LiDAR가 일부 가구를 안정적으로 못 보니 camera depth로 보강한다”는 전략 자체는 타당하다.

#### 이론 B: RealSense depth는 특정 재질/반사 조건에서 false depth를 만들 수 있다

- [출처 - Intel RealSense Optical Filters for D400] https://dev.intelrealsense.com/docs/optical-filters-for-intel-realsense-depth-cameras-d400
- 이 문서는 shiny surface, reflective surface, specular reflection이 false depth artefact를 만들 수 있다고 설명한다.
- 반사 물체나 shiny floor/table에서 실제보다 잘못된 depth object가 생길 수 있고, 경우에 따라 polarizer가 이를 줄일 수 있다고 한다.

- [출처 - Intel RealSense Post-processing Filters] https://dev.intelrealsense.com/docs/post-processing-filters
- Intel은 depth quality/noise reduction을 위해 post-processing filter chain을 권장한다.
  - Decimation
  - Spatial
  - Temporal
  - Hole Filling

- [출처 - Intel Support: improve depth quality] https://www.intel.com/content/www/us/en/support/articles/000036539/emerging-technologies/intel-realsense-technology.html
- Intel은 depth quality 향상을 위해 post-processing example과 texture 사용을 권장한다.

- [추론]
  - RealSense가 “가죽/광택 가구를 잘 볼 수 있다”는 장점과 “반사/근거리 노이즈 false depth를 만들 수 있다”는 단점은 서로 모순이 아니다.
  - 두 센서는 서로 다른 failure mode를 가진다.
  - 즉 지금 필요한 것은 “카메라 제거”가 아니라, **카메라를 raw obstacle source로 쓰지 않도록 역할을 좁히고 필터링하는 것**이다.

#### 이론 A와 B는 서로 다르지만 충돌하지 않는다

- [출처 - Livox 공식 스펙] LiDAR는 근거리 저반사/광택/특정 표면에서 놓칠 수 있다.
- [출처 - Intel 공식 문서] Stereo depth는 shiny/specular surface에서 false depth를 만들 수 있다.

정리:

- LiDAR는 **놓치는 문제**가 있고
- Camera depth는 **잘못 보는 문제**가 있다

따라서 센서 결합 전략은 “둘 다 켠다”가 아니라,

- LiDAR는 중장거리 기본 obstacle source
- Camera depth는 근거리, 특정 재질 보강용 보조 source

로 역할을 분리해야 한다.


## 7. 해결 전략

이번 문제는 “한 번에 큰 튜닝”보다 **원인 source를 분리하는 A/B 방식**으로 가야 한다.

### 전략 원칙

1. 먼저 source를 분리한다.
2. mapping용 포인트클라우드와 navigation용 포인트클라우드를 분리한다.
3. local obstacle false positive 억제는 local 전용 보수 필터로 해결한다.
4. 실제 장애물 회피 성능을 해치지 않도록 단계적으로 조정한다.
5. `depth_mark`는 다시 켜더라도 **raw `/camera/camera/depth/color/points`를 직접 obstacle layer에 물리지 않는다.**


## 8. 가장 먼저 해야 할 A/B 테스트

### Phase A: `depth_mark` 끄고 재현 확인 [완료]

목적:

- blob이 depth에서 오는지 확인

방법:

- local costmap `observation_sources`에서 `depth_mark`를 잠시 제거
- `lidar_mark lidar_clear`만 남긴다

예시:

```yaml
observation_sources: lidar_mark lidar_clear
```

판정:

- [사실 - 실험] blob이 사라졌다.
- 따라서 depth point cloud가 1차 원인으로 확정되었다.

현재는 이 단계가 끝났다.  
이제 더 이상 “원인 분리”가 아니라 **카메라를 어떻게 안전하게 재도입할지**를 고민해야 한다.


### Phase B: LiDAR만 남긴 상태에서 전방 blob 위치가 고정적인지 본다

목적:

- 자기 몸체/마운트 반사인지 확인

판정 포인트:

- blob이 거의 항상 같은 상대 위치(전방 정중앙 또는 약간 한쪽)에 붙는다
- 회전해도 센서 기준 비슷한 거리/각도에서 나타난다

그러면 self-reflection / near-field artifact 가능성이 커진다.


## 9. 해결방법 우선순위

아래는 내가 권장하는 실제 수정 우선순위다.

### 해결 0: raw `depth_mark`는 당분간 다시 켜지 않는다

이건 가장 먼저 고정해야 할 운영 원칙이다.

- [사실 - 실험] raw `depth_mark`를 제거하자 노이즈가 사라졌다.
- 따라서 raw `/camera/camera/depth/color/points`를 local obstacle layer에 직접 연결하는 현재 방식은, 현재 환경/가구 재질/마운트 조건에서는 맞지 않는다.

즉 “camera를 유지한다”와 “raw depth_mark를 바로 복구한다”는 같은 말이 아니다.

현재 정답은:

- camera 유지
- raw depth_mark 직접 사용 중단
- filtered depth 전용 nav topic으로 재도입


### 해결 1: camera depth를 **직접** 쓰지 말고 `depth_nav_filtered` 토픽으로 재도입한다

이게 이번 문제의 핵심 해결책이다.

권장 새 구조:

- 입력: `/camera/camera/depth/color/points`
- 출력: `/camera/camera/depth/color/nav_points`
- local obstacle layer는 앞으로 이 새 토픽만 본다.

권장 필터 단계:

1. `base_link` 기준 self-mask
2. 전방 근거리 blind-zone 제거
3. 높이 제한
4. Range 제한
5. 점군 희소 노이즈 제거
6. 짧은 temporal confirmation

이유:

- 지금 문제는 “camera가 필요 없다”가 아니라, **camera를 nav에 너무 raw하게 넣고 있다**는 것이다.
- depth pointcloud를 navigation-friendly pointcloud로 한 단계 바꾸는 것이 맞다.

권장 설계:

```text
/camera/camera/depth/color/points
  -> depth_nav_filter
  -> /camera/camera/depth/color/nav_points
  -> local_costmap.depth_mark_filtered
```

- [추론]
  - 이 구조가 가장 좋은 이유는 LiDAR blind spot 보완 능력은 유지하면서, 현재의 flickering false obstacle 생성 경로만 잘라낼 수 있기 때문이다.


### 해결 2: `depth_nav_filter`는 아래 규칙을 최소 세트로 가져간다

이건 구현 방향이다.

권장 방향:

1. `base_link` 기준 self-mask
2. 전방 10~20cm blind-zone 제거
3. `z_min` 상향
4. ROR 또는 min-cluster-size
5. temporal confirmation 2~3 frame
6. 필요 시 좌우 폭 제한

권장값 예시:

```yaml
depth_mark_filtered:
  topic: /camera/camera/depth/color/nav_points
  data_type: PointCloud2
  sensor_frame: camera_link
  marking: true
  clearing: false
  obstacle_range: 0.8
  min_obstacle_height: 0.10
  max_obstacle_height: 1.2
  observation_persistence: 0.10
```

이유:

- 근거리 false positive를 줄인다.
- 아주 낮은 바닥/엣지 점을 덜 장애물로 본다.
- 오탐이 생겨도 더 빨리 사라진다.
- depth를 완전히 잃지 않고, 근거리 재질 보강 역할은 유지할 수 있다.

주의:

- depth는 검은 물체/저반사 물체 보강용이므로, 너무 공격적으로 줄이면 실제 근거리 검은 장애물 검출이 약해질 수 있다.
- 그래서 **depth 제거**가 아니라 **depth 역할 축소 + 전처리 강화** 방향으로 가야 한다.


### 해결 3: RealSense 자체 post-processing도 같이 튜닝한다

이건 depth_nav_filter와 별개로 upstream depth quality를 좋아지게 하는 방향이다.

- [출처 - Intel 공식 문서] Intel은 D400 depth quality 개선을 위해 post-processing filters를 권장한다.
  - https://dev.intelrealsense.com/docs/post-processing-filters
  - https://www.intel.com/content/www/us/en/support/articles/000036539/emerging-technologies/intel-realsense-technology.html

현재 워크스페이스 기준 `run_all.sh`는 Realsense를 다음처럼 해상도만 지정해서 띄운다.

```bash
ros2 launch realsense2_camera rs_launch.py \
  rgb_camera.color_profile:=848x480x30 \
  depth_module.depth_profile:=848x480x30
```

즉 현재는 **navigation false positive 억제를 겨냥한 depth post-processing 파라미터가 별도로 붙어 있지 않다.**

권장 방향:

1. Threshold/clip으로 유효 range 축소
2. Spatial filter
3. Temporal filter
4. 필요 시 decimation
5. hole filling은 보수적으로

- [추론]
  - navigation false positive 관점에서는 hole filling을 공격적으로 쓰는 것은 오히려 “없던 depth를 메워서” blob을 키울 수 있다.
  - 따라서 초기에는 threshold/spatial/temporal 중심이 더 안전하다.

#### 현재 구현 반영

- [구현] `depth_nav_filter_node`를 추가했다.
  - 입력: `/camera/camera/depth/color/points`
  - 출력: `/camera/camera/depth/color/nav_points`
  - 적용 규칙:
    1. `base_link` 기준 self-mask
    2. 센서 근거리 blind-zone 제거
    3. `x/y/z/range` 제한
    4. `VoxelGrid + RadiusOutlierRemoval`
    5. 2프레임 temporal confirmation

- [구현] local costmap의 depth source를 raw depth가 아닌 `/camera/camera/depth/color/nav_points`로 변경했다.

- [구현] `run_all.sh`의 RealSense launch에 다음 post-processing 기본값을 추가했다.
  - `align_depth.enable:=true`
  - `enable_sync:=true`
  - `clip_distance:=1.5`
  - `decimation_filter.enable:=true`
  - `spatial_filter.enable:=true`
  - `temporal_filter.enable:=true`
  - `hole_filling_filter.enable:=false`

- [추론]
  - 이 조합은 shiny/specular 재질을 완벽하게 해결하는 값이라기보다,
    **현재 환경에서 raw depth false obstacle을 줄이면서도 depth 보강 능력은 남겨두기 위한 보수적 1차 기본값**이다.

#### 구현 한 줄 요약

- `depth_nav_filter_node`는 **raw depth를 받아 `base_link` 기준으로 옮긴 뒤, 로봇 자기 몸체/근거리 반사/바닥성 노이즈/희소 점/한 프레임짜리 깜빡이 점을 순서대로 제거해서 navigation용 point cloud만 다시 publish하는 노드**다.


### 해결 4: shiny/specular 가구가 계속 문제면 하드웨어 광학 대책도 옵션이다

- [출처 - Intel Optical Filters 문서] shiny/specular reflection false depth에는 linear polarizer가 도움이 될 수 있다고 설명한다.
  - https://dev.intelrealsense.com/docs/optical-filters-for-intel-realsense-depth-cameras-d400

- [추론]
  - 이건 소프트웨어보다 뒤 순위다.
  - 하지만 특정 광택 바닥/가죽/유광 가구 환경이 반복적으로 문제라면, 소프트웨어 필터만으로는 한계가 있을 수 있다.
  - 그 경우 polarizer는 현장 맞춤형 옵션이 될 수 있다.


### 해결 5: local costmap용 LiDAR 입력을 “navigation 전용 보수 필터”로 분리한다

이게 구조적으로 가장 좋은 해결이다.

현재 문제:

- `/livox/lidar/filtered`는 mapping에도 쓰기 좋은 쪽으로 남겨둔 필터라서
- navigation false positive 억제에는 부족하다.

권장 구조:

- mapping/SLAM용: 기존 `/livox/lidar/filtered`
- navigation/local obstacle용: 새 토픽 `/livox/lidar/nav_filtered`

새 nav filter에서 넣어야 할 것:

1. 더 강한 ROR
2. base_link 기준 self-mask
3. 근거리 blind-zone 제거
4. 필요 시 z 하한 상향

권장 초기값:

```yaml
leaf_size: 0.05
ror_radius: 0.25
ror_min_neighbors: 4
```

그리고 가장 중요한 추가 기능:

- **base_link 기준 crop/self-mask**

예:

- 로봇 footprint 내부 및 바로 앞 작은 마진 영역 제거
- 센서 마운트/상판 반사 구간 제거

이건 현재 `livox_pointcloud_filter_node.cpp`에 없다.  
따라서 이 문제를 구조적으로 끝내려면, 결국 nav 전용 self-mask 로직이 들어가는 게 가장 좋다.


### 해결 6: local costmap에서 LiDAR mark source를 temporal static filter 결과와 병행하거나 분리 사용한다

현재 `dynamic_object_filter_node`가 이미 `/livox/lidar/static_filtered`를 만들고 있다.

하지만 local mark는 여전히 `/livox/lidar/filtered`를 본다.

여기서 선택지는 두 가지다.

#### 선택지 A
local mark를 `/livox/lidar/static_filtered`로 바꾼다.

장점:

- false positive가 크게 줄 가능성 높음

단점:

- 동적 장애물 반응이 늦어질 수 있음
- local obstacle avoidance용으로는 너무 보수적일 수 있음

그래서 이건 “확인용 A/B 테스트”에는 좋지만, 최종 기본 설정으로는 조심해야 한다.

#### 선택지 B
local은 `nav_filtered`라는 새 보수 필터 토픽을 따로 만든다.

이게 더 바람직하다.

- 동적 장애물은 살리고
- 자기 몸체/근거리 반사/희소 노이즈만 줄이는 방향으로 설계 가능


### 해결 7: observation persistence를 낮춰 false blob 체류 시간을 줄인다

현재:

- local LiDAR mark: `0.3`
- depth mark: `0.5`

false positive가 짧게 들어왔다가 사라지는 유형이면, persistence가 체감 문제를 키운다.

권장 조정:

- LiDAR mark: `0.3 -> 0.1~0.15`
- depth mark: `0.5 -> 0.1~0.2`

주의:

- 센서 드롭이 있는 환경에서는 장애물 깜빡임이 심해질 수 있다.
- 따라서 source 분리 후 마지막 완화 수단으로 쓰는 것이 좋다.


## 10. 내가 권장하는 최종 실행 순서

가장 안전하고 정보량이 많은 순서는 아래다.

### Step 1 [완료]
`depth_mark`를 local에서 잠시 끄고 동일 상황 재현

목표:

- depth culprit 여부 확정

결과:

- [사실 - 실험] blob이 사라졌다.

### Step 2 [다음]
raw `depth_mark`를 복구하지 말고, `depth_nav_filter` 설계를 먼저 한다.

핵심:

- `/camera/camera/depth/color/nav_points` 신설
- self-mask
- range/z 제한
- 희소 노이즈 제거
- temporal confirmation

### Step 3 [다음]
filtered depth source를 local costmap에 좁은 역할로 재도입한다.

우선순위:

1. obstacle_range 축소
2. min_obstacle_height 상향
3. persistence 축소
4. raw topic이 아니라 filtered topic 사용

### Step 4 [병행]
LiDAR nav 전용 필터(`/livox/lidar/nav_filtered`)도 준비한다.

이유:

- camera는 false positive를 줄이기 위해 보수적으로 만들 수밖에 없고
- LiDAR는 여전히 주력 obstacle source여야 하기 때문이다.

### Step 5 [마지막]
필요할 때만 persistence와 Realsense post-processing을 미세조정한다.

이건 본질 해결 뒤의 품질 튜닝 단계다.


## 11. 현재 시점의 결론

현재 코드베이스와 로그를 기준으로 가장 타당한 결론은 이것이다.

1. 이번 전방 obstacle blob은 `/rtabmap/map` 고스팅보다 **Nav2 obstacle marking noise** 쪽이다.
2. [사실 - 실험] `depth_mark`를 끄자 노이즈가 사라졌으므로, 현재 blob의 1차 원인은 `depth_mark`다.
3. 하지만 LiDAR는 저반사/광택/특정 근거리 재질에서 놓칠 수 있으므로, camera depth를 완전히 버리면 실제 장애물 인식이 약해질 수 있다.  
   [출처] Livox MID-360 공식 스펙: https://www.livoxtech.com/de/mid-360/specs
4. 반대로 Intel RealSense stereo depth는 shiny/specular surface에서 false depth를 만들 수 있다.  
   [출처] Intel Optical Filters 문서: https://dev.intelrealsense.com/docs/optical-filters-for-intel-realsense-depth-cameras-d400
5. 따라서 정답은 “카메라 제거”가 아니라 **raw depth의 직접 사용을 중단하고, filtered depth를 근거리 보조 센서로 다시 도입하는 것**이다.
6. 구조적으로는 **mapping용 cloud / LiDAR nav cloud / depth nav cloud**를 분리하는 것이 가장 깔끔한 해결책이다.


## 12. 다음 작업 제안

이 문서 기준으로 바로 이어서 할 작업은 아래 둘 중 하나다.

### 옵션 1: depth nav filter 먼저 구현

새 토픽:

- `/camera/camera/depth/color/nav_points`

할 일:

- raw depth pointcloud를 navigation 전용 필터를 거쳐 재발행
- local obstacle layer는 이 토픽만 사용

장점:

- camera의 장점은 살리고, 현재 false blob 경로는 직접 차단 가능

### 옵션 2: LiDAR nav filter까지 같이 구현

새 토픽:

- `/livox/lidar/nav_filtered`

할 일:

- LiDAR도 navigation 전용 보수 필터 경로를 따로 만든다
- local obstacle layer는 depth + LiDAR 모두 nav 전용 토픽만 사용

장점:

- false positive를 구조적으로 줄일 수 있다
- mapping 품질과 navigation false positive 억제를 동시에 분리할 수 있다
- 최종 구조로 가려면 이쪽이 더 좋다


## 13. 요약 한 줄

이번 전방 가짜 장애물은 맵 고스팅이 아니라 **raw `depth_mark`가 local obstacle layer에 직접 들어가며 생긴 false obstacle**로 보는 것이 가장 타당하며,  
지금부터의 올바른 방향은 **카메라를 제거하는 것**이 아니라, **LiDAR의 재질 blind spot을 보완할 수 있도록 camera depth를 navigation 전용으로 강하게 필터링한 뒤 다시 도입하는 것**이다.
