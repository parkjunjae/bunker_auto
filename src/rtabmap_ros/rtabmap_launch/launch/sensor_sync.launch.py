"""
센서 시간동기화 + Deskew + Static TF 통합 런치 파일

파이프라인:
1. LiDAR 타임스탬프 오프셋 보정
2. IMU 바이어스 보정 + Madgwick 필터
3. LiDAR Deskew (base_link 기준)
4. Static TF (센서 마운트 위치)

사용법:
  ros2 launch rtabmap_launch sensor_sync.launch.py

  # 타임스탬프 오프셋 측정 후 조정:
  ros2 launch rtabmap_launch sensor_sync.launch.py lidar_offset_sec:=-0.42
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # ==================== Launch Arguments ====================
    use_sim_time = LaunchConfiguration('use_sim_time')

    # LiDAR 타임스탬프 오프셋 (ros2 topic echo로 측정 후 조정)
    lidar_offset_sec = LaunchConfiguration('lidar_offset_sec')

    # 센서 마운트 위치 (실측값으로 조정)
    livox_x = LaunchConfiguration('livox_x')
    livox_y = LaunchConfiguration('livox_y')
    livox_z = LaunchConfiguration('livox_z')
    livox_roll = LaunchConfiguration('livox_roll')
    livox_pitch = LaunchConfiguration('livox_pitch')
    livox_yaw = LaunchConfiguration('livox_yaw')

    camera_x = LaunchConfiguration('camera_x')
    camera_y = LaunchConfiguration('camera_y')
    camera_z = LaunchConfiguration('camera_z')
    camera_roll = LaunchConfiguration('camera_roll')
    camera_pitch = LaunchConfiguration('camera_pitch')
    camera_yaw = LaunchConfiguration('camera_yaw')


    # ==================== 1. LiDAR 타임스탬프 오프셋 ====================
    # Livox 타임스탬프가 시스템 시간과 다를 경우 보정
    # 측정 방법: ros2 topic echo /livox/lidar --field header.stamp 와 현재 시간 비교
    lidar_timestamp_offset_node = Node(
        package='livox_timestamp_offset',
        executable='livox_timestamp_offset_node',
        name='livox_timestamp_offset',
        output='screen',
        parameters=[{
            'input_topic': '/livox/lidar',
            'output_topic': '/livox/lidar/synced',
            'offset_sec': lidar_offset_sec,
        }],
    )

    # ==================== 2. IMU 파이프라인 ====================
    # 바이어스 보정 (정지 상태에서 캘리브레이션)
    imu_bias_corrector_node = Node(
        package='camera_imu_pipeline_cpp',
        executable='camera_imu_bias_corrector_cpp',
        name='camera_imu_bias_corrector',
        output='screen',
        parameters=[{
            'input_topic': '/camera/camera/imu',
            'output_topic': '/camera/camera/imu_bias_corrected',
            'target_frame': 'base_link',  # 프레임 변환 안 함
            'tf_timeout_sec': 0.05,
            'calib_samples': 1000,  # 실제 IMU ~50Hz × 20초 = 충분한 평균으로 bias 정확도 향상
            'stationary_threshold': 0.01,  # 좀 낮게 정지 판단 (켈레브레이션 지연이 너무 큼 약 80초)
            'gyro_cov': 0.01,  # 휠 vyaw(0.02)보다 낮게: IMU가 주도 → 제자리 회전 시 슬립 보정 효과
            'accel_cov': 0.01,
            # 정지 구간 yaw drift 억제: EKF에 0 yaw rate를 강하게 주입
            # 아래 파라미터는 LOCKED/UNLOCKED 2상태를 유지한 채,
            # 저속 yaw도 "yaw evidence"로 moving 판정하게 만드는 값이다.
            # 원칙:
            # - hard_wz: 강한 회전은 즉시 해제
            # - yaw evidence: 보통/저속 회전은 yaw 크기 + 지속성으로 해제
            # - xy_hold: 마지막 fallback
            'yaw_zeroing_enable': True,
            'yaw_zero_threshold': 0.05,             # 하위 호환용 기본 yaw 기준. 1차 튜닝 기준과 맞춘다.
            'gyro_xy_stationary_threshold': 0.05,   # 하위 호환용 기본 xy gyro 기준. launch/코드 기본값 동기화용.
            'accel_stationary_threshold': 0.7,      # 하위 호환용 기본 accel 기준. launch/코드 기본값 동기화용.
            'yaw_lock_enter_wz': 0.02,              # LOCK 진입용 |wz| 상한. 이 값보다 충분히 작아야 "거의 회전 없음"으로 본다.
            'yaw_lock_exit_wz': 0.05,               # 하위 호환용 기존 파라미터. 현재 primary unlock은 evidence score가 담당한다.
            'yaw_lock_hard_exit_wz': 0.10,          # 강한 회전 즉시 해제 기준. 이 값 초과면 hold 없이 바로 UNLOCK.
            'yaw_lock_enter_xy': 0.03,              # LOCK 진입용 roll/pitch rate 기준. 차체 흔들림이 있으면 잠그지 않기 위한 값.
            'yaw_lock_exit_xy': 0.05,               # xy fallback 기준. 이제 xy는 주 해제 조건이 아니라 마지막 보조 탈출 경로다.
            'yaw_lock_enter_accel_dev': 0.25,       # LOCK 진입용 ||a|-g| 기준. 전후진/가감속 중에는 잠기지 않게 한다.
            'yaw_lock_enter_hold_sec': 0.50,        # LOCK 진입 유지시간. 0.5초 연속 안정일 때만 다시 잠가 채터링을 줄인다.
            'yaw_lock_exit_hold_sec': 0.03,         # evidence unlock hold. 점수가 기준을 넘으면 30ms 유지 후 해제한다.
            'yaw_lock_exit_xy_hold_sec': 0.08,      # xy 기반 보조 해제 유지시간. body shake만으로 풀리지 않게 wz보다 더 길게 둔다.
            'yaw_lock_evidence_wz_low': 0.008,      # 저속 yaw deadband. 이 값 아래는 evidence를 거의 쌓지 않는다.
            'yaw_lock_evidence_wz_high': 0.025,     # yaw magnitude 정규화 상단. 이 값이면 magnitude evidence를 충분히 본다.
            'yaw_lock_evidence_yaw_ref_rad': 0.012, # same-sign yaw 누적 기준. 약 0.7도 누적되면 persistence evidence가 충분해진다.
            'yaw_lock_evidence_decay_sec': 0.40,    # yaw가 사라졌을 때 evidence를 얼마나 빨리 0으로 되돌릴지 정하는 감쇠 시간.
            'yaw_lock_evidence_unlock_threshold': 0.48, # yaw evidence score 해제 임계값. 보통/저속 실제 회전을 moving으로 승격하는 기준.
            'yaw_lock_evidence_unlock_hold_sec': 0.04,  # evidence score가 기준 이상으로 유지되어야 하는 최소 시간.
            'yaw_lock_evidence_relock_threshold': 0.15, # 저속 yaw가 계속 남아 있을 때 다시 LOCK으로 재진입하지 않게 막는 기준.
            'yaw_lock_filter_tau_sec': 0.10,        # gyro EMA 시정수. 샘플 노이즈는 줄이고 회전 응답은 크게 늦추지 않는 값.
            'yaw_lock_accel_filter_tau_sec': 0.15,  # accel EMA 시정수. accel은 더 튀므로 gyro보다 약간 더 느리게 평활화.
            'gravity_mps2': 9.81,
            'yaw_stationary_cov': 1.0e-4,           # 정지 시 z축 공분산(작게)
            'yaw_moving_cov': 0.01,                 # 회전 시 z축 공분산
            'publish_during_calib': False,
            'continuous_calib': True,   # 초기 교정 후 정지 상태에서 EMA로 bias 지속 추적
            'ema_alpha': 0.001,         # 50Hz × 1000 스텝 ≈ 20초 시정수 (온도 드리프트 추적)
        }],
    )

    # Madgwick 필터 (roll/pitch 추정)
    imu_madgwick_node = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter',
        output='screen',
        parameters=[{
            'use_mag': False,
            'publish_tf': False,
            'world_frame': 'enu',
            'gain': 0.1,  # 낮을수록 안정적, 높을수록 반응 빠름
            'zeta': 0.0,  # gyro drift correction 비활성화 → angular_velocity 값 변조 방지
        }],
        remappings=[
            ('imu/data_raw', '/camera/camera/imu_bias_corrected'),
            ('imu/data', '/camera/camera/imu_fixed'),
        ],
    )

    # ==================== 3. LiDAR Deskew ====================
    # base_link 기준으로 deskew (odom 드리프트 영향 최소화)
    lidar_deskew_node = Node(
        package='rtabmap_util',
        executable='lidar_deskewing',
        name='lidar_deskewing',
        output='screen',
        parameters=[{
            'fixed_frame_id': 'base_link',  # 로컬 움직임 기준 deskew → odom 드리프트 영향 차단
            'queue_size': 3,
            'qos': 2,
            'wait_for_transform': 0.5,  # TF 준비 전 skewed cloud 통과 방지
            'slerp': True,
            'use_sim_time': use_sim_time,
        }],
        remappings=[
            ('input_cloud', '/livox/lidar/synced'),  # 타임스탬프 보정된 것 사용
            ('output_cloud', '/livox/lidar/synced/deskewed'),
        ],
    )

    # ==================== 4. Static TF (센서 마운트) ====================
    # Livox LiDAR → base_link
    livox_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='livox_tf',
        arguments=[
            '--x', livox_x,
            '--y', livox_y,
            '--z', livox_z,
            '--roll', livox_roll,
            '--pitch', livox_pitch,
            '--yaw', livox_yaw,
            '--frame-id', 'base_link',
            '--child-frame-id', 'livox_frame',
        ],
    )

    # RealSense Camera → base_link
    camera_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_tf',
        arguments=[
            '--x', camera_x,
            '--y', camera_y,
            '--z', camera_z,
            '--roll', camera_roll,
            '--pitch', camera_pitch,
            '--yaw', camera_yaw,
            '--frame-id', 'base_link',
            '--child-frame-id', 'camera_link',
        ],
    )

    # ==================== Launch Description ====================
    return LaunchDescription([
        # Arguments
        DeclareLaunchArgument('use_sim_time', default_value='false'),

        # LiDAR 타임스탬프 오프셋 (측정 후 조정 필요)
        DeclareLaunchArgument('lidar_offset_sec', default_value='0.003',
            description='LiDAR timestamp offset in seconds. Measure: now - lidar_stamp'),

        # Livox 마운트 위치 (base_link 기준)
        DeclareLaunchArgument('livox_x', default_value='0.3'),  # 전방 30cm
        DeclareLaunchArgument('livox_y', default_value='0.0'),
        DeclareLaunchArgument('livox_z', default_value='0.63'),  # 로봇 위 63cm
        DeclareLaunchArgument('livox_roll', default_value='0.0'),
        DeclareLaunchArgument('livox_pitch', default_value='0.0'),
        DeclareLaunchArgument('livox_yaw', default_value='0.0'),

        # Camera 마운트 위치 (base_link 기준)
        DeclareLaunchArgument('camera_x', default_value='0.3'),  # 전방 30cm
        DeclareLaunchArgument('camera_y', default_value='0.0'),
        DeclareLaunchArgument('camera_z', default_value='0.55'),  # 로봇 위 55cm
        DeclareLaunchArgument('camera_roll', default_value='0.0'),
        DeclareLaunchArgument('camera_pitch', default_value='0.0'),
        DeclareLaunchArgument('camera_yaw', default_value='0.0'),
        # Nodes
        lidar_timestamp_offset_node,
        imu_bias_corrector_node,
        imu_madgwick_node,
        lidar_deskew_node,
        livox_tf_node,
        camera_tf_node,
    ])
