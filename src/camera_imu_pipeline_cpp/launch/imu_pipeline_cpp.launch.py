from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 입력/출력 토픽을 런치 인자로 분리
    imu_raw = LaunchConfiguration('imu_raw')
    imu_bias = LaunchConfiguration('imu_bias')
    imu_fixed = LaunchConfiguration('imu_fixed')
    imu_target_frame = LaunchConfiguration('imu_target_frame')
    tf_timeout_sec = LaunchConfiguration('tf_timeout_sec')

    bias_corrector = Node(
        package='camera_imu_pipeline_cpp',
        executable='camera_imu_bias_corrector_cpp',
        name='camera_imu_bias_corrector',
        output='screen',
        parameters=[{
            # raw IMU를 받아 바이어스 보정 후 imu_bias 토픽으로 발행
            'input_topic': imu_raw,
            'output_topic': imu_bias,
            # IMU 벡터를 이 프레임 기준으로 회전(옵션)
            'target_frame': imu_target_frame,
            'tf_timeout_sec': tf_timeout_sec,
            # 필요하면 보정 샘플 수/정지 판정 기준을 여기서 조정
            'calib_samples': 1000,
            'stationary_threshold': 0.01,
            'gyro_cov': 0.1,
            'accel_cov': 0.1,
            # 정지 구간 yaw drift 억제
            'yaw_zeroing_enable': True,
            'yaw_zero_threshold': 0.03,
            'gyro_xy_stationary_threshold': 0.05,
            'accel_stationary_threshold': 0.7,
            'gravity_mps2': 9.81,
            'yaw_stationary_cov': 1.0e-4,
            'yaw_moving_cov': 0.1,
            'publish_during_calib': True,
        }],
    )

    madgwick = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter',
        output='screen',
        parameters=[{
            # 자력계 없이 roll/pitch 중심으로 자세 생성
            'use_mag': False,
            'publish_tf': False,
        }],
        remappings=[
            # 바이어스 보정된 IMU를 입력으로 사용
            ('imu/data_raw', imu_bias),
            # orientation 포함된 IMU 출력
            ('imu/data', imu_fixed),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('imu_raw', default_value='/camera/camera/imu', description='원본 IMU 토픽'),
        DeclareLaunchArgument('imu_bias', default_value='/camera/camera/imu_bias_corrected', description='바이어스 보정 IMU 토픽'),
        DeclareLaunchArgument('imu_fixed', default_value='/camera/camera/imu_fixed', description='orientation 포함 IMU 토픽'),
        DeclareLaunchArgument('imu_target_frame', default_value='', description='IMU 벡터를 회전시킬 목표 프레임(비우면 변환 안 함)'),
        DeclareLaunchArgument('tf_timeout_sec', default_value='0.05', description='IMU TF 조회 타임아웃(초)'),
        bias_corrector,
        madgwick,
    ])
