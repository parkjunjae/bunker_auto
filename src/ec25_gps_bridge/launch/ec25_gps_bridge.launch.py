from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('device', default_value='/dev/ttyUSB3'),
        DeclareLaunchArgument('baudrate', default_value='115200'),
        DeclareLaunchArgument('poll_rate_hz', default_value='2.0'),
        DeclareLaunchArgument('frame_id', default_value='gps_link'),
        DeclareLaunchArgument('topic_name', default_value='/gps/fix'),
        DeclareLaunchArgument('enable_gps_on_start', default_value='true'),
        DeclareLaunchArgument('qgpsloc_command', default_value='AT+QGPSLOC=2'),
        DeclareLaunchArgument('read_timeout_sec', default_value='1.2'),
        DeclareLaunchArgument('publish_static_tf', default_value='true'),
        DeclareLaunchArgument('parent_frame', default_value='base_link'),
        DeclareLaunchArgument('gps_x', default_value='0.3'),
        DeclareLaunchArgument('gps_y', default_value='0.0'),
        DeclareLaunchArgument('gps_z', default_value='0.475'),
        DeclareLaunchArgument('gps_roll', default_value='0.0'),
        DeclareLaunchArgument('gps_pitch', default_value='0.0'),
        DeclareLaunchArgument('gps_yaw', default_value='0.0'),
        Node(
            package='ec25_gps_bridge',
            executable='ec25_navsat_bridge',
            name='ec25_navsat_bridge',
            output='screen',
            parameters=[{
                'device': ParameterValue(LaunchConfiguration('device'), value_type=str),
                'baudrate': ParameterValue(LaunchConfiguration('baudrate'), value_type=int),
                'poll_rate_hz': ParameterValue(LaunchConfiguration('poll_rate_hz'), value_type=float),
                'frame_id': ParameterValue(LaunchConfiguration('frame_id'), value_type=str),
                'topic_name': ParameterValue(LaunchConfiguration('topic_name'), value_type=str),
                'enable_gps_on_start': ParameterValue(LaunchConfiguration('enable_gps_on_start'), value_type=bool),
                'qgpsloc_command': ParameterValue(LaunchConfiguration('qgpsloc_command'), value_type=str),
                'read_timeout_sec': ParameterValue(LaunchConfiguration('read_timeout_sec'), value_type=float),
            }],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='gps_static_tf',
            condition=IfCondition(LaunchConfiguration('publish_static_tf')),
            arguments=[
                '--x', LaunchConfiguration('gps_x'),
                '--y', LaunchConfiguration('gps_y'),
                '--z', LaunchConfiguration('gps_z'),
                '--roll', LaunchConfiguration('gps_roll'),
                '--pitch', LaunchConfiguration('gps_pitch'),
                '--yaw', LaunchConfiguration('gps_yaw'),
                '--frame-id', LaunchConfiguration('parent_frame'),
                '--child-frame-id', LaunchConfiguration('frame_id'),
            ],
        ),
    ])
