from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare('bunker_sim')
    world = PathJoinSubstitution([pkg, 'worlds', 'empty.sdf'])
    model = PathJoinSubstitution([pkg, 'models', 'bunker_tracked', 'model.sdf'])

    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])
        ]),
        launch_arguments={'gz_args': ['-r ', world]}.items(),
    )

    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'bunker', '-file', model, '-x', '0', '-y', '0', '-z', '0.05'],
        output='screen',
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
        ],
        output='screen',
    )

    return LaunchDescription([
        gz,
        spawn,
        bridge,
    ])
