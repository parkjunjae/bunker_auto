from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    gui = LaunchConfiguration('gui')
    pkg = FindPackageShare('bunker_sim')
    world = PathJoinSubstitution([pkg, 'worlds', 'empty.sdf'])
    model = PathJoinSubstitution([pkg, 'models', 'bunker_tracked', 'model.sdf'])

    gz_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])
        ]),
        launch_arguments={'gz_args': ['-r ', world]}.items(),
        condition=IfCondition(gui),
    )

    gz_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])
        ]),
        launch_arguments={'gz_args': ['-r -s ', world]}.items(),
        condition=UnlessCondition(gui),
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
        DeclareLaunchArgument(
            'gui',
            default_value='false',
            description='Run Gazebo with GUI window (true/false)',
        ),
        gz_gui,
        gz_headless,
        spawn,
        bridge,
    ])
