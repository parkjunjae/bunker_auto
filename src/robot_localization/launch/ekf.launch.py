import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory("robot_localization"),
        "params",
        "ekf.yaml",
    )

    debug = LaunchConfiguration("debug")
    debug_out_file = LaunchConfiguration("debug_out_file")
    print_diagnostics = LaunchConfiguration("print_diagnostics")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "debug",
                default_value="false",
                description="Enable robot_localization core debug file output",
            ),
            DeclareLaunchArgument(
                "debug_out_file",
                default_value="robot_localization_debug.txt",
                description="Path to robot_localization debug output file",
            ),
            DeclareLaunchArgument(
                "print_diagnostics",
                default_value="false",
                description="Enable robot_localization diagnostics output",
            ),
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node",
                output="screen",
                parameters=[
                    params_file,
                    {
                        "debug": ParameterValue(debug, value_type=bool),
                        "debug_out_file": ParameterValue(debug_out_file, value_type=str),
                        "print_diagnostics": ParameterValue(
                            print_diagnostics, value_type=bool
                        ),
                    },
                ],
            ),
        ]
    )
