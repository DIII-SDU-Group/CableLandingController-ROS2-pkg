from struct import pack
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('cable_landing_control'),
        'config',
        'params.yaml'
    )

    tf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("cable_landing_control"),
                "launch/tf-sim.launch.py"
            ])
        ])
    )

    trajectory_controller = Node(
        package="cable_landing_control",
        executable="trajectory_controller",
        parameters=[config]
    )

    cable_drum_controller = Node(
        package="cable_landing_control",
        executable="cable_drum_controller",
        parameters=[config]
    )

    double_cable_lander = Node(
        package="cable_landing_control",
        executable="double_cable_lander",
        parameters=[config]
    )

    return LaunchDescription([
        tf_launch,
        trajectory_controller,
        cable_drum_controller,
        double_cable_lander
    ])
