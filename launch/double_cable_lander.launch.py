from struct import pack
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    tf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("cable_landing_control"),
                "launch/tf.launch.py"
            ])
        ])
    )

    pl_tracker = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("iii_drone"),
                "launch/pl_tracker_SIM.launch.py"
            ])
        ])
    )

    return LaunchDescription([
        tf_launch,
        pl_tracker
    ])
