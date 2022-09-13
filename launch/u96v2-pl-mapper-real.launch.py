from struct import pack
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    sensors = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("iii_drone"),
                "launch/sensors_real.launch.py"
            ])
        ])
    )

    pl_tracker = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("iii_drone"),
                "launch/pl_mapper_HW.launch.py"
            ])
        ])
    )

    return LaunchDescription([
        sensors,
        pl_tracker
    ])
