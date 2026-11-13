from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    livox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("livox_ros_driver2"),
                    "launch_ROS2",
                    "msg_MID360_launch.py",
                ]
            )
        )
    )

    small_point_lio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("small_point_lio"),
                    "launch",
                    "small_point_lio.launch.py",
                ]
            )
        ),
        launch_arguments={
            "enable_mapping_visualization": "true",
        }.items(),
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="small_point_lio_rviz",
        output="screen",
        arguments=[
            "-d",
            PathJoinSubstitution(
                [
                    FindPackageShare("small_point_lio"),
                    "config",
                    "small_point_lio_mapping.rviz",
                ]
            ),
        ],
    )

    return LaunchDescription([
        livox_launch,
        small_point_lio_launch,
        rviz_node,
    ])
