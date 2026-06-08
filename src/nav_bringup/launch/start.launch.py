from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 包含livox_ros_driver2的launch文件
    livox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('livox_ros_driver2'),
                'launch_ROS2',
                'msg_MID360_launch.py'  
            ])
        ])
    )
    
    # 包含small_point_lio的launch文件
    small_point_lio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('small_point_lio'),
                'launch',
                'small_point_lio.launch.py' 
            ])
        ])
    )
    
    # 包含path_planning
    path_planning_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('path_planning'),
                'launch',
                'path_planning.launch.py'  
            ])
        ])
    )
    
    # 包含go_go_go的launch文件
    go_go_go_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('go_go_go'),
                'launch',
                'go_go_go.launch.py'
            ])
        ])
    )
    
    # 启动RViz2节点，使用指定的rviz配置文件
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', PathJoinSubstitution([FindPackageShare('nav_bringup'), 'rviz', '1.rviz'])]
    )

    # odom_point -> map_point（odom原点在map下的坐标，记录的是map->odom的坐标系变换）
    static_odom_to_map = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "--x",
            "3.5",
            "--y",
            "7.5",
            "--z",
            "0.0",
            "--roll",
            "0.0",
            "--pitch",
            "0.0",
            "--yaw",
            "0.0",
            "--frame-id",
            "map",
            "--child-frame-id",
            "odom",
        ],
    )

    return LaunchDescription([
        livox_launch,
        small_point_lio_launch,
        path_planning_launch,
        go_go_go_launch,  # 添加go_go_go节点
        static_odom_to_map,
        rviz_node
    ])