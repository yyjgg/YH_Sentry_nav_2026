from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path

def generate_launch_description():
    params_file = Path(get_package_share_directory('path_planning')) / 'config' / 'path_planning.params.yaml'

    return LaunchDescription([
        Node(
            package='path_planning',
            executable='path_planning_node',
            name='path_planning_node',
            output='screen',
            parameters=[str(params_file)]
        ),
    ])