from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path

def generate_launch_description():
    params_file = Path(get_package_share_directory('go_go_go')) / 'config' / 'go_go_go.params.yaml'

    return LaunchDescription([
        Node(
            package='go_go_go',
            executable='go_go_go_node',
            name='go_go_go_node',
            output='screen',
            emulate_tty=True, 
            parameters=[str(params_file)],
            remappings=[]
        ),
    ])