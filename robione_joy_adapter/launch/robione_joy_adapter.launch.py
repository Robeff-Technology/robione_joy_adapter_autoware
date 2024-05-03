import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('robione_joy_adapter'),
        'config',
        'robione_joy_adapter.param.yaml'
    )

    return LaunchDescription([
        Node(
            package='robione_joy_adapter',
            name="robione_joy_adapter",
            executable = 'robione_joy_adapter_node',
            output = 'screen',
            parameters=[config]
        ),
        ExecuteProcess(
            cmd=['ros2', 'run', 'joy', 'joy_node'],
            output='screen',
        )
    ])