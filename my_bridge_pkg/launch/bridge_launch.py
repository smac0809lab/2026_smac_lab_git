from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        #만든 파이썬 브릿지 노드 실행
        Node(
            package='my_bridge_pkg',
            executable='bridge_node',
            output='screen'
        )
    ])