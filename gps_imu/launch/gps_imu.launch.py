import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 노드들이 시뮬레이션 시간이 아닌 시스템 시간을 쓰도록 명시적으로 정의하는 것이 안전합니다.
        Node(package='iahrs_driver', executable='driver', parameters=[{'use_sim_time': False}], output='screen'),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('ublox_gps'), 'launch', 'ublox_gps_node.launch.py')]),
            launch_arguments={'use_sim_time': 'false'}.items()
        ),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('ntrip_client'), 'ntrip_client.launch.py')]),
            launch_arguments={'use_sim_time': 'false'}.items()
        )
    ])