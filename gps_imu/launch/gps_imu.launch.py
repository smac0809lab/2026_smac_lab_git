import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # 다른 패키지의 런치 파일들을 가져옵니다.
    # 시스템이 알아서 'ublox_gps' 패키지 경로를 찾아내기 때문에 걱정 마세요!
    
    ublox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ublox_gps'), 'launch', 'ublox_gps_node.launch.py')
        ])
    )

    ntrip_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ntrip_client'), 'ntrip_client.launch.py')
        ])
    )

    ebimu_node = Node(
        package='ebimu_pkg',
        executable='ebimu_node',
        output='screen'
    )

    ekf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ekf_localization'), 'launch', 'ekf_localization.launch.py')
        ])
    )

    return LaunchDescription([
        ublox_launch,
        ntrip_launch,
        ebimu_node,
        ekf_launch
    ])