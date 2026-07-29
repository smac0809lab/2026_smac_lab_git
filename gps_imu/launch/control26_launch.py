import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 패키지 이름 정의 (본인의 ROS 2 패키지 이름으로 수정 필요)
    pkg_name = 'python' 

    # 1. 센서 드라이버 및 통신 노드들
    iahrs_node = Node(
        package='iahrs_driver',
        executable='driver',
        name='iahrs_driver',
        parameters=[{'use_sim_time': False}],
        output='screen'
    )

    ublox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ublox_gps'), 'launch', 'ublox_gps_node.launch.py')
        ]),
        launch_arguments={'use_sim_time': 'false'}.items()
    )

    ntrip_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ntrip_client'), 'ntrip_client.launch.py')
        ]),
        launch_arguments={'use_sim_time': 'false'}.items()
    )

    # 2. 직접 만든 파이썬 스크립트 노드들 (/src/python 하위 폴더에 있다고 가정)
    # executable은 setup.py의 entry_points에 등록된 이름이어야 합니다.
    odom_parser_node = Node(
        package=pkg_name,
        executable='odom_parser', # setup.py에 등록된 entry_point 이름
        name='odom_parser',
        parameters=[{'use_sim_time': False}],
        output='screen'
    )

    path_pub_node = Node(
        package=pkg_name,
        executable='path_pub', # setup.py에 등록된 entry_point 이름
        name='path_pub',
        parameters=[{'use_sim_time': False}],
        output='screen'
    )

    control_node = Node(
        package=pkg_name,
        executable='gps_imu_stanley', # control26(GpsImuStanley) 노드의 entry_point 이름
        name='gps_imu_stanley',
        parameters=[{'use_sim_time': False}],
        output='screen'
    )

    return LaunchDescription([
        iahrs_node,
        ublox_launch,
        ntrip_launch,
        odom_parser_node,
        path_pub_node,
        control_node
    ])