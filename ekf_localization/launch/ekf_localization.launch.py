import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('ekf_localization')
    # EKF 설정 파일 경로
    config_file = os.path.join(pkg_share, 'config', 'ekf.yaml')

    # 1. GPS 센서 구동 (ublox)
    ublox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ublox_gps'), 'launch', 'ublox_gps_node.launch.py')
        ])
    )

    # 2. RTK 보정 데이터 수신 (NTRIP)
    ntrip_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ntrip_client'), 'ntrip_client.launch.py')
        ])
    )

    # 3. IMU 센서 구동 (EBIMU 원본 데이터 발행)
    ebimu_node = Node(
        package='ebimu_pkg',
        executable='ebimu_node',
        name='ebimu_node',
        output='screen'
    )

    

    # 5. Navsat Transform 노드 (GPS -> XY 변환)
    navsat_node = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform',
        output='screen',
        parameters=[config_file],
        remappings=[
            ('gps/fix', '/ublox_gps_node/fix'),
            ('imu', '/imu/data'),       # 필터링된 데이터 사용
            ('odometry/filtered', '/odometry/filtered')
        ]
    )

    # 6. EKF 노드 (최종 위치 추정)
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[config_file],
        remappings=[
            ('imu0', '/imu/data')       # yaml 설정에 맞춰 필터링된 데이터 사용
        ]
    )

    # 7. Static TF (좌표계 정의)
    static_tf_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'imu_link']
    )
    static_tf_gps = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'gps']
    )

    return LaunchDescription([
        ublox_launch,
        ntrip_launch,
        ebimu_node,  
        navsat_node,
        ekf_node,
        static_tf_imu,
        static_tf_gps
    ])