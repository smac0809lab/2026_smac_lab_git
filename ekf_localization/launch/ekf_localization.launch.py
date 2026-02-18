import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('ekf_localization')
    config_file = os.path.join(pkg_share, 'config', 'ekf.yaml')

    # gps_vel.py의 절대 경로
    gps_vel_path = '/home/user/ros2_ws/src/python/gps_vel.py'

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

    # 3. IMU 센서 구동
    ebimu_node = Node(
        package='ebimu_pkg',
        executable='ebimu_node',
        name='ebimu_node',
        output='screen'
    )

    # 4. IMU + Encoder 융합 노드 (작성하신 Python 코드 패키지)
    # imu_encoder 패키지에 정의된 imu_encoder_node 실행
    imu_encoder_node = Node(
        package='imu_encoder',
        executable='imu_encoder_node',
        name='imu_encoder_node',
        output='screen'
    )

    # 5. gps_vel인데 엔코더 대용으로 만든거 (python직접실행)
    gps_vel_exec = ExecuteProcess(
        cmd=['python3', gps_vel_path],
        output='screen'
    )

    # 6. Navsat Transform 노드
    navsat_node = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform',
        output='screen',
        parameters=[config_file],
        remappings=[
            ('gps/fix', '/ublox_gps_node/fix'),
            ('imu', '/imu/data'),
            ('odometry/filtered', '/odometry/filtered')
        ]
    )

    # 7. EKF 노드
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[config_file],
        remappings=[
            ('imu0', '/imu/data')
        ]
    )

    # 8. Static TF
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
        imu_encoder_node, # 추가됨
        gps_vel_exec,     # 추가됨
        navsat_node,
        ekf_node,
        static_tf_imu,
        static_tf_gps
    ])