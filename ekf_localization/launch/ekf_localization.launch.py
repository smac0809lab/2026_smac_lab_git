import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 설정 파일 경로 (본인의 패키지 이름과 경로에 맞춰 수정하세요)
    config_file = os.path.join(get_package_share_directory('ekf_localization'), 'config', 'ekf.yaml')

    # 1. GPS 신뢰도 필터 노드
    gps_filter_node = Node(
        package='ekf_localization',
        executable='gps_reliability_filtered_node',
        name='gps_reliability_filter',
        output='screen',
        remappings=[
            ('input_gps', '/ublox_gps_node/fix'),
            ('output_gps', '/gps_reliability_filtered')
        ]
    )

    # 2. Navsat Transform 노드 (GPS -> XY 변환)
    navsat_node = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform',
        output='screen',
        parameters=[config_file],
        remappings=[
            ('gps/fix', '/gps_reliability_filtered'),
            ('imu', '/imu/data'),
            ('odometry/filtered', '/odometry/filtered')
        ]
    )

    # 3. EKF 노드 (최종 위치 추정)
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[config_file]
    )

    # 4. Static TF (좌표계 정의)
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
        gps_filter_node,
        navsat_node,
        ekf_node,
        static_tf_imu,
        static_tf_gps
    ])