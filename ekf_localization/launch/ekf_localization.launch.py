import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('ekf_localization')
    ekf_config = os.path.join(pkg_dir, 'config', 'ekf.yaml')
    # 이전 에러 메시지에서 확인된 경로 문제를 방지하기 위해 파일 존재 확인 권장
    gps_config = os.path.join(pkg_dir, 'config', 'ublox_rover.yaml')

    return LaunchDescription([
        # 1. GPS 노드 (ublox)
        Node(
            package='ublox_gps',
            executable='ublox_gps_node',
            parameters=[gps_config],
            remappings=[('/fix', '/ublox_gps_node/fix')] # 토픽 이름 통일
        ),

        # 2. IMU 노드 (ebimu)
        Node(
            package='ebimu_pkg',
            executable='ebimu_node',  # .py 제거된 entry_point 이름
            output='screen'
        ),

        # 3. 정적 TF 브로드캐스터 (중요: 좌표계 연결)
        # base_link를 기준으로 센서들의 위치를 정의합니다. (x y z yaw pitch roll)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_imu',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'imu_link']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_gps',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'gps']
        ),

        # 4. GPS -> XY 변환 노드 (navsat_transform_node)
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform',
            output='screen',
            parameters=[ekf_config],
            remappings=[
                ('imu/data', '/imu/data'),
                ('gps/fix', '/ublox_gps_node/fix'),
                ('gps/filtered', '/gps/filtered'),
                ('odometry/gps', '/odometry/gps'),
                ('odometry/filtered', '/odometry/filtered')
            ]
        ),

        # 5. EKF 노드 (최종 센서 융합)
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_config],
            remappings=[
                ('odometry/filtered', '/odometry/filtered')
            ]
        )
    ])