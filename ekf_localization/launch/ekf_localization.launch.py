from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 1. 패키지 공유 폴더 경로 가져오기
    pkg_name = 'ekf_localization'
    pkg_share = get_package_share_directory(pkg_name)
    
    # 2. YAML 설정 파일 절대 경로 설정
    # 패키지의 config 폴더 안에 ekf.yaml이 위치해야 합니다.
    config_file = os.path.join(pkg_share, 'config', 'ekf.yaml')

    return LaunchDescription([
        # [A] EKF 노드: 센서 융합의 핵심 (IMU + GPS)
        Node(
            package='robot_localization', # 실제 바이너리가 위치한 패키지명
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[config_file]
        ),

        # [B] Navsat Transform 노드: GPS(위경도) -> XY(미터) 변환
        Node(
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
        ),
        
        # [C] Static TF: 센서의 위치 정의 (base_link 기준)
        # base_link -> imu_link (위치 0, 회전 0)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_imu',
            arguments = ['0', '0', '0', '0', '0', '0', 'base_link', 'imu_link']
        ),
        # base_link -> gps (위치 0, 회전 0)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_gps',
            arguments = ['0', '0', '0', '0', '0', '0', 'base_link', 'gps']
        ),
    ])