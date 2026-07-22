from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='auto_parking',
            executable='gps_imu_parking_node',
            name='gps_imu_parking_node',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'auto_start': False,
                'enable_heading_calibration': False,
                'output_twist_mode': 'kmh_deg',

                'utm_zone': 52,

                'parking_spot_lat': 36.9683204,
                'parking_spot_lon': 127.8734188,
                'parking_spot_alt': 121.65,
                'parking_spot_qx': 0.01553492990444338,
                'parking_spot_qy': 0.01581687613452909,
                'parking_spot_qz': -0.3674679570552528,
                'parking_spot_qw': 0.9297719037053378,

                'wheelbase': 0.78,
                'max_steer_deg': 22.0,
                'reverse_speed_kmh': 1.2,
                'calibration_speed_kmh': 2.0,
                'creep_speed_kmh': 0.6,
                'control_hz': 20.0,
                'max_steer_rate_deg': 12.0,
                'max_speed_rate_kmh': 1.5,
                'steer_deadband_deg': 1.5,

                'line_follow_kp_cte': 0.35,
                'line_follow_kp_heading': 1.00,
                'reverse_line_follow_kp_cte': 0.35,
                'reverse_line_follow_kp_heading': 0.90,
                'steering_hold_kp': 0.70,
                'steering_hold_max_comp_deg': 8.0,

                # 10m 이내 감시, 최단거리 후 약 1m 멀어지면 자동 트리거
                'auto_arm_enabled': True,
                'auto_arm_distance_m': 10.0,
                'auto_arm_increase_delta_m': 1.0,
                'auto_arm_persist_cycles': 3,
                'auto_arm_min_speed_kmh': 0.4,

                'steering_feedback_topic': '/steering_pot_deg',
                'steering_feedback_mode': 'deg',
                'steering_pot_center_raw': 0.0,
                'steering_pot_min_raw': -1.0,
                'steering_pot_max_raw': 1.0,
                'steering_pot_max_deg': 22.0,

                'parking_side': -1,
                'reverse_turn_entry_offset_m': 1.2,

                'status_log_period_sec': 1.0,
                'traj_history_limit': 500,
                'traj_min_point_dist': 0.05,
                'state_hold_cycles': 4,
                'calibration_min_travel_m': 2.0,
                'calibration_min_elapsed_sec': 4.0,
                'calibration_min_samples': 5,
                'gps_ema_alpha': 0.35,
                'gps_fusion_alpha': 0.60,

                'pos_tol': 0.30,
                'yaw_tol_deg': 6.0,
                'timeout_calibrate': 20.0,
                'timeout_reverse_turn': 20.0,
                'timeout_reverse_straight': 20.0,
                'timeout_final_align': 15.0,
            }]
        )
    ])
