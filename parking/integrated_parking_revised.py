#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
GPS / IMU / steering-pot 기반 직각주차 통합 노드 (ROS2)

동작 개요
- 평소에는 차량을 직접 제어하지 않고, 현재 위치/방향/조향피드백만 감시한다.
- 목표 주차구역을 기준으로 거리를 관찰하다가,
-1) 목표점에서 일정 거리(auto_arm_distance_m) 안으로 들어오고
-2) 그 이후 최단거리를 갱신한 뒤 다시 멀어지기 시작하면
-자동으로 주차 FSM을 시작한다.
- 주차 시작 후에만 /cmd_vel 을 발행한다.
- APPROACH 단계는 제거했고, 바로 REVERSE_TURN 으로 진입한다.

센서 역할
- GPS(/ublox_gps_node/fix): 절대 위치 보정
- GPS 속도(/ublox_gps_node/fix_velocity): 단기 속도 보조
- NavPVT(/ublox_gps_node/navpvt): g_speed, heading, head_veh를 이용한 속도/방향 보정
- IMU(/imu/data): yaw(방향) 보정
- IMU magnetometer(/imu/mag): 자기장 상태 확인 및 보조 입력
- steering pot(가변저항): 실제 조향각 피드백

RViz 출력
- /parking_traj_path : 실시간 누적 주행 경로
- /parking_goal_line : 현재 위치 -> 목표점 직선
- /parking_path      : 주차 계획 경로(정적)

주차 상태
IDLE(감시) -> [CALIBRATE, optional] -> REVERSE_TURN -> REVERSE_STRAIGHT -> FINAL_ALIGN -> DONE

주의
- 장애물 회피 없음
- 저속 실주행용
- steering pot 값은 실제 조향각(deg) 또는 raw ADC로 들어올 수 있다.
"""

import math
from threading import Lock

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, Quaternion, PoseStamped, TwistWithCovarianceStamped
from nav_msgs.msg import Path, Odometry
from sensor_msgs.msg import NavSatFix, Imu, MagneticField
from std_msgs.msg import Bool, Float32
from tf_transformations import quaternion_from_euler
from ublox_msgs.msg import NavPVT

try:
    from pyproj import Proj
    HAS_PYPROJ = True
except Exception:
    Proj = None
    HAS_PYPROJ = False


class GpsImuParkingNode(Node):
    def __init__(self):
        super().__init__('gps_imu_parking_node')
        self.lock = Lock()

        # =========================================================
        # 실행 모드
        # =========================================================
        self.auto_start = bool(self.declare_parameter('auto_start', False).value)
        self.enable_heading_calibration = bool(self.declare_parameter('enable_heading_calibration', False).value)

        # /cmd_vel 출력 규약
        # - kmh_deg : linear.x = km/h, angular.z = deg
        # - si      : linear.x = m/s,  angular.z = rad/s
        self.declare_parameter('output_twist_mode', 'kmh_deg')
        self.declare_parameter('output_mode', 'kmh_deg')
        self.output_twist_mode = str(self.get_parameter('output_twist_mode').value)
        if self.output_twist_mode not in ('kmh_deg', 'si'):
            legacy_mode = str(self.get_parameter('output_mode').value)
            self.output_twist_mode = legacy_mode if legacy_mode in ('kmh_deg', 'si') else 'kmh_deg'

        # =========================================================
        # 차량 / 제어
        # =========================================================
        self.wheelbase = float(self.declare_parameter('wheelbase', 0.78).value)
        self.max_steer_deg = float(self.declare_parameter('max_steer_deg', 22.0).value)
        self.max_steer_rad = math.radians(self.max_steer_deg)

        self.reverse_speed_kmh = float(self.declare_parameter('reverse_speed_kmh', 1.2).value)
        self.calibration_speed_kmh = float(self.declare_parameter('calibration_speed_kmh', 2.0).value)
        self.creep_speed_kmh = float(self.declare_parameter('creep_speed_kmh', 0.6).value)

        self.control_hz = float(self.declare_parameter('control_hz', 20.0).value)
        self.max_steer_rate_deg = float(self.declare_parameter('max_steer_rate_deg', 12.0).value)
        self.max_speed_rate_kmh = float(self.declare_parameter('max_speed_rate_kmh', 1.5).value)
        self.steer_deadband_deg = float(self.declare_parameter('steer_deadband_deg', 1.5).value)

        # 직진 보정용 이득
        self.line_follow_kp_cte = float(self.declare_parameter('line_follow_kp_cte', 0.35).value)
        self.line_follow_kp_heading = float(self.declare_parameter('line_follow_kp_heading', 1.00).value)
        self.reverse_line_follow_kp_cte = float(self.declare_parameter('reverse_line_follow_kp_cte', 0.35).value)
        self.reverse_line_follow_kp_heading = float(self.declare_parameter('reverse_line_follow_kp_heading', 0.90).value)

        # steering pot feedback 보정
        self.steering_hold_kp = float(self.declare_parameter('steering_hold_kp', 0.70).value)
        self.steering_hold_max_comp_deg = float(self.declare_parameter('steering_hold_max_comp_deg', 8.0).value)

        # =========================================================
        # 자동 주차 트리거
        # =========================================================
        self.auto_arm_enabled = bool(self.declare_parameter('auto_arm_enabled', True).value)
        self.auto_arm_distance_m = float(self.declare_parameter('auto_arm_distance_m', 10.0).value)
        self.auto_arm_increase_delta_m = float(self.declare_parameter('auto_arm_increase_delta_m', 1.0).value)
        self.auto_arm_persist_cycles = int(self.declare_parameter('auto_arm_persist_cycles', 3).value)
        self.auto_arm_min_speed_kmh = float(self.declare_parameter('auto_arm_min_speed_kmh', 0.4).value)

        # =========================================================
        # 좌표계 / GPS
        # =========================================================
        self.utm_zone = int(self.declare_parameter('utm_zone', 52).value)
        self.proj_utm = Proj(proj='utm', zone=self.utm_zone, ellps='WGS84', preserve_units=False) if HAS_PYPROJ else None

        # =========================================================
        # 목표 주차구역
        # =========================================================
        self.goal_lat = float(self.declare_parameter('parking_spot_lat', 36.9683204).value)
        self.goal_lon = float(self.declare_parameter('parking_spot_lon', 127.8734188).value)
        self.goal_alt = float(self.declare_parameter('parking_spot_alt', 121.65).value)

        self.goal_qx = float(self.declare_parameter('parking_spot_qx', 0.01553492990444338).value)
        self.goal_qy = float(self.declare_parameter('parking_spot_qy', 0.01581687613452909).value)
        self.goal_qz = float(self.declare_parameter('parking_spot_qz', -0.3674679570552528).value)
        self.goal_qw = float(self.declare_parameter('parking_spot_qw', 0.9297719037053378).value)

        self.reference_lat = self.goal_lat
        self.reference_lon = self.goal_lon
        self.goal_x, self.goal_y = self.latlon_to_xy(self.goal_lat, self.goal_lon)
        self.goal_yaw = self.quat_to_yaw(self.goal_qx, self.goal_qy, self.goal_qz, self.goal_qw)
        self.parking_side = int(self.declare_parameter('parking_side', -1).value)

        # REVERSE_TURN 시작 위치
        self.reverse_turn_entry_offset_m = float(self.declare_parameter('reverse_turn_entry_offset_m', 1.2).value)

        # =========================================================
        # 허용오차 / 타임아웃
        # =========================================================
        self.pos_tol = float(self.declare_parameter('pos_tol', 0.30).value)
        self.yaw_tol_deg = float(self.declare_parameter('yaw_tol_deg', 6.0).value)
        self.yaw_tol = math.radians(self.yaw_tol_deg)

        self.timeout_calibrate = float(self.declare_parameter('timeout_calibrate', 20.0).value)
        self.timeout_reverse_turn = float(self.declare_parameter('timeout_reverse_turn', 20.0).value)
        self.timeout_reverse_straight = float(self.declare_parameter('timeout_reverse_straight', 20.0).value)
        self.timeout_final_align = float(self.declare_parameter('timeout_final_align', 15.0).value)

        self.calibration_min_travel_m = float(self.declare_parameter('calibration_min_travel_m', 2.0).value)
        self.calibration_min_elapsed_sec = float(self.declare_parameter('calibration_min_elapsed_sec', 4.0).value)
        self.calibration_min_samples = int(self.declare_parameter('calibration_min_samples', 5).value)
        self.gps_ema_alpha = float(self.declare_parameter('gps_ema_alpha', 0.35).value)
        self.gps_fusion_alpha = float(self.declare_parameter('gps_fusion_alpha', 0.60).value)
        self.status_log_period_sec = float(self.declare_parameter('status_log_period_sec', 1.0).value)

        # RViz trajectory history
        self.traj_history_limit = int(self.declare_parameter('traj_history_limit', 500).value)
        self.traj_min_point_dist = float(self.declare_parameter('traj_min_point_dist', 0.05).value)

        # =========================================================
        # steering pot / 가변저항
        # =========================================================
        self.steering_feedback_topic = str(self.declare_parameter('steering_feedback_topic', '/steering_pot_deg').value)
        self.steering_feedback_mode = str(self.declare_parameter('steering_feedback_mode', 'deg').value)
        self.steering_pot_center_raw = float(self.declare_parameter('steering_pot_center_raw', 0.0).value)
        self.steering_pot_min_raw = float(self.declare_parameter('steering_pot_min_raw', -1.0).value)
        self.steering_pot_max_raw = float(self.declare_parameter('steering_pot_max_raw', 1.0).value)
        self.steering_pot_max_deg = float(self.declare_parameter('steering_pot_max_deg', self.max_steer_deg).value)

        # =========================================================
        # 상태 변수
        # =========================================================
        self.gps_ready = False
        self.imu_ready = False
        self.steer_ready = False
        self.mag_ready = False

        self.raw_lat = 0.0
        self.raw_lon = 0.0
        self.raw_alt = 0.0

        self.gps_x = 0.0
        self.gps_y = 0.0
        self.dr_x = 0.0
        self.dr_y = 0.0
        self.dr_yaw = 0.0
        self.fused_x = 0.0
        self.fused_y = 0.0
        self.fused_yaw = 0.0
        self.prev_fused_x = None
        self.prev_fused_y = None

        self.raw_yaw = 0.0
        self.yaw = 0.0
        self.yaw_offset_rad = 0.0
        self.imu_yaw_samples = []

        self.current_steer_deg = 0.0
        self.current_steer_deg_filtered = 0.0
        self.last_raw_steer_value = 0.0

        self.mag_x = 0.0
        self.mag_y = 0.0
        self.mag_z = 0.0

        self.nav_speed_mps = 0.0
        self.nav_heading_rad = None
        self.nav_heading_valid = False

        self.filtered_initialized = False
        self.calib_start_x = None
        self.calib_start_y = None
        self.calib_start_time = None
        self.calib_ref_yaw = 0.0
        self.is_yaw_aligned = not self.enable_heading_calibration

        # 상태기계
        self.state = 'IDLE'
        self.state_entry_time = self.get_clock().now()
        self.parking_mode_on = False
        self.pending_auto_start = bool(self.auto_start)
        self.state_hold_count = 0

        # 자동 주차 감시 상태
        self.auto_arm_state = 'WAIT'
        self.auto_arm_min_distance = None
        self.auto_arm_last_distance = None
        self.auto_arm_increase_count = 0
        self.auto_arm_triggered = False

        self.latched_goal_x = self.goal_x
        self.latched_goal_y = self.goal_y
        self.latched_goal_yaw = self.goal_yaw
        self.latched_parking_side = self.parking_side

        self.prev_speed_cmd_kmh = 0.0
        self.prev_steer_cmd_deg = 0.0
        self.last_cmd_speed_kmh = 0.0
        self.last_control_time = self.get_clock().now()

        # RViz / path helpers
        self.entry_pose = None
        self.path_points = []
        self.reverse_line_start = None
        self.reverse_line_end = None
        self.traj_history = []
        self.last_traj_pose = None

        # 로그 / 디버그
        self._last_wait_log_sec = 0.0
        self._last_status_log_sec = 0.0
        self.distance_to_goal_m = 0.0
        self.distance_to_target_m = 0.0
        self.active_target_name = 'goal'

        # =========================================================
        # ROS I/O
        # =========================================================
        self.create_subscription(Bool, '/parking_mode', self.mode_cb, 10)
        self.create_subscription(NavSatFix, '/ublox_gps_node/fix', self.gps_cb, 10)
        self.create_subscription(TwistWithCovarianceStamped, '/ublox_gps_node/fix_velocity', self.fix_velocity_cb, 10)
        self.create_subscription(NavPVT, '/ublox_gps_node/navpvt', self.navpvt_cb, 10)
        self.create_subscription(Imu, '/imu/data', self.imu_cb, 10)
        self.create_subscription(MagneticField, '/imu/mag', self.mag_cb, 10)
        self.create_subscription(Float32, self.steering_feedback_topic, self.steer_cb, 10)
        self.create_subscription(PoseStamped, '/slot_pose', self.slot_pose_cb, 10)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.path_pub = self.create_publisher(Path, '/parking_path', 10)
        self.traj_pub = self.create_publisher(Path, '/parking_traj_path', 10)
        self.goal_line_pub = self.create_publisher(Path, '/parking_goal_line', 10)
        self.odom_pub = self.create_publisher(Odometry, '/parking_odom', 10)
        self.current_pose_pub = self.create_publisher(PoseStamped, '/parking_current_pose', 10)
        self.goal_distance_pub = self.create_publisher(Float32, '/parking_distance_to_goal', 10)
        self.target_distance_pub = self.create_publisher(Float32, '/parking_distance_to_target', 10)

        self.timer = self.create_timer(1.0 / self.control_hz, self.control_loop)

        self.get_logger().info('GpsImuParkingNode started.')
        if not HAS_PYPROJ:
            self.get_logger().warn('pyproj not available: using local fallback coordinates around the parking spot.')
        self.get_logger().info(f'Goal XY=({self.goal_x:.3f}, {self.goal_y:.3f}), goal_yaw={self.goal_yaw:.3f} rad')

    # =========================================================
    # Callbacks
    # =========================================================
    def mode_cb(self, msg: Bool):
        with self.lock:
            self.parking_mode_on = bool(msg.data)
            if self.parking_mode_on and self.state == 'IDLE':
                if self.gps_ready and self.imu_ready:
                    self.start_parking()
                else:
                    self.pending_auto_start = True
            elif (not self.parking_mode_on) and self.state != 'IDLE':
                self.stop_vehicle()
                self.pending_auto_start = False
                self.reset_to_idle()

    def slot_pose_cb(self, msg: PoseStamped):
        with self.lock:
            self.goal_x = msg.pose.position.x
            self.goal_y = msg.pose.position.y
            q = msg.pose.orientation
            self.goal_yaw = self.quat_to_yaw(q.x, q.y, q.z, q.w)

    def gps_cb(self, msg: NavSatFix):
        if msg.latitude == 0.0 and msg.longitude == 0.0:
            return

        with self.lock:
            self.raw_lat = float(msg.latitude)
            self.raw_lon = float(msg.longitude)
            self.raw_alt = float(msg.altitude)

            x, y = self.latlon_to_xy(self.raw_lat, self.raw_lon)
            self.gps_x, self.gps_y = x, y

            if not self.filtered_initialized:
                self.dr_x, self.dr_y = x, y
                self.fused_x, self.fused_y = x, y
                self.filtered_initialized = True
            else:
                self.fused_x = self.gps_fusion_alpha * x + (1.0 - self.gps_fusion_alpha) * self.dr_x
                self.fused_y = self.gps_fusion_alpha * y + (1.0 - self.gps_fusion_alpha) * self.dr_y

            self.gps_ready = True
            self.publish_current_pose()

    def fix_velocity_cb(self, msg: TwistWithCovarianceStamped):
        with self.lock:
            vx = float(msg.twist.twist.linear.x)
            vy = float(msg.twist.twist.linear.y)
            self.nav_speed_mps = math.hypot(vx, vy)
            self.nav_speed_mps = max(self.nav_speed_mps, 0.0)

    def navpvt_cb(self, msg: NavPVT):
        with self.lock:
            g_speed_mm_s = getattr(msg, 'g_speed', getattr(msg, 'gSpeed', 0.0))
            self.nav_speed_mps = max(float(g_speed_mm_s) / 1000.0, self.nav_speed_mps)

            head_valid_mask = getattr(NavPVT, 'FLAGS_HEAD_VEH_VALID', 0)
            flags = int(getattr(msg, 'flags', 0))
            head_veh = getattr(msg, 'head_veh', getattr(msg, 'headVeh', None))
            heading = getattr(msg, 'heading', None)

            if head_veh is not None and (head_valid_mask == 0 or (flags & head_valid_mask) != 0):
                self.nav_heading_rad = math.radians(float(head_veh) * 1e-5)
                self.nav_heading_valid = True
            elif heading is not None:
                self.nav_heading_rad = math.radians(float(heading) * 1e-5)
                self.nav_heading_valid = True
            else:
                self.nav_heading_valid = False

    def imu_cb(self, msg: Imu):
        with self.lock:
            q = msg.orientation
            raw_yaw = self.quat_to_yaw(q.x, q.y, q.z, q.w)
            self.raw_yaw = raw_yaw

            if not self.is_yaw_aligned:
                if self.calib_start_x is not None:
                    self.imu_yaw_samples.append(raw_yaw)
                self.yaw = raw_yaw
            else:
                self.yaw = self.wrap_angle(raw_yaw + self.yaw_offset_rad)

            self.imu_ready = True
            self.publish_current_pose()

    def mag_cb(self, msg: MagneticField):
        with self.lock:
            self.mag_x = float(msg.magnetic_field.x)
            self.mag_y = float(msg.magnetic_field.y)
            self.mag_z = float(msg.magnetic_field.z)
            self.mag_ready = True

    def steer_cb(self, msg: Float32):
        with self.lock:
            raw = float(msg.data)
            self.last_raw_steer_value = raw

            if self.steering_feedback_mode == 'raw':
                span = max(self.steering_pot_max_raw - self.steering_pot_min_raw, 1e-6)
                normalized = (raw - self.steering_pot_center_raw) / (span * 0.5)
                normalized = self.clamp(normalized, -1.0, 1.0)
                self.current_steer_deg = normalized * self.steering_pot_max_deg
            else:
                self.current_steer_deg = raw

            self.current_steer_deg_filtered = 0.7 * self.current_steer_deg_filtered + 0.3 * self.current_steer_deg
            self.steer_ready = True

    # =========================================================
    # FSM start / reset
    # =========================================================
    def start_parking(self):
        self.latch_goal()
        self.generate_visual_path()
        self.publish_visual_path()
        self.state_hold_count = 0
        self.auto_arm_triggered = False
        self.auto_arm_state = 'WAIT'

        if self.enable_heading_calibration:
            self.state = 'CALIBRATE'
            self.calib_start_x = self.fused_x
            self.calib_start_y = self.fused_y
            self.calib_start_time = self.get_clock().now()
            self.calib_ref_yaw = self.yaw
            self.imu_yaw_samples = []
            self.is_yaw_aligned = False
            self.active_target_name = 'calibration'
        else:
            self.is_yaw_aligned = True
            self.transition('REVERSE_TURN', stop_vehicle=False)

        self.state_entry_time = self.get_clock().now()
        self.pending_auto_start = False
        self.get_logger().info(f'Parking started. state={self.state}')

    def reset_to_idle(self):
        self.state = 'IDLE'
        self.state_entry_time = self.get_clock().now()
        self.calib_start_x = None
        self.calib_start_y = None
        self.calib_start_time = None
        self.imu_yaw_samples = []
        self.is_yaw_aligned = not self.enable_heading_calibration
        self.prev_speed_cmd_kmh = 0.0
        self.prev_steer_cmd_deg = 0.0
        self.state_hold_count = 0
        self.active_target_name = 'goal'
        self.auto_arm_state = 'WAIT'
        self.auto_arm_min_distance = None
        self.auto_arm_last_distance = None
        self.auto_arm_increase_count = 0
        self.auto_arm_triggered = False
        self.traj_history = []
        self.last_traj_pose = None

    def latch_goal(self):
        self.latched_goal_x = self.goal_x
        self.latched_goal_y = self.goal_y
        self.latched_goal_yaw = self.goal_yaw
        self.latched_parking_side = self.parking_side

    # =========================================================
    # Main loop
    # =========================================================
    def control_loop(self):
        with self.lock:
            now = self.get_clock().now()
            dt = max((now - self.last_control_time).nanoseconds * 1e-9, 1e-3)
            self.last_control_time = now

            self.update_dead_reckoning(dt)
            self.update_fused_position()
            self.update_distance_metrics()
            self.update_traj_history(now)

            # 주차 전 감시 단계: 차량 명령은 내리지 않고 거리만 본다.
            if self.auto_arm_enabled and not self.parking_mode_on and self.state == 'IDLE':
                self.update_auto_arm_watch()
                if self.auto_arm_triggered and self.gps_ready and self.imu_ready:
                    self.parking_mode_on = True
                    self.start_parking()

            if not self.gps_ready or not self.imu_ready:
                current_sec = now.nanoseconds * 1e-9
                if current_sec - self._last_wait_log_sec > 2.0:
                    self.get_logger().warn('Waiting for GPS / IMU...')
                    self._last_wait_log_sec = current_sec
                self.publish_internal_odom()
                self.publish_current_pose()
                self.publish_distance_topics()
                self.publish_goal_line()
                self.publish_traj_history()
                return

            if not self.parking_mode_on and self.state == 'IDLE':
                self.publish_internal_odom()
                self.publish_current_pose()
                self.publish_distance_topics()
                self.publish_goal_line()
                self.publish_traj_history()
                self.maybe_log_status()
                return

            if self.pending_auto_start and self.state == 'IDLE' and self.gps_ready and self.imu_ready:
                self.start_parking()

            if self.state == 'IDLE':
                self.publish_internal_odom()
                self.publish_current_pose()
                self.publish_distance_topics()
                self.publish_goal_line()
                self.publish_traj_history()
                self.maybe_log_status()
                return

            if self.state_timed_out(now):
                self.get_logger().warn(f'Timeout in state {self.state}')
                self.stop_vehicle()
                self.reset_to_idle()
                self.publish_internal_odom()
                self.publish_current_pose()
                self.publish_distance_topics()
                self.publish_goal_line()
                self.publish_traj_history()
                self.maybe_log_status()
                return

            if self.state == 'CALIBRATE':
                self.run_calibration(dt)
            elif self.state == 'REVERSE_TURN':
                self.run_reverse_turn(dt)
            elif self.state == 'REVERSE_STRAIGHT':
                self.run_reverse_straight(dt)
            elif self.state == 'FINAL_ALIGN':
                self.run_final_align(dt)
            elif self.state == 'DONE':
                self.stop_vehicle()
            else:
                self.stop_vehicle()

            self.update_fused_position()
            self.update_distance_metrics()
            self.update_traj_history(now)
            self.publish_internal_odom()
            self.publish_current_pose()
            self.publish_distance_topics()
            self.publish_goal_line()
            self.publish_traj_history()
            self.maybe_log_status()

    # =========================================================
    # Auto-arm watching
    # =========================================================
    def update_auto_arm_watch(self):
        if not self.gps_ready:
            return

        motion_speed_kmh = abs(self.get_motion_speed_mps()) * 3.6
        dist = self.distance_to_goal_m

        if self.auto_arm_state == 'WAIT':
            if dist <= self.auto_arm_distance_m:
                self.auto_arm_state = 'ARMED'
                self.auto_arm_min_distance = dist
                self.auto_arm_last_distance = dist
                self.auto_arm_increase_count = 0
                self.get_logger().info(
                    f'Auto-arm window entered: d_goal={dist:.2f}m (watch <= {self.auto_arm_distance_m:.1f}m)'
                )
            return

        if self.auto_arm_state == 'ARMED':
            if self.auto_arm_min_distance is None or dist < self.auto_arm_min_distance:
                self.auto_arm_min_distance = dist
                self.auto_arm_increase_count = 0
            else:
                is_moving = motion_speed_kmh >= self.auto_arm_min_speed_kmh
                is_past_min = dist > (self.auto_arm_min_distance + self.auto_arm_increase_delta_m)
                is_increasing = self.auto_arm_last_distance is None or dist > self.auto_arm_last_distance

                if is_moving and is_past_min and is_increasing:
                    self.auto_arm_increase_count += 1
                else:
                    self.auto_arm_increase_count = 0

            self.auto_arm_last_distance = dist

            if self.auto_arm_increase_count >= self.auto_arm_persist_cycles:
                self.auto_arm_triggered = True
                self.get_logger().info(
                    f'Auto parking triggered: min={self.auto_arm_min_distance:.2f}m, now={dist:.2f}m'
                )

    # =========================================================
    # Speed / motion helpers
    # =========================================================
    def get_motion_speed_mps(self):
        if self.nav_speed_mps > 0.0:
            sign = -1.0 if self.last_cmd_speed_kmh < 0.0 else 1.0
            return self.nav_speed_mps * sign

        if self.last_cmd_speed_kmh != 0.0:
            return self.last_cmd_speed_kmh / 3.6

        return 0.0

    # =========================================================
    # DR / fusion
    # =========================================================
    def update_dead_reckoning(self, dt):
        if not self.imu_ready:
            return

        actual_steer_deg = self.current_steer_deg_filtered if self.steer_ready else self.prev_steer_cmd_deg
        actual_steer_rad = math.radians(actual_steer_deg)
        motion_speed_mps = self.get_motion_speed_mps()

        self.dr_yaw = self.wrap_angle(
            self.dr_yaw + (motion_speed_mps / max(self.wheelbase, 1e-6)) * math.tan(actual_steer_rad) * dt
        )
        self.dr_x += motion_speed_mps * math.cos(self.dr_yaw) * dt
        self.dr_y += motion_speed_mps * math.sin(self.dr_yaw) * dt

        # DR yaw가 IMU yaw와 너무 멀리 벌어지지 않도록 약하게 따라가게 한다.
        self.dr_yaw = self.wrap_angle(0.97 * self.dr_yaw + 0.03 * self.yaw)

    def update_fused_position(self):
        if self.gps_ready:
            self.fused_x = self.gps_fusion_alpha * self.gps_x + (1.0 - self.gps_fusion_alpha) * self.dr_x
            self.fused_y = self.gps_fusion_alpha * self.gps_y + (1.0 - self.gps_fusion_alpha) * self.dr_y
        else:
            self.fused_x = self.dr_x
            self.fused_y = self.dr_y

        self.fused_yaw = self.yaw

    # =========================================================
    # Distance metrics
    # =========================================================
    def update_distance_metrics(self):
        self.distance_to_goal_m = self.euclidean(self.fused_x, self.fused_y, self.latched_goal_x, self.latched_goal_y)

        if self.state == 'CALIBRATE' and self.calib_start_x is not None and self.calib_start_y is not None:
            self.distance_to_target_m = self.euclidean(self.fused_x, self.fused_y, self.calib_start_x, self.calib_start_y)
            self.active_target_name = 'calibration'
        elif self.state == 'REVERSE_TURN' and self.entry_pose is not None:
            tx, ty, _ = self.entry_pose
            self.distance_to_target_m = self.euclidean(self.fused_x, self.fused_y, tx, ty)
            self.active_target_name = 'entry'
        else:
            self.distance_to_target_m = self.distance_to_goal_m
            self.active_target_name = 'goal'

    # =========================================================
    # Trajectory history for RViz
    # =========================================================
    def update_traj_history(self, now):
        if not self.gps_ready:
            return

        if self.last_traj_pose is None:
            self.traj_history.append((self.fused_x, self.fused_y, self.fused_yaw, now))
            self.last_traj_pose = (self.fused_x, self.fused_y)
            return

        dx = self.fused_x - self.last_traj_pose[0]
        dy = self.fused_y - self.last_traj_pose[1]
        if math.hypot(dx, dy) >= self.traj_min_point_dist:
            self.traj_history.append((self.fused_x, self.fused_y, self.fused_yaw, now))
            self.last_traj_pose = (self.fused_x, self.fused_y)

        if len(self.traj_history) > self.traj_history_limit:
            self.traj_history = self.traj_history[-self.traj_history_limit:]

    # =========================================================
    # Timeout
    # =========================================================
    def state_timed_out(self, now):
        elapsed = (now - self.state_entry_time).nanoseconds * 1e-9
        if self.state == 'CALIBRATE':
            return elapsed > self.timeout_calibrate
        if self.state == 'REVERSE_TURN':
            return elapsed > self.timeout_reverse_turn
        if self.state == 'REVERSE_STRAIGHT':
            return elapsed > self.timeout_reverse_straight
        if self.state == 'FINAL_ALIGN':
            return elapsed > self.timeout_final_align
        return False

    # =========================================================
    # Calibration (optional)
    # =========================================================
    def run_calibration(self, dt):
        if self.calib_start_x is None or self.calib_start_y is None:
            self.calib_start_x = self.fused_x
            self.calib_start_y = self.fused_y
        if self.calib_start_time is None:
            self.calib_start_time = self.get_clock().now()

        traveled = self.euclidean(self.fused_x, self.fused_y, self.calib_start_x, self.calib_start_y)
        elapsed = (self.get_clock().now() - self.calib_start_time).nanoseconds * 1e-9
        self.active_target_name = 'calibration'

        if self.steer_ready:
            cmd_steer_deg = math.degrees(self.line_follow_steer(
                self.calib_start_x, self.calib_start_y,
                self.calib_start_x + math.cos(self.calib_ref_yaw) * 3.0,
                self.calib_start_y + math.sin(self.calib_ref_yaw) * 3.0,
                reverse=False,
            ))
        else:
            cmd_steer_deg = self.heading_hold_to_deg(self.calib_ref_yaw)

        self.publish_cmd(self.calibration_speed_kmh, cmd_steer_deg, dt)

        if traveled < self.calibration_min_travel_m or elapsed < self.calibration_min_elapsed_sec:
            self.state_hold_count = 0
            self.get_logger().info(
                f'Calibrating heading... traveled={traveled:.2f}m / {self.calibration_min_travel_m:.2f}m, '
                f'elapsed={elapsed:.1f}s / {self.calibration_min_elapsed_sec:.1f}s',
                throttle_duration_sec=1.0,
            )
            return

        if len(self.imu_yaw_samples) < self.calibration_min_samples:
            self.state_hold_count = 0
            self.get_logger().warn(f'Calibration samples 부족: {len(self.imu_yaw_samples)} / {self.calibration_min_samples}')
            return

        self.state_hold_count += 1
        if self.state_hold_count < self.state_hold_cycles:
            self.get_logger().info(
                f'Calibration settling... {self.state_hold_count}/{self.state_hold_cycles}',
                throttle_duration_sec=0.8,
            )
            return

        gps_h = math.atan2(self.fused_y - self.calib_start_y, self.fused_x - self.calib_start_x)
        imu_h = self.circular_mean(self.imu_yaw_samples)
        self.yaw_offset_rad = self.wrap_angle(gps_h - imu_h)
        self.is_yaw_aligned = True
        self.state_hold_count = 0
        self.get_logger().info(f'Calibration success. yaw_offset={math.degrees(self.yaw_offset_rad):.2f} deg')
        self.transition('REVERSE_TURN', stop_vehicle=False)

    # =========================================================
    # Parking states
    # =========================================================
    def run_reverse_turn(self, dt):
        tx, ty, tyaw = self.entry_pose
        pos_err = self.euclidean(self.fused_x, self.fused_y, tx, ty)
        yaw_err = self.wrap_angle(tyaw - self.yaw)

        if pos_err < 0.25 and abs(yaw_err) < math.radians(10.0):
            self.state_hold_count += 1
            if self.state_hold_count >= self.state_hold_cycles:
                self.state_hold_count = 0
                self.transition('REVERSE_STRAIGHT', stop_vehicle=False)
                return
        else:
            self.state_hold_count = 0

        cmd_steer_deg = math.degrees(self.reverse_steer_to(tx, ty, tyaw))
        cmd_speed_kmh = -self.scale_speed_from_distance(pos_err, self.reverse_speed_kmh, min_kmh=0.3, gain=0.80)
        if pos_err < 0.8:
            cmd_speed_kmh = max(cmd_speed_kmh, -0.8)
        self.publish_cmd(cmd_speed_kmh, cmd_steer_deg, dt)

    def run_reverse_straight(self, dt):
        gx, gy, gyaw = self.latched_goal_x, self.latched_goal_y, self.latched_goal_yaw
        pos_err = self.euclidean(self.fused_x, self.fused_y, gx, gy)
        yaw_err = self.wrap_angle(gyaw - self.yaw)

        if pos_err < 0.35 and abs(yaw_err) < math.radians(8.0):
            self.state_hold_count += 1
            if self.state_hold_count >= self.state_hold_cycles:
                self.state_hold_count = 0
                self.transition('FINAL_ALIGN', stop_vehicle=False)
                return
        else:
            self.state_hold_count = 0

        cmd_steer_deg = math.degrees(self.line_follow_steer(
            self.reverse_line_start[0], self.reverse_line_start[1],
            self.reverse_line_end[0], self.reverse_line_end[1],
            reverse=True,
        ))
        cmd_steer_deg = self.clamp(cmd_steer_deg, -15.0, 15.0)
        cmd_speed_kmh = -self.scale_speed_from_distance(pos_err, self.reverse_speed_kmh, min_kmh=0.3, gain=0.70)
        cmd_speed_kmh = max(cmd_speed_kmh, -0.8)
        self.publish_cmd(cmd_speed_kmh, cmd_steer_deg, dt)

    def run_final_align(self, dt):
        gx, gy, gyaw = self.latched_goal_x, self.latched_goal_y, self.latched_goal_yaw
        pos_err = self.euclidean(self.fused_x, self.fused_y, gx, gy)
        yaw_err = self.wrap_angle(gyaw - self.yaw)

        if pos_err < self.pos_tol and abs(yaw_err) < self.yaw_tol:
            self.state_hold_count += 1
            if self.state_hold_count >= self.state_hold_cycles:
                self.stop_vehicle()
                self.state = 'DONE'
                self.state_hold_count = 0
                self.get_logger().info('Parking complete.')
                return
        else:
            self.state_hold_count = 0

        local_x, local_y = self.global_to_local(gx, gy, gyaw, self.fused_x, self.fused_y)
        cmd_steer_deg = math.degrees(1.35 * local_y + 0.85 * yaw_err)
        cmd_steer_deg = self.clamp(cmd_steer_deg, -12.0, 12.0)
        if abs(cmd_steer_deg) < self.steer_deadband_deg:
            cmd_steer_deg = 0.0

        if abs(local_x) > 0.08:
            cmd_speed_kmh = -self.creep_speed_kmh if local_x > 0.0 else self.creep_speed_kmh
        else:
            cmd_speed_kmh = 0.0
        self.publish_cmd(cmd_speed_kmh, cmd_steer_deg, dt)

    # =========================================================
    # Steering helpers
    # =========================================================
    def line_follow_steer(self, x0, y0, x1, y1, reverse=False):
        dx = x1 - x0
        dy = y1 - y0
        line_len = math.hypot(dx, dy)
        if line_len < 1e-6:
            return 0.0

        line_heading = math.atan2(dy, dx)
        cross = ((self.fused_x - x0) * dy - (self.fused_y - y0) * dx) / line_len
        heading_error = self.wrap_angle(line_heading - self.yaw)

        if reverse:
            steer = -(self.reverse_line_follow_kp_cte * cross + self.reverse_line_follow_kp_heading * heading_error)
        else:
            steer = self.line_follow_kp_cte * cross + self.line_follow_kp_heading * heading_error
        return self.clamp(steer, -self.max_steer_rad, self.max_steer_rad)

    def heading_hold_to_deg(self, ref_yaw):
        yaw_err = self.wrap_angle(ref_yaw - self.yaw)
        steer = self.line_follow_kp_heading * yaw_err
        return math.degrees(self.clamp(steer, -self.max_steer_rad, self.max_steer_rad))

    def scale_speed_from_distance(self, dist_m, max_kmh, min_kmh=0.5, gain=0.8):
        scaled = gain * dist_m * 3.6
        return min(max_kmh, max(min_kmh, scaled))

    def reverse_steer_to(self, tx, ty, tyaw):
        dx = tx - self.fused_x
        dy = ty - self.fused_y
        x_local, y_local = self.global_to_vehicle(dx, dy)
        yaw_err = self.wrap_angle(tyaw - self.yaw)
        steer = -(1.30 * y_local + 0.75 * yaw_err)
        return self.clamp(steer, -self.max_steer_rad, self.max_steer_rad)

    def apply_steering_feedback_compensation(self, cmd_steer_deg):
        if not self.steer_ready:
            return cmd_steer_deg
        actual = self.current_steer_deg_filtered
        error = cmd_steer_deg - actual
        compensation = self.steering_hold_kp * error
        compensation = self.clamp(compensation, -self.steering_hold_max_comp_deg, self.steering_hold_max_comp_deg)
        return self.clamp(cmd_steer_deg + compensation, -self.max_steer_deg, self.max_steer_deg)

    def publish_cmd(self, cmd_speed_kmh, cmd_steer_deg, dt):
        cmd_speed_kmh = self.rate_limit(cmd_speed_kmh, self.prev_speed_cmd_kmh, self.max_speed_rate_kmh, dt)
        cmd_steer_deg = self.rate_limit(cmd_steer_deg, self.prev_steer_cmd_deg, self.max_steer_rate_deg, dt)
        cmd_steer_deg = self.apply_steering_feedback_compensation(cmd_steer_deg)

        cmd_speed_kmh = self.clamp(cmd_speed_kmh, -self.reverse_speed_kmh, self.calibration_speed_kmh)
        cmd_steer_deg = self.clamp(cmd_steer_deg, -self.max_steer_deg, self.max_steer_deg)

        self.prev_speed_cmd_kmh = cmd_speed_kmh
        self.prev_steer_cmd_deg = cmd_steer_deg
        self.last_cmd_speed_kmh = cmd_speed_kmh

        msg = Twist()
        if self.output_twist_mode == 'si':
            msg.linear.x = float(cmd_speed_kmh / 3.6)
            msg.angular.z = float(math.radians(cmd_steer_deg))
        else:
            msg.linear.x = float(cmd_speed_kmh)
            msg.angular.z = float(cmd_steer_deg)
        self.cmd_pub.publish(msg)

    def stop_vehicle(self):
        self.prev_speed_cmd_kmh = 0.0
        self.prev_steer_cmd_deg = 0.0
        self.last_cmd_speed_kmh = 0.0
        self.publish_cmd(0.0, 0.0, 1.0 / self.control_hz)

    # =========================================================
    # RViz / pose / odom / distance publish
    # =========================================================
    def publish_internal_odom(self):
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'map'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = float(self.fused_x)
        odom.pose.pose.position.y = float(self.fused_y)
        odom.pose.pose.position.z = float(self.raw_alt)

        q = quaternion_from_euler(0.0, 0.0, self.yaw)
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]
        odom.twist.twist.linear.x = float(self.last_cmd_speed_kmh / 3.6)
        self.odom_pub.publish(odom)

    def publish_current_pose(self):
        if not self.gps_ready:
            return

        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'map'
        pose.pose.position.x = float(self.fused_x)
        pose.pose.position.y = float(self.fused_y)
        pose.pose.position.z = float(self.raw_alt)
        q = quaternion_from_euler(0.0, 0.0, self.yaw)
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]
        self.current_pose_pub.publish(pose)

    def publish_distance_topics(self):
        goal_msg = Float32()
        goal_msg.data = float(self.distance_to_goal_m)
        self.goal_distance_pub.publish(goal_msg)

        target_msg = Float32()
        target_msg.data = float(self.distance_to_target_m)
        self.target_distance_pub.publish(target_msg)

    def publish_goal_line(self):
        if not self.gps_ready:
            return

        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = self.get_clock().now().to_msg()

        start = (self.fused_x, self.fused_y, self.fused_yaw)
        end = (self.latched_goal_x, self.latched_goal_y, self.latched_goal_yaw)
        for x, y, yaw in [start, end]:
            ps = PoseStamped()
            ps.header.frame_id = 'map'
            ps.pose.position.x = float(x)
            ps.pose.position.y = float(y)
            q = quaternion_from_euler(0.0, 0.0, yaw)
            ps.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
            path.poses.append(ps)

        self.goal_line_pub.publish(path)

    def publish_traj_history(self):
        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = self.get_clock().now().to_msg()

        for x, y, yaw, stamp in self.traj_history:
            ps = PoseStamped()
            ps.header.frame_id = 'map'
            ps.header.stamp = stamp.to_msg() if hasattr(stamp, 'to_msg') else self.get_clock().now().to_msg()
            ps.pose.position.x = float(x)
            ps.pose.position.y = float(y)
            q = quaternion_from_euler(0.0, 0.0, yaw)
            ps.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
            path.poses.append(ps)

        self.traj_pub.publish(path)

    def update_traj_history(self, now):
        if not self.gps_ready:
            return

        if self.last_traj_pose is None:
            self.traj_history.append((self.fused_x, self.fused_y, self.fused_yaw, now))
            self.last_traj_pose = (self.fused_x, self.fused_y)
            return

        dx = self.fused_x - self.last_traj_pose[0]
        dy = self.fused_y - self.last_traj_pose[1]
        if math.hypot(dx, dy) >= self.traj_min_point_dist:
            self.traj_history.append((self.fused_x, self.fused_y, self.fused_yaw, now))
            self.last_traj_pose = (self.fused_x, self.fused_y)

        if len(self.traj_history) > self.traj_history_limit:
            self.traj_history = self.traj_history[-self.traj_history_limit:]

    def maybe_log_status(self):
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        if now_sec - self._last_status_log_sec < self.status_log_period_sec:
            return
        self._last_status_log_sec = now_sec

        if self.state == 'IDLE' and not self.parking_mode_on and self.auto_arm_enabled:
            runtime_mode = f'WATCHING({self.auto_arm_state})'
        elif self.state == 'IDLE' and self.parking_mode_on:
            runtime_mode = 'IDLE(armed)'
        else:
            runtime_mode = self.state

        self.get_logger().info(
            f'[STATUS] mode={runtime_mode} '
            f'pose=({self.fused_x:.3f},{self.fused_y:.3f}) '
            f'yaw={math.degrees(self.yaw):.1f}deg '
            f'steer={self.current_steer_deg_filtered:.1f}deg '
            f'cmd=({self.prev_speed_cmd_kmh:.2f}km/h, {self.prev_steer_cmd_deg:.1f}deg) '
            f'd_goal={self.distance_to_goal_m:.2f}m '
            f'd_target={self.distance_to_target_m:.2f}m '
            f'gps={int(self.gps_ready)} imu={int(self.imu_ready)} steer_fb={int(self.steer_ready)} '
            f'target={self.active_target_name}'
        )

    # =========================================================
    # Path visualization
    # =========================================================
    def generate_visual_path(self):
        self.path_points = []
        gx, gy, gyaw = self.latched_goal_x, self.latched_goal_y, self.latched_goal_yaw
        side = self.latched_parking_side

        entry_local = (-max(1.0, self.reverse_turn_entry_offset_m), 0.0)
        entry_gx, entry_gy = self.local_to_global(gx, gy, gyaw, entry_local[0], entry_local[1])
        self.entry_pose = (entry_gx, entry_gy, gyaw)

        for t in self.linspace(0.0, math.pi / 2.0, 25):
            x_local = entry_local[0] + self.reverse_turn_entry_offset_m * math.sin(t)
            y_local = side * self.reverse_turn_entry_offset_m * math.cos(t) * 0.6
            px, py = self.local_to_global(gx, gy, gyaw, x_local, y_local)
            pyaw = self.wrap_angle(gyaw + side * t)
            self.path_points.append((px, py, pyaw))

        for x_local in self.linspace(entry_local[0], 0.0, 18):
            px, py = self.local_to_global(gx, gy, gyaw, x_local, 0.0)
            self.path_points.append((px, py, gyaw))

        self.path_points.append((gx, gy, gyaw))
        self.reverse_line_start = (entry_gx, entry_gy)
        self.reverse_line_end = (gx, gy)

    def publish_visual_path(self):
        path_msg = Path()
        path_msg.header.frame_id = 'map'
        path_msg.header.stamp = self.get_clock().now().to_msg()

        for x, y, yaw in self.path_points:
            ps = PoseStamped()
            ps.header.frame_id = 'map'
            ps.pose.position.x = float(x)
            ps.pose.position.y = float(y)
            q = quaternion_from_euler(0.0, 0.0, yaw)
            ps.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
            path_msg.poses.append(ps)

        self.path_pub.publish(path_msg)

    def linspace(self, start, stop, num):
        if num <= 1:
            return [float(start)]
        step = (stop - start) / float(num - 1)
        return [start + i * step for i in range(num)]

    # =========================================================
    # Math helpers
    # =========================================================
    def latlon_to_xy(self, lat, lon):
        if HAS_PYPROJ and self.proj_utm is not None:
            x, y = self.proj_utm(lon, lat)
            return float(x), float(y)
        lat_scale = 110540.0
        lon_scale = 111320.0 * math.cos(math.radians(self.reference_lat))
        x = (lon - self.reference_lon) * lon_scale
        y = (lat - self.reference_lat) * lat_scale
        return float(x), float(y)

    def quat_to_yaw(self, x, y, z, w):
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    def global_to_vehicle(self, dx, dy):
        x_local = math.cos(self.yaw) * dx + math.sin(self.yaw) * dy
        y_local = -math.sin(self.yaw) * dx + math.cos(self.yaw) * dy
        return x_local, y_local

    def global_to_local(self, ox, oy, oyaw, x, y):
        dx = x - ox
        dy = y - oy
        c = math.cos(oyaw)
        s = math.sin(oyaw)
        lx = c * dx + s * dy
        ly = -s * dx + c * dy
        return lx, ly

    def local_to_global(self, ox, oy, oyaw, lx, ly):
        c = math.cos(oyaw)
        s = math.sin(oyaw)
        gx = ox + c * lx - s * ly
        gy = oy + s * lx + c * ly
        return gx, gy

    def euclidean(self, x1, y1, x2, y2):
        return math.hypot(x2 - x1, y2 - y1)

    def wrap_angle(self, ang):
        return math.atan2(math.sin(ang), math.cos(ang))

    def circular_mean(self, angles):
        if not angles:
            return 0.0
        s = sum(math.sin(a) for a in angles)
        c = sum(math.cos(a) for a in angles)
        return math.atan2(s, c)

    def clamp(self, v, lo, hi):
        return max(lo, min(hi, v))

    def rate_limit(self, target, prev, max_rate_per_sec, dt):
        delta = target - prev
        max_delta = max_rate_per_sec * dt
        return prev + self.clamp(delta, -max_delta, max_delta)

    # =========================================================
    # State transition
    # =========================================================
    def transition(self, new_state, stop_vehicle=False):
        if stop_vehicle:
            self.stop_vehicle()
        self.state = new_state
        self.state_entry_time = self.get_clock().now()
        self.state_hold_count = 0
        self.get_logger().info(f'State -> {new_state}')


def main(args=None):
    rclpy.init(args=args)
    node = GpsImuParkingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_vehicle()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
