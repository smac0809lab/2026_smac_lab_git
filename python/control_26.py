#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
import numpy as np
import os
from datetime import datetime
from pyproj import Proj
from threading import Lock
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
from sensor_msgs.msg import NavSatFix, Imu
from std_msgs.msg import Float32
from tf_transformations import euler_from_quaternion

class GpsDirectStanley(Node):
    def __init__(self):
        super().__init__('gps_direct_stanley')
        self.lock = Lock()

        # [차량 파라미터]
        self.L = 0.72               # 축거 (Wheelbase)
        self.k = 0.2                # Stanley Gain (속도에 따른 민감도)
        self.max_steer_deg = 22.0
        self.max_steer_rad = np.deg2rad(self.max_steer_deg) 
        self.target_speed_kmh = 5.0 # 주행 목표 속도
        
        # [자동 정렬(Auto Alignment) 변수]
        self.is_yaw_aligned = False
        self.yaw_offset_deg = 0.0
        self.start_pos = None       # 정렬 시작점 UTM 좌표
        self.align_dist_threshold = 2.0 # 2미터 전진 후 정렬 완료
        self.imu_yaw_samples = []   # 정렬 중 IMU 데이터 수집용

        # [상태 변수]
        self.proj_utm = Proj(proj='utm', zone=52, ellps='WGS84', preserve_units=False)
        self.pos_x, self.pos_y, self.yaw = 0.0, 0.0, 0.0
        self.current_v = 0.0        # ESP32 수신 속도 (km/h)
        self.target_path = None     # /local_path 수신 경로
        self.is_gps_init = False
        self.is_imu_init = False
        self.is_path_init = False
        self.raw_imu = Imu()
        
        # 로그 설정
        self.log_dir = '/home/user/ros2_ws/src/python/logs'
        self.init_csv_logger()

        # 통신 설정 (Sub/Pub)
        self.create_subscription(NavSatFix, '/ublox_gps_node/fix', self.gps_cb, 10)
        self.create_subscription(Imu, '/imu/data', self.imu_cb, 10)
        self.create_subscription(Path, '/local_path', self.path_cb, 10)
        self.create_subscription(Float32, '/calculateSpeed', self.speed_cb, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # 메인 루프 (20Hz)
        self.create_timer(0.05, self.main_control_loop)

    def main_control_loop(self):
        # 센서 데이터 수신 대기
        if not (self.is_gps_init and self.is_imu_init and self.is_path_init):
            self.get_logger().info("Waiting for Sensors or Path...", throttle_duration_sec=2.0)
            return

        with self.lock:
            curr_x, curr_y, curr_yaw, curr_v = self.pos_x, self.pos_y, self.yaw, self.current_v
            path, imu = self.target_path, self.raw_imu

        # [단계 1] 자동 정렬 단계 (2m 전진)
        if not self.is_yaw_aligned:
            self.process_auto_alignment(curr_x, curr_y)
            return

        # [단계 2] Stanley 경로 추종 단계
        # 프런트 액슬(전륜) 위치 계산
        fx = curr_x + self.L * np.cos(curr_yaw)
        fy = curr_y + self.L * np.sin(curr_yaw)
        
        # 가장 가까운 경로점 찾기
        dists = np.hypot(path[:, 0] - fx, path[:, 1] - fy)
        target_idx = np.argmin(dists)

        # 경로의 헤딩 계산
        if target_idx < len(path) - 1:
            path_yaw = np.arctan2(path[target_idx+1, 1] - path[target_idx, 1], 
                                  path[target_idx+1, 0] - path[target_idx, 0])
        else:
            path_yaw = curr_yaw

        # CTE(Cross Track Error) 및 헤딩 에러 계산
        dx, dy = fx - path[target_idx, 0], fy - path[target_idx, 1]
        cte = dy * np.cos(path_yaw) - dx * np.sin(path_yaw)
        theta_e = np.arctan2(np.sin(path_yaw - curr_yaw), np.cos(path_yaw - curr_yaw))

        # Stanley 공식 적용 (v_eff는 m/s 단위 사용)
        v_eff = max(abs(curr_v / 3.6), 0.5) 
        steer_rad = theta_e + np.arctan2(self.k * cte, v_eff)
        clipped_steer_rad = np.clip(steer_rad, -self.max_steer_rad, self.max_steer_rad)

        # ESP32 전송용 단위 변환 (km/h, Degree)
        steer_deg = np.rad2deg(clipped_steer_rad)
        self.publish_cmd(self.target_speed_kmh, steer_deg)

        # 터미널 및 CSV 로그 기록
        self.print_status_to_terminal(curr_x, curr_y, curr_yaw, cte, steer_deg, curr_v)
        self.save_to_csv(curr_x, curr_y, np.rad2deg(theta_e), cte, curr_v, imu, self.target_speed_kmh, steer_deg)

    def process_auto_alignment(self, curr_x, curr_y):
        """2m 전진하며 GPS와 IMU 사이의 각도 오프셋 계산"""
        dist = np.hypot(curr_x - self.start_pos[0], curr_y - self.start_pos[1])
        
        if dist < self.align_dist_threshold:
            # 2m 지점까지는 바퀴를 정면(0도)으로 정렬하고 5km/h 전진
            self.get_logger().info(f"Aligning... [{dist:.2f}m / {self.align_dist_threshold}m]", throttle_duration_sec=1.0)
            self.publish_cmd(5.0, 0.0) 
        else:
            # 2m 도달: GPS 궤적 각도와 모아둔 IMU 평균 각도 비교
            gps_h = np.arctan2(curr_y - self.start_pos[1], curr_x - self.start_pos[0])
            imu_h = np.mean(self.imu_yaw_samples)
            
            # 오프셋 산출 (GPS헤딩 - IMU헤딩)
            diff = np.arctan2(np.sin(gps_h - imu_h), np.cos(gps_h - imu_h))
            self.yaw_offset_deg = np.rad2deg(diff)
            self.is_yaw_aligned = True
            
            self.get_logger().info(f"!!! Alignment Success !!! Offset: {self.yaw_offset_deg:.2f} deg")
            # 잠시 정지하여 정렬 완료를 알림
            self.publish_cmd(0.0, 0.0) 

    def gps_cb(self, msg):
        if msg.latitude == 0: return
        with self.lock:
            self.pos_x, self.pos_y = self.proj_utm(msg.longitude, msg.latitude)
            if self.start_pos is None: 
                self.start_pos = (self.pos_x, self.pos_y)
            self.is_gps_init = True

    def imu_cb(self, msg):
        with self.lock:
            self.raw_imu = msg
            # 쿼터니언을 오일러 Yaw로 변환
            _, _, raw_yaw = euler_from_quaternion([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
            
            # 자동 정렬 완료 전에는 Raw 데이터를 수집, 완료 후에는 오프셋 적용
            if not self.is_yaw_aligned:
                if self.start_pos is not None:
                    self.imu_yaw_samples.append(raw_yaw)
                self.yaw = raw_yaw
            else:
                corrected_yaw = raw_yaw + np.deg2rad(self.yaw_offset_deg)
                self.yaw = np.arctan2(np.sin(corrected_yaw), np.cos(corrected_yaw))
            
            self.is_imu_init = True

    def path_cb(self, msg):
        if len(msg.poses) < 2: return
        with self.lock:
            self.target_path = np.array([[p.pose.position.x, p.pose.position.y] for p in msg.poses])
            self.is_path_init = True

    def speed_cb(self, msg):
        with self.lock: 
            self.current_v = msg.data # 이미 km/h 단위

    def init_csv_logger(self):
        if not os.path.exists(self.log_dir): os.makedirs(self.log_dir)
        now = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.log_path = f"{self.log_dir}/stanley_log_{now}.csv"
        header = "time,x,y,heading_err_deg,cte,v_kmh,acc_x,acc_y,acc_z,gyro_x,gyro_y,gyro_z,q_x,q_y,q_z,q_w,target_kmh,steer_deg\n"
        with open(self.log_path, 'w') as f: f.write(header)

    def print_status_to_terminal(self, x, y, yaw, cte, steer_deg, v_kmh):
        self.get_logger().info(
            f"[Stanley] V:{v_kmh:.1f}km/h | Steer:{steer_deg:.1f}deg | CTE:{cte:.3f}",
            throttle_duration_sec=0.5
        )

    def save_to_csv(self, x, y, h_err_deg, cte, v_kmh, imu, target_kmh, steer_deg):
        t = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        line = (f"{t},{x:.4f},{y:.4f},{h_err_deg:.2f},{cte:.3f},{v_kmh:.2f},"
                f"{imu.linear_acceleration.x:.3f},{imu.linear_acceleration.y:.3f},{imu.linear_acceleration.z:.3f},"
                f"{imu.angular_velocity.x:.3f},{imu.angular_velocity.y:.3f},{imu.angular_velocity.z:.3f},"
                f"{imu.orientation.x:.4f},{imu.orientation.y:.4f},{imu.orientation.z:.4f},{imu.orientation.w:.4f},"
                f"{target_kmh:.1f},{steer_deg:.2f}\n")
        with open(self.log_path, 'a') as f: f.write(line)

    def publish_cmd(self, kmh, deg):
        cmd = Twist()
        cmd.linear.x = float(kmh)
        cmd.angular.z = float(deg)
        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = GpsDirectStanley()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_cmd(0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()