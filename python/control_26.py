#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
import numpy as np
import os
from datetime import datetime
from threading import Lock

from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Path, Odometry
from sensor_msgs.msg import Imu, Joy
from std_msgs.msg import Float32, Int32, String
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from tf2_ros import TransformBroadcaster

class PIDController:
    def __init__(self, kp, ki, kd, dt=0.05):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        self.prev_error = 0.0
        self.integral = 0.0

    def calculate(self, target, current):
        error = target - current
        self.integral += error * self.dt
        derivative = (error - self.prev_error) / self.dt
        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        self.prev_error = error
        return np.clip(output, -255.0, 255.0)

class GpsImuStanley(Node):
    def __init__(self):
        super().__init__('gps_imu_stanley')
        self.lock = Lock()
        self.tf_broadcaster = TransformBroadcaster(self)

        self.is_autonomous = False 
        self.prev_x_btn_state = 0 
        self.init_drive_counter = 0

        self.manual_throttle = 0.0
        self.manual_steer = 0.0  
        
        self.L = 0.72           
        self.k = 2.0
        self.max_steer_deg = 22.0
        self.max_steer_rad = np.deg2rad(self.max_steer_deg)
        
        self.target_speed = 70.0 
        self.steer_pid = PIDController(kp=10.0, ki=0.0, kd=0.5, dt=0.05)

        self.pot_min = 0.0
        self.pot_max = 4096.0
        self.pot_center = 2048.0

        self.current_steer_pot = 2048.0
        self.current_steer_deg = 0.0
        self.target_steer_deg = 0.0
        self.pid_out = 0.0
        self.closet_cluster_dist = 99.0  
        
        # Odom 및 IMU 관련 변수
        self.pos_x, self.pos_y, self.yaw = 0.0, 0.0, 0.0
        self.raw_sensor_yaw = 0.0   
        self.imu_yaw_bias = 0.0
        self.current_v = 0.0        
        
        # 경로 관련 변수 (Global / Local 분리 관리)
        self.global_path = None
        self.local_path = None
        self.target_path = None     # 현재 실제로 추종할 경로
        self.path_type_str = 'none' # 로그용 ('global' 또는 'local')
        
        self.is_odom_init = False
        self.is_imu_init = False
        self.is_global_path_init = False
        
        # 신호등/정지선 제어용 변수
        self.r_state = 0
        self.y_state = 0
        self.g_state = 0
        self.f_state = 0
        self.l_state = 0
        self.traffic_light_area = 0.0
        self.distance_to_stopline = 999.0
        
        self.current_light = 'green'
        self.brake_start_dist = 150.0  
        self.emergency_area = 2000.0   

        self.log_dir = '/home/user/ros2_ws/src/python/logs'
        self.init_csv_logger()

        # 서브스크라이버 설정 (Odometry 및 분리된 Path 구독)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.create_subscription(Imu, '/imu/data', self.imu_cb, 10)
        self.create_subscription(Path, '/global_path', self.global_path_cb, 10)
        self.create_subscription(Path, '/local_path', self.local_path_cb, 10)
        self.create_subscription(Float32, '/calculateSpeed', self.speed_cb, 10)
        self.create_subscription(Int32, '/steer_pot_raw', self.pot_cb, 10)
        self.create_subscription(Joy, '/joy', self.joy_cb, 10)
        self.create_subscription(Float32, '/closest_cluster_distance', self.closest_cluster_cb, 10)
        self.create_subscription(String, '/traffic_light_info', self.traffic_cb, 10)
        
        self.throttle_pub = self.create_publisher(Float32, '/final_throttle', 10)
        self.steer_pub = self.create_publisher(Float32, '/final_steer', 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/current_pose', 10)

        self.create_timer(0.05, self.main_control_loop)

    def init_csv_logger(self):
        if not os.path.exists(self.log_dir): os.makedirs(self.log_dir)
        now = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.log_path = f"{self.log_dir}/stanley_log_{now}.csv"
        
        # [수정] utm_x, utm_y 다음 위치에 path_type 컬럼 추가
        header = "time,utm_x,utm_y,path_type,vehicle_yaw_deg,path_yaw_deg,cte_meter,k_gain,v_kmh,target_steer_deg,current_steer_deg,closest_cluster_dist,r_state,y_state,g_state,f_state,l_state,light_area,stopline_dist\n"
        with open(self.log_path, 'w') as f: f.write(header)

    def save_to_csv(self, x, y, p_type, v_yaw, p_yaw, cte, k_gain, v_kmh, target_steer, curr_steer, cluster_dist, r, y_flag, g, f, l, area, dist):
        t = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        line = (f"{t},{x:.4f},{y:.4f},{p_type},{np.rad2deg(v_yaw):.2f},{np.rad2deg(p_yaw):.2f},"
                f"{cte:.3f},{k_gain:.2f},{v_kmh:.2f},"
                f"{target_steer:.2f},{curr_steer:.2f},{cluster_dist:.2f},"
                f"{r},{y_flag},{g},{f},{l},{area:.2f},{dist:.2f}\n")
        with open(self.log_path, 'a') as f: f.write(line)

    def publish_cmd(self, speed_val, angular_val):
        throttle_msg = Float32()
        throttle_msg.data = float(speed_val)
        self.throttle_pub.publish(throttle_msg)

        steer_msg = Float32()
        steer_msg.data = float(np.clip(angular_val, -255.0, 255.0))
        self.steer_pub.publish(steer_msg)

    def traffic_cb(self, msg):
        with self.lock:
            try:
                data = msg.data.split(',')
                self.r_state = int(data[1])
                self.y_state = int(data[2])
                self.g_state = int(data[3])
                self.f_state = int(data[4])
                self.l_state = int(data[5])
                self.traffic_light_area = float(data[6]) 
                dist = float(data[7])

                if self.r_state == 1 or self.y_state == 1: 
                    self.current_light = 'red'
                elif self.g_state == 1: 
                    self.current_light = 'green'
                
                if self.l_state == 1:
                    self.distance_to_stopline = dist
                else:
                    self.distance_to_stopline = 999.0 
            except Exception as e:
                pass

    def joy_cb(self, msg):
        with self.lock:
            current_x_btn = msg.buttons[0] 
            if current_x_btn == 1 and self.prev_x_btn_state == 0:
                if not self.is_autonomous:
                    if self.is_odom_init and self.is_imu_init and self.is_global_path_init:
                        # 자율주행 시작 시점의 IMU Yaw Bias 자동 설정
                        path_to_use = self.local_path if self.local_path is not None and len(self.local_path) >= 2 else self.global_path
                        dists = np.hypot(path_to_use[:, 0] - self.pos_x, path_to_use[:, 1] - self.pos_y)
                        idx = np.argmin(dists)
                        if idx < len(path_to_use) - 1:
                            init_path_yaw = np.arctan2(path_to_use[idx+1, 1] - path_to_use[idx, 1], 
                                                       path_to_use[idx+1, 0] - path_to_use[idx, 0])
                        else:
                            init_path_yaw = 0.0
                        
                        self.imu_yaw_bias = init_path_yaw - self.raw_sensor_yaw
                        self.steer_pid.prev_error = 0.0
                        self.steer_pid.integral = 0.0
                        self.is_autonomous = True
                        self.get_logger().info(f"자율 주행 시작! IMU Bias 설정됨: {np.rad2deg(self.imu_yaw_bias):.2f}°")
                    else:
                        self.get_logger().error("오도메트리, IMU 또는 글로벌 경로가 준비되지 않았습니다.")
                else:
                    self.is_autonomous = False
                    self.get_logger().info("자율 주행 종료 (수동 모드)")
            self.prev_x_btn_state = current_x_btn

            if not self.is_autonomous:
                self.manual_throttle = msg.axes[1] * 255.0 
                self.manual_steer = -msg.axes[3] * 255.0

    def pot_cb(self, msg):
        with self.lock:
            raw_val = float(msg.data)
            self.current_steer_pot = raw_val
            self.current_steer_deg = float(np.interp(
                raw_val,
                [self.pot_min, self.pot_center, self.pot_max],
                [-self.max_steer_deg, 0.0, self.max_steer_deg]
            ))

    def speed_cb(self, msg):
        with self.lock: 
            self.current_v = msg.data / 3.6 
    
    def odom_cb(self, msg):
        with self.lock:
            self.pos_x = msg.pose.pose.position.x
            self.pos_y = msg.pose.pose.position.y
            self.is_odom_init = True
    
    def imu_cb(self, msg):
        with self.lock:
            _, _, raw_yaw = euler_from_quaternion([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
            self.raw_sensor_yaw = raw_yaw
            corrected_yaw = self.raw_sensor_yaw + self.imu_yaw_bias
            self.yaw = np.arctan2(np.sin(corrected_yaw), np.cos(corrected_yaw)) 
            self.is_imu_init = True

    def global_path_cb(self, msg):
        if len(msg.poses) < 2: return
        with self.lock:
            self.global_path = np.array([[p.pose.position.x, p.pose.position.y] for p in msg.poses])
            self.is_global_path_init = True

    def local_path_cb(self, msg):
        with self.lock:
            if len(msg.poses) >= 2:
                self.local_path = np.array([[p.pose.position.x, p.pose.position.y] for p in msg.poses])
            else:
                self.local_path = None # 로컬 경로가 없으면 초기화하여 글로벌 경로를 사용하도록 함

    def closest_cluster_cb(self, msg):
        with self.lock:
            self.closet_cluster_dist = msg.data

    def main_control_loop(self):
        with self.lock:
            curr_autonomous = self.is_autonomous
            curr_x, curr_y, curr_yaw = self.pos_x, self.pos_y, self.yaw
            curr_v = self.current_v
            
            # 경로 우선순위 결정: Local Path가 존재하면 Local Path 추종, 아니면 Global Path 추종
            if self.local_path is not None and len(self.local_path) >= 2:
                self.target_path = self.local_path
                self.path_type_str = 'local'
            else:
                self.target_path = self.global_path
                self.path_type_str = 'global'
                
            path = self.target_path
            curr_steer = self.current_steer_deg
            curr_closet_dist = self.closet_cluster_dist
        
        if not curr_autonomous:
            self.init_drive_counter = 0
            self.publish_cmd(self.manual_throttle, self.manual_steer)
            return
        
        if not (self.is_odom_init and self.is_imu_init and self.is_global_path_init):
            self.publish_cmd(0.0, 0.0) 
            return
        
        if path is None or len(path) < 2:
            self.publish_cmd(0.0, 0.0)
            self.is_autonomous = False
            return

        if self.init_drive_counter < 10:
            self.init_drive_counter += 1
            self.publish_cmd(0.0, 0.0)
            return

        self.publish_rviz_data(curr_x, curr_y, curr_yaw)

        dists = np.hypot(path[:, 0] - curr_x, path[:, 1] - curr_y)
        target_idx = np.argmin(dists)

        if target_idx < len(path) - 1:
            path_yaw = np.arctan2(path[target_idx+1, 1] - path[target_idx, 1], 
                                  path[target_idx+1, 0] - path[target_idx, 0])
        else:
            path_yaw = curr_yaw

        dx = curr_x - path[target_idx, 0]
        dy = curr_y - path[target_idx, 1]
        cte = -dx * np.sin(path_yaw) + dy * np.cos(path_yaw)

        theta_e = np.arctan2(np.sin(curr_yaw - path_yaw), np.cos(curr_yaw - path_yaw))
        v_eff = max(abs(curr_v), 1.0) 
        
        steer_rad = theta_e + np.arctan2(self.k * cte, v_eff)
        self.target_steer_deg = np.rad2deg(np.clip(steer_rad, -self.max_steer_rad, self.max_steer_rad))
        self.pid_out = self.steer_pid.calculate(self.target_steer_deg, curr_steer)

        # 종방향 제어: 신호등/정지선 감속 로직
        final_speed = self.target_speed

        cond_distance = (self.distance_to_stopline <= self.brake_start_dist)
        cond_area = (self.traffic_light_area >= self.emergency_area)

        if self.current_light == 'red':
            if cond_distance:
                p_gain = self.target_speed / self.brake_start_dist
                final_speed = self.distance_to_stopline * p_gain
                if self.distance_to_stopline < 20.0:
                    final_speed = 0.0
            elif cond_area:
                self.get_logger().info("정지선 미인식! 신호등 면적 기반 비상 정지!")
                final_speed = 0.0

        self.publish_cmd(final_speed, self.pid_out)
        
        # CSV 로그 기록 (path_type 포함)
        self.save_to_csv(curr_x, curr_y, self.path_type_str, curr_yaw, path_yaw, cte, self.k, curr_v * 3.6, 
                         self.target_steer_deg, curr_steer, curr_closet_dist, 
                         self.r_state, self.y_state, self.g_state, self.f_state, self.l_state, 
                         self.traffic_light_area, self.distance_to_stopline)

    def publish_rviz_data(self, curr_x, curr_y, curr_yaw):
        now = self.get_clock().now().to_msg()
        pose_msg = PoseStamped()
        pose_msg.header.stamp = now
        pose_msg.header.frame_id = 'map'
        pose_msg.pose.position.x = float(curr_x)
        pose_msg.pose.position.y = float(curr_y)
        pose_msg.pose.position.z = 0.0

        q = quaternion_from_euler(0.0, 0.0, float(curr_yaw))
        pose_msg.pose.orientation.x = q[0]
        pose_msg.pose.orientation.y = q[1]
        pose_msg.pose.orientation.z = q[2]
        pose_msg.pose.orientation.w = q[3]
        self.pose_pub.publish(pose_msg)

        t_vehicle = TransformStamped()
        t_vehicle.header.stamp = now
        t_vehicle.header.frame_id = 'map'
        t_vehicle.child_frame_id = 'base_link'
        t_vehicle.transform.translation.x = float(curr_x)
        t_vehicle.transform.translation.y = float(curr_y)
        t_vehicle.transform.translation.z = 0.0
        t_vehicle.transform.rotation.x = q[0]
        t_vehicle.transform.rotation.y = q[1]
        t_vehicle.transform.rotation.z = q[2]
        t_vehicle.transform.rotation.w = q[3]
        self.tf_broadcaster.sendTransform(t_vehicle)

def main(args=None):
    rclpy.init(args=args)
    node = GpsImuStanley()
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