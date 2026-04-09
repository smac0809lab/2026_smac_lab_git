#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
import os
import re
from math import sqrt
import numpy as np
from pyproj import Proj  # 위경도 -> UTM 변환용

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from sensor_msgs.msg import NavSatFix

class PathPublisher(Node):
    def __init__(self):
        super().__init__('path_publisher')

        # 1. UTM 변환 설정 (한국 지역 Zone 52)
        self.proj_utm = Proj(proj='utm', zone=52, ellps='WGS84', preserve_units=False)

        # 2. 파라미터 및 변수 설정
        self.local_path_size = 50
        self.is_gps = False
        self.curr_x, self.curr_y = 0.0, 0.0
        
        # 3. 데이터 로드 (절대 경로)
        # CSV 파일의 x, y도 UTM 좌표계여야 합니다.
        self.target_path = "/home/user/ros2_ws/src/python/utm_path.csv"
        self.full_global_path = self.load_waypoints()

        if not self.full_global_path:
            self.get_logger().error("Path data is missing or CSV format is invalid!")
            return

        # 4. Publisher & Subscriber 설정
        self.global_path_pub = self.create_publisher(Path, '/global_path', 10)
        self.local_path_pub = self.create_publisher(Path, '/local_path', 10)
        
        # [수정] EKF 대신 순수 GPS 토픽 구독
        self.gps_sub = self.create_subscription(
            NavSatFix,
            '/ublox_gps_node/fix',
            self.gps_callback,
            10)

        # 5. 타이머 설정 (20Hz)
        self.timer = self.create_timer(0.05, self.run)

    def load_waypoints(self):
        """CSV에서 UTM 경로 로드"""
        waypoints = []
        if not os.path.exists(self.target_path):
            self.get_logger().error(f"File not found at: {self.target_path}")
            return []

        try:
            with open(self.target_path, 'r') as f:
                for line in f:
                    clean_line = line.strip()
                    if clean_line:
                        parts = re.split(r'[\s,]+', clean_line)
                        if parts and re.match(r'^-?\d', parts[0]):
                            pose = PoseStamped()
                            pose.header.frame_id = 'map' 
                            pose.pose.position.x = float(parts[0])
                            pose.pose.position.y = float(parts[1])
                            pose.pose.position.z = 0.0
                            pose.pose.orientation.w = 1.0
                            waypoints.append(pose)
            
            self.get_logger().info(f"Successfully loaded {len(waypoints)} waypoints.")
            return waypoints
        except Exception as e:
            self.get_logger().error(f"Failed to load path file: {e}")
            return []

    def gps_callback(self, msg):
        """GPS 위경도를 UTM으로 즉시 변환하여 현재 위치 업데이트"""
        if msg.latitude == 0 or msg.longitude == 0:
            return
            
        # 위경도 -> UTM 변환
        self.curr_x, self.curr_y = self.proj_utm(msg.longitude, msg.latitude)
        
        if not self.is_gps:
            self.get_logger().info(f"GPS Connected! First UTM: ({self.curr_x:.2f}, {self.curr_y:.2f})")
            self.is_gps = True

    def run(self):
        if not self.full_global_path or not self.is_gps:
            # 데이터가 준비될 때까지 경고 출력 (3초 간격)
            self.get_logger().warn("Waiting for GPS and Path data...", throttle_duration_sec=3.0)
            return

        # 1. 현재 UTM 위치에서 가장 가까운 웨이포인트 찾기
        min_dis = float('inf')
        current_idx = 0
        
        for i, pose in enumerate(self.full_global_path):
            dx = self.curr_x - pose.pose.position.x
            dy = self.curr_y - pose.pose.position.y
            dist = sqrt(dx**2 + dy**2)
            if dist < min_dis:
                min_dis = dist
                current_idx = i

        now = self.get_clock().now().to_msg()

        # 2. Global Path 발행 (전체 경로)
        global_path_msg = Path()
        global_path_msg.header.frame_id = 'map'
        global_path_msg.header.stamp = now
        global_path_msg.poses = self.full_global_path
        self.global_path_pub.publish(global_path_msg)

        # 3. Local Path 발행 (현재 위치부터 일정 거리만)
        local_path_msg = Path()
        local_path_msg.header.frame_id = 'map'
        local_path_msg.header.stamp = now
        
        # 현재 인덱스부터 local_path_size만큼 자르기
        end_idx = min(len(self.full_global_path), current_idx + self.local_path_size)
        local_path_msg.poses = self.full_global_path[current_idx : end_idx]
        self.local_path_pub.publish(local_path_msg)

        # 상태 확인용 로거
        self.get_logger().info(f"Publishing Path | Closest Idx: {current_idx}, Dist: {min_dis:.2f}m", throttle_duration_sec=1.0)

def main(args=None):
    rclpy.init(args=args)
    node = PathPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()