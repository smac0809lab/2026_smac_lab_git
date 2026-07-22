#!/usr/init/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
import os
import re
from math import sqrt
import numpy as np
from pyproj import Proj 

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path, Odometry

class PathPublisher(Node):
    def __init__(self):
        super().__init__('path_publisher')

        # 1. UTM 변환 설정 (한국 지역 Zone 52)
        self.proj_utm = Proj(proj='utm', zone=52, ellps='WGS84', preserve_units=False)

        # 2. 파라미터 및 변수 설정
        self.local_path_size = 50 
        self.is_odom = False
        self.curr_x, self.curr_y = 0.0, 0.0
        
        # 외부에서 발행되는 동적 로컬 경로 수신을 위한 변수
        self.external_local_path = None
        
        # 3. 데이터 로드 (Global Path)
        self.target_path_file = "/home/user/ros2_ws/src/python/시스템관주차장_path.csv"
        self.full_global_path = self.load_waypoints()

        if not self.full_global_path:
            self.get_logger().error("Path data is missing or CSV format is invalid!")
            return

        # 4. Publisher & Subscriber 설정
        # RViz2에서 Global Path는 초록색(Green), Local Path는 빨간색(Red)으로 설정하여 시각화할 수 있습니다.
        self.global_path_pub = self.create_publisher(Path, '/global_path', 10)
        self.local_path_pub = self.create_publisher(Path, '/local_path', 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/current_pose', 10) 
        
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)
            
        # [추가] 주차나 장애물 회피 등으로 외부에서 생성된 동적 로컬 경로를 구독하는 토픽 (예: /dynamic_local_path)
        self.dynamic_local_sub = self.create_subscription(
            Path,
            '/dynamic_local_path',
            self.dynamic_local_callback,
            10)

        # 5. 타이머 설정 (20Hz)
        self.timer = self.create_timer(0.05, self.run)

    def load_waypoints(self):
        waypoints = []
        if not os.path.exists(self.target_path_file):
            self.get_logger().error(f"File not found at: {self.target_path_file}")
            return []

        try:
            with open(self.target_path_file, 'r') as f:
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

    def odom_callback(self, msg):
        self.curr_x = msg.pose.pose.position.x
        self.curr_y = msg.pose.pose.position.y
        
        now = self.get_clock().now().to_msg()
        p = PoseStamped()
        p.header.stamp = now
        p.header.frame_id = 'map'
        p.pose.position.x = self.curr_x
        p.pose.position.y = self.curr_y
        p.pose.orientation = msg.pose.pose.orientation
        self.pose_pub.publish(p)

        if not self.is_odom:
            self.get_logger().info(f"Odom Connected! Current Position: ({self.curr_x:.2f}, {self.curr_y:.2f})")
            self.is_odom = True

    def dynamic_local_callback(self, msg):
        # 주차나 장애물 회피 경로 등 동적 로컬 경로가 들어올 경우 저장
        if len(msg.poses) > 0:
            self.external_local_path = msg.poses
        else:
            self.external_local_path = None

    def run(self):
        if not self.full_global_path or not self.is_odom:
            self.get_logger().warn("Waiting for Odom and Global Path data...", throttle_duration_sec=3.0)
            return

        # 1. 현재 위치에서 가장 가까운 글로벌 웨이포인트 찾기
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

        # 2. Global Path 발행 (초록색 시각화용)
        global_path_msg = Path()
        global_path_msg.header.frame_id = 'map'
        global_path_msg.header.stamp = now
        global_path_msg.poses = self.full_global_path
        self.global_path_pub.publish(global_path_msg)

        # 3. Local Path 발행 (빨간색 시각화용)
        local_path_msg = Path()
        local_path_msg.header.frame_id = 'map'
        local_path_msg.header.stamp = now
        
        # 만약 외부(주차/장애물 회피 등)에서 동적 로컬 경로가 들어왔다면 그것을 우선 발행하고,
        # 그렇지 않다면 글로벌 경로 중 현재 위치부터 일정 구간을 잘라서 기본 Local Path로 발행합니다.
        if self.external_local_path is not None:
            local_path_msg.poses = self.external_local_path
        else:
            end_idx = min(len(self.full_global_path), current_idx + self.local_path_size)
            local_path_msg.poses = self.full_global_path[current_idx : end_idx]
            
        self.local_path_pub.publish(local_path_msg)

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