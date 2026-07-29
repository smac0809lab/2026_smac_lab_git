#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
import os
import re
from math import sqrt
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path, Odometry

class PathPublisher(Node):
    def __init__(self):
        super().__init__('path_publisher')

        self.is_odom = False
        self.curr_x, self.curr_y = 0.0, 0.0
        
        # CSV 경로 파일 경로 (시스템관 주차장 경로)
        self.target_path_file = "/home/user/ros2_ws/src/python/python/시스템관주차장_path.csv"
        self.full_global_path = self.load_waypoints()

        if not self.full_global_path:
            self.get_logger().error("Path data is missing or CSV format is invalid!")
            return

        self.global_path_pub = self.create_publisher(Path, '/global_path', 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/current_pose', 10) 
        
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)

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
                            # CSV 파일에 저장된 값이 UTM 미터 좌표계라고 가정
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

    def run(self):
        if not self.full_global_path or not self.is_odom:
            self.get_logger().warn("Waiting for Odom and Global Path data...", throttle_duration_sec=3.0)
            return

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

        global_path_msg = Path()
        global_path_msg.header.frame_id = 'map'
        global_path_msg.header.stamp = now
        global_path_msg.poses = self.full_global_path
        self.global_path_pub.publish(global_path_msg)

        self.get_logger().info(f"Publishing Global Path | Closest Idx: {current_idx}, Dist to Path: {min_dis:.2f}m", throttle_duration_sec=1.0)

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