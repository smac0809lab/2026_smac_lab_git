#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
import csv
import os
from math import sqrt
from pyproj import Transformer

class UtmPathRecorder(Node):
    def __init__(self):
        super().__init__('utm_path_recorder')
        
        self.subscription = self.create_subscription(
            NavSatFix, 
            '/ublox_gps_node/fix', 
            self.gps_callback, 
            10)
        
        # Odom 노드와 100% 동일한 EPSG:32652 Transformer 적용
        self.transformer = Transformer.from_crs("epsg:4326", "epsg:32652", always_xy=True)

        self.file_path = '/home/user/ros2_ws/src/python/python/시스템관주차장_path.csv'
        self.file = open(self.file_path, 'w', newline='')
        self.writer = csv.writer(self.file)
        self.writer.writerow(['x', 'y', 'alt']) 
        
        self.last_x, self.last_y = 0.0, 0.0
        self.first_record = True
        
        self.get_logger().info(f"🚀 [표준 좌표계 동기화 완료] 시스템관주차장 경로 녹화 시작! 저장 경로: {self.file_path}")

    def gps_callback(self, msg):
        if msg.status.status < 0:
            self.get_logger().warn("GPS Fix가 잡히지 않았습니다.", throttle_duration_sec=5.0)
            return

        # Odom 노드와 완전히 동일한 함수로 변환
        utm_x, utm_y = self.transformer.transform(msg.longitude, msg.latitude)
        utm_z = msg.altitude

        dist = sqrt((utm_x - self.last_x)**2 + (utm_y - self.last_y)**2)
        
        if self.first_record or dist > 0.1:
            self.writer.writerow([f"{utm_x:.6f}", f"{utm_y:.6f}", f"{utm_z:.3f}"])
            self.file.flush()
            self.last_x, self.last_y = utm_x, utm_y
            self.first_record = False
            self.get_logger().info(f"📝 촘촘하게 기록 중: X={utm_x:.2f}, Y={utm_y:.2f} (이동거리: {dist:.2f}m)")

def main():
    rclpy.init()
    node = UtmPathRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 녹화가 중단되었습니다. 파일을 안전하게 저장하고 닫습니다.")
    finally:
        node.file.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()