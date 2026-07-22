#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
import csv
import os
from math import sqrt
# ⚠️ 주행/발행 코드와 100% 동일한 좌표 수식을 위해 Proj 라이브러리를 사용합니다.
from pyproj import Proj 

class UtmPathRecorder(Node):
    def __init__(self):
        super().__init__('utm_path_recorder')
        
        # 1. GPS 토픽 구독
        self.subscription = self.create_subscription(
            NavSatFix, 
            '/ublox_gps_node/fix', 
            self.gps_callback, 
            10)
        
        # 2. 주행 및 발행 노드와 완벽히 일치하는 UTM Zone 52 타원체 설정
        self.proj_utm = Proj(proj='utm', zone=52, ellps='WGS84', preserve_units=False)

        # 3. 파일 저장 설정 (경로 이름: 시스템관주차장_path.csv)
        self.file_path = '/home/user/ros2_ws/src/python/시스템관주차장_path.csv'
        self.file = open(self.file_path, 'w', newline='')
        self.writer = csv.writer(self.file)
        self.writer.writerow(['x', 'y', 'alt']) 
        
        self.last_x, self.last_y = 0.0, 0.0
        self.first_record = True
        
        self.get_logger().info(f"🚀 [동기화 완료] 시스템관주차장 경로 녹화 시작! 저장 경로: {self.file_path}")

    def gps_callback(self, msg):
        # GPS 수신 상태 체크
        if msg.status.status < 0:
            self.get_logger().warn("GPS Fix가 잡히지 않았습니다.", throttle_duration_sec=5.0)
            return

        # 경위도 -> UTM 변환
        utm_x, utm_y = self.proj_utm(msg.longitude, msg.latitude)
        utm_z = msg.altitude

        # 곡선 구간 정밀도를 위해 0.1m(10cm) 이상 이동 시 기록
        dist = sqrt((utm_x - self.last_x)**2 + (utm_y - self.last_y)**2)
        
        if self.first_record or dist > 0.1:
            self.writer.writerow([f"{utm_x:.6f}", f"{utm_y:.6f}", f"{utm_z:.3f}"])
            self.file.flush() # 파일에 즉시 쓰기
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