#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
import numpy as np
from pyproj import Transformer

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, NavSatFix
from geometry_msgs.msg import TransformStamped
from tf_transformations import quaternion_from_euler, euler_from_quaternion
from tf2_ros import TransformBroadcaster

class GpsImuToOdom(Node):
    def __init__(self):
        super().__init__('gps_imu_to_odom')

        # 퍼블리셔 및 TF 브로드캐스터 설정
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # 서브스크라이버 설정
        self.create_subscription(NavSatFix, '/ublox_gps_node/fix', self.gps_cb, 10)
        self.create_subscription(Imu, '/imu/data', self.imu_cb, 10)

        # pyproj Transformer 설정 (WGS84 위경도 -> UTM 52N 존)
        self.transformer = Transformer.from_crs("epsg:4326", "epsg:32652", always_xy=True)

        # 상태 변수 (원점 빼기 로직 제거, 실제 UTM 원본 값 유지)
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.yaw = 0.0

        self.is_gps_init = False
        self.is_imu_init = False

        self.get_logger().info("실제 UTM 좌표 기반 GPS & IMU to Odom 노드 초기화 완료.")

    def gps_cb(self, msg):
        lon = msg.longitude
        lat = msg.latitude

        # pyproj를 이용해 실제 UTM 미터 좌표로 변환 (4백만 대 원본 값 유지)
        utm_x, utm_y = self.transformer.transform(lon, lat)

        self.pos_x = utm_x
        self.pos_y = utm_y
        self.is_gps_init = True

        self.publish_odom()

    def imu_cb(self, msg):
        q_x = msg.orientation.x
        q_y = msg.orientation.y
        q_z = msg.orientation.z
        q_w = msg.orientation.w
        
        _, _, yaw = euler_from_quaternion([q_x, q_y, q_z, q_w])
        self.yaw = yaw
        self.is_imu_init = True

    def publish_odom(self):
        if not (self.is_gps_init and self.is_imu_init):
            return

        now = self.get_clock().now().to_msg()

        # Odometry 메시지 생성
        odom_msg = Odometry()
        odom_msg.header.stamp = now
        odom_msg.header.frame_id = 'map'
        odom_msg.child_frame_id = 'base_link'

        # 위치 설정 (실제 UTM 원본 좌표 그대로 사용)
        odom_msg.pose.pose.position.x = float(self.pos_x)
        odom_msg.pose.pose.position.y = float(self.pos_y)
        odom_msg.pose.pose.position.z = 0.0

        # 자세(Yaw) 설정
        q = quaternion_from_euler(0.0, 0.0, float(self.yaw))
        odom_msg.pose.pose.orientation.x = q[0]
        odom_msg.pose.pose.orientation.y = q[1]
        odom_msg.pose.pose.orientation.z = q[2]
        odom_msg.pose.pose.orientation.w = q[3]

        # /odom 퍼블리시
        self.odom_pub.publish(odom_msg)

        # TF 변환 발행 (map -> base_link)
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = 'map'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = float(self.pos_x)
        t.transform.translation.y = float(self.pos_y)
        t.transform.translation.z = 0.0
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = GpsImuToOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()