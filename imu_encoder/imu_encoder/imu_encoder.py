#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistWithCovarianceStamped
import numpy as np
from scipy.spatial.transform import Rotation as R

class IMUEncoderFusion(Node):
    def __init__(self):
        super().__init__('imu_encoder_node')

        # --- [파라미터 설정] ---
        # 인코더 속도를 100% 신뢰하도록 설정 (가속도 적분 오차 배제)
        self.encoder_trust_ratio = 1.0 
        # GPS 궤적과 비교해서 벌어지는 각도 조절 (라디안)
        self.heading_offset = np.radians(0.0) 
        
        # 상태 변수
        self.current_vel_x = 0.0
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.yaw = 0.0
        
        self.last_imu_time = None

        # 구독 및 발행
        self.sub_imu = self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)
        self.sub_vel = self.create_subscription(TwistWithCovarianceStamped, '/vehicle/encoder_velocity', self.encoder_callback, 10)
        self.pub_odom = self.create_publisher(Odometry, '/odometry/imu_encoder', 10)

        self.get_logger().info('🔥 IMU-Encoder Node: Simplified Mode (No Direction Detection)')

    def encoder_callback(self, msg):
        # 인코더에서 들어오는 속도를 그대로 사용 (음수 값이 들어오면 자동으로 후진 처리됨)
        # 만약 gps_vel.py에서 항상 양수만 준다면 abs()를 빼고 방향 로직을 별도로 넣어야 합니다.
        self.current_vel_x = abs(msg.twist.twist.linear.x)

    def imu_callback(self, msg):
        curr_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.last_imu_time is None:
            self.last_imu_time = curr_time
            return
        dt = curr_time - self.last_imu_time
        self.last_imu_time = curr_time

        # 1. Yaw 정보 활용 (IMU의 Orientation 직접 사용)
        quat = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        rot = R.from_quat(quat)
        self.yaw = rot.as_euler('zyx')[0] + self.heading_offset

        # 2. 위치 업데이트
        # 방향 판별 없이 현재 속도(vx)와 각도(yaw)만으로 위치 적분
        self.pos_x += self.current_vel_x * np.cos(self.yaw) * dt
        self.pos_y += self.current_vel_x * np.sin(self.yaw) * dt

        self.publish_odom(msg.header, quat)

    def publish_odom(self, header, quat):
        odom = Odometry()
        odom.header = header
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        
        odom.pose.pose.position.x = self.pos_x
        odom.pose.pose.position.y = self.pos_y
        
        # 보정된 Yaw 반영
        corrected_quat = R.from_euler('z', self.yaw).as_quat()
        odom.pose.pose.orientation.x = corrected_quat[0]
        odom.pose.pose.orientation.y = corrected_quat[1]
        odom.pose.pose.orientation.z = corrected_quat[2]
        odom.pose.pose.orientation.w = corrected_quat[3]

        # 속도 정보 포함
        odom.twist.twist.linear.x = float(self.current_vel_x)
        
        # [메모 준수] IMU 공분산이 0이므로 EKF가 이 데이터를 매우 강하게 신뢰하게 됨
        self.pub_odom.publish(odom)

def main(args=None):
    rclpy.init(args=args)
    node = IMUEncoderFusion()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()