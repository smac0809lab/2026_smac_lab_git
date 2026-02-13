#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistWithCovarianceStamped
import numpy as np

class EncoderSimFromGPS(Node):
    def __init__(self):
        super().__init__('encoder_sim_node')
        
        # 시작 안내 로그
        self.get_logger().info('Encoder Simulation Node Started')

        # 입력/출력 설정
        self.sub_gps = self.create_subscription(Odometry, '/odometry/gps', self.gps_callback, 10)
        self.pub_encoder = self.create_publisher(TwistWithCovarianceStamped, '/vehicle/encoder_velocity', 10)

        self.last_pose = None
        self.last_time = None
        self.filter_alpha = 0.2 
        self.current_speed_m_s = 0.0

    def gps_callback(self, msg):
        current_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        current_pose = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])

        if self.last_pose is not None:
            dt = current_time - self.last_time
            if dt <= 1e-6: return

            distance = np.linalg.norm(current_pose - self.last_pose)
            raw_speed = distance / dt

            # 필터 적용
            self.current_speed_m_s = (1 - self.filter_alpha) * self.current_speed_m_s + self.filter_alpha * raw_speed

            # --- [로그 출력 추가] ---
            # m/s와 km/h를 동시에 보여줍니다.
            km_h = self.current_speed_m_s * 3.6
            self.get_logger().info(f' Speed: {self.current_speed_m_s:.2f} m/s | {km_h:.1f} km/h')

            # 메시지 발행
            encoder_msg = TwistWithCovarianceStamped()
            encoder_msg.header = msg.header
            encoder_msg.header.frame_id = 'base_link'
            encoder_msg.twist.twist.linear.x = self.current_speed_m_s
            encoder_msg.twist.covariance[0] = 0.02
            
            self.pub_encoder.publish(encoder_msg)

        self.last_pose = current_pose
        self.last_time = current_time

def main(args=None):
    try:
        rclpy.init(args=args)
        node = EncoderSimFromGPS()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()