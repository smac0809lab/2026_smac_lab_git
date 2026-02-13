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

        # --- [커브 반응성 파라미터] ---
        self.vel_gain = 0.9           
        self.heading_offset = np.radians(15.0)
        
        # 상태 변수
        self.current_vel_x = 0.0
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.yaw = 0.0
        
        self.is_calibrated = False
        self.base_gravity_world = np.zeros(3)
        self.calib_samples = []
        self.last_imu_time = None

        # 구독 및 발행
        self.sub_imu = self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)
        self.sub_vel = self.create_subscription(TwistWithCovarianceStamped, '/vehicle/encoder_velocity', self.encoder_callback, 10)
        self.pub_odom = self.create_publisher(Odometry, '/odometry/imu_encoder', 10)

        self.get_logger().info('🔥 INS Node: High-G Curve Response Mode')

    def encoder_callback(self, msg):
        raw_speed = msg.twist.twist.linear.x
        actual_speed = raw_speed if self.current_vel_x >= -0.1 else -raw_speed

        if abs(actual_speed) < 0.005:
            self.current_vel_x = 0.0
        else:
            # 급커브 시에는 GPS 속도가 실제 주행 궤적보다 느리게 찍힐 수 있음
            # 하지만 요청하신 대로 90% 신뢰도는 유지합니다.
            self.current_vel_x = (0.1 * self.current_vel_x) + (0.9 * actual_speed)

    def imu_callback(self, msg):
        curr_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.last_imu_time is None:
            self.last_imu_time = curr_time
            return
        dt = curr_time - self.last_imu_time
        self.last_imu_time = curr_time

        # 1. 헤딩 계산 (모든 회전각 즉시 반영)
        quat = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        rot = R.from_quat(quat)
        self.yaw = rot.as_euler('zyx')[0] + self.heading_offset

        # 2. 가속도 처리
        accel_raw = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
        accel_world = rot.apply(accel_raw)

        if not self.is_calibrated:
            self.calib_samples.append(accel_world)
            if len(self.calib_samples) >= 100:
                self.base_gravity_world = np.mean(self.calib_samples, axis=0)
                self.is_calibrated = True
            return

        # 3. [핵심] 횡가속도(Lateral Accel)를 포함한 속도 벡터 계산
        pure_accel_world = accel_world - self.base_gravity_world
        pure_accel_body = rot.inv().apply(pure_accel_world)
        
        accel_x = pure_accel_body[0] # 전진 가속도
        accel_y = pure_accel_body[1] # 횡방향 가속도 (커브 시 발생)

        # 전진 속도 업데이트
        self.current_vel_x += accel_x * dt

        # 4. [개선] 커브 시 Slip이나 원심력을 고려한 위치 업데이트
        # 단순히 x속도만 쓰는 게 아니라, y방향 밀림(accel_y)도 미세하게 반영
        vx = self.current_vel_x
        vy = accel_y * dt # 횡방향 속도 성분 추가
        
        # 세계 좌표계로 변환하여 위치 적분
        cos_y = np.cos(self.yaw)
        sin_y = np.sin(self.yaw)
        
        self.pos_x += (vx * cos_y - vy * sin_y) * dt
        self.pos_y += (vx * sin_y + vy * cos_y) * dt

        self.publish_odom(msg.header, quat)

    def publish_odom(self, header, quat):
        odom = Odometry()
        odom.header = header
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.pos_x
        odom.pose.pose.position.y = self.pos_y
        
        # 보정된 헤딩 반영
        corrected_quat = R.from_euler('z', self.yaw).as_quat()
        odom.pose.pose.orientation.x = corrected_quat[0]
        odom.pose.pose.orientation.y = corrected_quat[1]
        odom.pose.pose.orientation.z = corrected_quat[2]
        odom.pose.pose.orientation.w = corrected_quat[3]

        odom.twist.twist.linear.x = float(self.current_vel_x)
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