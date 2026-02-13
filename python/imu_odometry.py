import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import numpy as np
from scipy.spatial.transform import Rotation as R

class IMUOdometry(Node):
    def __init__(self):
        super().__init__('imu_odometry_node')

        # IMU 데이터 구독
        self.subscription = self.create_subscription(
            Imu,
            '/imu/data',  # 토픽명
            self.imu_callback,
            10)
        
        # 상태 변수 초기화
        self.pose = np.zeros(3)  # [x, y, z]
        self.vel = np.zeros(3)  # [vx, vy, vz]
        self.gravity = np.array([0, 0, 9.81]) # 표준 중력
        self.last_time = None

    def imu_callback(self, msg):
        # 1. 메시지에서 시간 추출 (초 단위)
        current_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        # 2. 메시지에서 가속도 및 쿼터니언 추출
        accel_raw = np.array([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z
        ])
        
        quat_xyzw = [
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        ]

        # 3. (Update) 실행
        current_pose = self.update(accel_raw, quat_xyzw, current_time)

        # 4. 결과 확인용 로그 (선택 사항)
        self.get_logger().info(f'Pose: {current_pose}')

        def update(self, accel_raw, quat_xyzw, current_time):
            if self.last_time is None:
                self.last_time = current_time
                return self.pose    
        
            dt = current_time - self.last_time
            self.last_time = current_time

            # 1. 쿼터니언을 회전 객체로 변환
            rot = R.from_quat(quat_xyzw)

            # 2. 가속도 센서 값을 세계 좌표계(Global Frame)로 변환
            accel_world = rot.apply(accel_raw)

            # 3. 중력 제거
            pure_accel = accel_world - self.gravity

            # 4. 정지 상태 임계값 처리 (Threshold)
            if np.linalg.norm(pure_accel) < 0.05:
                pure_accel = np.zeros(3)
                # 팁: 멈췄을 때 속도가 야금야금 쌓이는 걸 막기 위해 속도도 살짝 죽여주는게 좋습니다.
                self.vel *= 0.9

            # 5. 1차 적분: 속도 업데이트 (v = v0 + a*dt)
            self.vel += pure_accel * dt    

            # 6. 2차 적분: 위치 업데이트 (p = p0 + v*dt)
            self.pose += self.vel * dt    

            return self.pose
        def main(args=None):
            rclpy.init(args=args)
            node = IMUOdometry()
            try:
                rclpy.spin(node)
            except KeyboardInterrupt:
                pass
            finally:
                node.destroy_node()
                rclpy.shutdown()

        if __name__ == '__main__':
            main()