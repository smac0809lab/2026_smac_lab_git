import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField
from geometry_msgs.msg import Quaternion
import serial
import math
import numpy as np

class EBTImuNode(Node):
    def __init__(self):
        super().__init__('ebt_imu_node')
        
        # 1. 시리얼 포트 설정 (포트 이름과 보드레이트 확인 필수)
        # EBTerminal에서 확인한 포트(예: /dev/ttyUSB0)로 수정하세요.
        self.serial_port = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.1)
        
        self.imu_pub = self.create_publisher(Imu, 'imu/data', 10)
        self.mag_pub = self.create_publisher(MagneticField, 'imu/mag', 10)
        
        self.timer = self.create_timer(0.01, self.read_serial) # 100Hz 주기
        self.get_logger().info('EBT-IMU Node Started. Parsing RPY, Gyro, Acc, Mag...')

    def euler_to_quaternion(self, r, p, y):
        """오일러 각(deg)을 쿼터니언으로 변환"""
        y = -y
        r, p, y = np.radians([r, p, y])
        qx = np.sin(r/2) * np.cos(p/2) * np.cos(y/2) - np.cos(r/2) * np.sin(p/2) * np.sin(y/2)
        qy = np.cos(r/2) * np.sin(p/2) * np.cos(y/2) + np.sin(r/2) * np.cos(p/2) * np.sin(y/2)
        qz = np.cos(r/2) * np.cos(p/2) * np.sin(y/2) - np.sin(r/2) * np.sin(p/2) * np.cos(y/2)
        qw = np.cos(r/2) * np.cos(p/2) * np.cos(y/2) + np.sin(r/2) * np.sin(p/2) * np.sin(y/2)
        return [qx, qy, qz, qw]

    def read_serial(self):
        if self.serial_port.in_waiting > 0:
            line = self.serial_port.readline().decode('utf-8').strip()
            
            if line.startswith('*'):
                try:
                    # '*' 제거 후 콤마로 분리
                    data = line[1:].split(',')
                    if len(data) < 13: return # 데이터 개수 부족 시 무시
                    
                    # 데이터 매칭
                    # 0,1,2: RPY / 3,4,5: Gyro / 6,7,8: Acc / 9,10,11: Mag / 12: Time
                    r, p, y = float(data[0]), float(data[1]), float(data[2])
                    gx, gy, gz = float(data[3]), float(data[4]), float(data[5])
                    ax, ay, az = float(data[6]), float(data[7]), float(data[8])
                    mx, my, mz = float(data[9]), float(data[10]), float(data[11])

                    # 2. IMU 메시지 생성
                    imu_msg = Imu()
                    imu_msg.header.stamp = self.get_clock().now().to_msg()
                    imu_msg.header.frame_id = 'imu_link'

                    # 쿼터니언 변환 (ROS 표준)
                    q = self.euler_to_quaternion(r, p, y)
                    imu_msg.orientation.x, imu_msg.orientation.y, imu_msg.orientation.z, imu_msg.orientation.w = q

                    # 각속도 (deg/s -> rad/s 변환)
                    imu_msg.angular_velocity.x = np.radians(gx)
                    imu_msg.angular_velocity.y = np.radians(gy)
                    imu_msg.angular_velocity.z = -np.radians(gz)

                    # 가속도 (g -> m/s^2 변환, 1g = 9.80665)
                    imu_msg.linear_acceleration.x = ax * 9.80665
                    imu_msg.linear_acceleration.y = ay * 9.80665
                    imu_msg.linear_acceleration.z = az * 9.80665

                    # 3. 지자기 메시지 생성 (uT -> Tesla 변환)
                    mag_msg = MagneticField()
                    mag_msg.header = imu_msg.header
                    mag_msg.magnetic_field.x = mx * 1e-6
                    mag_msg.magnetic_field.y = my * 1e-6
                    mag_msg.magnetic_field.z = mz * 1e-6

                    self.imu_pub.publish(imu_msg)
                    self.mag_pub.publish(mag_msg)

                except Exception as e:
                    self.get_logger().error(f'Error parsing data: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = EBTImuNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()