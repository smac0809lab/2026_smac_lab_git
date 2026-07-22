import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField
import serial
import numpy as np

class EBTImuNode(Node):
    def __init__(self):
        super().__init__('ebt_imu_node')
        
        # 포트 및 설정
        self.port_name = '/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0'
        self.baudrate = 115200
        
        try:
            self.ser = serial.Serial(self.port_name, self.baudrate, timeout=0.1)
        except Exception as e:
            self.get_logger().error(f'Failed to connect: {e}')
            exit()

        self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)
        self.mag_pub = self.create_publisher(MagneticField, '/imu/mag', 10)
        
        self.timer = self.create_timer(0.01, self.read_serial)
        self.get_logger().info('EBT-IMU Node Started (Full Data Parsing)')

    def euler_to_quaternion(self, r, p, y):
        r, p, y = np.radians([r, p, y])
        cy, sy = np.cos(y * 0.5), np.sin(y * 0.5)
        cp, sp = np.cos(p * 0.5), np.sin(p * 0.5)
        cr, sr = np.cos(r * 0.5), np.sin(r * 0.5)
        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        return [qx, qy, qz, qw]

    def read_serial(self):
        if self.ser.in_waiting > 0:
            try:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if not line.startswith('*'): return

                # 데이터 파싱
                data = line[1:].split(',')
                if len(data) < 12: return
                
                # 순서: R, P, Y, Gx, Gy, Gz, Ax, Ay, Az, Mx, My, Mz
                r, p, y = float(data[0]), float(data[1]), -float(data[2])
                gx, gy, gz = float(data[3]), float(data[4]), float(data[5])
                ax, ay, az = float(data[6]), float(data[7]), float(data[8])
                mx, my, mz = float(data[9]), float(data[10]), float(data[11])

                now = self.get_clock().now().to_msg()
                
                # 1. IMU 메시지 생성
                imu_msg = Imu()
                imu_msg.header.stamp = now
                imu_msg.header.frame_id = 'imu_link'
                
                # 쿼터니언
                q = self.euler_to_quaternion(r, p, y)
                imu_msg.orientation.x, imu_msg.orientation.y, imu_msg.orientation.z, imu_msg.orientation.w = q
                
                # 각속도 (rad/s)
                imu_msg.angular_velocity.x = np.radians(gx)
                imu_msg.angular_velocity.y = np.radians(gy)
                imu_msg.angular_velocity.z = np.radians(gz)
                
                # 가속도 (m/s^2)
                imu_msg.linear_acceleration.x = ax * 9.80665
                imu_msg.linear_acceleration.y = ay * 9.80665
                imu_msg.linear_acceleration.z = az * 9.80665

                # 2. 지자기 메시지 생성
                mag_msg = MagneticField()
                mag_msg.header.stamp = now
                mag_msg.header.frame_id = 'imu_link'
                mag_msg.magnetic_field.x = mx * 1e-6
                mag_msg.magnetic_field.y = my * 1e-6
                mag_msg.magnetic_field.z = mz * 1e-6

                self.imu_pub.publish(imu_msg)
                self.mag_pub.publish(mag_msg)

            except Exception as e:
                pass

def main(args=None):
    rclpy.init(args=args)
    node = EBTImuNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()