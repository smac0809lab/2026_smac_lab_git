import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String
import serial

class EbimuPublisher(Node):
    def __init__(self):
        super().__init__('ebimu_publisher')
        qos_profile = QoSProfile(depth=10)

        self.port_name = '/dev/ttyIMU'
        self.baudrate = 115200 # <-- 만약 안나오면 9600으로도 테스트 필수

        try:
            self.ser = serial.Serial(port=self.port_name, baudrate=self.baudrate, timeout=0.1)
            self.get_logger().info(f'Serial port connected: {self.port_name}')
        except Exception as e:
            self.get_logger().error(f'Serial port error: {e}')
            return

        self.publisher = self.create_publisher(String, 'ebimu_data', qos_profile)
        self.timer = self.create_timer(0.01, self.timer_callback)

    def timer_callback(self):
        # 핵심: 함수 내부 로직은 반드시 들여쓰기가 되어야 함
        if self.ser.in_waiting > 0:
            try:
                # 데이터가 있을 때 한 줄을 읽음
                raw_data = self.ser.readline()
                if raw_data:
                    # 1. 터미널에 Raw 데이터 강제 출력 (디버깅용)
                    self.get_logger().info(f'Raw: {raw_data}')
                    
                    # 2. 디코딩 (에러 무시)
                    decoded_line = raw_data.decode('utf-8', errors='ignore').strip()
                    
                    if decoded_line:
                        msg = String()
                        msg.data = decoded_line
                        self.publisher.publish(msg)
            except Exception as e:
                self.get_logger().warn(f'Read error: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = EbimuPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()