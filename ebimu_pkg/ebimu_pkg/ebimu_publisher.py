import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String
import serial

class EbimuPublisher(Node):

    def __init__(self):
        super().__init__('ebimu_publisher')
        qos_profile = QoSProfile(depth=10)

        # 1. 설정값 고정
        self.port_name = '/dev/ttyUSB0'
        self.baudrate = 115200

        # 2. 시리얼 포트 초기화
        try:
            self.ser = serial.Serial(port=self.port_name, baudrate=self.baudrate, timeout=1)
            self.get_logger().info(f'Serial port connected: {self.port_name}')
        except Exception as e:
            self.get_logger().error(f'Serial port error: {e}')
            exit()

        # 3. 퍼블리셔 및 타이머 설정
        self.publisher = self.create_publisher(String, 'ebimu_data', qos_profile)
        
        # 주기 설정 (0.01초 = 100Hz 정도가 IMU 데이터 수신에 적당합니다)
        timer_period = 0.01 
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        if self.ser.in_waiting > 0:
            try:
                # 시리얼 데이터 한 줄 읽기
                ser_data = self.ser.readline()
                
                # 데이터가 비어있지 않은지 확인 후 디코딩 및 발행
                if ser_data:
                    msg = String()
                    msg.data = ser_data.decode('utf-8').strip()
                    self.publisher.publish(msg)
                    # 확인용 로그 (필요 없으면 삭제 가능)
                    # self.get_logger().info(f'Publishing: {msg.data}')
            except Exception as e:
                self.get_logger().warn(f'Read error: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = EbimuPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
