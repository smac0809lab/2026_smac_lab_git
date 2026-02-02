import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import serial

class PS5SplitTeleop(Node):
    def __init__(self):
        super().__init__('ps5_split_teleop')
        
        # 아두이노 시리얼 포트 설정 (본인 환경에 맞게 수정)
        # 예: '/dev/ttyUSB0' 또는 '/dev/ttyACM0'
        try:
            self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.1)
            self.get_logger().info("✅ 아두이노와 연결되었습니다.")
        except Exception as e:
            self.get_logger().error(f"❌ 시리얼 포트 열기 실패: {e}")

        # /joy 토픽 구독
        self.subscription = self.create_subscription(
            Joy, 'joy', self.joy_callback, 10)

    def joy_callback(self, msg):
        # 1. 왼쪽 스틱 (상/하) -> 속도 (Velocity)
        # PS5 axes[1]은 위가 1.0, 아래가 -1.0입니다.
        velocity = int(msg.axes[1] * 255)

        # 2. 오른쪽 스틱 (좌/우) -> 조향 (Steering)
        # PS5 axes[3]은 왼쪽이 1.0, 오른쪽이 -1.0입니다. 
        # 아두이노 로직 상 오른쪽이 양수(+)여야 한다면 앞에 마이너스를 붙이세요.
        steering = int(msg.axes[3] * 255)

        # 아두이노가 기다리는 "V,A\n" 형식 생성
        data = f"{velocity},{steering}\n"
        
        try:
            self.ser.write(data.encode())
            # 디버깅용 로그 (필요 없으면 주석 처리)
            # self.get_logger().info(f"전송 데이터: {data.strip()}")
        except Exception as e:
            self.get_logger().error(f"시리얼 전송 중 에러: {e}")

    def destroy_node(self):
        if self.ser.is_open:
            self.ser.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = PS5SplitTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()