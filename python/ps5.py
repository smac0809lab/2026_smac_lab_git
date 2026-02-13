import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import serial

class PS5SplitTeleop(Node):
    def __init__(self):
        super().__init__('ps5_split_teleop')
        
        # [중요] 변수를 미리 선언해두어야 'AttributeError'를 방지할 수 있습니다.
        self.ser = None
        
        # 아까 확인한 고유 ID 경로
        device_path = '/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0'
        
        try:
            self.ser = serial.Serial(device_path, 115200, timeout=0.1)
            self.get_logger().info(f"✅ 아두이노 연결 성공: {device_path}")
        except Exception as e:
            self.get_logger().error(f"❌ 아두이노 연결 실패: {e}")
            self.get_logger().warn("시리얼 포트 없이 노드를 시작합니다. 컨트롤러 입력이 무시됩니다.")

        self.subscription = self.create_subscription(
            Joy, 'joy', self.joy_callback, 10)

    def joy_callback(self, msg):
        # 시리얼 연결이 안 되어 있으면 아무것도 하지 않고 리턴
        if self.ser is None or not self.ser.is_open:
            return

        # PS5 스틱 값 변환
        velocity = int(msg.axes[1] * 255)
        steering = -int(msg.axes[3] * 255)

        data = f"{velocity},{steering}\n"
        
        try:
            self.ser.write(data.encode())
            self.get_logger().info(f"전송 데이터: {data.strip()}")
        except Exception as e:
            self.get_logger().error(f"시리얼 전송 중 에러: {e}")

    def destroy_node(self):
        # 종료 시 안전하게 닫기
        if self.ser is not None and self.ser.is_open:
            self.ser.close()
            self.get_logger().info("시리얼 포트를 닫았습니다.")
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