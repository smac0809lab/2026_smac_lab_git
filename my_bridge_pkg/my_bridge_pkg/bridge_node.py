import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import TwistWithCovarianceStamped
import traceback

class SpeedBridge(Node):
    def __init__(self):
        super().__init__('speed_bridge_debug')
        self.get_logger().info("브릿지 노드 시작 중...")
        
        # 아두이노 데이터 구독 (에러 방지를 위해 QoS 설정)
        self.sub = self.create_subscription(
            Float32, '/calculateSpeed', self.listener_callback, 10)
        
        self.pub = self.create_publisher(
            TwistWithCovarianceStamped, '/wheel_twist_stamped', 10)
        
        self.get_logger().info("구독 및 발행 준비 완료.")

    def listener_callback(self, msg):
        try:
            # 데이터 수신 확인 로그
            # self.get_logger().debug(f"데이터 수신: {msg.data}")
            
            twist = TwistWithCovarianceStamped()
            twist.header.stamp = self.get_clock().now().to_msg()
            twist.header.frame_id = "base_link"
            
            twist.twist.twist.linear.x = float(msg.data)
            twist.twist.covariance[0] = 0.01
            
            self.pub.publish(twist)
            
        except Exception as e:
            self.get_logger().error(f"메시지 변환 중 에러 발생: {e}")
            self.get_logger().error(traceback.format_exc())

def main(args=None):
    rclpy.init(args=args)
    node = SpeedBridge()
    try:
        rclpy.spin(node)
    except Exception as e:
        print(f"노드 실행 중 치명적 오류: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()