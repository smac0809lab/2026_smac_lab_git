#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class PS5TeleopNode(Node):
    def __init__(self):
        super().__init__('ps5_teleop_node')
        
        # 조이스틱 토픽 구독
        self.subscription = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10)
            
        # cmd_vel 토픽 발행
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 1)
        
        self.get_logger().info('PS5 Teleop Node (Left Stick: Speed, Right Stick: Steering) has been started.')

    def joy_callback(self, msg):
        twist = Twist()
        
        # PS5 듀얼센스 매핑 변경:
        # - axes[1]: 왼쪽 아날로그 스틱 상/하 (Linear X - 앞/뒤)
        # - axes[3]: 오른쪽 아날로그 스틱 좌/우 (Angular Z - 좌/우)
        forward_speed = msg.axes[1]
        steer_value = msg.axes[3]
        
        # linear.x는 0~1 사이로 정규화 (최대 속도 0.5로 제한, 후진 포함)
        twist.linear.x = float(forward_speed) 
        
        # angular.z는 조이스틱의 좌/우 회전값을 그대로 적용
        twist.angular.z = -float(steer_value)
        
        # 토픽 발행
        self.publisher_.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = PS5TeleopNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()