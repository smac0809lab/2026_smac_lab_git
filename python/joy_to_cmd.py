#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class JoyToCmdVelRaw(Node):
    def __init__(self):
        super().__init__('joy_to_cmd_vel_node')

        # 하드코딩 인덱스 (표준 패드 기준)
        # 왼쪽 스틱 상하: 1, 오른쪽 스틱 좌우: 3
        self.linear_axis = 1   
        self.angular_axis = 3  

        # Publisher & Subscription
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10
        )
        self.get_logger().info("Raw 값 매핑 노드가 시작되었습니다. (스틱 값 그대로 pub)")

    def joy_callback(self, joy_msg):
        # 배열 크기 안전성 체크
        if len(joy_msg.axes) > max(self.linear_axis, self.angular_axis):
            twist = Twist()

            # 조이스틱 스틱의 -1.0 ~ 1.0 값을 그대로 cmd_vel에 대입
            twist.linear.x = joy_msg.axes[self.linear_axis]
            twist.angular.z = joy_msg.axes[self.angular_axis]
            
            self.publisher_.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = JoyToCmdVelRaw()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()