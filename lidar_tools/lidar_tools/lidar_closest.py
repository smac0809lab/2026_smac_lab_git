#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class ClosestObstacle(Node):
    def __init__(self):
        super().__init__('closest_obstacle')
        self.sub = self.create_subscription(LaserScan, '/scan_front', self.cb, qos_profile_sensor_data)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)


    def cb(self, msg: LaserScan):
        best_r = float('inf')
        best_i = -1

        for i, r in enumerate(msg.ranges):
            # NaN/Inf 제거 + 센서 범위(min/max) 밖 제거
            if not math.isfinite(r):
                continue
            if r <= msg.range_min or r >= msg.range_max:
                continue
            
            # index → 각도 계산
            angle_rad = msg.angle_min + i * msg.angle_increment
            angle_deg = math.degrees(angle_rad)

            # 전방 -90~+90도만 사용(-90~90은 라이다 기준 뒷쪽이라 반대 사용)
            if not (abs(angle_deg) >= 90.0):
                continue

            if r < best_r:
                best_r = r
                best_i = i

        if best_i == -1:
            self.get_logger().warn("유효한 거리값이 없음 (NaN/Inf 또는 범위 밖)")
            return

        angle_rad = msg.angle_min + best_i * msg.angle_increment
        angle_deg = math.degrees(angle_rad)

        cmd = Twist()

        STOP = 0.6
        SLOW = 1.2

        if best_r < STOP:
            cmd.linear.x = 0.0
            state = "STOP"
            cmd.angular.z = 0.0
        elif best_r < SLOW:
            cmd.linear.x = 0.15
            state = "SLOW"
            cmd.angular.z = 0.0
        else:
            cmd.linear.x = 0.35
            state = "GO"
            cmd.angular.z = 0.0

        self.cmd_pub.publish(cmd)
        self.get_logger().info(
            f"[STATE={state}] dist={best_r:.2f} m, angle={angle_deg:.1f} deg",
            throttle_duration_sec=1.0
        )

def main():
    rclpy.init()
    node = ClosestObstacle()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
