import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import csv
import time

class OdomLogger(Node):
    def __init__(self):
        super().__init__('odom_logger')
        
        # CSV 파일 열기
        self.file = open('odom_data.csv', 'w', newline='')
        self.writer = csv.writer(self.file)
        self.writer.writerow(['timestamp', 'type', 'x', 'y', 'z'])
        
        # Subscriber 설정
        self.sub_filtered = self.create_subscription(
            Odometry, '/odometry/filtered', self.filtered_callback, 10)
        self.sub_gps = self.create_subscription(
            Odometry, '/odometry/gps', self.gps_callback, 10)
        
        self.get_logger().info('데이터 기록 시작: odom_data.csv')

    def filtered_callback(self, msg):
        self.log_data('filtered', msg)

    def gps_callback(self, msg):
        self.log_data('gps', msg)

    def log_data(self, label, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z
        self.writer.writerow([t, label, x, y, z])

    def destroy_node(self):
        self.file.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = OdomLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()