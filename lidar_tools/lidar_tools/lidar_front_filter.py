import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import LaserScan


class LidarFront180Filter(Node):
    """
    Subscribes:  /scan (sensor_msgs/LaserScan)
    Publishes:   /scan_front (sensor_msgs/LaserScan)

    Keeps only the front angular sector (default: -90deg ~ +90deg).
    """

    def __init__(self):
        super().__init__('lidar_front_180_filter')

        # Parameters (degrees)
        self.declare_parameter('lower_deg', -90.0)
        self.declare_parameter('upper_deg',  90.0)
        self.declare_parameter('in_topic',   '/scan')
        self.declare_parameter('out_topic',  '/scan_front')

        self.lower_deg = float(self.get_parameter('lower_deg').value)
        self.upper_deg = float(self.get_parameter('upper_deg').value)
        self.in_topic  = str(self.get_parameter('in_topic').value)
        self.out_topic = str(self.get_parameter('out_topic').value)

        # Sensor QoS (many lidar drivers use best-effort)
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.sub = self.create_subscription(LaserScan, self.in_topic, self.cb_scan, qos)
        self.pub = self.create_publisher(LaserScan, self.out_topic, qos)

        self.get_logger().info(
            f"Filtering LaserScan: in={self.in_topic} out={self.out_topic} "
            f"keep=[{self.lower_deg}°, {self.upper_deg}°] (180° flipped)"
        )

    def normalize_angle(self, a):
        while a > math.pi:
            a -= 2 * math.pi
        while a < -math.pi:
            a += 2 * math.pi
        return a

    def cb_scan(self, msg: LaserScan):
        lower = math.radians(self.lower_deg)
        upper = math.radians(self.upper_deg)

        # Ensure order
        if lower > upper:
            lower, upper = upper, lower

        out = LaserScan()
        out.header = msg.header
        out.angle_min = msg.angle_min
        out.angle_max = msg.angle_max
        out.angle_increment = msg.angle_increment
        out.time_increment = msg.time_increment
        out.scan_time = msg.scan_time
        out.range_min = msg.range_min
        out.range_max = msg.range_max

        n = len(msg.ranges)
        ranges = list(msg.ranges)

        has_int = len(msg.intensities) == n and n > 0
        intensities = list(msg.intensities) if has_int else []

        for i in range(n):
            ang = msg.angle_min + i * msg.angle_increment

            # 180도 뒤집기
            ang = self.normalize_angle(ang + math.pi)

            if ang < lower or ang > upper:
                ranges[i] = float('inf')
                if has_int:
                    intensities[i] = 0.0

        out.ranges = ranges
        if has_int:
            out.intensities = intensities

        self.pub.publish(out)


def main():
    rclpy.init()
    node = LidarFront180Filter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
