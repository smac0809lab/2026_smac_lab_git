import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix

class GpsReliabilityFilter(Node):
    def __init__(self):
        super().__init__('gps_reliability')
        
        # 파라미터로 선언 (나중에 ros2 param set으로 실시간 변경 가능)
        self.declare_parameter('cov_threshold', 0.5)
        
        self.sub = self.create_subscription(NavSatFix, '/ublox_gps_node/fix', self.callback, 10)
        self.pub = self.create_publisher(NavSatFix, '/gps_reliability_filtered', 10)
        
    def callback(self, msg):
        # 파라미터 값 읽기
        threshold = self.get_parameter('cov_threshold').get_parameter_value().double_value
        
        # 새로운 메시지 객체 생성 (데이터 보호)
        filtered_msg = msg
        current_cov = msg.position_covariance[0]
        
        if current_cov > threshold:
            # 음영 구역: 공분산 극대화
            filtered_msg.position_covariance[0] = 99999.0
            filtered_msg.position_covariance[4] = 99999.0
            filtered_msg.position_covariance[8] = 99999.0
            # EKF가 이 값을 확실히 인지하도록 타입 고정 (Known)
            filtered_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_KNOWN
            
            self.get_logger().warn(f"⚠️ GPS Bad (Cov: {current_cov:.4f}) -> Ignoring GPS")
        else:
            # 상태 양호
            self.get_logger().info(f"✅ GPS Good (Cov: {current_cov:.4f})")
            
        self.pub.publish(filtered_msg)

def main(args=None):
    rclpy.init(args=args)
    node = GpsReliabilityFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()