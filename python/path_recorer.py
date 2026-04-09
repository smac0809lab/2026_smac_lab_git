import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
import csv
import os
# ProtoRPC를 제외하고 Transformer만 임포트합니다.
from pyproj import Transformer

class UtmPathRecorder(Node):
    def __init__(self):
        super().__init__('utm_path_recorder')
        
        # 1. GPS 토픽 구독 (메시지 타입: NavSatFix)
        # 터미널에서 'ros2 topic list'를 확인하여 실제 GPS 토픽명으로 수정하세요.
        self.subscription = self.create_subscription(
            NavSatFix, 
            '/ublox_gps_node/fix', 
            self.gps_callback, 
            10)
        
        # 2. WGS84 -> UTM Zone 52N 변환기 (충주 및 한국 중동부 표준)
        self.transformer = Transformer.from_crs("epsg:4326", "epsg:32652", always_xy=True)

        # 3. 파일 저장 설정
        self.file_path = '/home/user/ros2_ws/src/python/utm_path.csv'
        self.file = open(self.file_path, 'w', newline='')
        self.writer = csv.writer(self.file)
        self.writer.writerow(['x', 'y', 'alt']) 
        
        self.last_x, self.last_y = 0.0, 0.0
        self.first_record = True
        
        self.get_logger().info(f"UTM 녹화 시작! 저장 경로: {self.file_path}")

    def gps_callback(self, msg):
        # GPS 수신 상태 체크 (0 이상이어야 정상 Fix)
        if msg.status.status < 0:
            self.get_logger().warn("GPS Fix가 잡히지 않았습니다.", throttle_duration_sec=5.0)
            return

        # 경위도 -> UTM 변환 (always_xy=True 설정으로 x, y 순서 보장)
        utm_x, utm_y = self.transformer.transform(msg.longitude, msg.latitude)
        utm_z = msg.altitude

        # 처음 기록하거나, 0.3m 이상 이동했을 때만 저장
        dist = ((utm_x - self.last_x)**2 + (utm_y - self.last_y)**2)**0.5
        
        if self.first_record or dist > 0.3:
            self.writer.writerow([f"{utm_x:.6f}", f"{utm_y:.6f}", f"{utm_z:.3f}"])
            self.file.flush() # 파일에 즉시 쓰기
            self.last_x, self.last_y = utm_x, utm_y
            self.first_record = False
            self.get_logger().info(f"기록 중: X={utm_x:.2f}, Y={utm_y:.2f}")

def main():
    rclpy.init()
    node = UtmPathRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("녹화 완료. 파일을 닫습니다.")
    finally:
        node.file.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()