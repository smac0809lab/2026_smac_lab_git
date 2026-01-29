import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage  # CompressedImage 추가
from std_msgs.msg import Header
from cv_bridge import CvBridge
import cv2

class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher')
        
        # 1. 원본 이미지 퍼블리셔
        self.publisher_raw = self.create_publisher(Image, 'video_frames', 10)
        
        # 2. 압축 이미지 퍼블리셔 (네트워크 전송용)
        self.publisher_compressed = self.create_publisher(CompressedImage, 'video_frames/compressed', 10)
        
        # 10Hz (0.1초) 주기로 실행
        self.timer = self.create_timer(0.1, self.timer_callback)

        # 카메라 설정 (USB0 고정)
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error('카메라를 열 수 없습니다!')
            exit()

        self.br = CvBridge()

    def timer_callback(self):
        ret, frame = self.cap.read()

        if ret:
            # 현재 시간 정보(Timestamp) 생성
            current_time = self.get_clock().now().to_msg()
            
            # --- [1] 원본 이미지 메시지 작성 ---
            img_msg = self.br.cv2_to_imgmsg(frame, encoding="bgr8")
            img_msg.header.stamp = current_time
            img_msg.header.frame_id = 'camera_link'
            self.publisher_raw.publish(img_msg)

            # --- [2] 압축 이미지 메시지 작성 ---
            comp_msg = CompressedImage()
            comp_msg.header.stamp = current_time
            comp_msg.header.frame_id = 'camera_link'
            comp_msg.format = "jpeg"
            # OpenCV를 사용하여 JPEG로 압축 후 메시지에 담기
            comp_msg.data = cv2.imencode('.jpg', frame)[1].tobytes()
            self.publisher_compressed.publish(comp_msg)

            # self.get_logger().info('이미지 및 압축 이미지 발행 중...')
        else:
            self.get_logger().warn('프레임을 읽지 못했습니다.')

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    image_publisher = ImagePublisher()
    
    try:
        rclpy.spin(image_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        image_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()