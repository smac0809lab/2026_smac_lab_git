import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Int32
import torch
import torch.nn as nn
from torchvision import transforms
from cv_bridge import CvBridge
import cv2
import numpy as np

# 훈련 때 사용한 모델 구조
class PilotNet(nn.Module):
    def __init__(self, input_shape=(3, 120, 160)):
        super(PilotNet, self).__init__()
        self.conv_layers = nn.Sequential(
            nn.Conv2d(3, 24, 5, stride=2), nn.ReLU(),
            nn.Conv2d(24, 36, 5, stride=2), nn.ReLU(),
            nn.Conv2d(36, 48, 5, stride=2), nn.ReLU(),
            nn.Conv2d(48, 64, 3), nn.ReLU(),
            nn.Conv2d(64, 64, 3), nn.ReLU(),
        )
        self.fc_layers = nn.Sequential(
            nn.Flatten(),
            nn.Linear(6656, 100), nn.ReLU(), # Flatten 사이즈 6656으로 고정
            nn.Linear(100, 50), nn.ReLU(),
            nn.Linear(50, 10), nn.ReLU(),
            nn.Linear(10, 2)
        )

    def forward(self, x):
        x = self.conv_layers(x)
        x = self.fc_layers(x)
        return x

class AutonomousDriveNode(Node):
    def __init__(self):
        super().__init__('autonomous_drive_node')
        
        # 1. 모델 로드 (CUDA 가능 시 GPU 사용)
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.model = PilotNet().to(self.device)
        try:
            # model.pth가 현재 실행 위치와 같은 폴더에 있어야 함
            self.model.load_state_dict(torch.load('model.pth', map_location=self.device))
            self.get_logger().info(f'모델 로드 완료! (Device: {self.device})')
        except FileNotFoundError:
            self.get_logger().error('model.pth 파일을 찾을 수 없습니다! 경로를 확인하세요.')
            
        self.model.eval()
        self.data_received = False
        
        self.bridge = CvBridge()
        self.transform = transforms.Compose([
            transforms.ToPILImage(),
            transforms.Resize((120, 160)),
            transforms.ToTensor(),
            transforms.Normalize((0.5, 0.5, 0.5), (0.5, 0.5, 0.5))
        ])

        # 2. 구독 및 발행 (토픽 이름: /ros_steer, /ros_throttle)
        self.subscription = self.create_subscription(
            CompressedImage,
            '/video_frames/compressed', 
            self.image_callback,
            10)
        
        # 수동 주행(final)과 구분하기 위해 자동 주행 전용 토픽 사용
        self.steer_pub = self.create_publisher(Int32, '/ros_steer', 10)
        self.throttle_pub = self.create_publisher(Int32, '/ros_throttle', 10)
        
        self.get_logger().info('카메라 데이터 대기 중... (/ros_steer, /ros_throttle 준비 완료)')

    def image_callback(self, msg):
        if not self.data_received:
            self.get_logger().info('카메라 데이터 수신됨! 자동 주행 추론을 시작합니다.')
            self.data_received = True

        # 1. 이미지 디코딩 및 RGB 변환 (필수!!)
        np_arr = np.frombuffer(msg.data, np.uint8)
        cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB) # BGR -> RGB 필수
        
        # 2. 전처리 (학습 때 사용한 transform 적용)
        input_tensor = self.transform(cv_image).unsqueeze(0).to(self.device)
        
        with torch.no_grad():
            outputs = self.model(input_tensor)
            # 학습 시 /128 했으므로 출력에 *128을 해야 정확한 값이 나옴
            steer_out = outputs[0][0].item()
            throttle_out = outputs[0][1].item()

        # 3. 값 복원 (256 -> 128로 수정)
        ros_steer = int(steer_out * 256) 
        ros_throttle = 80 # 속도는 80 고정 잘하셨습니다.
        
        # 4. 안전 범위 제한 (ESP32 코드의 map 범위인 -250 ~ 250에 맞춤)
        ros_steer = max(min(ros_steer, 250), -250)
        
        # 5. 메시지 발행 (확실하게 int로 형변환)
        steer_msg = Int32()
        steer_msg.data = int(ros_steer)
        self.steer_pub.publish(steer_msg)

        throttle_msg = Int32()
        throttle_msg.data = int(ros_throttle)
        self.throttle_pub.publish(throttle_msg)
        # 디버깅용 로그
        self.get_logger().info(f'[Auto Mode] Steer: {ros_steer}, Throttle: {ros_throttle}')

def main(args=None):
    rclpy.init(args=args)
    node = AutonomousDriveNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('사용자에 의해 중단되었습니다.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()