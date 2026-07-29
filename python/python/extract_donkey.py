import os
import cv2
import csv
import rosbag2_py
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message
from cv_bridge import CvBridge

def extract_multiple_bags(bag_paths, storage_id='mcap'):
    # 설정
    output_dir = "extracted_data"
    image_dir = os.path.join(output_dir, "images")
    csv_path = os.path.join(output_dir, "driving_log.csv")
    
    if not os.path.exists(image_dir):
        os.makedirs(image_dir)

    bridge = CvBridge()
    
    # [수정] 모든 bag 데이터를 하나로 통합할 리스트
    data_log = []
    global_frame_count = 0  # 전체 프레임 카운트 (겹침 방지)

    # 헤더는 처음 한 번만 쓰기 위해 파일 오픈
    with open(csv_path, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['image_path', 'steer', 'throttle'])

        # 3개의 bag 파일을 순차적으로 순회
        for bag_path in bag_paths:
            print(f"현재 처리 중인 파일: {bag_path}")
            
            # bag 파일 이름 추출 (파일명 구분용)
            bag_name = os.path.basename(bag_path)

            # ROS2 Bag 설정
            reader = rosbag2_py.SequentialReader()
            storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id=storage_id)
            converter_options = rosbag2_py.ConverterOptions(
                input_serialization_format='cdr',
                output_serialization_format='cdr'
            )
            reader.open(storage_options, converter_options)

            topic_types = reader.get_all_topics_and_types()
            type_map = {topic.name: topic.type for topic in topic_types}
            
            last_steer = 0
            last_throttle = 0
            bag_frame_count = 0

            while reader.has_next():
                (topic, data, t) = reader.read_next()
                msg_type = get_message(type_map[topic])
                msg = deserialize_message(data, msg_type)

                if topic == '/final_steer':
                    last_steer = msg.data
                elif topic == '/final_throttle':
                    last_throttle = msg.data
                
                elif topic == '/video_frames/compressed':
                    # [수정] 파일명에 bag 이름을 포함시켜 절대 안 겹치게 설정
                    frame_filename = f"{bag_name}_{bag_frame_count:05d}.jpg"
                    
                    # CompressedImage -> OpenCV 이미지 변환
                    cv_img = bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
                    cv2.imwrite(os.path.join(image_dir, frame_filename), cv_img)
                    
                    # CSV 기록용 경로 (train.py에서 읽을 형식)
                    csv_image_path = os.path.join("images", frame_filename)
                    writer.writerow([csv_image_path, last_steer, last_throttle])
                    
                    bag_frame_count += 1
                    global_frame_count += 1

            print(f"--- {bag_name} 추출 완료 ({bag_frame_count} 프레임)")

    print(f"\n모든 추출 완료! 총 {global_frame_count}개의 프레임이 저장되었습니다.")
    print(f"결과물 위치: {output_dir}")

if __name__ == "__main__":
    # [수정] 가공할 3개의 bag 파일 경로 리스트
    BAG_PATHS = [
        '/home/user/ros2_ws/rosbag/복도주행1',
        '/home/user/ros2_ws/rosbag/복도주행2',
        '/home/user/ros2_ws/rosbag/복도주행3'
    ]
    extract_multiple_bags(BAG_PATHS)