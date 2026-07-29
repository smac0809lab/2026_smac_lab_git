import csv
import os
from mcap_ros2.reader import read_ros2_messages
from datetime import datetime

# === 설정 부분 ===
NEW_BAG_PATH = "/home/user/ros2_ws/rosbag/0326.mcap" 
OUTPUT_DIR = "log"

if not os.path.exists(OUTPUT_DIR):
    os.makedirs(OUTPUT_DIR)

# 추출할 토픽 리스트 및 컬럼 정의
TOPICS = {
    '/ublox_gps_node/fix': ['human_time', 'timestamp', 'lat', 'lon', 'alt'],
    '/imu/data': ['human_time', 'timestamp', 'qx', 'qy', 'qz', 'qw', 'wx', 'wy', 'wz', 'ax', 'ay', 'az'], 
    '/odometry/gps': ['human_time', 'timestamp', 'x', 'y', 'z'],
    '/odometry/filtered': ['human_time', 'timestamp', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'],
    # --- [새로 추가된 토픽들] ---
    '/odometry/imu_encoder': ['human_time', 'timestamp', 'x', 'y', 'vx', 'yaw'], # 융합 위치 및 속도
    '/vehicle/encoder_velocity': ['human_time', 'timestamp', 'speed_ms']        # 인코더 원본 속도
}

files = {topic: open(f"{OUTPUT_DIR}/{topic.replace('/', '_')}.csv", 'w', newline='') for topic in TOPICS}
writers = {topic: csv.writer(files[topic]) for topic in TOPICS}

for topic, header in TOPICS.items():
    writers[topic].writerow(header)

print(f"🚀 추출 시작 : {NEW_BAG_PATH}...")

try:
    for msg in read_ros2_messages(NEW_BAG_PATH):
        topic = msg.channel.topic 
        
        if topic in TOPICS:
            # 시간 처리
            if isinstance(msg.publish_time, datetime):
                t = msg.publish_time.timestamp()
            else:
                t = msg.publish_time / 1e9
            
            dt_object = datetime.fromtimestamp(t)
            human_time = dt_object.strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
                
            ros_msg = msg.ros_msg
            
            if topic == '/ublox_gps_node/fix':
                writers[topic].writerow([human_time, t, ros_msg.latitude, ros_msg.longitude, ros_msg.altitude])
            
            elif topic == '/imu/data':
                o = ros_msg.orientation
                w = ros_msg.angular_velocity
                a = ros_msg.linear_acceleration
                writers[topic].writerow([human_time, t, o.x, o.y, o.z, o.w, w.x, w.y, w.z, a.x, a.y, a.z])
                
            elif topic == '/odometry/gps':
                p = ros_msg.pose.pose.position
                writers[topic].writerow([human_time, t, p.x, p.y, p.z])
                
            elif topic == '/odometry/filtered':
                p = ros_msg.pose.pose.position
                q = ros_msg.pose.pose.orientation
                writers[topic].writerow([human_time, t, p.x, p.y, p.z, q.x, q.y, q.z, q.w])

            # --- [추가된 로직 시작] ---
            elif topic == '/odometry/imu_encoder':
                p = ros_msg.pose.pose.position
                v = ros_msg.twist.twist.linear
                q = ros_msg.pose.pose.orientation
                # 쿼터니언에서 간단한 Yaw 값 계산 (필요시 사용)
                import math
                siny_cosp = 2 * (q.w * q.z + q.x * q.y)
                cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
                yaw = math.atan2(siny_cosp, cosy_cosp)
                
                writers[topic].writerow([human_time, t, p.x, p.y, v.x, yaw])

            elif topic == '/vehicle/encoder_velocity':
                v_x = ros_msg.twist.twist.linear.x
                writers[topic].writerow([human_time, t, v_x])
            # --- [추가된 로직 끝] ---

finally:
    for f in files.values():
        f.close()

print(f"✅ 추출 완료! log 폴더 내 아래 파일들이 생성되었습니다:")
for t in TOPICS.keys():
    print(f" - {t.replace('/', '_')}.csv")