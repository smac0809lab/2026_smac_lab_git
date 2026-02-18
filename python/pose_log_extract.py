import pandas as pd
import matplotlib.pyplot as plt
import os

# === 경로 설정 ===
LOG_DIR = "/home/user/ros2_ws/src/python/log"

# 파일명 매핑 (추출하신 파일명과 일치하는지 확인하세요)
files_config = [
    {'label': 'IMU-Encoder', 'file': '_odometry_imu_encoder.csv', 'color': 'green'},
    {'label': 'EKF-Filtered', 'file': '_odometry_filtered.csv', 'color': 'blue'},
    {'label': 'GPS-Raw', 'file': '_odometry_gps.csv', 'color': 'red'}
]

plt.figure(figsize=(10, 10))

for config in files_config:
    file_path = os.path.join(LOG_DIR, config['file'])
    
    if os.path.exists(file_path):
        # CSV 읽기
        df = pd.read_csv(file_path)
        
        # 3번째 열(index 2)이 X, 4번째 열(index 3)이 Y
        # 만약 CSV 헤더 순서가 다르면 아래 인덱스를 조정하세요.
        try:
            x = df.iloc[:, 2]
            y = df.iloc[:, 3]
            
            plt.plot(x, y, label=config['label'], color=config['color'], linewidth=1.5, alpha=0.8)
            
            # 시작점 표시
            plt.scatter(x.iloc[0], y.iloc[0], color=config['color'], edgecolors='black', s=50, zorder=5)
        except Exception as e:
            print(f"❌ {config['label']} 데이터 처리 중 오류 발생: {e}")
    else:
        print(f"⚠️ 파일을 찾을 수 없습니다: {file_path}")

# 그래프 설정
plt.title("Odom Trajectory Comparison (X-Y Plane)", fontsize=14)
plt.xlabel("X Coordinate [m]", fontsize=12)
plt.ylabel("Y Coordinate [m]", fontsize=12)
plt.legend()
plt.grid(True, linestyle=':', alpha=0.6)
plt.axis('equal') # 실제 주행 궤적 비율 유지

# 이미지 저장 및 출력
output_plot = os.path.join(LOG_DIR, "trajectory_comparison.png")
plt.savefig(output_plot)
print(f"✅ 그래프가 저장되었습니다: {output_plot}")
plt.show()