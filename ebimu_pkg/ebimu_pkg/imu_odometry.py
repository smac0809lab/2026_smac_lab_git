import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Pose, TransformStamped, PoseStamped # PoseStamped 추가
import numpy as np
from scipy.spatial.transform import Rotation as R
import tf2_ros

class IMUOdometry(Node):
    def __init__(self):
        super().__init__('imu_odometry_node')

        # --- [튜닝 파라미터] ---
        self.accel_threshold = 0.5   
        self.gyro_threshold = 0.1    
        self.alpha = 0.001            
        self.damping = 0.99          
        # ----------------------

        # 구독 및 발행 설정
        self.subscription = self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)
        
        # [추가] Pose 전용 토픽 퍼블리셔
        self.pose_publisher = self.create_publisher(PoseStamped, '/current_pose', 10)
        
        self.odom_publisher = self.create_publisher(Odometry, '/odom', 10)
        self.path_publisher = self.create_publisher(Path, '/robot_path', 10)
        
        self.path_msg = Path()
        self.path_msg.header.frame_id = 'odom'
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.pose = np.zeros(3)
        self.vel = np.zeros(3)
        self.last_time = None
        self.is_calibrated = False
        self.calib_samples = []
        self.base_gravity_world = np.zeros(3) 

    def imu_callback(self, msg):
        current_time_msg = msg.header.stamp
        current_time = current_time_msg.sec + current_time_msg.nanosec * 1e-9
        
        if msg.linear_acceleration.z == 0:
            return

        accel_raw = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
        gyro_raw = np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
        quat_xyzw = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]

        if not self.is_calibrated:
            rot = R.from_quat(quat_xyzw)
            accel_world = rot.apply(accel_raw)
            self.calib_samples.append(accel_world)
            if len(self.calib_samples) >= 100:
                self.base_gravity_world = np.mean(self.calib_samples, axis=0)
                self.is_calibrated = True
                self.get_logger().info(f'✅ 보정 완료! Pose 토픽 발행 시작')
            return

        self.update(accel_raw, gyro_raw, quat_xyzw, current_time)
        self.publish_data(current_time_msg, quat_xyzw)

    def update(self, accel_raw, gyro_raw, quat_xyzw, current_time):
        if self.last_time is None:
            self.last_time = current_time
            return
        
        dt = current_time - self.last_time
        self.last_time = current_time

        rot = R.from_quat(quat_xyzw)
        accel_world = rot.apply(accel_raw)
        gyro_norm = np.linalg.norm(gyro_raw)

        if gyro_norm < self.gyro_threshold:
            self.base_gravity_world = (1 - self.alpha) * self.base_gravity_world + self.alpha * accel_world

        pure_accel = accel_world - self.base_gravity_world
        accel_norm = np.linalg.norm(pure_accel[:2])

        if accel_norm < self.accel_threshold and gyro_norm < self.gyro_threshold:
            self.vel = np.zeros(3)
            display_accel = 0.0
        else:
            self.vel += pure_accel * dt
            self.vel *= self.damping 
            display_accel = accel_norm

        self.pose += self.vel * dt
        self.pose[2] = 0.0

        self.get_logger().info(f'P: [{self.pose[0]:.2f}, {self.pose[1]:.2f}] | V: {np.linalg.norm(self.vel):.2f} | A: {display_accel:.2f}')

    def publish_data(self, stamp, quat_xyzw):
        # --- [1] PoseStamped 발행 (X, Y, Orientation 모니터링용) ---
        ps = PoseStamped()
        ps.header.stamp = stamp
        ps.header.frame_id = 'odom'
        ps.pose.position.x = self.pose[0]
        ps.pose.position.y = self.pose[1]
        ps.pose.position.z = 0.0
        ps.pose.orientation.x = quat_xyzw[0]
        ps.pose.orientation.y = quat_xyzw[1]
        ps.pose.orientation.z = quat_xyzw[2]
        ps.pose.orientation.w = quat_xyzw[3]
        self.pose_publisher.publish(ps)

        # --- [2] Odometry 발행 (공분산 0 유지) ---
        odom = Odometry()
        odom.header = ps.header
        odom.child_frame_id = 'base_link'
        odom.pose.pose = ps.pose
        self.odom_publisher.publish(odom)

        # --- [3] TF 발행 ---
        t = TransformStamped()
        t.header = ps.header
        t.child_frame_id = 'base_link'
        t.transform.translation.x = ps.pose.position.x
        t.transform.translation.y = ps.pose.position.y
        t.transform.translation.z = 0.0
        t.transform.rotation = ps.pose.orientation
        self.tf_broadcaster.sendTransform(t)

        # --- [4] Path 발행 ---
        self.path_msg.poses.append(ps)
        if len(self.path_msg.poses) > 1000:
            self.path_msg.poses.pop(0)
        self.path_publisher.publish(self.path_msg)

def main(args=None):
    rclpy.init(args=args)
    node = IMUOdometry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()