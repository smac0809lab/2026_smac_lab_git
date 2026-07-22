import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField
from geometry_msgs.msg import TransformStamped
from iahrs_driver_interface.srv import Set

import tf2_ros
import serial
import time
import math

class IahrsDriver(Node):
    def __init__(self):
        super().__init__("iahrs_driver_node")
        
        # --- 설정부 ---
        self.port_name = "/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0"
        self.baud_rate = 921600 
        self.DEBUG_MODE = True  # True로 설정하면 터미널에 데이터 개수와 raw 데이터가 찍힙니다.
        
        self._tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        self._tf_prefix = ""
        self._is_send_tf = True
        self._ser = None
        
        # 메시지 객체 초기화
        self._imu_msg = Imu()
        self._mag_msg = MagneticField()
        self._init_msgs()

        # 퍼블리셔 및 서비스 등록
        self._imu_pub_handler = self.create_publisher(Imu, "imu/data", 10)
        self._mag_pub_handler = self.create_publisher(MagneticField, "imu/mag", 10)
        self.create_service(Set, "reset_sensor", self._reset_sensor_callback)
        
        # 타이머 (10ms 주기 체크)
        self.timer = self.create_timer(0.01, self._main_loop)
        self._connect_serial()

    def _init_msgs(self):
        # 공분산 초기화 (0.0은 미사용 의미, 추후 필요시 수정 가능)
        for i in range(9):
            self._imu_msg.linear_acceleration_covariance[i] = 0.0
            self._imu_msg.angular_velocity_covariance[i] = 0.0
            self._imu_msg.orientation_covariance[i] = 0.0
            self._mag_msg.magnetic_field_covariance[i] = 0.0

    def _connect_serial(self):
        try:
            if self._ser is not None: self._ser.close()
            self._ser = serial.Serial(self.port_name, self.baud_rate, timeout=0.1)
            self.get_logger().info(f"[{self.port_name}] {self.baud_rate}bps 연결 성공")
            self._reset_sensor_hardware()
        except Exception as e:
            self.get_logger().error(f"시리얼 연결 실패: {e}")
            self._ser = None

    def _main_loop(self):
        if self._ser is None or not self._ser.is_open:
            return

        try:
            while self._ser.in_waiting > 0:
                line = self._ser.readline()
                if not line: break
                
                try:
                    decoded = line.decode("utf-8", errors="ignore").strip()
                    if not decoded or "=" in decoded: continue

                    # 데이터 파싱
                    raw_values = decoded.split(",")
                    v = [float(x) for x in raw_values if x.strip()]
                    
                    if self.DEBUG_MODE:
                        # 터미널에서 데이터가 제대로 들어오는지 감시용
                        self.get_logger().info(f"RAW(Count:{len(v)}): {v[:3]}...", throttle_duration_sec=1.0)

                    # 16개 데이터(Acc, Gyro, Mag, Angl, Quat)가 들어올 때 처리
                    if len(v) >= 16:
                        self._process_and_publish(v)
                        
                except ValueError:
                    continue 
        except Exception as e:
            self.get_logger().error(f"루프 에러: {e}")

    def _process_and_publish(self, v):
        now = self.get_clock().now().to_msg()
        frame_id = self._tf_prefix + "imu_link"

        # 1. IMU 데이터 (v[0:3]: Acc, v[3:6]: Gyro)
        self._imu_msg.header.stamp = now
        self._imu_msg.header.frame_id = frame_id
        
        # 선가속도 (g -> m/s^2)
        self._imu_msg.linear_acceleration.x = v[0] * 9.80665
        self._imu_msg.linear_acceleration.y = v[1] * 9.80665
        self._imu_msg.linear_acceleration.z = v[2] * 9.80665
        
        # 각속도 (deg/s -> rad/s)
        self._imu_msg.angular_velocity.x = v[3] * (math.pi / 180.0)
        self._imu_msg.angular_velocity.y = v[4] * (math.pi / 180.0)
        self._imu_msg.angular_velocity.z = v[5] * (math.pi / 180.0)

        # 쿼터니언 (v[12:16] -> w, x, y, z)
        self._imu_msg.orientation.w = v[12]
        self._imu_msg.orientation.x = v[13]
        self._imu_msg.orientation.y = v[14]
        self._imu_msg.orientation.z = v[15]

        self._imu_pub_handler.publish(self._imu_msg)

        # 2. 지자계 데이터 (v[6:9] -> Mag)
        self._mag_msg.header.stamp = now
        self._mag_msg.header.frame_id = frame_id
        # mG -> Tesla (1e-7)
        self._mag_msg.magnetic_field.x = v[6] * 1e-7
        self._mag_msg.magnetic_field.y = -v[7] * 1e-7
        self._mag_msg.magnetic_field.z = v[8] * 1e-7
        self._mag_pub_handler.publish(self._mag_msg)

        if self._is_send_tf:
            self._send_tf()

    def _send_tf(self):
        t = TransformStamped()
        t.header.stamp = self._imu_msg.header.stamp
        t.header.frame_id = self._tf_prefix + "base_link"
        t.child_frame_id = self._tf_prefix + "imu_link"
        t.transform.rotation = self._imu_msg.orientation
        self._tf_broadcaster.sendTransform(t) # 미리 선언한 객체 사용

    def _reset_sensor_hardware(self):
        if self._ser and self._ser.is_open:
            self._ser.reset_input_buffer()
            self._ser.write(b"so=1\n") # 스트리밍 시작
            self.get_logger().info("센서 하드웨어 스트리밍 명령 전송 완료")

    def _reset_sensor_callback(self, req, res):
        self._reset_sensor_hardware()
        res.result = True
        return res

def main(args=None):
    rclpy.init(args=args)
    node = IahrsDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()