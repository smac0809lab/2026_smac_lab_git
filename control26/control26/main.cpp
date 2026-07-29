#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <Wire.h>

// 메시지 타입 헤더
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32.h>

// =====================================================
// 1. 설정 및 핀 맵
// =====================================================
struct MotorPin { uint8_t pwm; uint8_t ena; uint8_t enb; };

const MotorPin FRONT_MOTOR = {2, 3, 4};
const MotorPin REAR_MOTOR  = {5, 6, 7};
const MotorPin STEER_MOTOR = {8, 9, 10};

const uint8_t STR_POT_PIN      = A8;    // 조향 가변저항
const uint8_t ENCODER_I2C_ADDR = 0x17;  // I2C 엔코더 주소

const float WHEEL_DIAMETER_MM  = 270.0;
const float TICKS_PER_REV      = 150.0;

// =====================================================
// 2. 전역 변수 및 ROS 2 객체
// =====================================================
int target_drive_pwm = 0;
int target_steer_pwm = 0;

int32_t enc1_count      = 0;
long    prev_encoder    = 0;
unsigned long prev_speed_time = 0;

rclc_support_t    support;
rcl_node_t        node;
rcl_allocator_t   allocator;
rclc_executor_t   executor;

// Subscriptions (수동 cmd_vel 제거 완료)
rcl_subscription_t final_steer_sub, final_throttle_sub;
// Publishers
rcl_publisher_t    encoder_pub, steer_pot_pub, speed_pub;

// Messages
std_msgs__msg__Float32    final_steer_msg, final_throttle_msg, speed_kmh_msg;
std_msgs__msg__Int32      enc_raw_msg, steer_raw_msg;

enum states { WAITING_AGENT, AGENT_AVAILABLE, AGENT_CONNECTED, AGENT_DISCONNECTED } state;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){return false;}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// =====================================================
// 3. 하드웨어 제어 함수
// =====================================================

void initMotorPins(MotorPin m) {
  pinMode(m.pwm, OUTPUT); pinMode(m.ena, OUTPUT); pinMode(m.enb, OUTPUT);
}

void moveMotor(MotorPin motor, int speed) {
  if (speed == 0) {
    digitalWrite(motor.ena, LOW); digitalWrite(motor.enb, LOW);
    analogWrite(motor.pwm, 0);
  } else if (speed > 0) {
    digitalWrite(motor.ena, HIGH); digitalWrite(motor.enb, LOW);
    analogWrite(motor.pwm, min(speed, 255));
  } else {
    digitalWrite(motor.ena, LOW); digitalWrite(motor.enb, HIGH);
    analogWrite(motor.pwm, min(abs(speed), 255));
  }
}

void readI2CEncoder() {
  Wire.requestFrom(ENCODER_I2C_ADDR, (uint8_t)8);
  if (Wire.available() >= 8) {
    int32_t c1 = 0;
    c1 |= (int32_t)Wire.read() << 24;
    c1 |= (int32_t)Wire.read() << 16;
    c1 |= (int32_t)Wire.read() << 8;
    c1 |= (int32_t)Wire.read();
    enc1_count = -c1; // 엔코더 방향성 유지
    for(int i = 0; i < 4; i++) Wire.read();
  }
}

float calculateSpeed(long current_enc) {
  unsigned long current_time = millis();
  float dt = (current_time - prev_speed_time) / 1000.0f;
  if (dt <= 0.005f) return speed_kmh_msg.data;

  long delta_pulse = current_enc - prev_encoder;
  float speed_mm_s = (float(delta_pulse) / TICKS_PER_REV) * (WHEEL_DIAMETER_MM * PI) / dt;
  float speed_kmh  = speed_mm_s * 0.0036f; // km/h 단위 산출

  prev_encoder = current_enc;
  prev_speed_time = current_time;
  return speed_kmh;
}

// =====================================================
// 4. ROS 2 콜백
// =====================================================

void final_steer_callback(const void * msin) {
  const auto * msg_in = (const std_msgs__msg__Float32 *)msin;
  target_steer_pwm = constrain((int)msg_in->data, -250, 250);
}

void final_throttle_callback(const void * msin) {
  const auto * msg_in = (const std_msgs__msg__Float32 *)msin;
  target_drive_pwm = constrain((int)msg_in->data, -250, 250);
}

// =====================================================
// 5. ROS 2 엔티티 관리
// =====================================================

bool create_entities() {
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "due_drive_node", "", &support));

  // 구독(Sub) 설정
  RCCHECK(rclc_subscription_init_default(&final_steer_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "/final_steer"));
  RCCHECK(rclc_subscription_init_default(&final_throttle_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "/final_throttle"));

  // 발행(Pub) 설정
  RCCHECK(rclc_publisher_init_default(&encoder_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "/encoder_raw"));
  RCCHECK(rclc_publisher_init_default(&steer_pot_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "/steer_pot_raw"));
  RCCHECK(rclc_publisher_init_default(&speed_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "/calculateSpeed"));

  executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator)); // 핸들 개수 3 -> 2 변경
  RCCHECK(rclc_executor_add_subscription(&executor, &final_steer_sub, &final_steer_msg, &final_steer_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &final_throttle_sub, &final_throttle_msg, &final_throttle_callback, ON_NEW_DATA));

  return true;
}

void destroy_entities() {
  rcl_publisher_fini(&encoder_pub, &node);
  rcl_publisher_fini(&steer_pot_pub, &node);
  rcl_publisher_fini(&speed_pub, &node);
  rcl_subscription_fini(&final_steer_sub, &node);
  rcl_subscription_fini(&final_throttle_sub, &node);
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
}

// =====================================================
// 6. 메인 루프
// =====================================================

void setup() {
  Serial.begin(115200);
  set_microros_transports();

  Wire.begin();
  analogReadResolution(12);

  pinMode(LED_BUILTIN, OUTPUT);
  initMotorPins(FRONT_MOTOR); initMotorPins(REAR_MOTOR); initMotorPins(STEER_MOTOR);

  state = WAITING_AGENT;
}

void loop() {
  static unsigned long last_pub_time = 0;
  static unsigned long last_control_time = 0;
  static unsigned long last_ping_time = 0;

  switch (state) {
    case WAITING_AGENT:
      if (rmw_uros_ping_agent(100, 1) == RMW_RET_OK) state = AGENT_AVAILABLE;
      else { digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN)); delay(200); }
      break;

    case AGENT_AVAILABLE:
      state = create_entities() ? AGENT_CONNECTED : WAITING_AGENT;
      if (state == AGENT_CONNECTED) {
        digitalWrite(LED_BUILTIN, HIGH);
        prev_encoder = 0; prev_speed_time = millis();
      }
      break;

    case AGENT_CONNECTED:
      if (millis() - last_ping_time > 1000) {
        last_ping_time = millis();
        if (rmw_uros_ping_agent(100, 1) != RMW_RET_OK) { state = AGENT_DISCONNECTED; break; }
      }

      rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));

      // 센서 데이터 읽기 및 발행 (10Hz)
      if (millis() - last_pub_time > 100) {
        last_pub_time = millis();
        
        readI2CEncoder();
        int current_pot = analogRead(STR_POT_PIN);
        
        enc_raw_msg.data   = enc1_count;   
        steer_raw_msg.data = current_pot;  
        speed_kmh_msg.data = calculateSpeed(enc1_count);

        RCSOFTCHECK(rcl_publish(&encoder_pub, &enc_raw_msg, NULL));
        RCSOFTCHECK(rcl_publish(&steer_pot_pub, &steer_raw_msg, NULL));
        RCSOFTCHECK(rcl_publish(&speed_pub, &speed_kmh_msg, NULL));

        Serial.print("POT: "); Serial.print(current_pot);
        Serial.print(" | ENC: "); Serial.println(enc1_count);
      }

      // 모터 제어 명령 출력 (20Hz)
      if (millis() - last_control_time > 50) {
        last_control_time = millis();
        moveMotor(FRONT_MOTOR, target_drive_pwm);
        moveMotor(REAR_MOTOR,  target_drive_pwm);
        moveMotor(STEER_MOTOR, target_steer_pwm);
      }
      break;

    case AGENT_DISCONNECTED:
      moveMotor(FRONT_MOTOR, 0); moveMotor(REAR_MOTOR, 0); moveMotor(STEER_MOTOR, 0);
      destroy_entities();
      state = WAITING_AGENT;
      break;
  }
}