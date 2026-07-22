#include <Arduino.h>
#include <SPI.h>
#include <ps5Controller.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/int32.h> 

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){return false;}}

// 핀 설정
struct MotorPin { uint8_t pwm; uint8_t ena; uint8_t enb; };
const MotorPin FRONT_MOTOR = {32, 33, 22};
const MotorPin REAR_MOTOR  = {4, 13, 14};
const MotorPin STEER_MOTOR = {25, 26, 27};
const uint8_t ENC_CS_PIN = 5;

// 글로벌 변수
int steer_from_due = 0; 
int ros_throttle = 0, ros_steer = 0;
unsigned long last_msg_time = 0;
bool isAutoMode = false;
bool lastXButtonState = false;

// ROS 2 관련
rclc_support_t support;
rcl_node_t node;
rcl_allocator_t allocator;
rclc_executor_t executor;
rcl_subscription_t subscriber;
rcl_publisher_t encoder_publisher, steer_pot_publisher;
std_msgs__msg__Int32 encoder_msg, steer_pot_msg;
geometry_msgs__msg__Twist msg;

enum states { WAITING_AGENT, AGENT_AVAILABLE, AGENT_CONNECTED, AGENT_DISCONNECTED } state;

// ==========================================
// Due 데이터 수신 (안정화 버전)
// ==========================================
void updateFromDue() {
    // Serial2에 데이터가 있을 때만 읽음
    while (Serial2.available() > 0) {
        String data = Serial2.readStringUntil('\n');
        data.trim(); // 공백 제거
        
        if (data.startsWith("P")) {
            String valStr = data.substring(1);
            if (valStr.length() > 0) {
                steer_from_due = valStr.toInt();
            }
        }
    }
}

// ==========================================
// 모터 제어 함수
// ==========================================
void moveMotor(MotorPin motor, int channel, int speed, bool stop) {
    if (stop) {
        digitalWrite(motor.ena, HIGH); digitalWrite(motor.enb, HIGH);
        ledcWrite(channel, 255); return;
    }
    int s = constrain(abs(speed), 0, 255);
    if (speed > 0) { digitalWrite(motor.ena, HIGH); digitalWrite(motor.enb, LOW); }
    else if (speed < 0) { digitalWrite(motor.ena, LOW); digitalWrite(motor.enb, HIGH); }
    else { digitalWrite(motor.ena, LOW); digitalWrite(motor.enb, LOW); s = 0; }
    ledcWrite(channel, s);
}

void driveVehicle(int drive_pwm, int steer_pwm, bool stop) {
    moveMotor(FRONT_MOTOR, 0, drive_pwm, stop);
    moveMotor(REAR_MOTOR, 1, drive_pwm, stop);
    moveMotor(STEER_MOTOR, 2, steer_pwm, stop);
}

// ==========================================
// ROS 2 콜백
// ==========================================
void subscription_callback(const void * msin) {
    const geometry_msgs__msg__Twist * m = (const geometry_msgs__msg__Twist *)msin;
    last_msg_time = millis();
    ros_throttle = constrain((int)(m->linear.x * 255.0f), -255, 255);
    ros_steer = constrain((int)(m->angular.z * 255.0f), -255, 255);
}

bool create_entities() {
    allocator = rcl_get_default_allocator();
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    RCCHECK(rclc_node_init_default(&node, "esp32_drive_node", "", &support));

    RCCHECK(rclc_subscription_init_default(&subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "/cmd_vel"));
    RCCHECK(rclc_publisher_init_default(&encoder_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "/encoder_raw"));
    RCCHECK(rclc_publisher_init_default(&steer_pot_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "/steer_pot_raw"));

    executor = rclc_executor_get_zero_initialized_executor();
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));
    return true;
}

void setup() {
    // PC 통신 (micro-ROS)
    Serial.begin(115200);
    // Due 통신 (Serial2 사용으로 Serial과 분리)
    Serial2.begin(115200, SERIAL_8N1, 21, 17); 
    
    set_microros_serial_transports(Serial);
    state = WAITING_AGENT;

    // 모터 핀 초기화
    MotorPin motors[3] = {FRONT_MOTOR, REAR_MOTOR, STEER_MOTOR};
    for (int i = 0; i < 3; i++) {
        pinMode(motors[i].ena, OUTPUT); pinMode(motors[i].enb, OUTPUT);
        ledcSetup(i, 5000, 8); ledcAttachPin(motors[i].pwm, i);
    }

    ps5.begin("D4:2F:4B:00:61:10"); 
}

void loop() {
    // 1. micro-ROS 에이전트 연결 관리
    switch (state) {
        case WAITING_AGENT:
            if (rmw_uros_ping_agent(100, 1) == RMW_RET_OK) state = AGENT_AVAILABLE;
            break;
        case AGENT_AVAILABLE:
            state = create_entities() ? AGENT_CONNECTED : WAITING_AGENT;
            break;
        case AGENT_CONNECTED:
            static unsigned long last_pub = 0;
            if (millis() - last_pub > 50) { // 20Hz 주기로 데이터 발행
                last_pub = millis();
                steer_pot_msg.data = steer_from_due;
                rcl_publish(&steer_pot_publisher, &steer_pot_msg, NULL);
            }
            if (rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)) != RCL_RET_OK) {
                state = WAITING_AGENT;
            }
            break;
        default: break;
    }

    // 2. Due 데이터 읽기 (항상 실행)
    updateFromDue();

    // 3. 주행 제어 로직
    int finalT = 0, finalS = 0;
    bool finalStop = false;

    if (ps5.isConnected()) {
        // 모드 전환 (X 버튼)
        if (ps5.Cross() && !lastXButtonState) isAutoMode = !isAutoMode;
        lastXButtonState = ps5.Cross();

        if (isAutoMode) {
            // 자율 주행 모드
            if (millis() - last_msg_time > 500) finalT = 0; 
            else { finalT = ros_throttle; finalS = ros_steer; }
        } else {
            // 수동 주행 모드 (PS5 스틱)
            finalT = map(ps5.LStickY(), -128, 127, -250, 250);
            finalS = map(ps5.RStickX(), -128, 127, -250, 250);
            if (abs(finalT) < 20) finalT = 0;
        }
    } else {
        // 컨트롤러 연결 끊김 시 정지
        finalStop = true;
    }

    driveVehicle(finalT, finalS, finalStop);
    delay(10);
}