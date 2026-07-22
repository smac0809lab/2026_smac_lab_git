#include <Arduino.h>
#include <SPI.h>
#include <ps5Controller.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/int32.h> 
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/bool.h>

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){return false;}}

// ==========================================
// Vehicle Configuration
// ==========================================
namespace Vehicle {
    const float WHEEL_DIAMETER_MM = 270.0;
    const float TICKS_PER_REV     = 300.0;
    const float CONTROL_DT_MS     = 50.0;
    const int PWM_FREQ = 500;
    const int PWM_RES  = 8;
}

struct MotorPin { uint8_t pwm; uint8_t ena; uint8_t enb; };

const MotorPin FRONT_MOTOR = {32, 33, 22};
const MotorPin REAR_MOTOR  = {4, 13, 14};
const MotorPin STEER_MOTOR = {25, 26, 27};

const uint8_t ENC_CS_PIN = 5;
const uint8_t STR_POT_PIN = 34;

// ==========================================
// Sensor Handler
// ==========================================
const float MM_PER_TICK = (Vehicle::WHEEL_DIAMETER_MM * PI) / Vehicle::TICKS_PER_REV;

void initSensors() {
    SPI.begin(18, 19, 23);
    pinMode(ENC_CS_PIN, OUTPUT);
    digitalWrite(ENC_CS_PIN, HIGH);
    
    // 엔코더 초기화 모드 설정
    digitalWrite(ENC_CS_PIN, LOW);
    SPI.transfer(0x88); SPI.transfer(0x03);
    digitalWrite(ENC_CS_PIN, HIGH);
    
    analogReadResolution(12);
}

long readEncoder() {
    uint32_t cnt = 0;
    digitalWrite(ENC_CS_PIN, LOW);
    SPI.transfer(0x60);
    for(int i = 0; i < 4; i++) cnt = (cnt << 8) | SPI.transfer(0x00);
    digitalWrite(ENC_CS_PIN, HIGH);
    return (long)cnt;
}

float calculateSpeed(long current_enc, long &prev_enc) {
    unsigned long current_time = millis();
    static unsigned long prev_time = 0;

    float dt = (current_time - prev_time) / 1000.0f;
    
    if (dt <= 0.001f) return 0;

    long delta_pulse = current_enc - prev_enc;
    float speed_mm_s = (float(delta_pulse) / Vehicle::TICKS_PER_REV) * (Vehicle::WHEEL_DIAMETER_MM * PI) / dt; 
    float speed_kmh = speed_mm_s * 0.0036f;

    prev_enc = current_enc;
    prev_time = current_time;

    return speed_kmh;
}

int getSteerPot() {
    return analogRead(STR_POT_PIN);
}

// ==========================================
// Motor Control
// ==========================================
void initMotors() {
    MotorPin motors[3] = {FRONT_MOTOR, REAR_MOTOR, STEER_MOTOR};
    
    for (int i = 0; i < 3; i++) {
        pinMode(motors[i].ena, OUTPUT);
        pinMode(motors[i].enb, OUTPUT);
        
        ledcSetup(i, 5000, 8);
        ledcAttachPin(motors[i].pwm, i);
        
        digitalWrite(motors[i].ena, LOW);
        digitalWrite(motors[i].enb, LOW);
        ledcWrite(i, 0);
    }
}

void moveMotor(MotorPin motor, int channel, int speed, bool stop) {
    if (stop) {
        digitalWrite(motor.ena, HIGH); 
        digitalWrite(motor.enb, HIGH);
        ledcWrite(channel, 255); 
        return;
    }

    int s = constrain(abs(speed), 0, 255);
    
    if (speed > 0) {
        digitalWrite(motor.ena, HIGH); 
        digitalWrite(motor.enb, LOW);
    } else if (speed < 0) {
        digitalWrite(motor.ena, LOW);  
        digitalWrite(motor.enb, HIGH);
    } else {
        digitalWrite(motor.ena, LOW);  
        digitalWrite(motor.enb, LOW);
        s = 0;
    }
    ledcWrite(channel, s);
}

void driveVehicle(int drive_pwm, int steer_pwm, bool stop) {
    moveMotor(FRONT_MOTOR, 0, drive_pwm, stop);
    moveMotor(REAR_MOTOR, 1, drive_pwm, stop);
    moveMotor(STEER_MOTOR, 2, steer_pwm, stop);
    moveMotor(STEER_MOTOR, 2, steer_pwm, stop);
}

// ==========================================
// ROS 2 Manager
// ==========================================
enum states { WAITING_AGENT, AGENT_AVAILABLE, AGENT_CONNECTED, AGENT_DISCONNECTED } state;

unsigned long last_msg_time = 0; 

rcl_subscription_t subscriber;
rcl_subscription_t stop_subscriber;
geometry_msgs__msg__Twist msg;
std_msgs__msg__Bool stop_msg_in;
rcl_publisher_t encoder_publisher, steer_pot_publisher, final_throttle_pub, final_steer_pub, calculateSpeed_pub; 
std_msgs__msg__Int32 encoder_msg, steer_pot_msg, final_throttle_msg, final_steer_msg;
std_msgs__msg__Float32 calculateSpeed_msg; 

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

int ros_throttle = 0;
int ros_steer = 0;
bool ros_stop = false; 
long prev_encoder_for_speed = 0; 

void subscription_callback(const void * msin) {
    const geometry_msgs__msg__Twist * msg_in = (const geometry_msgs__msg__Twist *)msin;
    last_msg_time = millis(); 

    if (abs(msg_in->linear.x) > 0.01f) {
        ros_stop = false; 
    }

    ros_throttle = constrain((int)(msg_in->linear.x * 25.0f), -250, 250);
    ros_steer = constrain((int)(msg_in->angular.z * (250.0f / 22.0f)), -250, 250);
}

void stop_callback(const void * msin) {
    const std_msgs__msg__Bool * msg_in = (const std_msgs__msg__Bool *)msin;
    ros_stop = msg_in->data; 
}

void destroy_entities() {
    (void) rclc_executor_fini(&executor);
    (void) rcl_subscription_fini(&subscriber, &node);
    (void) rcl_subscription_fini(&stop_subscriber, &node);
    (void) rcl_publisher_fini(&encoder_publisher, &node);
    (void) rcl_publisher_fini(&steer_pot_publisher, &node);
    (void) rcl_publisher_fini(&final_throttle_pub, &node);
    (void) rcl_publisher_fini(&final_steer_pub, &node);
    (void) rcl_publisher_fini(&calculateSpeed_pub, &node);
    (void) rcl_node_fini(&node);
    (void) rclc_support_fini(&support);
}

bool create_entities() {
    allocator = rcl_get_default_allocator();
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    RCCHECK(rclc_node_init_default(&node, "esp32_drive_node", "", &support));

    RCCHECK(rclc_subscription_init_default(&subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "/cmd_vel"));
    RCCHECK(rclc_publisher_init_default(&encoder_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "/encoder_raw"));
    RCCHECK(rclc_publisher_init_default(&steer_pot_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "/steer_pot_raw"));
    RCCHECK(rclc_publisher_init_default(&final_throttle_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "/final_throttle"));
    RCCHECK(rclc_publisher_init_default(&final_steer_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "/final_steer"));
    RCCHECK(rclc_publisher_init_default(&calculateSpeed_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "/calculateSpeed"));
    RCCHECK(rclc_subscription_init_default(&stop_subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "/vehicle_stop"));

    executor = rclc_executor_get_zero_initialized_executor();
    RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
    RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(&executor, &stop_subscriber, &stop_msg_in, &stop_callback, ON_NEW_DATA));
    
    prev_encoder_for_speed = readEncoder();
    prev_encoder_for_speed = readEncoder();
    return true;
}

void updateROS() {
    static unsigned long last_ping = 0, last_pub = 0;
    
    switch (state) {
        case WAITING_AGENT:
            if (millis() - last_ping > 500) {
                last_ping = millis();
                state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;
            }
            break;
        case AGENT_AVAILABLE:
            state = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
            break;
        case AGENT_CONNECTED:
            if (millis() - last_ping > 1000) {
                last_ping = millis();
                if (RMW_RET_OK != rmw_uros_ping_agent(100, 1)) state = AGENT_DISCONNECTED;
            }
            if (state == AGENT_CONNECTED) {
                if (millis() - last_pub > 100) { // 10Hz
                    last_pub = millis();
                    long current_enc = readEncoder();
                    float speed_kmh = calculateSpeed(current_enc, prev_encoder_for_speed);
                    prev_encoder_for_speed = current_enc;

                    calculateSpeed_msg.data = speed_kmh;
                    encoder_msg.data = current_enc;
                    steer_pot_msg.data = getSteerPot();
                    final_throttle_msg.data = ros_throttle;
                    final_steer_msg.data = ros_steer;

                    rcl_ret_t ret;
                    ret = rcl_publish(&encoder_publisher, &encoder_msg, NULL);
                    ret = rcl_publish(&steer_pot_publisher, &steer_pot_msg, NULL);
                    ret = rcl_publish(&final_throttle_pub, &final_throttle_msg, NULL);
                    ret = rcl_publish(&final_steer_pub, &final_steer_msg, NULL);
                    ret = rcl_publish(&calculateSpeed_pub, &calculateSpeed_msg, NULL);
                    (void)ret; 
                }
                (void) rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
            }
            break;
        case AGENT_DISCONNECTED:
            destroy_entities();
            state = WAITING_AGENT;
            break;
    }
}

unsigned long getROSMsgTime() { return last_msg_time; }
void getROSCmd(int &outT, int &outS, bool &outStop) { outT = ros_throttle; outS = ros_steer; outStop = ros_stop;}
void setFinalCmd(int t, int s) { ros_throttle = t; ros_steer = s;}
void initROS() { set_microros_serial_transports(Serial); state = WAITING_AGENT; }

// ==========================================
// Setup & Loop
// ==========================================
bool isAutoMode = false;
bool lastXButtonState = false;
const unsigned long TIMEOUT_MS = 500; 

void setup() {
    Serial.begin(115200);
    initROS(); 
    
    delay(2000); 
    
    initMotors();   
    initSensors();

    ps5.begin("D4:2F:4B:00:61:10"); 
    Serial.println("System Ready.");
}

void loop() {
    updateROS(); 

    int finalThrottle = 0;
    int finalSteer = 0;
    bool finalStop = false; 

    if (ps5.isConnected()) {
        bool currentXState = ps5.Cross();
        if (currentXState && !lastXButtonState) {
            isAutoMode = !isAutoMode; 
            Serial.printf("Mode Switched: %s\n", isAutoMode ? "AUTO" : "MANUAL");
        }
        lastXButtonState = currentXState;

        if (isAutoMode) {
            unsigned long lastMsg = getROSMsgTime();
            if (millis() - lastMsg > TIMEOUT_MS) {
                finalThrottle = 0;
                finalSteer = 0;
                finalStop = false; 
            } else {
                finalStop = false;
                getROSCmd(finalThrottle, finalSteer, finalStop);
            }
        } else {
            int rawT = ps5.LStickY();
            int rawS = ps5.RStickX();

            finalThrottle = map(rawT, -128, 127, -250, 250);
            finalSteer = map(rawS, -128, 127, -250, 250);

            if (abs(finalThrottle) < 20) {
                finalThrottle = 0; 
                finalStop = false;  
            } else {
                finalStop = false; 
            }

            if (abs(finalSteer) < 20) {
                finalSteer = 0;
            }
        }

        setFinalCmd(finalThrottle, finalSteer);
        driveVehicle(finalThrottle, finalSteer, finalStop);

    } else {
        driveVehicle(0, 0, true); 
        setFinalCmd(0, 0); 
        isAutoMode = false;
    }
    
    delay(10); 
}