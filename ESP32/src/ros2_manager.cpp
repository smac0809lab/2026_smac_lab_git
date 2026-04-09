#include "ros2_manager.h"
#include "sensor_handler.h"
#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <std_msgs/msg/int32.h> 
#include <std_msgs/msg/float32.h>
#include <geometry_msgs/msg/twist.h>

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){return false;}}

enum states { WAITING_AGENT, AGENT_AVAILABLE, AGENT_CONNECTED, AGENT_DISCONNECTED } state;

unsigned long last_msg_time = 0; 

// ROS 2 Entities
rcl_subscription_t subscriber;
geometry_msgs__msg__Twist msg;
rcl_publisher_t encoder_publisher, steer_pot_publisher, final_throttle_pub, final_steer_pub, calculateSpeed_pub; 
std_msgs__msg__Int32 encoder_msg, steer_pot_msg, final_throttle_msg, final_steer_msg;
std_msgs__msg__Float32 calculateSpeed_msg; 

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

int ros_throttle = 0;
int ros_steer = 0;
long prev_encoder_for_speed = 0; 

// --- 구독 콜백: Python 명령 수신 및 모터 제어값 매핑 ---
void subscription_callback(const void * msin) {
    const geometry_msgs__msg__Twist * msg_in = (const geometry_msgs__msg__Twist *)msin;
    last_msg_time = millis(); 

    // 1. 속도 (Linear X): Python km/h (예: 5.0) -> 모터 출력 (-250~250)
    // 10km/h일 때 250 출력 기준
    ros_throttle = constrain((int)(msg_in->linear.x * 25.0f), -250, 250);

    // 2. 조향 (Angular Z): Python Degree (예: 22.0) -> 모터 출력 (-250~250)
    // 22도일 때 250 출력 기준
    ros_steer = constrain((int)(msg_in->angular.z * (250.0f / 22.0f)), -250, 250);
}

// --- 엔티티 제거 함수 (컴파일 에러 해결 버전) ---
void destroy_entities() {
    (void) rclc_executor_fini(&executor);
    (void) rcl_subscription_fini(&subscriber, &node);
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

    executor = rclc_executor_get_zero_initialized_executor();
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));
    
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

                    // 반환값 변수 처리를 통해 컴파일러 경고 해결
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

// 인터페이스 함수
unsigned long getROSMsgTime() { return last_msg_time; }
void getROSCmd(int &outT, int &outS) { outT = ros_throttle; outS = ros_steer; }
void setFinalCmd(int t, int s) { ros_throttle = t; ros_steer = s; }
void initROS() { set_microros_serial_transports(Serial); state = WAITING_AGENT; }