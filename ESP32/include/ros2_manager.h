#ifndef ROS2_MANAGER_H
#define ROS2_MANAGER_H

#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>

// 외부(main.cpp)에서 사용할 함수들
void initROS();
void updateROS();
void getROSCmd(int &outThrottle, int &outSteer, bool &outStop);
void setFinalCmd(int throttle, int steer); // [추가]
unsigned long getROSMsgTime(); // 추가됨
#endif