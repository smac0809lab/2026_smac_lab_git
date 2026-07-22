#include "motor_control.h"

void initMotors() {
    MotorPin motors[3] = {FRONT_MOTOR, REAR_MOTOR, STEER_MOTOR};
    
    for (int i = 0; i < 3; i++) {
        pinMode(motors[i].ena, OUTPUT);
        pinMode(motors[i].enb, OUTPUT);
        
        ledcSetup(i, 5000, 8);
        ledcAttachPin(motors[i].pwm, i);
        
        // 초기 상태: 무부하 정지
        digitalWrite(motors[i].ena, LOW);
        digitalWrite(motors[i].enb, LOW);
        ledcWrite(i, 0);
    }
}

void moveMotor(MotorPin motor, int channel, int speed, bool stop) {
    // 1. stop이 true일 때만 강제 브레이크 (HIGH, HIGH)
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
        // speed가 0이고 stop이 false라면 기존처럼 무부하 정지 (LOW, LOW)
        digitalWrite(motor.ena, LOW);  
        digitalWrite(motor.enb, LOW);
        s = 0;
    }
    ledcWrite(channel, s);
}

void driveVehicle(int drive_pwm, int steer_pwm, bool stop) {
    // drive_pwm 신호를 앞/뒤 모터에 전송
    moveMotor(FRONT_MOTOR, 0, drive_pwm, stop);
    moveMotor(REAR_MOTOR, 1, drive_pwm, stop);
    moveMotor(STEER_MOTOR, 2, steer_pwm, stop);
    
    // steer_pwm 신호를 조향 모터에 전송
    moveMotor(STEER_MOTOR, 2, steer_pwm, stop);
}