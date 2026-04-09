#include "motor_control.h"

void initMotors() {
    // 1. 사용할 모터 배열 정의
    MotorPin motors[3] = {FRONT_MOTOR, REAR_MOTOR, STEER_MOTOR};
    
    for (int i = 0; i < 3; i++) {
        // 방향 제어 핀 설정
        pinMode(motors[i].ena, OUTPUT);
        pinMode(motors[i].enb, OUTPUT);
        
        // PWM 채널 설정 (i가 곧 채널 번호가 됨: 0, 1, 2)
        ledcSetup(i, 5000, 8); // 빈도 5kHz, 해상도 8비트(0-255)
        ledcAttachPin(motors[i].pwm, i); // 각 모터의 pwm 핀을 채널 i에 연결
        
        // 초기 상태 정지
        digitalWrite(motors[i].ena, LOW);
        digitalWrite(motors[i].enb, LOW);
        ledcWrite(i, 0);
    }
}

void moveMotor(MotorPin motor, int channel, int speed) {
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

void driveVehicle(int drive_pwm, int steer_pwm) {
    // drive_pwm 신호를 앞/뒤 모터에 각각 전송 (채널 0, 1)
    moveMotor(FRONT_MOTOR, 0, drive_pwm);
    moveMotor(REAR_MOTOR, 1, drive_pwm);
    
    // steer_pwm 신호를 조향 모터에 전송 (채널 2)
    moveMotor(STEER_MOTOR, 2, steer_pwm);
}