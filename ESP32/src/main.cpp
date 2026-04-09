#include <Arduino.h>
#include <ps5Controller.h>
#include "motor_control.h"
#include "sensor_handler.h"
#include "ros2_manager.h"

bool isAutoMode = false;
bool lastXButtonState = false;
const unsigned long TIMEOUT_MS = 500; // 0.5초간 명령 없으면 정지

void setup() {
    Serial.begin(115200);
    initROS(); // 내부에서 set_microros_serial_transports 실행
    
    delay(2000); 
    
    initMotors();   
    initSensors();

    // PS5 주소 설정 (본인 컨트롤러 주소 확인)
    ps5.begin("D4:2F:4B:00:61:10"); 
    Serial.println("System Ready.");
}

void loop() {
    updateROS(); 

    int finalThrottle = 0;
    int finalSteer = 0;

    if (ps5.isConnected()) {
        // 모드 전환 (X 버튼)
        bool currentXState = ps5.Cross();
        if (currentXState && !lastXButtonState) {
            isAutoMode = !isAutoMode; 
            Serial.printf("Mode Switched: %s\n", isAutoMode ? "AUTO" : "MANUAL");
        }
        lastXButtonState = currentXState;

        if (isAutoMode) {
            // [안전 장치] ROS 메시지 수신 시간 확인
            unsigned long lastMsg = getROSMsgTime();
            if (millis() - lastMsg > TIMEOUT_MS) {
                finalThrottle = 0;
                finalSteer = 0; // 통신 끊기면 즉시 정지
            } else {
                getROSCmd(finalThrottle, finalSteer);
            }
        } else {
            // 수동 모드 (PS5 컨트롤러)
            int rawT = ps5.LStickY();
            int rawS = ps5.RStickX();
            finalThrottle = map(rawT, -128, 127, -250, 250);
            finalSteer = map(rawS, -128, 127, -250, 250);

            if (abs(finalThrottle) < 20) finalThrottle = 0;
            if (abs(finalSteer) < 20) finalSteer = 0;
        }

        // 현재 제어값을 ROS로 피드백하고 모터 구동
        setFinalCmd(finalThrottle, finalSteer);
        driveVehicle(finalThrottle, finalSteer);

    } else {
        // 컨트롤러 연결 끊김 시 안전 정지
        driveVehicle(0, 0);
        setFinalCmd(0, 0); 
        isAutoMode = false;
    }
    
    delay(10); 
}