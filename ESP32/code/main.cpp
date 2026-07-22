#include <Arduino.h>
#include <ps5Controller.h>
#include "motor_control.h"
#include "sensor_handler.h"
#include "ros2_manager.h"

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
    bool finalStop = false; // 브레이크 신호 변수 추가

    if (ps5.isConnected()) {
        // 모드 전환 (X 버튼)
        bool currentXState = ps5.Cross();
        if (currentXState && !lastXButtonState) {
            isAutoMode = !isAutoMode; 
            Serial.printf("Mode Switched: %s\n", isAutoMode ? "AUTO" : "MANUAL");
        }
        lastXButtonState = currentXState;

        if (isAutoMode) {
            // [자동 모드]
            unsigned long lastMsg = getROSMsgTime();
            if (millis() - lastMsg > TIMEOUT_MS) {
                finalThrottle = 0;
                finalSteer = 0;
                finalStop = false; // 통신 두절 시 안전을 위해 급제동(Brake)
            } else {
                finalStop = false;
                // 수정된 getROSCmd를 통해 stop 신호까지 받아옴
                getROSCmd(finalThrottle, finalSteer, finalStop);
                
            }
        } else {
            // [수동 모드] PS5 컨트롤러
            int rawT = ps5.LStickY();
            int rawS = ps5.RStickX();

            finalThrottle = map(rawT, -128, 127, -250, 250);
            finalSteer = map(rawS, -128, 127, -250, 250);

            // 속도가 데드존(20) 이내일 때만 급제동(Brake)을 겁니다.
            if (abs(finalThrottle) < 20) {
                finalThrottle = 0; 
                finalStop = false;  // 정지 시 급제동
            } else {
                finalStop = false; // 주행 시 브레이크 해제 (이게 없으면 안 움직입니다!)
            }

            if (abs(finalSteer) < 20) {
                finalSteer = 0;
            }
        }

        // 제어값 적용
        setFinalCmd(finalThrottle, finalSteer);
        driveVehicle(finalThrottle, finalSteer, finalStop); // stop 인자 추가

    } else {
        // 컨트롤러 연결 끊김 시 즉시 급제동
        driveVehicle(0, 0, true); 
        setFinalCmd(0, 0); 
        isAutoMode = false;
    }
    
    delay(10); 
}