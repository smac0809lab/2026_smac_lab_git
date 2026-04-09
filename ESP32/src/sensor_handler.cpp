#include "sensor_handler.h"

// 틱당 이동 거리 (mm) - 기존 유지
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
    for(int i=0; i<4; i++) cnt = (cnt << 8) | SPI.transfer(0x00);
    digitalWrite(ENC_CS_PIN, HIGH);
    return (long)cnt;
}

/**
 * 속도 계산 함수 (km/h 단위 반환)
 */
float calculateSpeed(long current_enc, long &prev_enc) {
    unsigned long current_time = millis();
    static unsigned long prev_time = 0;

    // 시간 변화량 (초 단위)
    float dt = (current_time - prev_time) / 1000.0f;
    
    // 처음 실행되거나 dt가 너무 작을 경우 방지
    if (dt <= 0.001f) return 0;

    // 엔코더 펄스 변화량
    long delta_pulse = current_enc - prev_enc;

    // 1. mm/s 계산: (변화량 / 한바퀴펄스) * (바퀴둘레) / 시간
    float speed_mm_s = (float(delta_pulse) / Vehicle::TICKS_PER_REV) * (Vehicle::WHEEL_DIAMETER_MM * PI) / dt; 

    // 2. km/h 변환: (mm/s * 3600초 / 1,000,000mm) => mm/s * 0.0036
    float speed_kmh = speed_mm_s * 0.0036f;

    // 변수 업데이트
    prev_enc = current_enc;
    prev_time = current_time;

    return speed_kmh;
}

int getSteerPot() {
    return analogRead(STR_POT_PIN);
}