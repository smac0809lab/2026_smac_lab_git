#ifndef SENSOR_HANDLER_H
#define SENSOR_HANDLER_H

#include <SPI.h>
#include "vehicle_config.h"

void initSensors();
long readEncoder();
float calculateSpeed(long current_enc, long &prev_enc);
int getSteerPot();

#endif