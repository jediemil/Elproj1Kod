/*
 * Status.cpp
 *
 *  Created on: Apr 26, 2025
 *      Author: emilr
 */

#include "Status.h"
#include "main_cpp.h"

Status::Status() {
	motorInitialized = false;
    motorSpeed = 0;
    programRunning = false;
    programType = 0;
    programProgress = 0;
    programLen = 120;
}

void Status::setMotorSpeed(float speed) {
  this->motorSpeed = speed;
}

float Status::getMotorSpeed() {
    return this->motorSpeed;
}

uint16_t Status::getTimeLeft() {
    return (getTimeTicks() - startTick) / 100;
}

float Status::getWaterTemp() {
	return convertADCToTemp(getADCValue(0));
}

float Status::convertADCToTemp(uint16_t adcValue) {
	if (adcValue == 0 || adcValue == 16383) return -273.15;

	float R = (float)(22000*adcValue)/(float)(16383-adcValue);
	float temperature = (298.15 * 3380.0)/(298.15 * log(R / 10000.0) + 3380.0) - 273.15;
	return temperature;
}

float Status::getAirTemp() {
	return convertADCToTemp(getADCValue(1));
}


float Status::getADCValue(uint16_t adcChannel) {
	//return adcBuf[adcChannel]/16383.0 * 3.3;
	return adcBuf[adcChannel];

}
