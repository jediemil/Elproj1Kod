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
