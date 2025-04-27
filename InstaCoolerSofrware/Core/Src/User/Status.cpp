/*
 * Status.cpp
 *
 *  Created on: Apr 26, 2025
 *      Author: emilr
 */

#include "Status.h"

Status::Status() {
	motorInitialized = false;
    motorSpeed = 0;

}

void Status::setMotorSpeed(float speed) {
  this->motorSpeed = speed;
}

float Status::getMotorSpeed() {
    return this->motorSpeed;
}
