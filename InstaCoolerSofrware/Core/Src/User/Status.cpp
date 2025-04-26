/*
 * Status.cpp
 *
 *  Created on: Apr 26, 2025
 *      Author: emilr
 */

#include "Status.h"

Status::Status(int i) {
	// TODO Auto-generated constructor stub

}

void Status::setMotorSpeed(uint8_t speed) {
  this->motorSpeed = speed;
}

uint8_t Status::getMotorSpeed() {
    return this->motorSpeed;
}
