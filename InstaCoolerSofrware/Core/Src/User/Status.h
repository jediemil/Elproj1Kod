/*
 * Status.h
 *
 *  Created on: Apr 26, 2025
 *      Author: emilr
 */

#ifndef SRC_USER_STATUS_H_
#define SRC_USER_STATUS_H_

#include <stdint.h>

class Status {
public:
	Status(int i);
	void setMotorSpeed(uint8_t speed);
    uint8_t getMotorSpeed();

private:
  uint8_t motorSpeed;
};

#endif /* SRC_USER_STATUS_H_ */
