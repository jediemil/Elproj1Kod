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
	Status();
	void setMotorSpeed(float speed);
    float getMotorSpeed();
    uint16_t getTimeLeft();

	bool motorInitialized;
    bool programRunning;
    uint8_t programType;
    float programProgress;

    uint16_t programLen;
    unsigned long startTick;


private:
  float motorSpeed;
};

#endif /* SRC_USER_STATUS_H_ */
