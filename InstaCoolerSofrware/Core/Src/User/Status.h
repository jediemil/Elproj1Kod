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

	bool motorInitialized;

private:
  float motorSpeed;
};

#endif /* SRC_USER_STATUS_H_ */
