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
    uint32_t adcBuf[4];

	bool motorInitialized;
    bool programRunning;
    uint8_t programType;
    float programProgress;
	bool lidOpen;
	bool motorRunning;

	float selectedMotorSpeed; // 0-1
	float targetTemp; // degree C
	uint16_t drinkSize; // mL
    uint16_t programLen; //seconds

    uint64_t startTick;

    float getWaterTemp();
    float getAirTemp();
    float convertADCToTemp(uint16_t adcValue);
    float getADCValue(uint16_t adcChannel);


private:
  float motorSpeed;


};

#endif /* SRC_USER_STATUS_H_ */
