/*
 * main_cpp.cpp
 *
 *  Created on: Apr 26, 2025
 *      Author: emilr
 */

#include "main_cpp.h"

#include <stdlib.h>
#include <stdbool.h>
#include <stdio.h>

#include "stusb4500/stusb4500.h"

#include "tim.h"
#include "i2c.h"

#include "Status.h"
#include "include.h"
#include "UserInterface.h"

bool motor_initialized = false;
static Status status(1);
static UserInterface userInterface(&status);

void setPWM(TIM_HandleTypeDef *timer_handle, uint32_t timer_channel, uint8_t duty) {
    uint32_t counter_period = __HAL_TIM_GET_AUTORELOAD(timer_handle); // Get the ARR value (number of ticks per period)
    uint32_t new_duty = duty / 255.0f * counter_period; // Calculate new duty value
    __HAL_TIM_SET_COMPARE(timer_handle, timer_channel, new_duty); // Set compare value to new duty
    HAL_TIM_PWM_Start(timer_handle, timer_channel); // Start PWM
}

void setBuzzerFrequency(uint32_t frequency) { //TODO: SKRIV OM KOD SÅ DNE ÄR RIMLIG OCH VÅR EGEN
    TIM_HandleTypeDef *timer = &htim3;           			// Using the timer instance htim3
    uint32_t channel = TIM_CHANNEL_1;            			// Using channel 1

    uint32_t timer_clk_freq = HAL_RCC_GetSysClockFreq(); 	// Get the timer base clock frequency (after APB scaling)
    uint32_t prescaler = timer->Init.Prescaler;   			// Get the prescaler value from the timer setup
    uint32_t tick_freq = timer_clk_freq / (prescaler + 1); 	// Calculate the actual tick frequency

    uint32_t counter_period = (tick_freq / frequency) - 1;	// Calculate the counter period (ARR) based on the desired frequency (ARR = number of ticks per period)
    __HAL_TIM_SET_AUTORELOAD(timer, counter_period);		// Set the ARR value (the counter period)

    setPWM(timer, channel, 128);
}

void setRGB(uint8_t r, uint8_t g, uint8_t b) {
    TIM_HandleTypeDef *htim = &LED_TIMER_HANDLE;
    setPWM(htim, LED_R_CHANNEL, r);
    setPWM(htim, LED_G_CHANNEL, g);
    setPWM(htim, LED_B_CHANNEL, b);
}

void setLED(uint8_t value, uint32_t timer_channel) {
    setPWM(&LED_TIMER_HANDLE, timer_channel, value);
}

void initMotor() {
    TIM_HandleTypeDef *htim = &MOTOR_TIMER_HANDLE;
    setPWM(htim, MOTOR_TIMER_CHANNEL, 3); // Initial 1% duty cycle
    HAL_Delay(5000); // Let motor initialize
    motor_initialized = true;
}

void setMotorSpeed(float throttle) {
	if (!motor_initialized) return;

	TIM_HandleTypeDef *htim = &MOTOR_TIMER_HANDLE;
	uint8_t duty = (throttle / 100.0f + 0.01f) * 255;
	setPWM(htim, MOTOR_TIMER_CHANNEL, duty);
}

bool write_i2c(uint16_t addr, uint8_t reg, void const* buf, size_t len, void* context) {
	HAL_Delay(10);
	printf("Write\n");
	//__disable_irq();
	uint8_t status = HAL_I2C_Mem_Write(&hi2c2, addr, reg, I2C_MEMADD_SIZE_8BIT, (uint8_t*)buf, len, 1000);
	//__enable_irq();
	  if (status == HAL_OK) {
		  uint8_t color = rand() % 255;
		  setRGB(0, color, color);
		  return true;
	  } else if (status == HAL_ERROR) {
		  setRGB(255, 0, 0);
	  } else if (status == HAL_TIMEOUT) {
		  setRGB(0, 0, 255);
	  }
	  return false;
}

bool read_i2c(uint16_t addr, uint8_t reg, void* buf, size_t len, void* context) {
	HAL_Delay(10);
	printf("Read\n");
	//__disable_irq();
	uint8_t status = HAL_I2C_Mem_Read(&hi2c2, addr, reg, I2C_MEMADD_SIZE_8BIT, (uint8_t*)buf, len, 1000);
	//__enable_irq();
	if (status == HAL_OK) {
		uint8_t color = rand() % 255;
		setRGB(color, 0, color);
		return true;
	} else if (status == HAL_ERROR) {
		setRGB(255, 0, 20);
	} else if (status == HAL_TIMEOUT) {
		setRGB(0,0,20);
	}
	return false;
}

int main_cpp() {
	stusb4500_t device;
	device.addr = 0x28;
	device.write = &write_i2c;
	device.read = &read_i2c;
	device.context = (void*)1000;
	stusb4500_gpio_cfg_t gpio_cfg;
	gpio_cfg = STUSB4500_GPIO_CFG_SINK_POWER;
	  stusb4500_nvm_config_t config;
	  config.pdo1_current_ma = 500;
	  config.pdo2_voltage_mv = 1500;
	  config.pdo2_current_ma = 3000;
	  config.pdo3_voltage_mv = 12000;
	  config.pdo3_current_ma = 3000;
	  config.num_valid_pdos = 3;
	  config.pdo_current_fallback = 0;
	  config.use_src_current = false;
	  config.only_above_5v = false;
	  config.gpio_cfg = gpio_cfg;
	  setRGB(0, 0, 255);
	  HAL_Delay(1000);
	  setRGB(0, 0, 0);
	  HAL_Delay(1000);
	  setRGB(0, 0, 255);

	  //bool success = stusb4500_nvm_flash(&device, &config);
	  printf("Begin\n");
	  //bool success = stusb4500_set_gpio_state(&device, true);
	  //uint8_t nvm_buf = 10;
	  //bool success = stusb4500_nvm_read(&device, &nvm_buf);
	  //uint8_t buf;
	  /*uint8_t status = HAL_I2C_Mem_Read(&hi2c2, 0x28, 0x2FUL, I2C_MEMADD_SIZE_8BIT, &buf, 1, 1000);
	  printf("%lu", HAL_I2C_GetError(&hi2c2));
	  	//__enable_irq();
	  	if (status == HAL_OK) {
	  		uint8_t color = rand() % 255;
	  		setRGB(color, 0, color);
	  		return true;
	  	} else if (status == HAL_ERROR) {
	  		setRGB(255, 0, 20);
	  	} else if (status == HAL_TIMEOUT) {
	  		setRGB(0, 0, 20);
	  	}*/
	  /*if (success){
		  printf("True\n");
		  setRGB(0, 255, 0);
	  } else {
		  printf("False\n");
		  //setRGB(0, 0, 0);
	  }*/
	  //printf("%i", nvm_buf);


	  /*if (success) {
		 setRGB(0, 255, 0);
	  } else {
		  setRGB(255, 0, 0);
	  }*/
	  while (1) {

	  }
}
