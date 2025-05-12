/*
 * main_cpp.cpp
 *
 *  Created on: Apr 26, 2025
 *      Author: emilr
 */

// SSD1306 library: https://github.com/afiskon/stm32-ssd1306
// STUSB4500 library:  https://github.com/jefflongo/stusb4500

#include "main_cpp.h"

#include <stdlib.h>
#include <stdio.h>
#include <algorithm>

#include "stusb4500/stusb4500.h"

#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "app_freertos.h"
#include "adc.h"

#include "Status.h"
#include "include.h"
#include "UserInterface.h"
#include "bmp.h"

static Status status;
static UserInterface userInterface(&status);
uint64_t timeTicks = 0; // 0.01 s

osThreadId_t motorTaskHandle;
const osThreadAttr_t motorTaskAttributes = {
        .name = "motorTaskTask",
        .attr_bits=NULL,
        .cb_mem=NULL,
        .cb_size=NULL,
        .stack_mem=NULL,
        .stack_size = 128*4,
        .priority = (osPriority_t) osPriorityNormal,
        .tz_module=NULL,
        .reserved=NULL,
};

osThreadId_t userInterfaceTaskHandle;
const osThreadAttr_t userInterfaceTaskAttributes = {
        .name = "userInterfaceTask",
        .attr_bits=NULL,
        .cb_mem=NULL,
        .cb_size=NULL,
        .stack_mem=NULL,
        .stack_size = 128*16,
        .priority = (osPriority_t) osPriorityNormal,
        .tz_module=NULL,
        .reserved=NULL,
};

osThreadId_t buzzerMusicTaskHandle;
const osThreadAttr_t buzzerMusicTaskAttributes = {
        .name = "buzzerMusicTask",
        .attr_bits=NULL,
        .cb_mem=NULL,
        .cb_size=NULL,
        .stack_mem=NULL,
        .stack_size = 1024*1,
        .priority = (osPriority_t) osPriorityNormal,
        .tz_module=NULL,
        .reserved=NULL,
};

void addTick() {
    timeTicks++;
}

uint64_t getTimeTicks() { // 0.01 s
    return timeTicks;
}

void buttonPressIt() {
    userInterface.click();
}

void changeEncoderIt(int change) {
    //encoder_counter += change*10;
    //setRGB(encoder_counter, 0, encoder_counter);
    if (change > 0) {
        userInterface.rightSwipe();
    } else {
        userInterface.leftSwipe();
    }
}

void setPWM(TIM_HandleTypeDef *timer_handle, uint32_t timer_channel, float duty) {
    uint32_t counter_period = __HAL_TIM_GET_AUTORELOAD(timer_handle); // Get the ARR value (number of ticks per period)
    uint32_t new_duty = duty * counter_period; // Calculate new duty value
    __HAL_TIM_DISABLE(timer_handle);
    __HAL_TIM_SET_COMPARE(timer_handle, timer_channel, new_duty); // Set compare value to new duty
    __HAL_TIM_ENABLE(timer_handle);
    HAL_TIM_PWM_Start(timer_handle, timer_channel); // Start PWM
}

void setBuzzerFrequency(uint32_t frequency) {
	TIM_HandleTypeDef *timer = &BUZZER_TIMER_HANDLE;
	uint32_t channel = BUZZER_TIMER_CHANNEL;

	if (frequency) {
	    uint32_t timer_clk_freq = HAL_RCC_GetSysClockFreq();
	    uint32_t prescaler = timer->Instance->PSC + 1; // safer
	    uint32_t tick_freq = timer_clk_freq / prescaler;

	    if (frequency > 0 && frequency < 20000) {
	        uint32_t counter_period = (tick_freq / frequency) - 1;
	        __disable_irq();
	        __HAL_TIM_DISABLE(timer);
	        __HAL_TIM_SET_AUTORELOAD(timer, counter_period);
	        __HAL_TIM_SET_COMPARE(timer, channel, counter_period / 2);
	        __HAL_TIM_SET_COUNTER(timer, 0);
	        __HAL_TIM_ENABLE(timer);

	        HAL_TIM_PWM_Start(timer, channel);
	        __enable_irq();
	    }
	} else {
	    __HAL_TIM_SET_COMPARE(timer, channel, 0);
	    HAL_TIM_PWM_Stop(timer, channel);
	}
}

void setRGB(uint8_t r, uint8_t g, uint8_t b) {
    TIM_HandleTypeDef *htim = &LED_TIMER_HANDLE;
    setPWM(htim, LED_R_CHANNEL, r / 255.0);
    setPWM(htim, LED_G_CHANNEL, g / 255.0);
    setPWM(htim, LED_B_CHANNEL, b / 255.0);
}

void setLED(uint8_t value, uint32_t timer_channel) {
    setPWM(&LED_TIMER_HANDLE, timer_channel, value);
}

void initMotor() {
    TIM_HandleTypeDef *htim = &MOTOR_TIMER_HANDLE;
    setPWM(htim, MOTOR_TIMER_CHANNEL, 0.05); // Initial 1% duty cycle
    osDelay(6000); // Let motor initialize
    status.motorInitialized = true;
}

void setMotorSpeed(float throttle) {
    if (!status.motorInitialized) return;
	if (!status.motorRunning) throttle = 0;
	if (status.lidOpen) throttle = 0;

    TIM_HandleTypeDef *htim = &MOTOR_TIMER_HANDLE;
    float duty = throttle*5 / 100.0f + 0.05f;
    setPWM(htim, MOTOR_TIMER_CHANNEL, duty);
    status.setMotorSpeed(throttle);
}

bool write_i2c(uint16_t addr, uint8_t reg, void const *buf, size_t len, void *context) {
    return HAL_I2C_Mem_Write(&hi2c2, addr, reg, I2C_MEMADD_SIZE_8BIT, (uint8_t *) buf, len, 1000) == HAL_OK;
}

bool read_i2c(uint16_t addr, uint8_t reg, void *buf, size_t len, void *context) {
    return HAL_I2C_Mem_Read(&hi2c2, addr, reg, I2C_MEMADD_SIZE_8BIT, (uint8_t *) buf, len, 1000) == HAL_OK;
}

bool write_nvm() {
    stusb4500_t device;
    device.addr = 0x28;
    device.write = &write_i2c;
    device.read = &read_i2c;
    device.context = (void *) 1000;
    stusb4500_gpio_cfg_t gpio_cfg = STUSB4500_GPIO_CFG_SINK_POWER;
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
    printf("Begin\n");
    bool success = stusb4500_nvm_flash(&device, &config);
    return success;
}

void playBuzzerMusic(void* argument) {
	uint16_t* ptr = (uint16_t*) argument; // When playBuzzerMusic is called, a pointer to a music dara array is passed.
	while (*ptr != 0) { // The music data array is ended with a zero byte -> if *ptr is zero: the music has finished
		uint16_t value = *ptr;
		uint16_t freq = value >> 4; // The frequency is encoded in the 12 most significant bits
		uint16_t delay = ((*ptr)&0b1111) * 50; // The note play time is encoded in the bottom 4 bits

		setBuzzerFrequency(freq);
		osDelay(delay);
		ptr++; // Go to next element of the music data array
	}
	setBuzzerFrequency(0);
	osThreadExit(); // End the current thread
}

void startUserInterfaceTask(void *argument) {
    for (;;) {
        userInterface.drawScreen();
        osDelay(200);
    }
}

void rampDownMotor(float from) {
    for (int i = (int)(from*1000.0); i >= 50; i-=1) {
    	if (!status.motorRunning) break;
        setMotorSpeed(i / 1000.0);
        osDelay(50);
    }
}

void startMotorTask(void *argument) {
    for (;;) {
		setMotorSpeed(0);
        while (!status.programRunning) {
            osDelay(10);
        }
        if (status.programType == PROGRAM_TYPE_AUTO || status.programType == PROGRAM_TYPE_CUSTOM) {
        	float motorSpeed = status.selectedMotorSpeed;
        	if (status.programType == PROGRAM_TYPE_AUTO) {
        		motorSpeed = 0.1;
        	}
            status.programProgress = 0;
            status.startTick = getTimeTicks();
			status.motorRunning = true;
            for (int i = 500; i < (int)(motorSpeed*10000.0); i++) {
                setMotorSpeed(i / 10000.0);
                osDelay(15);
                if (!status.programRunning) break;
            }

            for (uint32_t i = 0; i < std::max((int) status.programLen - 15, 0); i++) {
                if (!status.programRunning) break;
                osDelay(1000);
            }

            rampDownMotor(status.getMotorSpeed());
            setMotorSpeed(0);
			status.motorRunning = false;
        }

        if (!status.lidOpen) {
        	userInterface.setupAutostart();
        }
        status.programRunning = false;
        setBuzzerFrequency(1000);
        osDelay(1000);
        setBuzzerFrequency(0);
    }
}

void terminateMotorTask() {
	status.motorRunning = false;
	setMotorSpeed(0);
    status.lidOpen = true;
	status.programLen = 0;
	status.programRunning = false;
	status.programType = 0;
    userInterface.changeLidStatus();
}

void restartMotorTask() {
    status.lidOpen = false;
    userInterface.changeLidStatus();
}

int main_cpp() {
    printf("Booted\n");
    setRGB(20, 20, 0);
    printf("Startar UI\n");
    userInterface.begin();
    userInterface.setupWelcome();

    //HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
    HAL_ADC_Start_DMA(&hadc1, status.adcBuf, 4);

    printf("Startar tasks\n");
    //osThreadDef(UserInterfaceTask, startUserInterfaceTask, osPriorityNormal, 0, 128);
    //userInterfaceTaskHandle = osThreadCreate(osThread(UserInterfaceTask), NULL);
    userInterfaceTaskHandle = osThreadNew(startUserInterfaceTask, NULL, &userInterfaceTaskAttributes);
    //osThreadDef(MotorTask, startMotorTask, osPriorityNormal, 0, 128);
    //motorTaskHandle = osThreadCreate(osThread(MotorTask), NULL);
    motorTaskHandle = osThreadNew(startMotorTask, NULL, &motorTaskAttributes);

    printf("Startar motor\n");
    initMotor();

    setRGB(0, 25, 0);
    //osDelay(1000);
    buzzerMusicTaskHandle = osThreadNew(playBuzzerMusic, (void*) windowsStartupStream, &buzzerMusicTaskAttributes);
    //playBuzzerMusic((void*) windowsStartupStream);
    //setBuzzerFrequency(1000);
    if (HAL_GPIO_ReadPin(GPIOB, LID_SENSOR_Pin) == GPIO_PIN_SET) {
    	terminateMotorTask();
    	while (HAL_GPIO_ReadPin(GPIOB, LID_SENSOR_Pin) == GPIO_PIN_SET) {
    		osDelay(100);
    	}
    }

    userInterface.setupAutostart();
    osDelay(1000);
    //setBuzzerFrequency(0);

    printf("Startad\n");
    while (true) {
        osDelay(100);
        /*if (HAL_GPIO_ReadPin(GPIOB, LID_SENSOR_Pin) == GPIO_PIN_SET && status.lidOpen == false) {
            terminateMotorTask();
        } else if (HAL_GPIO_ReadPin(GPIOB, LID_SENSOR_Pin) != GPIO_PIN_SET && status.lidOpen == true) {
            restartMotorTask();
        }*/
    }
}
