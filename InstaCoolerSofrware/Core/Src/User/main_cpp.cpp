/*
 * main_cpp.cpp
 *
 *  Created on: Apr 26, 2025
 *      Author: emilr
 */

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
volatile uint8_t encoder_counter = 0; //TODO: Varför volatile?
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
        .stack_size = 512,
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
    __HAL_TIM_SET_COMPARE(timer_handle, timer_channel, new_duty); // Set compare value to new duty
    HAL_TIM_PWM_Start(timer_handle, timer_channel); // Start PWM
}

void setBuzzerFrequency(uint32_t frequency) {
    //TODO: SKRIV OM KOD SÅ DNE ÄR RIMLIG OCH VÅR EGEN
    TIM_HandleTypeDef *timer = &BUZZER_TIMER_HANDLE; // Using the timer instance htim3
    uint32_t channel = BUZZER_TIMER_CHANNEL; // Using channel 1
    if (frequency) {
        uint32_t timer_clk_freq = HAL_RCC_GetSysClockFreq(); // Get the timer base clock frequency (after APB scaling)
        uint32_t prescaler = timer->Init.Prescaler; // Get the prescaler value from the timer setup
        uint32_t tick_freq = timer_clk_freq / (prescaler + 1); // Calculate the actual tick frequency

        uint32_t counter_period = (tick_freq / frequency) - 1;
        // Calculate the counter period (ARR) based on the desired frequency (ARR = number of ticks per period)
        __HAL_TIM_SET_AUTORELOAD(timer, counter_period); // Set the ARR value (the counter period)

        setPWM(timer, channel,0.5);
    } else {
        setPWM(timer, channel,0);
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
	uint16_t* ptr = (uint16_t*) argument;
	while (*ptr != 0) {
		setBuzzerFrequency((*ptr)>>4);
		osDelay(((*ptr)&0b1111) * 100);
		ptr++;
	}
	setBuzzerFrequency(0);
}

void startUserInterfaceTask(void *argument) {
    for (;;) {
        userInterface.drawScreen();
        osDelay(200);
    }
}

void rampDownMotor(int from) {
    for (int i = 100; i >= 0; i--) {
        setMotorSpeed(i/400.0);
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
            status.programProgress = 0;
            status.startTick = getTimeTicks();
			status.motorRunning = true;
            for (int i = 0; i < 100; i++) {
                setMotorSpeed(i / 400.0);
                osDelay(50);
                if (!status.programRunning) break;
            }

            for (uint32_t i = 0; i < std::max((int) status.programLen - 10, 0); i++) {
                if (!status.programRunning) break;
                osDelay(1000);
            }

            rampDownMotor(100);
			status.motorRunning = false;
        }

        if (status.programRunning) {
            status.programRunning = false;
            userInterface.continueEvent();
        } else {
            userInterface.continueEvent();
        }
        setBuzzerFrequency(1000);
        osDelay(1000);
        setBuzzerFrequency(0);
    }
}

void terminateMotorTask() {
    osThreadSuspend(motorTaskHandle);
	setMotorSpeed(0);
	status.motorRunning = false;
    status.lidOpen = true;
	status.programLen = 0;
	status.programRunning = false;
    userInterface.changeLidStatus();
}

void startMotorTask() {
    status.lidOpen = false;
	osThreadResume(motorTaskHandle);
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
    //setBuzzerFrequency(1000);
    if (HAL_GPIO_ReadPin(GPIOB, LID_SENSOR_Pin) == GPIO_PIN_SET) {
        terminateMotorTask();
    } else {
        userInterface.continueEvent();
    }

    osDelay(1000);
    //setBuzzerFrequency(0);

    printf("Startad\n");
    while (true) {
        osDelay(100);

    }
}
