/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32u5xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "definitions.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define ENCODER_A_Pin GPIO_PIN_13
#define ENCODER_A_GPIO_Port GPIOC
#define ENCODER_A_EXTI_IRQn EXTI13_IRQn
#define ENCODER_B_Pin GPIO_PIN_14
#define ENCODER_B_GPIO_Port GPIOC
#define ENCODER_B_EXTI_IRQn EXTI14_IRQn
#define ENCODER_SWITCH_Pin GPIO_PIN_15
#define ENCODER_SWITCH_GPIO_Port GPIOC
#define ENCODER_SWITCH_EXTI_IRQn EXTI15_IRQn
#define PD_SCL_Pin GPIO_PIN_13
#define PD_SCL_GPIO_Port GPIOB
#define PD_SDA_Pin GPIO_PIN_14
#define PD_SDA_GPIO_Port GPIOB
#define BLUE_Pin GPIO_PIN_8
#define BLUE_GPIO_Port GPIOA
#define RED_Pin GPIO_PIN_9
#define RED_GPIO_Port GPIOA
#define MOTOR_Pin GPIO_PIN_15
#define MOTOR_GPIO_Port GPIOA
#define BUZZER_Pin GPIO_PIN_4
#define BUZZER_GPIO_Port GPIOB
#define OLED_SCL_Pin GPIO_PIN_6
#define OLED_SCL_GPIO_Port GPIOB
#define OLED_SDA_Pin GPIO_PIN_7
#define OLED_SDA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
