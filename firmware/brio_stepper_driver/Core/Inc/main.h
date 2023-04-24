/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "stm32f3xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define STEPPER_RESET_Pin GPIO_PIN_0
#define STEPPER_RESET_GPIO_Port GPIOA
#define STEPPER3_DIR_Pin GPIO_PIN_1
#define STEPPER3_DIR_GPIO_Port GPIOA
#define STEPPER3_STEP_Pin GPIO_PIN_2
#define STEPPER3_STEP_GPIO_Port GPIOA
#define STEPPER3_SENS_Pin GPIO_PIN_3
#define STEPPER3_SENS_GPIO_Port GPIOA
#define STEPPER2_DIR_Pin GPIO_PIN_4
#define STEPPER2_DIR_GPIO_Port GPIOA
#define STEPPER2_STEP_Pin GPIO_PIN_5
#define STEPPER2_STEP_GPIO_Port GPIOA
#define STEPPER2_SENS_Pin GPIO_PIN_6
#define STEPPER2_SENS_GPIO_Port GPIOA
#define STEPPER1_DIR_Pin GPIO_PIN_7
#define STEPPER1_DIR_GPIO_Port GPIOA
#define STEPPER1_STEP_Pin GPIO_PIN_0
#define STEPPER1_STEP_GPIO_Port GPIOB
#define STEPPER1_SENS_Pin GPIO_PIN_1
#define STEPPER1_SENS_GPIO_Port GPIOB
#define STEPPER_SLEEP_Pin GPIO_PIN_8
#define STEPPER_SLEEP_GPIO_Port GPIOA
#define STEPPER_EMG_Pin GPIO_PIN_11
#define STEPPER_EMG_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
