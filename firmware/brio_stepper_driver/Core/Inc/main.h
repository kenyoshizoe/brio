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
#define stepper_reset_Pin GPIO_PIN_0
#define stepper_reset_GPIO_Port GPIOA
#define stepper_dir3_Pin GPIO_PIN_1
#define stepper_dir3_GPIO_Port GPIOA
#define stepper_step3_Pin GPIO_PIN_2
#define stepper_step3_GPIO_Port GPIOA
#define sens_in3_Pin GPIO_PIN_3
#define sens_in3_GPIO_Port GPIOA
#define stepper_dir2_Pin GPIO_PIN_4
#define stepper_dir2_GPIO_Port GPIOA
#define stepper_step2_Pin GPIO_PIN_5
#define stepper_step2_GPIO_Port GPIOA
#define sens_in2_Pin GPIO_PIN_6
#define sens_in2_GPIO_Port GPIOA
#define stepper_dir1_Pin GPIO_PIN_7
#define stepper_dir1_GPIO_Port GPIOA
#define stepper_step1_Pin GPIO_PIN_0
#define stepper_step1_GPIO_Port GPIOB
#define sens_in1_Pin GPIO_PIN_1
#define sens_in1_GPIO_Port GPIOB
#define sleep_Pin GPIO_PIN_8
#define sleep_GPIO_Port GPIOA
#define emg_Pin GPIO_PIN_11
#define emg_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
