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
#include "stm32f1xx_hal.h"

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
#define Set_Mode_Pin GPIO_PIN_0
#define Set_Mode_GPIO_Port GPIOB
#define Set_Mode_EXTI_IRQn EXTI0_IRQn
#define Count_UP_Pin GPIO_PIN_1
#define Count_UP_GPIO_Port GPIOB
#define Count_Down_Pin GPIO_PIN_2
#define Count_Down_GPIO_Port GPIOB
#define STAT_PULL_UP_Pin GPIO_PIN_10
#define STAT_PULL_UP_GPIO_Port GPIOA
#define STAT_Pin GPIO_PIN_11
#define STAT_GPIO_Port GPIOA
#define AIR_PUMP_Pin GPIO_PIN_6
#define AIR_PUMP_GPIO_Port GPIOB
#define Buzzer_Pin GPIO_PIN_7
#define Buzzer_GPIO_Port GPIOB
#define Sensor_Out_Pin GPIO_PIN_8
#define Sensor_Out_GPIO_Port GPIOB
#define Sensor_Clock_Pin GPIO_PIN_9
#define Sensor_Clock_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
