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
#include "stm32f4xx_hal.h"

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
#define button_1_Pin GPIO_PIN_2
#define button_1_GPIO_Port GPIOE
#define button_5_Pin GPIO_PIN_3
#define button_5_GPIO_Port GPIOE
#define button_2_Pin GPIO_PIN_4
#define button_2_GPIO_Port GPIOE
#define button_3_Pin GPIO_PIN_5
#define button_3_GPIO_Port GPIOE
#define button_4_Pin GPIO_PIN_6
#define button_4_GPIO_Port GPIOE
#define button_6_Pin GPIO_PIN_0
#define button_6_GPIO_Port GPIOF
#define button_7_Pin GPIO_PIN_1
#define button_7_GPIO_Port GPIOF
#define button_8_Pin GPIO_PIN_2
#define button_8_GPIO_Port GPIOF
#define button_9_Pin GPIO_PIN_7
#define button_9_GPIO_Port GPIOA
#define button_10_Pin GPIO_PIN_0
#define button_10_GPIO_Port GPIOG
#define blue_led_Pin GPIO_PIN_7
#define blue_led_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
