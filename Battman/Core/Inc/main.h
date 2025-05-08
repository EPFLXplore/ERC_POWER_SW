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
#include "stm32l5xx_hal.h"

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
#define A1_7_Pin GPIO_PIN_13
#define A1_7_GPIO_Port GPIOC
#define A1_8_Pin GPIO_PIN_0
#define A1_8_GPIO_Port GPIOH
#define A0_8_Pin GPIO_PIN_1
#define A0_8_GPIO_Port GPIOH
#define WDT_Stm_Pin GPIO_PIN_0
#define WDT_Stm_GPIO_Port GPIOC
#define HardwareFault_Pin GPIO_PIN_1
#define HardwareFault_GPIO_Port GPIOC
#define ChargeCurrent_Pin GPIO_PIN_2
#define ChargeCurrent_GPIO_Port GPIOC
#define ChargeCurrRef_Pin GPIO_PIN_3
#define ChargeCurrRef_GPIO_Port GPIOC
#define LTCSelfTestOK_Pin GPIO_PIN_0
#define LTCSelfTestOK_GPIO_Port GPIOA
#define LTCSelfTest_Pin GPIO_PIN_1
#define LTCSelfTest_GPIO_Port GPIOA
#define DischgCurr_Pin GPIO_PIN_2
#define DischgCurr_GPIO_Port GPIOA
#define DischgCurrRef_Pin GPIO_PIN_3
#define DischgCurrRef_GPIO_Port GPIOA
#define SPI1_CS_Pin GPIO_PIN_4
#define SPI1_CS_GPIO_Port GPIOA
#define Enable1_Pin GPIO_PIN_4
#define Enable1_GPIO_Port GPIOC
#define OutputEnable_Pin GPIO_PIN_5
#define OutputEnable_GPIO_Port GPIOC
#define Enable2_Pin GPIO_PIN_0
#define Enable2_GPIO_Port GPIOB
#define ErrorReset_Pin GPIO_PIN_1
#define ErrorReset_GPIO_Port GPIOB
#define LED_A_Pin GPIO_PIN_10
#define LED_A_GPIO_Port GPIOB
#define LED_R_Pin GPIO_PIN_11
#define LED_R_GPIO_Port GPIOB
#define LED_Y_Pin GPIO_PIN_12
#define LED_Y_GPIO_Port GPIOB
#define LED_G_Pin GPIO_PIN_13
#define LED_G_GPIO_Port GPIOB
#define Config1_Pin GPIO_PIN_14
#define Config1_GPIO_Port GPIOB
#define Config2_Pin GPIO_PIN_15
#define Config2_GPIO_Port GPIOB
#define Config3_Pin GPIO_PIN_6
#define Config3_GPIO_Port GPIOC
#define Config4_Pin GPIO_PIN_7
#define Config4_GPIO_Port GPIOC
#define A1_1_Pin GPIO_PIN_8
#define A1_1_GPIO_Port GPIOC
#define A0_1_Pin GPIO_PIN_9
#define A0_1_GPIO_Port GPIOC
#define A1_2_Pin GPIO_PIN_8
#define A1_2_GPIO_Port GPIOA
#define A1_3_Pin GPIO_PIN_9
#define A1_3_GPIO_Port GPIOA
#define A1_4_Pin GPIO_PIN_10
#define A1_4_GPIO_Port GPIOA
#define A1_5_Pin GPIO_PIN_10
#define A1_5_GPIO_Port GPIOC
#define A0_2_Pin GPIO_PIN_11
#define A0_2_GPIO_Port GPIOC
#define A0_3_Pin GPIO_PIN_12
#define A0_3_GPIO_Port GPIOC
#define A0_4_Pin GPIO_PIN_2
#define A0_4_GPIO_Port GPIOD
#define A0_5_Pin GPIO_PIN_4
#define A0_5_GPIO_Port GPIOB
#define A1_6_Pin GPIO_PIN_5
#define A1_6_GPIO_Port GPIOB
#define A0_6_Pin GPIO_PIN_6
#define A0_6_GPIO_Port GPIOB
#define A0_7_Pin GPIO_PIN_7
#define A0_7_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
