/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
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

/* Includes ------------------------------------------------------------------*/
#include "gpio.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins
     PC14-OSC32_IN (PC14)   ------> RCC_OSC32_IN
     PC15-OSC32_OUT (PC15)   ------> RCC_OSC32_OUT
     PA13 (JTMS/SWDIO)   ------> DEBUG_JTMS-SWDIO
     PA14 (JTCK/SWCLK)   ------> DEBUG_JTCK-SWCLK
     PA15 (JTDI)   ------> DEBUG_JTDI
     PB3 (JTDO/TRACESWO)   ------> DEBUG_JTDO-SWO
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, A1_7_Pin|WDT_Stm_Pin|Enable1_Pin|A1_1_Pin
                          |A0_1_Pin|A1_5_Pin|A0_2_Pin|A0_3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(A1_8_GPIO_Port, A1_8_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, Enable2_Pin|ErrorReset_Pin|LED_A_Pin|LED_R_Pin
                          |LED_Y_Pin|LED_G_Pin|A0_5_Pin|A1_6_Pin
                          |A0_6_Pin|A0_7_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, A1_2_Pin|A1_3_Pin|A1_4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(A0_4_GPIO_Port, A0_4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : A1_7_Pin WDT_Stm_Pin Enable1_Pin A1_1_Pin
                           A0_1_Pin A1_5_Pin A0_2_Pin A0_3_Pin */
  GPIO_InitStruct.Pin = A1_7_Pin|WDT_Stm_Pin|Enable1_Pin|A1_1_Pin
                          |A0_1_Pin|A1_5_Pin|A0_2_Pin|A0_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : A1_8_Pin */
  GPIO_InitStruct.Pin = A1_8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(A1_8_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : A0_8_Pin */
  GPIO_InitStruct.Pin = A0_8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(A0_8_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : HardwareFault_Pin OutputEnable_Pin */
  GPIO_InitStruct.Pin = HardwareFault_Pin|OutputEnable_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LTCSelfTestOK_Pin LTCSelfTest_Pin */
  GPIO_InitStruct.Pin = LTCSelfTestOK_Pin|LTCSelfTest_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : Enable2_Pin ErrorReset_Pin LED_A_Pin LED_R_Pin
                           LED_Y_Pin LED_G_Pin A0_5_Pin A1_6_Pin
                           A0_6_Pin A0_7_Pin */
  GPIO_InitStruct.Pin = Enable2_Pin|ErrorReset_Pin|LED_A_Pin|LED_R_Pin
                          |LED_Y_Pin|LED_G_Pin|A0_5_Pin|A1_6_Pin
                          |A0_6_Pin|A0_7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : Config1_Pin Config2_Pin */
  GPIO_InitStruct.Pin = Config1_Pin|Config2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : Config3_Pin Config4_Pin */
  GPIO_InitStruct.Pin = Config3_Pin|Config4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : A1_2_Pin A1_3_Pin A1_4_Pin */
  GPIO_InitStruct.Pin = A1_2_Pin|A1_3_Pin|A1_4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : A0_4_Pin */
  GPIO_InitStruct.Pin = A0_4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(A0_4_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */
