/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : SAI.c
  * Description        : This file provides code for the configuration
  *                      of the SAI instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "sai.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

SAI_HandleTypeDef hsai_BlockA3;

/* SAI3 init function */
void MX_SAI3_Init(void)
{

  /* USER CODE BEGIN SAI3_Init 0 */

  /* USER CODE END SAI3_Init 0 */

  /* USER CODE BEGIN SAI3_Init 1 */

  /* USER CODE END SAI3_Init 1 */

  hsai_BlockA3.Instance = SAI3_Block_A;
  hsai_BlockA3.Init.Protocol = SAI_FREE_PROTOCOL;
  hsai_BlockA3.Init.AudioMode = SAI_MODEMASTER_TX;
  hsai_BlockA3.Init.DataSize = SAI_DATASIZE_8;
  hsai_BlockA3.Init.FirstBit = SAI_FIRSTBIT_MSB;
  hsai_BlockA3.Init.ClockStrobing = SAI_CLOCKSTROBING_FALLINGEDGE;
  hsai_BlockA3.Init.Synchro = SAI_ASYNCHRONOUS;
  hsai_BlockA3.Init.OutputDrive = SAI_OUTPUTDRIVE_DISABLE;
  hsai_BlockA3.Init.NoDivider = SAI_MASTERDIVIDER_ENABLE;
  hsai_BlockA3.Init.FIFOThreshold = SAI_FIFOTHRESHOLD_EMPTY;
  hsai_BlockA3.Init.AudioFrequency = SAI_AUDIO_FREQUENCY_192K;
  hsai_BlockA3.Init.SynchroExt = SAI_SYNCEXT_DISABLE;
  hsai_BlockA3.Init.MonoStereoMode = SAI_STEREOMODE;
  hsai_BlockA3.Init.CompandingMode = SAI_NOCOMPANDING;
  hsai_BlockA3.Init.TriState = SAI_OUTPUT_NOTRELEASED;
  hsai_BlockA3.Init.PdmInit.Activation = DISABLE;
  hsai_BlockA3.Init.PdmInit.MicPairsNbr = 1;
  hsai_BlockA3.Init.PdmInit.ClockEnable = SAI_PDM_CLOCK1_ENABLE;
  hsai_BlockA3.FrameInit.FrameLength = 8;
  hsai_BlockA3.FrameInit.ActiveFrameLength = 1;
  hsai_BlockA3.FrameInit.FSDefinition = SAI_FS_STARTFRAME;
  hsai_BlockA3.FrameInit.FSPolarity = SAI_FS_ACTIVE_LOW;
  hsai_BlockA3.FrameInit.FSOffset = SAI_FS_FIRSTBIT;
  hsai_BlockA3.SlotInit.FirstBitOffset = 0;
  hsai_BlockA3.SlotInit.SlotSize = SAI_SLOTSIZE_DATASIZE;
  hsai_BlockA3.SlotInit.SlotNumber = 1;
  hsai_BlockA3.SlotInit.SlotActive = 0x00000000;
  if (HAL_SAI_Init(&hsai_BlockA3) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN SAI3_Init 2 */

  /* USER CODE END SAI3_Init 2 */

}
static uint32_t SAI3_client =0;

void HAL_SAI_MspInit(SAI_HandleTypeDef* saiHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct;
/* SAI3 */
    if(saiHandle->Instance==SAI3_Block_A)
    {
    /* SAI3 clock enable */
    if (SAI3_client == 0)
    {
       __HAL_RCC_SAI3_CLK_ENABLE();
    }
    SAI3_client ++;

    /**SAI3_A_Block_A GPIO Configuration
    PD0     ------> SAI3_SCK_A
    PD1     ------> SAI3_SD_A
    PD4     ------> SAI3_FS_A
    */
    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_4;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF6_SAI3;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    }
}

void HAL_SAI_MspDeInit(SAI_HandleTypeDef* saiHandle)
{

/* SAI3 */
    if(saiHandle->Instance==SAI3_Block_A)
    {
    SAI3_client --;
    if (SAI3_client == 0)
      {
      /* Peripheral clock disable */
       __HAL_RCC_SAI3_CLK_DISABLE();
      }

    /**SAI3_A_Block_A GPIO Configuration
    PD0     ------> SAI3_SCK_A
    PD1     ------> SAI3_SD_A
    PD4     ------> SAI3_FS_A
    */
    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_4);

    }
}

/**
  * @}
  */

/**
  * @}
  */
