/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "cmsis_os.h"
#include "adc.h"
#include "dma.h"
#include "fdcan.h"
#include "icache.h"
#include "rtc.h"
#include "spi.h"
#include "tim.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "BMS.h"
#include "LTC6811.h"
#include "stm32l5xx_hal.h"

//#include "usbd_cdc_if.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
bool flagTIM2= false;	//flag for TIM2
bool FLAG_TIM3= false;	//flag for TIM3
bool FLAG_TIM4= false;	//flag for TIM4
bool FLAG_TIM5= false;	//flag for TIM5
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */
//void LTC6811_init(void);	//Initializes the LTC and the SPI communication
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_FDCAN1_Init();
  MX_RTC_Init();
  MX_ADC1_Init();
  MX_SPI1_Init();
  MX_ICACHE_Init();
  MX_ADC2_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM8_Init();
  MX_TIM1_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */
  LTC6811_init();	//initializes the LTC (and SPI communication)

  //START ADC
  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adcVal[0], 1);
  HAL_ADC_Start_DMA(&hadc2, (uint32_t *)adcVal[1], 1);

  //START TIMERS
  HAL_TIM_Base_Start(&htim8);	//Triggers ADCs

  HAL_TIM_Base_Start_IT(&htim5); // Triggers Reading of aux conversion
  HAL_TIM_Base_Start_IT(&htim4); // Triggers aux conversion
  HAL_TIM_Base_Start_IT(&htim3); // Triggers Reading of voltage conversion
  HAL_TIM_Base_Start_IT(&htim2); // Triggers voltage conversion

  /* USER CODE END 2 */

  /* Init scheduler */
//  osKernelInitialize();

  /* Call init function for freertos objects (in cmsis_os2.c) */
//  MX_FREERTOS_Init();

  /* Start scheduler */
//  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if(flagTIM2){
		  wakeup_idle(TOTAL_IC);	//wake up daisy-chain from sleep mode
		  LTC6811_adcv(MD_27KHZ_14KHZ, DCP_DISABLED, CELL_CH_ALL); //should take 1.1 ms for 27KHz mode and 1.3 ms for 14KHz mode
		  flagTIM2 = false;	//sets the flag to false
	  }
	  if(FLAG_TIM3){
		  wakeup_idle(TOTAL_IC);	//wake up daisy-chain from sleep mode
		  readVoltages();	//reads the cell voltages
		  FLAG_TIM3 = false;	//sets the flag to false
	  }
	  if(FLAG_TIM4){
		  wakeup_idle(TOTAL_IC);	//wake up daisy-chain from sleep mode
		  LTC6811_adax(MD_27KHZ_14KHZ, AUX_CH_ALL); //should take 1.1 ms for 27KHz mode and 1.3 ms for 14KHz mode
		  FLAG_TIM4 = false;	//sets the flag to false
	  }
	  if(FLAG_TIM5){
		  wakeup_idle(TOTAL_IC);	//wake up daisy-chain from sleep mode
		  readTemperatures();	//reads the cell voltages
		  tempConvert();	//converts the ADC values into temperature
		  FLAG_TIM5 = false;	//sets the flag to false
	  }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE0) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_7;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 48;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV4;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

/* USER CODE BEGIN 4 */

//handle the DMA interrupt
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{
	currentConvert();	//convert ADC values into current sensor skaling 19.8mV/A
}

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
	//timers to control the LTC6811 value reading
	//TIM2 starts first and then triggers TIM3, which triggers TIM4, which triggers TIM5.
	//These 4 timers run in a loop at 5Hz each, and 20Hz overall
	//TIM2 starts cell voltage conversion

	if(htim->Instance == TIM2){
//		__HAL_TIM_CLEAR_IT(&htim2 ,TIM_IT_UPDATE);	//clears the IT flag
//		__HAL_TIM_SET_COUNTER(&htim2, 0);	//resets the timer's counter to 0
		flagTIM2 = true;	//sets the flag to true
//		wakeup_idle(TOTAL_IC);	//wake up daisy-chain from sleep mode
//		LTC6811_adcv(MD_27KHZ_14KHZ, DCP_DISABLED, CELL_CH_ALL); //should take 1.1 ms for 27KHz mode and 1.3 ms for 14KHz mode
		HAL_GPIO_TogglePin(LED_R_GPIO_Port, LED_R_Pin);	//toggles the red LED to show that the timer is running

		//TIM3 reads cell voltages
	}else if(htim->Instance == TIM3){
//		__HAL_TIM_CLEAR_IT(&htim3 ,TIM_IT_UPDATE);	//clears the IT flag
//		__HAL_TIM_SET_COUNTER(&htim3, 0);	//resets the timer's counter to 0
		FLAG_TIM3 = true;	//sets the flag to true
//		wakeup_idle(TOTAL_IC);	//wake up daisy-chain from sleep mode
//		readVoltages();	//reads the cell voltages
		HAL_GPIO_TogglePin(LED_A_GPIO_Port, LED_A_Pin);	//toggles the amber LED to show that the timer is running

	//TIM4 starts aux voltage conversion
	}else if(htim->Instance == TIM4){
//		__HAL_TIM_CLEAR_IT(&htim4 ,TIM_IT_UPDATE);	//clears the IT flag
//		__HAL_TIM_SET_COUNTER(&htim4, 0);	//resets the timer's counter to 0
		FLAG_TIM4 = true;	//sets the flag to true
//		wakeup_idle(TOTAL_IC);	//wake up daisy-chain from sleep mode
//		LTC6811_adax(MD_27KHZ_14KHZ, AUX_CH_ALL); //should take 1.1 ms for 27KHz mode and 1.3 ms for 14KHz mode
		HAL_GPIO_TogglePin(LED_Y_GPIO_Port, LED_Y_Pin);	//toggles the yellow LED to show that the timer is running

		//TIM5 reads aux voltage conversion
	}else if(htim->Instance == TIM5){
//		__HAL_TIM_CLEAR_IT(&htim5 ,TIM_IT_UPDATE);	//clears the IT flag
//		__HAL_TIM_SET_COUNTER(&htim5, 0);	//resets the timer's counter to 0
//		flagErrorCheck = true;	//enables the error check and state change code block
		FLAG_TIM5 = true;	//sets the flag to true
//		wakeup_idle(TOTAL_IC);	//wake up daisy-chain from sleep mode
//		readTemperatures();
		HAL_GPIO_TogglePin(LED_G_GPIO_Port, LED_G_Pin);	//toggles the green LED to show that the timer is running
	}

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
