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
#define ADC_CNT_MAX 10 //maximum number of ADC values to read before averaging
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
bool FLAG_TIM2 = false;	//flag for TIM2
bool FLAG_TIM3 = false;	//flag for TIM3
bool FLAG_TIM4 = false;	//flag for TIM4
bool FLAG_TIM5 = false;	//flag for TIM5
bool FLAG_MainTask1=false, FLAG_MainTask2=false, FLAG_MainTask3=false;	//flags for main task
bool FLAG_WDT = true;	//flag for WDT

//uint32_t adc1CNT = 0, adc2CNT = 0;
//uint32_t adcVal[2][ADC_CNT_MAX];	//ADC values for current sensors
float alpha = 0.01f;	//alpha value for exponential moving average filter
uint32_t adcVal;	//ADC values for current sensors
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
/* USER CODE BEGIN PFP */

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

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_FDCAN1_Init();
  MX_RTC_Init();
  MX_USB_Device_Init();
  MX_ADC1_Init();
  MX_SPI1_Init();
  MX_ICACHE_Init();
  MX_ADC2_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM1_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */
  LTC6811_init();	//initializes the LTC (and SPI communication)

  SetHardwareProtection();	//sets the hardware protection configuration pins to the correct values
  HAL_Delay(100);	//wait for 100ms to ensure the hardware protection is set

  HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET);	//turn off red LED
  HAL_GPIO_WritePin(LED_A_GPIO_Port, LED_A_Pin, GPIO_PIN_RESET);	//turn off amber LED
  HAL_GPIO_WritePin(LED_Y_GPIO_Port, LED_Y_Pin, GPIO_PIN_RESET);	//turn off yellow LED
  HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_RESET);	//turn off green LED

  //START ADCs

  HAL_ADC_Start_IT(&hadc2);
  HAL_ADC_Start_IT(&hadc1);
  HAL_ADCEx_Calibration_Start(&hadc1, ADC_DIFFERENTIAL_ENDED) ;
  HAL_ADCEx_Calibration_Start(&hadc2, ADC_DIFFERENTIAL_ENDED);

//  HAL_ADCEx_MultiModeStart_DMA(&hadc1, (uint32_t*)adcVal, 1);	//start ADC conversion for current sensors, 2 channels in multi mode

  //START TIMERS
  HAL_TIM_Base_Start_IT(&htim5); // Triggers Reading of aux conversion
  HAL_TIM_Base_Start_IT(&htim4); // Triggers aux conversion
  HAL_TIM_Base_Start_IT(&htim3); // Triggers Reading of voltage conversion
  HAL_TIM_Base_Start_IT(&htim2); // Triggers voltage conversion
  HAL_TIM_Base_Start_IT(&htim7); // 3khz clock for LTC6801
//  HAL_TIM_Base_Start_IT(&htim8); // Triggers ADC

  HAL_Delay(100);	//wait for 100ms to ensure the system is stable
  resetOutputLatch();	//reset the output latch
  HAL_TIM_Base_Start_IT(&htim1); // pseudo watchdog timer, needs to be reset every 0.5s
  adcOffsetZero();	//zero the ADC offset, this is done once at the beginning of the program

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if(FLAG_TIM2){
		  wakeup_idle(TOTAL_IC);	//wake up daisy-chain from sleep mode
		  LTC6811_adcv(MD_422HZ_1KHZ, DCP_DISABLED, CELL_CH_ALL); //should take 1.1 ms for 27KHz mode and 1.3 ms for 14KHz mode
		  FLAG_TIM2 = false;	//sets the flag to false
//		  HAL_GPIO_TogglePin(LED_R_GPIO_Port, LED_R_Pin);
	  }
	  if(FLAG_TIM3){
		  wakeup_idle(TOTAL_IC);	//wake up daisy-chain from sleep mode
		  readVoltages();	//reads the cell voltages
		  FLAG_TIM3 = false;	//sets the flag to false
//		  HAL_GPIO_TogglePin(LED_A_GPIO_Port, LED_A_Pin);
	  }
	  if(FLAG_TIM4){
		  wakeup_idle(TOTAL_IC);	//wake up daisy-chain from sleep mode
		  LTC6811_adax(MD_422HZ_1KHZ, AUX_CH_ALL); //should take 1.1 ms for 27KHz mode and 1.3 ms for 14KHz mode
		  FLAG_TIM4 = false;	//sets the flag to false
//		  HAL_ADC_Start(&hadc1);	//start ADC conversion for current sensor
//		  HAL_ADC_Start(&hadc2);	//start ADC conversion for current sensor
//		  HAL_GPIO_TogglePin(LED_G_GPIO_Port, LED_Y_Pin);
	  }
	  if(FLAG_TIM5){
		  wakeup_idle(TOTAL_IC);	//wake up daisy-chain from sleep mode
		  readTemperatures();	//reads the cell voltages
		  tempConvert();	//converts the ADC values into temperature
		  currentConvert();
//		  HAL_GPIO_TogglePin(LED_G_GPIO_Port, LED_G_Pin);

		  FLAG_TIM5 = false;
		  FLAG_MainTask1 = true;
		  FLAG_MainTask2 = true;
		  FLAG_MainTask3 = true;
	  }
	  //reset the timer 1 counter (watchdog)
	  __HAL_TIM_SET_COUNTER(&htim1, 0);
//	  if(HAL_GPIO_ReadPin(OutputEnable_GPIO_Port, OutputEnable_Pin) == GPIO_PIN_RESET){
//		  HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_SET);	//turn on red LED
//	  }
	  /*//debug manual output reset
	  if(HAL_GPIO_ReadPin(Config1_GPIO_Port, Config1_Pin) == GPIO_PIN_RESET){
		  HAL_GPIO_WritePin(Enable1_GPIO_Port, Enable1_Pin, GPIO_PIN_SET);
	  }else{
		  HAL_GPIO_WritePin(Enable1_GPIO_Port, Enable1_Pin, GPIO_PIN_RESET);
	  }
	  if(HAL_GPIO_ReadPin(Config2_GPIO_Port, Config2_Pin) == GPIO_PIN_RESET){
		  HAL_GPIO_WritePin(Enable2_GPIO_Port, Enable2_Pin, GPIO_PIN_SET);
	  }else{
		  HAL_GPIO_WritePin(Enable2_GPIO_Port, Enable2_Pin, GPIO_PIN_RESET);
	  }
	  */
	  if(HAL_GPIO_ReadPin(Config1_GPIO_Port, Config1_Pin) == GPIO_PIN_RESET){
		  resetOutputLatch();	//reset the output latch if Config1 is pressed
	  }
	  if(FLAG_MainTask1){
		  errorCheck();
		  FLAG_MainTask1 = false;
	  }else if(FLAG_MainTask2){
//		  balancingControl();
		  FLAG_MainTask2 = false;
	  }else if(FLAG_MainTask3){
		  outputControl();
		  LEDControl();
		  FLAG_MainTask3 = false;
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI
                              |RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.LSIDiv = RCC_LSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_7;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 24;
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

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the common periph clock
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLLSAI1;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSAI1SOURCE_MSI;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 12;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_48M2CLK|RCC_PLLSAI1_ADC1CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{

	if(hadc->Instance == ADC1){
		//read the ADC value for current sensor

		adcVal = HAL_ADC_GetValue(&hadc1);
		adc1Val = (float)((float)adc1Val*(1-alpha) + (float)adcVal*alpha);	//apply a low-pass filter to the ADC value
//		adc1CNT++;	//increment the adc counter
//		if(adc1CNT >= ADC_CNT_MAX){	//if the adc counter reaches the maximum value
//			adc1CNT = 0;	//reset the adc counter
//			adc1Val = 0;	//reset the adc1Val to 0
//			for(int i = 0; i < ADC_CNT_MAX; i++){	//copy the adc values to the adc1Val array
//				adc1Val += adcVal[0][i];
//			}
//			adc1Val /= ADC_CNT_MAX;	//average the adc values
//		}
//		HAL_GPIO_TogglePin(Test_GPIO_Port, Test_Pin);	//toggle test pin for debugging
	}else if(hadc->Instance == ADC2){
		//read the ADC value for current sensor
		adcVal = HAL_ADC_GetValue(&hadc2);
		adc2Val = (float)((float)adc2Val*(1-alpha) + (float)adcVal*alpha);	//apply a low-pass filter to the ADC value
//		adcVal[1][adc2CNT] = HAL_ADC_GetValue(&hadc2);
//		adc2CNT++;	//increment the adc counter
//		if(adc2CNT >= ADC_CNT_MAX){	//if the adc counter reaches the maximum value
//			adc2CNT = 0;	//reset the adc counter
//			adc2Val = 0;	//reset the adc2Val to 0
//			for(int i = 0; i < ADC_CNT_MAX; i++){	//copy the adc values to the adc2Val array
//				adc2Val += adcVal[1][i];
//			}
//			adc2Val /= ADC_CNT_MAX;	//average the adc values
//		}
	}


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
		FLAG_TIM2 = true;	//sets the flag to true

	//TIM3 reads cell voltages
	}else if(htim->Instance == TIM3){
		FLAG_TIM3 = true;	//sets the flag to true

	//TIM4 starts aux voltage conversion
	}else if(htim->Instance == TIM4){
		FLAG_TIM4 = true;	//sets the flag to true

	//TIM5 reads aux voltage conversion
	}else if(htim->Instance == TIM5){
		FLAG_TIM5 = true;	//sets the flag to true
//		HAL_GPIO_TogglePin(LED_G_GPIO_Port, LED_G_Pin);	//toggles green LED
	}
	if(htim->Instance == TIM7){
		if(FLAG_WDT){
			HAL_GPIO_TogglePin(WDT_Stm_GPIO_Port, WDT_Stm_Pin);	//toggles external watchdog
		}
	}
	if(htim->Instance == TIM1){
//		FLAG_WDT = false;	//watchdog fired, stop output toggling
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
