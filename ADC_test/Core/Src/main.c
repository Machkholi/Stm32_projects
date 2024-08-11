/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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


/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include<stdio.h>
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
ADC_HandleTypeDef hadc1;

/* USER CODE BEGIN PV */
uint16_t adc_value0 = 0;
uint16_t adc_value1 = 0;
uint16_t adc_value2 = 0;
double analog_voltage=0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t arr1[]={D0_Pin,D1_Pin,D2_Pin,D3_Pin,D4_Pin,D5_Pin,D6_Pin,D7_Pin};
char name[]="rahul";
uint8_t value[10];
uint8_t value2[10];
uint8_t value3[10];
uint8_t i=0;
uint16_t arr2[]={0x80,0x81,0x82,0x83,0x84,0x85,0x86,0x87,0x88,0x89,0x8a,0x8b,0x8c,0x8d,0x8e,0x8f};
uint16_t arr3[]={0xC0,0xc1,0xc2,0xc3,0xc4,0xc5,0xc6,0xc7,0xc8,0xc9,0xca,0xcb,0xcc,0xcd,0xce,0xcf};
#define ch1 1
#define ch2 2
#define ch5 3
typedef union
{
	struct
	{
		uint8_t b0:1;
		uint8_t b1:1;
		uint8_t b2:1;
		uint8_t b3:1;
		uint8_t b4:1;
		uint8_t b5:1;
		uint8_t b6:1;
		uint8_t b7:1;
	};
	struct
	{
		uint8_t nib1:4;
		uint8_t nib2:4;
	};
	uint8_t x;
}typedef_pinconfig;

void byte_write(uint8_t x)
{
	typedef_pinconfig y =(typedef_pinconfig)x;

	HAL_GPIO_WritePin(GPIOB, D0_Pin, y.b0);
	HAL_GPIO_WritePin(GPIOB, D1_Pin, y.b1);
	HAL_GPIO_WritePin(GPIOB, D2_Pin, y.b2);
	HAL_GPIO_WritePin(GPIOB, D3_Pin, y.b3);
	HAL_GPIO_WritePin(GPIOB, D4_Pin, y.b4);
	HAL_GPIO_WritePin(GPIOB, D5_Pin, y.b5);
	HAL_GPIO_WritePin(GPIOB, D6_Pin, y.b6);
	HAL_GPIO_WritePin(GPIOB, D7_Pin, y.b7);
}

//void byte_write(uint8_t x)
//{
//	for(int i=0;i<8;i++)
//		{
//			HAL_GPIO_WritePin(GPIOB, arr1[i], ((x>>i)&0x01)?1:0);
//		}
//}

void high_nibble_write(uint8_t x)
{
	typedef_pinconfig y =(typedef_pinconfig)x;

	HAL_GPIO_WritePin(GPIOB, D4_Pin, y.b4);
	HAL_GPIO_WritePin(GPIOB, D5_Pin, y.b5);
	HAL_GPIO_WritePin(GPIOB, D6_Pin, y.b6);
	HAL_GPIO_WritePin(GPIOB, D7_Pin, y.b7);
}

void low_nibble_write(uint8_t x)
{
	typedef_pinconfig y =(typedef_pinconfig)x;

	HAL_GPIO_WritePin(GPIOB, D4_Pin, y.b0);
	HAL_GPIO_WritePin(GPIOB, D5_Pin, y.b1);
	HAL_GPIO_WritePin(GPIOB, D6_Pin, y.b2);
	HAL_GPIO_WritePin(GPIOB, D7_Pin, y.b3);
}

void lcd_data(uint8_t data)
{
//	byte_write(data);
	high_nibble_write(data);
	HAL_GPIO_WritePin(RS_GPIO_Port, RS_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(RW_GPIO_Port, RW_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, GPIO_PIN_SET);
	HAL_Delay(5);
	HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, GPIO_PIN_RESET);
	HAL_Delay(5);

	low_nibble_write(data);
	HAL_GPIO_WritePin(RS_GPIO_Port, RS_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(RW_GPIO_Port, RW_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, GPIO_PIN_SET);
	HAL_Delay(5);
	HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, GPIO_PIN_RESET);
	HAL_Delay(5);
}

void lcd_cmd(uint8_t cmd)
{
//	byte_write(cmd);
	high_nibble_write(cmd);
	HAL_GPIO_WritePin(RS_GPIO_Port, RS_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(RW_GPIO_Port, RW_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, GPIO_PIN_SET);
	HAL_Delay(5);
	HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, GPIO_PIN_RESET);
	HAL_Delay(5);

	low_nibble_write(cmd);
	HAL_GPIO_WritePin(RS_GPIO_Port, RS_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(RW_GPIO_Port, RW_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, GPIO_PIN_SET);
	HAL_Delay(5);
	HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, GPIO_PIN_RESET);
	HAL_Delay(5);
}
void lcd_string(uint8_t *ptr)
{
	while(*ptr!='\0')
	{
		lcd_data(*ptr++);
//		(*ptr)++;
	}
}



void adc_select_ch1(void)
{
	  ADC_ChannelConfTypeDef sConfig = {0};
	    sConfig.Channel = ADC_CHANNEL_1;
	    sConfig.Rank = 1;
	    sConfig.SingleDiff = ADC_SINGLE_ENDED;
	    sConfig.SamplingTime = ADC_SAMPLETIME_601CYCLES_5;
	    sConfig.OffsetNumber = ADC_OFFSET_NONE;
	    sConfig.Offset = 0;
	    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	    {
	      Error_Handler();
	    }

		  HAL_ADC_Start(&hadc1);

		  if (HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK)
		  {
			  adc_value0 = HAL_ADC_GetValue(&hadc1);
		  }
		  HAL_ADC_Stop(&hadc1);
		  HAL_Delay(10);
}

void adc_select_ch2(void)
{
	  ADC_ChannelConfTypeDef sConfig = {0};
	  sConfig.Channel = ADC_CHANNEL_2;
	  sConfig.Rank = 1;
	  sConfig.SingleDiff = ADC_SINGLE_ENDED;
	  sConfig.SamplingTime = ADC_SAMPLETIME_601CYCLES_5;
	  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }

	  HAL_ADC_Start(&hadc1);

	  if (HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK)
	  {
		  adc_value1 = HAL_ADC_GetValue(&hadc1);
	  }
	  HAL_ADC_Stop(&hadc1);
	  HAL_Delay(10);
}

void adc_select_ch5(void)
{
	  ADC_ChannelConfTypeDef sConfig = {0};
	    sConfig.Channel = ADC_CHANNEL_5;
	    sConfig.Rank = 1;
	    sConfig.SingleDiff = ADC_SINGLE_ENDED;
	    sConfig.SamplingTime = ADC_SAMPLETIME_601CYCLES_5;
	    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	    {
	      Error_Handler();
	    }

		  HAL_ADC_Start(&hadc1);

		  if (HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK)
		  {
			  adc_value2 = HAL_ADC_GetValue(&hadc1);
		  }
		  HAL_ADC_Stop(&hadc1);
		  HAL_Delay(10);
}

void print_adc()
{

	 adc_select_ch1();

	  lcd_cmd(0x80);
	  sprintf((char*)value,"%04d",(int)adc_value0);
	 lcd_string(value);
HAL_Delay(100);
	 adc_select_ch2();
	  lcd_cmd(0x8b);
	  sprintf((char*)value2,"%04d",(int)adc_value1);
	 lcd_string(value2);
	 HAL_Delay(100);

	 adc_select_ch5();
	  lcd_cmd(0xc0);
	  sprintf((char*)value3,"%04d",(int)adc_value2);
	 lcd_string(value3);
	 HAL_Delay(100);

//	  analog_voltage= (adc_value *3.3)/4095;
//	  analog_voltage=analog_voltage*1000;
}

//void print(uint8_t channel,uint8_t location)
//{
//	ADC_ChannelConfTypeDef sConfig = {0};
//	switch(channel)
//	{
//	case ch1:
//
//
//		    sConfig.Channel = ADC_CHANNEL_1;
//		    sConfig.Rank = 1;
//		    sConfig.SingleDiff = ADC_SINGLE_ENDED;
//		    sConfig.SamplingTime = ADC_SAMPLETIME_601CYCLES_5;
//		    sConfig.OffsetNumber = ADC_OFFSET_NONE;
//		    sConfig.Offset = 0;
//		    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
//		    {
//		      Error_Handler();
//		    }
//
//			  HAL_ADC_Start(&hadc1);
//
//			  if (HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK)
//			  {
//				  adc_value0 = HAL_ADC_GetValue(&hadc1);
//			  }
//			  HAL_ADC_Stop(&hadc1);
//			  HAL_Delay(10);
//
//			  lcd_cmd(location);
//			  sprintf((char*)value,"%04d",(int)adc_value0);
//			 lcd_string(value);
//		HAL_Delay(100);
//
//	case ch2:
////		  ADC_ChannelConfTypeDef sConfig = {0};
//		  sConfig.Channel = ADC_CHANNEL_2;
//		  sConfig.Rank = 1;
//		  sConfig.SingleDiff = ADC_SINGLE_ENDED;
//		  sConfig.SamplingTime = ADC_SAMPLETIME_601CYCLES_5;
//		  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
//		  {
//		    Error_Handler();
//		  }
//
//		  HAL_ADC_Start(&hadc1);
//
//		  if (HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK)
//		  {
//			  adc_value1 = HAL_ADC_GetValue(&hadc1);
//		  }
//		  HAL_ADC_Stop(&hadc1);
//		  HAL_Delay(10);
//		  lcd_cmd(location);
//		  sprintf((char*)value2,"%04d",(int)adc_value1);
//		 lcd_string(value2);
//		 HAL_Delay(100);
//
//	case ch5:
////		  ADC_ChannelConfTypeDef sConfig = {0};
//		    sConfig.Channel = ADC_CHANNEL_5;
//		    sConfig.Rank = 1;
//		    sConfig.SingleDiff = ADC_SINGLE_ENDED;
//		    sConfig.SamplingTime = ADC_SAMPLETIME_601CYCLES_5;
//		    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
//		    {
//		      Error_Handler();
//		    }
//
//			  HAL_ADC_Start(&hadc1);
//
//			  if (HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK)
//			  {
//				  adc_value2 = HAL_ADC_GetValue(&hadc1);
//			  }
//			  HAL_ADC_Stop(&hadc1);
//			  HAL_Delay(10);
//			  lcd_cmd(location);
//			  sprintf((char*)value3,"%04d",(int)adc_value2);
//			 lcd_string(value3);
//			 HAL_Delay(100);
//	}
//}

void location(void (*channel),uint8_t row,uint8_t col)
{
	if(row==1)
	{
		lcd_cmd(0x80|col);
	}
	else if(row==2)
	{
		lcd_cmd(0xc0|col);
	}
}
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
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  lcd_cmd(0x01);
//  lcd_cmd(0x02);
//  lcd_cmd(0x06);
  lcd_cmd(0x28);
  lcd_cmd(0x0c);
  lcd_cmd(0x0E);
  lcd_cmd(0x80);

//  lcd_string("ADC VALUE ");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
location(adc_selectch5,2,5);
//	  print_adc();
	  //print(ch1,0x85);



//
//	  adc_select_ch2();
//
//
//	  adc_select_ch5();



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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC1;
  PeriphClkInit.Adc1ClockSelection = RCC_ADC1PLLCLK_DIV1;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

//  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
//  sConfig.Channel = ADC_CHANNEL_1;
//  sConfig.Rank = ADC_REGULAR_RANK_1;
//  sConfig.SingleDiff = ADC_SINGLE_ENDED;
//  sConfig.SamplingTime = ADC_SAMPLETIME_601CYCLES_5;
//  sConfig.OffsetNumber = ADC_OFFSET_NONE;
//  sConfig.Offset = 0;
//  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, RS_Pin|RW_Pin|EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, D0_Pin|D1_Pin|D2_Pin|D3_Pin
                          |D4_Pin|D5_Pin|D6_Pin|D7_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : RS_Pin RW_Pin EN_Pin */
  GPIO_InitStruct.Pin = RS_Pin|RW_Pin|EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : D0_Pin D1_Pin D2_Pin D3_Pin
                           D4_Pin D5_Pin D6_Pin D7_Pin */
  GPIO_InitStruct.Pin = D0_Pin|D1_Pin|D2_Pin|D3_Pin
                          |D4_Pin|D5_Pin|D6_Pin|D7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

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
