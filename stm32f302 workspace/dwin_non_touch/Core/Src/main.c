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
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint32_t counter=0;
int16_t count=0;
uint8_t button_press=0;
int16_t reading=0;
uint8_t led_state1=1;
uint8_t led_state2=1;

//uint8_t page =0;
//uint8_t first =1;
//uint8_t led_page=2;
//uint8_t adc_page=3;

typedef enum{
	first_page,
	led_page,
	adc_page,
}page_state;
page_state current_page = first_page;

uint8_t led_s[] = {0x5a,0xa5,0x17,0x82,0x10,0x00,0x00,0x06,0x00,0x01,0x00,0x01,0x00,0x4d,0x00,0x59,0x00,0xba,0x00,0xbd,0x00,0x4d,0x00,0x59,0xff,0x00};

uint8_t led_us[]= {0x5a,0xa5,0x17,0x82,0x10,0x00,0x00,0x06,0x00,0x00,0x00,0x01,0x00,0x4d,0x00,0x59,0x00,0xba,0x00,0xbd,0x00,0x4d,0x00,0x59,0xff,0x00};

uint8_t adc_s[] = {0x5a,0xa5,0x17,0x82,0x10,0x10,0x00,0x06,0x00,0x01,0x00,0x01,0x01,0x12,0x00,0x53,0x01,0x78,0x00,0xb8,0x01,0x12,0x00,0x53,0xff,0x00};

uint8_t adc_us[] = {0x5a,0xa5,0x17,0x82,0x10,0x10,0x00,0x06,0x00,0x00,0x00,0x01,0x01,0x12,0x00,0x53,0x01,0x78,0x00,0xb8,0x01,0x12,0x00,0x53,0xff,0x00};

uint8_t led1_s[] = {0x5a,0xa5,0x17,0x82,0x10,0x20,0x00,0x06,0x00,0x01,0x00,0x03,0x00,0x52,0x00,0x5c,0x00,0xb6,0x00,0xb6,0x00,0x52,0x00,0x5c,0xff,0x00};

uint8_t led1_us[]= {0x5a,0xa5,0x17,0x82,0x10,0x20,0x00,0x06,0x00,0x00,0x00,0x03,0x00,0x52,0x00,0x5c,0x00,0xb6,0x00,0xb6,0x00,0x52,0x00,0x5c,0xff,0x00};

uint8_t led2_s[] = {0x5a,0xa5,0x17,0x82,0x10,0x30,0x00,0x06,0x00,0x01,0x00,0x03,0x01,0x11,0x00,0x5b,0x01,0x75,0x00,0xb5,0x01,0x11,0x00,0x5b,0xff,0x00};

uint8_t led2_us[] = {0x5a,0xa5,0x17,0x82,0x10,0x30,0x00,0x06,0x00,0x00,0x00,0x03,0x01,0x11,0x00,0x5b,0x01,0x75,0x00,0xb5,0x01,0x11,0x00,0x5b,0xff,0x00};

uint8_t led1_on[]=  {0x5a,0xa5,0x17,0x82,0x10,0x50,0x00,0x06,0x00,0x01,0x00,0x03,0x00,0x6b,0x00,0xb9,0x00,0x9e,0x00,0xcf,0x00,0x6b,0x00,0xb9,0xff,0x00};

uint8_t led1_off[]= {0x5a,0xa5,0x17,0x82,0x10,0x50,0x00,0x06,0x00,0x00,0x00,0x03,0x00,0x6b,0x00,0xb9,0x00,0x9e,0x00,0xcf,0x00,0x6b,0x00,0xb9,0xff,0x00};

uint8_t led2_on[]=  {0x5a,0xa5,0x17,0x82,0x10,0x60,0x00,0x06,0x00,0x01,0x00,0x03,0x01,0x2a,0x00,0xb8,0x01,0x61,0x00,0xd3,0x01,0x2a,0x00,0xb8,0xff,0x00};

uint8_t led2_off[]= {0x5a,0xa5,0x17,0x82,0x10,0x60,0x00,0x06,0x00,0x00,0x00,0x03,0x01,0x2a,0x00,0xb8,0x01,0x61,0x00,0xd3,0x01,0x2a,0x00,0xb8,0xff,0x00};

uint8_t led_back_s[]= {0x5a,0xa5,0x17,0x82,0x10,0x40,0x00,0x06,0x00,0x01,0x00,0x03,0x00,0x02,0x00,0xe4,0x00,0x58,0x01,0x10,0x00,0x02,0x00,0xe4,0xff,0x00};

uint8_t led_back_us[]= {0x5a,0xa5,0x17,0x82,0x10,0x40,0x00,0x06,0x00,0x00,0x00,0x03,0x00,0x02,0x00,0xe4,0x00,0x58,0x01,0x10,0x00,0x02,0x00,0xe4,0xff,0x00};

uint8_t adc_back_s[]= {0x5a,0xa5,0x17,0x82,0x10,0x70,0x00,0x06,0x00,0x01,0x00,0x05,0x00,0x02,0x00,0xe1,0x00,0x5a,0x01,0x10,0x00,0x02,0x00,0xe1,0xff,0x00};

uint8_t adc_back_us[]= {0x5a,0xa5,0x17,0x82,0x10,0x70,0x00,0x06,0x00,0x00,0x00,0x05,0x00,0x02,0x00,0xe1,0x00,0x5a,0x01,0x10,0x00,0x02,0x00,0xe1,0xff,0x00};

uint8_t data_send_command_adc[]= {0x5a,0xa5,0x05,0x82,0x50,0x00,0x00,0x01};

uint8_t screen_change_command[]={0x5A, 0xA5, 0x07, 0x82, 0x00, 0x84, 0x5A, 0x01, 0x00, 0x00};







void sendCommandToDwin(uint8_t *cmd,uint16_t length)
{
//	  while(HAL_UART_GetState(&huart1)!=HAL_UART_STATE_READY);
//	HAL_UART_Transmit_DMA(&huart1, cmd, length);
	HAL_UART_Transmit_IT(&huart1, cmd, length);
//	HAL_UART_Transmit(&huart1, cmd, length, HAL_MAX_DELAY);
	HAL_Delay(100);
//	memset(rxbuffer,0,9);

//	__HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
////	HAL_Delay(20);
//	HAL_UART_Receive_IT(&huart1, rxbuffer, 10);


//	HAL_UART_Transmit_DMA(&huart1, cmd, length);

}

void page_change(uint8_t page_no)
{
	screen_change_command[9]=page_no;

		sendCommandToDwin(screen_change_command,sizeof(screen_change_command));
}

void led_select()
{
	sendCommandToDwin(led_s,sizeof(led_s));

}

void led_unselect()
{
	sendCommandToDwin(led_us,sizeof(led_us));

}

void adc_select()
{
	sendCommandToDwin(adc_s,sizeof(adc_s));

}

void adc_unselect()
{
	sendCommandToDwin(adc_us,sizeof(adc_us));

}

void led1_select()
{
sendCommandToDwin(led1_s,sizeof(led1_s));

}

void led1_unselect()
{
sendCommandToDwin(led1_us,sizeof(led1_us));

}

void led2_select()
{
sendCommandToDwin(led2_s,sizeof(led2_s));

}

void led2_unselect()
{
sendCommandToDwin(led2_us,sizeof(led2_us));

}

void led1_ON()
{
sendCommandToDwin(led1_on,sizeof(led1_on));
}

void led1_OFF()
{
sendCommandToDwin(led1_off,sizeof(led1_off));
}

void led2_ON()
{
sendCommandToDwin(led2_on,sizeof(led2_on));
}

void led2_OFF()
{
sendCommandToDwin(led2_off,sizeof(led2_off));
}

void led_back_select()
{
sendCommandToDwin(led_back_s,sizeof(led_back_s));
}

void led_back_unselect()
{
sendCommandToDwin(led_back_us,sizeof(led_back_us));
}

void adc_back_select()
{
sendCommandToDwin(adc_back_s,sizeof(adc_back_s));
}

void adc_back_unselect()
{
sendCommandToDwin(adc_back_us,sizeof(adc_back_us));
}

void display_value(uint8_t arr[],uint16_t value)
{
	arr[7]=(uint8_t)value;
	sendCommandToDwin(data_send_command_adc,sizeof(data_send_command_adc));
}

//void adc_value()
//{
//	scale_adc_val=(adc_val*100)/4095;
//
//	show_value[7]=scale_adc_val;
//
//	if(HAL_GetTick()-tim2>50)
//	{
//	sendCommandToDwin(data_send_command_adc,sizeof(data_send_command_adc));
//	tim2=HAL_GetTick();
//	}
//}



void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	counter=__HAL_TIM_GET_COUNTER(htim);
	count=(int16_t)counter;

	count=count/4;

//	if(count<0)
//	{
//		count=0;
//	}
}

void ResetTimerCounter(TIM_HandleTypeDef *htim,uint16_t value)
{
    // Reset the timer counter to 0
    __HAL_TIM_SET_COUNTER(&htim2, value);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin==GPIO_PIN_8)
	{
	button_press=1;
	}
}

void handle_counter(uint8_t value)
{
	if(count>=value)
	{
		count=value-1;
		ResetTimerCounter(&htim2,count*4);
	}
	else if(count<0)
	{
		count=0;
		ResetTimerCounter(&htim2,count*4);
	}
}

void input_reading()
{
	if(current_page==first_page)
	{
		handle_counter(2);
		switch(count)
		{
		case 0:
			led_select();
			adc_unselect();
			break;
		case 1:
			adc_select();
			led_unselect();
			break;
		}
	}
	else if(current_page==led_page)
	{
		handle_counter(3);
		switch(count)
		{
		case 0:
			led1_select();
			led2_unselect();
			led_back_unselect();
			break;
		case 1:
			led1_unselect();
			led2_select();
			led_back_unselect();
			break;
		case 2:
			led1_unselect();
			led2_unselect();
			led_back_select();
			break;
		}
	}
}

void handle_page_switch()
{
	if(button_press)
	{
		switch(current_page)
		{
		case first_page:
			if(count==0)
			{
				page_change(2);
				ResetTimerCounter(&htim2,0);
				count=0;
				current_page=led_page;
			}
			else if(count==1)
			{
				page_change(4);
				ResetTimerCounter(&htim2,0);
				count=0;
				current_page=adc_page;
			}
			break;
		case led_page:
			if(count==0)
			{
				led_state1=!led_state1;
				if(led_state1)
				{
					led1_OFF();

				}
				else
				{
					led1_ON();

				}
			}
			else if(count==1)
			{
				led_state2=!led_state2;
				if(led_state2)
				{
					led2_OFF();

				}
				else
				{
					led2_ON();

				}
			}
			else if(count==2)
			{
				page_change(0);
				ResetTimerCounter(&htim2,0);
				count=0;
				current_page = first_page;
			}
			break;
		}
		button_press=0;
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
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Encoder_Start_IT(&htim2,TIM_CHANNEL_ALL);
  page_change(0);
//  page=first;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

		 input_reading();

		 handle_page_switch();
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
