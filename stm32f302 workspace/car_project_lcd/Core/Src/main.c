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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//uint8_t arr1[]={D0_Pin,D1_Pin,D2_Pin,D3_Pin,D4_Pin,D5_Pin,D6_Pin,D7_Pin};
extern uint8_t flag;
char name[]="rahul";
 uint8_t count_I=0;
uint8_t count_O=0;
uint8_t count_T=0;

uint8_t b1_flag=0;
uint8_t b2_flag=0;
uint8_t buffer[10];
uint8_t i=0;
uint16_t arr2[]={0x80,0x81,0x82,0x83,0x84,0x85,0x86,0x87,0x88,0x89,0x8a,0x8b,0x8c,0x8d,0x8e,0x8f};
uint16_t arr3[]={0xC0,0xc1,0xc2,0xc3,0xc4,0xc5,0xc6,0xc7,0xc8,0xc9,0xca,0xcb,0xcc,0xcd,0xce,0xcf};

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

//void byte_write(uint8_t x)
//{
//	typedef_pinconfig y =(typedef_pinconfig)x;
//
//	HAL_GPIO_WritePin(GPIOB, D0_Pin, y.b0);
//	HAL_GPIO_WritePin(GPIOB, D1_Pin, y.b1);
//	HAL_GPIO_WritePin(GPIOB, D2_Pin, y.b2);
//	HAL_GPIO_WritePin(GPIOB, D3_Pin, y.b3);
//	HAL_GPIO_WritePin(GPIOB, D4_Pin, y.b4);
//	HAL_GPIO_WritePin(GPIOB, D5_Pin, y.b5);
//	HAL_GPIO_WritePin(GPIOB, D6_Pin, y.b6);
//	HAL_GPIO_WritePin(GPIOB, D7_Pin, y.b7);
//}

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

void toggle_char_down(uint8_t data)
{
int k=15;
int l=0;
	while(l<=15)
	{
		lcd_cmd(0x01);
		lcd_cmd(arr3[k]);
		lcd_data(data);
		k--;
		l++;
//		if(k==0)
//		{
//			break;
//		}
		HAL_Delay(100);
	}
}

void delete_char()
{
	lcd_cmd(0x10);

	lcd_data(' ');
	lcd_cmd(0x10);
//	lcd_cmd(0x04);



}

void lcd_string(uint8_t *ptr)
{
	while(*ptr!='\0')
	{
		lcd_data(*ptr++);
//		*ptr++;
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
//	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_13);
//	flag++;
//	 if(flag==12)
//			  {
//				  flag=0;
//			  }
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
  /* USER CODE BEGIN 2 */
  lcd_cmd(0x01);
//  lcd_cmd(0x02);
//  lcd_cmd(0x06);
  lcd_cmd(0x28);
  lcd_cmd(0x0c);
  lcd_cmd(0x80);

  lcd_string((uint8_t *)"Car I ");

  lcd_cmd(0x89);
  lcd_string((uint8_t *)"CarO ");

  lcd_cmd(0xc0);
  lcd_string((uint8_t *)"Total ");

  lcd_cmd(0xc8);
  lcd_data('0');

  lcd_cmd(0x86);
  lcd_data('0');

  lcd_cmd(0x8e);
  lcd_data('0');




//  lcd_data('A');
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//		if(flag!=0)
//		{
//			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
//			  HAL_Delay(arr[flag]);
//			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
//			  HAL_Delay(arr[flag]);
//	  itoa(flag,(char *)buffer,10);
//			lcd_cmd(0x86);
//			lcd_string(buffer);
//
//
//			lcd_cmd(0xc8);
//			lcd_string(buffer);
//
//			lcd_cmd(0x8e);
//			lcd_string(buffer);

			if(b1_flag)
			{
				count_I++;
				count_T++;
				HAL_Delay(100);
//				  itoa(count_I,(char *)buffer,10);
				if(count_I>9)
				{
					sprintf((char *)buffer,"%02d",count_I);
				}
				else
				{
					sprintf((char *)buffer,"%d",count_I);
				}

						lcd_cmd(0x86);
						lcd_string(buffer);

//						itoa(count_T,(char *)buffer,10);
						if(count_T>9)
						{
							sprintf((char *)buffer,"%02d",count_T);
						}
						else
						{
							sprintf((char *)buffer,"%d",count_T);
						}

						lcd_cmd(0xc8);
						lcd_string(buffer);

						b1_flag=0;
			}
			if(b2_flag)
			{


				if(count_T<=0)
				{
					count_T=0;
				}
				else
				{
					count_T--;
				}
				if(count_O==count_I)
				{
					count_O=count_I;
				}
				else
				{
				count_O++;
				}
HAL_Delay(100);
//				  itoa(count_O,(char *)buffer,10);
if(count_O>9)
{
sprintf((char *)buffer,"%02d",count_O);
}
else
{
	sprintf((char *)buffer,"%d",count_O);
}
						lcd_cmd(0x8e);
						lcd_string(buffer);

//						itoa(count_T,(char *)buffer,10);
						if(count_T>9)
						{
							sprintf((char *)buffer,"%02d",count_T);
						}
						else{
							sprintf((char *)buffer,"%d",count_T);
						}

						lcd_cmd(0xc8);
						lcd_string(buffer);

						b2_flag=0;
			}



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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, RS_Pin|RW_Pin|EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, D4_Pin|D5_Pin|D6_Pin|D7_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : RS_Pin RW_Pin EN_Pin */
  GPIO_InitStruct.Pin = RS_Pin|RW_Pin|EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : D4_Pin D5_Pin D6_Pin D7_Pin */
  GPIO_InitStruct.Pin = D4_Pin|D5_Pin|D6_Pin|D7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

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
