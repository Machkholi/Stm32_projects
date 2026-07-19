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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t arr1[]={D0_Pin,D1_Pin,D2_Pin,D3_Pin,D4_Pin,D5_Pin,D6_Pin,D7_Pin};
char name[]="rahul";
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
		lcd_data(*ptr);
		*ptr++;
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
  /* USER CODE BEGIN 2 */

  lcd_cmd(0x01);
//  lcd_cmd(0x02);
//  lcd_cmd(0x06);
  lcd_cmd(0x28);
  lcd_cmd(0x0c);
  lcd_cmd(0x0E);
  lcd_cmd(0x80);
//	  while( i<sizeof(name)-1)
//	  {
//
//		  lcd_data(name[i]);
//
//		  i++;
//	  }
lcd_string("hello");


//  lcd_data("RAHUL");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {



		 if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13)==0)
		  {
			  delete_char();
			  while(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13)==0);
		  }




//	  for(int i=0;i<sizeof(name);i++)
//	  {
//	  lcd_data(name[i]);
//	  }
//	  while( i<sizeof(name)-1)
//	  {
//
//		  lcd_data(name[i]);
//		  i++;
//	  }
//	  toggle_char('A');
//	  toggle_char_down('A');
//	  while(i<=15)
//		{
//		lcd_cmd(0x01);
//		lcd_cmd(arr2[i]);
	//lcd_data('a');
//		i++;
//		HAL_Delay(500);
//		}

//		for(uint8_t i=0;i<10;i++)
//		{

//			HAL_Delay(500);
			//lcd_cmd(0x01);
//			lcd_cmd(0x81);
//			lcd_data('A');
//			HAL_Delay(500);
//			lcd_cmd(0x01);
//			lcd_cmd(0x82);
//			lcd_data('A');
//			HAL_Delay(500);
////			lcd_cmd(0x01);
//		}
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, RS_Pin|RW_Pin|EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, D0_Pin|D1_Pin|D2_Pin|D3_Pin
                          |D4_Pin|D5_Pin|D6_Pin|D7_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

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


void toggle_char(uint8_t data)
{
int j=0;
	while(j<=15)
	{
		lcd_cmd(0x01);
		lcd_cmd(arr2[j]);
		lcd_data(data);
		j++;
		HAL_Delay(100);
	}
}

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
