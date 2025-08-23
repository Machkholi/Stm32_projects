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
uint8_t button_status=0;
uint8_t flag=0;
uint8_t count=0;
uint32_t previous_time=0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

const uint8_t led_array[10][7]={    { 0,0,0,0,0,0,1 }, // display '0'
                                 	{ 1,0,0,1,1,1,1 }, // display '1'
                                 	{ 0,0,1,0,0,1,0 }, // display '2'
								 	{ 0,0,0,0,1,1,0 }, // display '3'
									{ 1,0,0,1,1,0,0 }, // display '4'
									{ 0,1,0,0,1,0,0 }, // display '5'
									{ 0,1,0,0,0,0,0 }, // display '6'
									{ 0,0,0,1,1,1,1 }, // display '7'
									{ 0,0,0,0,0,0,0 }, // display '8'
									{ 0,0,0,1,1,0,0 }  // display '9'
								};

uint8_t arr[]={0xc0,0xf9,0xa4,0xb0,0x99,0x92,0x82,0xf8,0x80,0x90};
uint8_t arr1[]={GPIO_PIN_0,GPIO_PIN_1,GPIO_PIN_2,GPIO_PIN_3,GPIO_PIN_4,GPIO_PIN_5,GPIO_PIN_6,GPIO_PIN_7};
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
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  for(int i=0;i<10;i++)
	  {
		  byte_write(arr[i]);
		  HAL_Delay(1000);
	  }



//	  button_status=HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13);
//
//	  	  if(button_status==0)
//	  	  {
//	  		  if(HAL_GetTick()-previous_time>200)
//	  		  {
//	  			  previous_time=HAL_GetTick();
//
//	  				  if(flag==0)
//	  				  {
//
//	  			  show_numbers(count);           //0
//	  			  flag=1;
//	  			  count++;
//	  			  //play_sound();
//	  				  }
//	  				  else if(flag==1)
//	  				  {
//	  					  show_numbers(count);    //1
//	  					  flag=2;
//	  					  count++;
//	  				  }
//	  				  else if(flag==2)
//	  				  {
//	  					  show_numbers(count);     //2
//	  					  flag=3;
//	  					  count++;
//	  				  }
//	  				  else if(flag==3)
//	  				  {
//	  					  show_numbers(count);      //3
//	  					  flag=4;
//	  					  count++;
//	  				  }
//	  				  else if(flag==4)
//	  				  {
//	  					  show_numbers(count);      //4
//	  					  flag=5;
//	  					  count++;
//	  				  }
//	  				  else if(flag==5)
//	  				  {
//	  					  show_numbers(count);      //5
//	  					  flag=6;
//	  					  count++;
//	  				  }
//	  				  else if(flag==6)
//	  				  {
//	  					  show_numbers(count);      //6
//	  					  flag=7;
//	  					  count++;
//	  				  }
//	  				  else if(flag==7)
//	  				  {
//	  					  show_numbers(count);      //7
//	  					  flag=8;
//	  					  count++;
//	  				  }
//	  				  else if(flag==8)
//	  				  {
//	  					  show_numbers(count);      //8
//	  					  flag=9;
//	  					  count++;
//	  				  }
//	  				  else if(flag==9)
//	  				  {
//	  					  show_numbers(count);      //9
//	  					  flag=0;
//	  					  count=0;
//	  				  }
//
//
//	  	  }
//	  	  }
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
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_SET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PC0 PC1 PC3 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 PB3
                           PB4 PB5 PB6 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void byte_write(uint8_t x)
{
	for(int i=0;i<8;i++)
		{
			HAL_GPIO_WritePin(GPIOB, arr1[i], ((x>>i)&0x01)?1:0);
		}
}

//void show_numbers(uint8_t number)
//{
//	if(number>9)
//	{
//		return;
//	}
//
//	HAL_GPIO_WritePin(GPIOB, 0x0001, led_array[number][0]);
//	HAL_GPIO_WritePin(GPIOB, 0x0002, led_array[number][1]);
//	HAL_GPIO_WritePin(GPIOB, 0x0004, led_array[number][2]);
//	HAL_GPIO_WritePin(GPIOB, 0x0008, led_array[number][3]);
//	HAL_GPIO_WritePin(GPIOB, 0x0010, led_array[number][4]);
//	HAL_GPIO_WritePin(GPIOB, 0x0020, led_array[number][5]);
//	HAL_GPIO_WritePin(GPIOB, 0x0040, led_array[number][6]);
//	HAL_GPIO_WritePin(GPIOB, 0x0080, led_array[number][7]);
//
//}
//
//void play_sound()
//{
//	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);
//}
///* USER CODE END 4 */

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
