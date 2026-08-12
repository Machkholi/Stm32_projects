/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include <stdint.h>
#include<string.h>
#include<stdlib.h>
#include<stdio.h>
#include<math.h>
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
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t rx_byte;

 uint8_t rx_buffer[32];

 uint8_t rx_index = 0;


 void DWIN_Send(uint8_t *data, uint16_t length)
 {
     HAL_UART_Transmit(&huart1,
                       data,
                       length,
                       100);
 }

 void DWIN_SetPage(uint16_t page)
 {
     uint8_t tx_buffer[10];


     tx_buffer[0] = 0x5A;
     tx_buffer[1] = 0xA5;

     tx_buffer[2] = 0x07;

     tx_buffer[3] = 0x82;

     tx_buffer[4] = 0x00;
     tx_buffer[5] = 0x84;

     tx_buffer[6] = 0x5A;

     tx_buffer[7] = 0x01;

     tx_buffer[8] = (uint8_t)(page >> 8);

     tx_buffer[9] = (uint8_t)(page & 0xFF);


     DWIN_Send(tx_buffer,
               sizeof(tx_buffer));
 }

 void DWIN_ProcessVP(uint16_t vp, uint16_t value)
 {
     switch (vp)
     {
         /* --------------------------------------------------------------
          * VP = 0x1000
          *
          * Key 0001 -> Page 1
          * Key 0002 -> Page 2
          * -------------------------------------------------------------- */

         case 0x1000:

             if (value == 0x0001)
             {
                 DWIN_SetPage(1);
             }
             else if (value == 0x0002)
             {
                 DWIN_SetPage(2);
             }

             break;


         /* --------------------------------------------------------------
          * VP = 0x1002
          *
          * Flow value
          * -------------------------------------------------------------- */

         case 0x1002:



             break;


         /* --------------------------------------------------------------
          * VP = 0x1004
          *
          * Pressure value
          * -------------------------------------------------------------- */

         case 0x1004:



             break;


         /* --------------------------------------------------------------
          * VP = 0x1006
          *
          * Oxygen value
          * -------------------------------------------------------------- */

         case 0x1006:



             break;


         /* --------------------------------------------------------------
          * Unknown VP
          * -------------------------------------------------------------- */

         default:

             break;
     }
 }

 void DWIN_ProcessFrame(uint8_t *data)
 {
     uint16_t vp;
     uint16_t value;


     /* ----------------------------------------------------------------------
      * Check DWIN header
      * ---------------------------------------------------------------------- */

     if (data[0] != 0x5A ||
         data[1] != 0xA5)
     {
         return;
     }


     /* ----------------------------------------------------------------------
      * Check command
      *
      * 0x83 = DWIN VP response
      * ---------------------------------------------------------------------- */

     if (data[3] != 0x83)
     {
         return;
     }


     /* ----------------------------------------------------------------------
      * Extract VP
      *
      * Example:
      *
      * data[4] = 10
      * data[5] = 00
      *
      * VP = 0x1000
      * ---------------------------------------------------------------------- */

     vp = ((uint16_t)data[4] << 8) |
           data[5];


     /* ----------------------------------------------------------------------
      * Extract value
      *
      * Example:
      *
      * data[7] = 00
      * data[8] = 01
      *
      * value = 0x0001
      * ---------------------------------------------------------------------- */

     value = ((uint16_t)data[7] << 8) |
              data[8];


     /* ----------------------------------------------------------------------
      * Send VP and value to application layer
      * ---------------------------------------------------------------------- */

     DWIN_ProcessVP(vp, value);
 }

 void  DWIN_ReceivePolling()
 {
	 /* --------------------------------------------------------------
	 	     * Wait for first header byte: 0x5A
	 	     * -------------------------------------------------------------- */

	 	    if (rx_index == 0)
	 	    {
	 	        if (HAL_UART_Receive(&huart1,
	 	                             &rx_byte,
	 	                             1,
	 	                             100) == HAL_OK)
	 	        {
	 	            if (rx_byte == 0x5A)
	 	            {
	 	                rx_buffer[rx_index++] = rx_byte;
	 	            }
	 	        }
	 	    }


	 	    /* --------------------------------------------------------------
	 	     * Wait for second header byte: 0xA5
	 	     * -------------------------------------------------------------- */

	 	    else if (rx_index == 1)
	 	    {
	 	        if (HAL_UART_Receive(&huart1,
	 	                             &rx_byte,
	 	                             1,
	 	                             100) == HAL_OK)
	 	        {
	 	            if (rx_byte == 0xA5)
	 	            {
	 	                rx_buffer[rx_index++] = rx_byte;
	 	            }
	 	            else
	 	            {
	 	                /* Wrong header - start again */

	 	                rx_index = 0;
	 	            }
	 	        }
	 	    }


	 	    /* --------------------------------------------------------------
	 	     * Receive the rest of the frame
	 	     * -------------------------------------------------------------- */

	 	    else
	 	    {
	 	        if (HAL_UART_Receive(&huart1,
	 	                             &rx_byte,
	 	                             1,
	 	                             100) == HAL_OK)
	 	        {
	 	            rx_buffer[rx_index++] = rx_byte;


	 	            /* ------------------------------------------------------
	 	             * rx_buffer[2] contains DWIN frame length.
	 	             *
	 	             * Total frame size = length + 3
	 	             *
	 	             * Example:
	 	             *
	 	             * 5A A5 06 83 10 00 01 00 01
	 	             *
	 	             * Length = 06
	 	             * Total = 06 + 3 = 9 bytes
	 	             * ------------------------------------------------------ */

	 	            if (rx_index >= 3)
	 	            {
	 	                uint8_t frame_length = rx_buffer[2];

	 	                uint8_t total_length = frame_length + 3;


	 	                if (rx_index >= total_length)
	 	                {
	 	                    DWIN_ProcessFrame(rx_buffer);

	 	                    rx_index = 0;
	 	                }
	 	            }


	 	            /* ------------------------------------------------------
	 	             * Safety
	 	             * ------------------------------------------------------ */

	 	            if (rx_index >= sizeof(rx_buffer))
	 	            {
	 	                rx_index = 0;
	 	            }
	 	        }
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
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  DWIN_SetPage(0);
  HAL_Delay(1000);
  DWIN_SetPage(1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  DWIN_ReceivePolling();



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
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

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
#ifdef USE_FULL_ASSERT
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
