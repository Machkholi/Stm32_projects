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
#include<string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define RX_BUFFER_SIZE 256
uint8_t rxBuffer[RX_BUFFER_SIZE];
volatile uint8_t rxIndex = 0;
volatile uint8_t newDataReceived = 0;

typedef enum {
    STATE_IDLE,
    STATE_RECEIVE_TOUCH,
    STATE_SEND_PAGE_CHANGE,
    STATE_WAIT_FOR_ACK
} State;

volatile State currentState = STATE_IDLE;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
void processTouchInput(uint8_t* data, uint16_t length);
void sendPageChangeCommand(void);
void processAck(void);
void stateMachine(void);
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
uint8_t rx_buffer[20];
uint8_t screen_change_command[]= {0x5a,0xa5,0x07,0x82,0x00,0x84,0x5a,0x01,0x00,0x01};
uint8_t data_ready=0;
uint8_t graphic[]=               {0x5A,0xA5,0x17,0x82,0x50,0x00,0x00,0x06,0x00,0x01,0x00,0x00,0x00,0x90,0x00,0xb6,0x00,0xd8,0x00,0xf6,0x00,0x90,0x00,0xf6,0xff,0x00};
uint8_t msg[] = "hello";
uint8_t count=0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void sendCommandToDwin(uint8_t *cmd,uint16_t length)
{
	HAL_UART_Transmit(&huart1, cmd, length, HAL_MAX_DELAY);
//	HAL_Delay(50);
//	HAL_UART_Receive_IT(&huart1, rx_buffer, 9);

}
void page_change(uint8_t data)
{
	screen_change_command[9]=data;
	sendCommandToDwin(screen_change_command,sizeof(screen_change_command));
}

void  read_display()
{

	HAL_UART_Receive(&huart1, rx_buffer, 9, HAL_MAX_DELAY);


		if((rx_buffer[7]==0x10)&&(rx_buffer[8]==0x00))
		{
			page_change(1);
			HAL_Delay(10);
			memset(rx_buffer,0,9);
		}
		else if((rx_buffer[7]==0x10)&&(rx_buffer[8]==0x02))
		{
			page_change(0);
			HAL_Delay(10);
//			memset(rx_buffer,0,9);
		}
//	return 0;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance==USART1)
	{
        uint8_t receivedByte = (uint8_t)(huart->Instance->RDR & 0xFF);

        // Store the received byte in the buffer
        rxBuffer[rxIndex++] = receivedByte;
        newDataReceived = 1;

        // Wrap the buffer index if necessary
        if (rxIndex >= RX_BUFFER_SIZE)
        {
            rxIndex = 0;
        }
	}
}
void stateMachine(void)
{
    static uint8_t touchData[9];
    static uint8_t ackData[6];
    static uint8_t touchIndex = 0;
    static uint8_t ackIndex = 0;

    newDataReceived = 0;

    switch (currentState)
    {
    case STATE_IDLE:
        currentState = STATE_RECEIVE_TOUCH;
        break;

    case STATE_RECEIVE_TOUCH:
        touchData[touchIndex++] = rxBuffer[rxIndex - 1];

        if (touchIndex >= 9)
        {
            touchIndex = 0;
            processTouchInput(touchData, 9);
            currentState = STATE_SEND_PAGE_CHANGE;
        }
        break;

    case STATE_SEND_PAGE_CHANGE:
//        sendPageChangeCommand();
    	//sendCommandToDwin
        currentState = STATE_WAIT_FOR_ACK;
        break;

    case STATE_WAIT_FOR_ACK:
        ackData[ackIndex++] = rxBuffer[rxIndex - 1];

        if (ackIndex >= 6)
        {
            ackIndex = 0;
            processAck();
            currentState = STATE_IDLE;
        }
        break;
    }
}

void processTouchInput(uint8_t* data, uint16_t length)
{
    // Extract touch input and decide on the page change command
    // For example, let's assume data[2] contains the key code
//    uint8_t keyCode = data[2];
	if((data[7]==0x10)&&(data[8]==0x00))
	{
		page_change(1);
	}
	else if((data[7]==0x10)&&(data[8]==0x02))
	{
		page_change(0);
	}
    // Process the keyCode to decide on the page change
    // ...
}

void sendPageChangeCommand(void)
{
    uint8_t pageChangeCmd[] = { /* Your page change command bytes */ };
    HAL_UART_Transmit(&huart1, pageChangeCmd, sizeof(pageChangeCmd), HAL_MAX_DELAY);
}

void processAck(void)
{
    // Process the acknowledgment if necessary, or simply ignore
}

//void scan_display()
//{
//	uint8_t touched=read_display();
//	switch(touched)
//	{
//	case
//	}
//}
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
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
HAL_UART_Receive_IT(&huart1, rx_buffer, 9);
//  __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      if (newDataReceived)
      {
          stateMachine();
      }


//		HAL_UART_Receive(&huart1, rx_buffer, 9, HAL_MAX_DELAY);
//
//
//			if((rx_buffer[7]==0x10)&&(rx_buffer[8]==0x00))
//			{
//				page_change(1);
//				HAL_Delay(10);
////				memset(rx_buffer,0,9);
//			}
//			else if((rx_buffer[7]==0x10)&&(rx_buffer[8]==0x02))
//			{
//				page_change(0);
//				HAL_Delay(10);
//	//			memset(rx_buffer,0,9);
//			}
//	  scan_display();
//	  read_display();
//		if((rx_buffer[7]==0x10)&&(rx_buffer[8]==0x00))
//			{
//				page_change(1);
//				HAL_Delay(10);
//	//			memset(rx_buffer,0,9);
//			}
//			else if((rx_buffer[7]==0x10)&&(rx_buffer[8]==0x02))
//			{
//				page_change(0);
//				HAL_Delay(10);
//	//			memset(rx_buffer,0,9);
//			}
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_SYSCLK;
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
  huart2.Init.BaudRate = 38400;
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
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 38400;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PB13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
