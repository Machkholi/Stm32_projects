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
#define DWIN_RX_BUFFER_SIZE  64
#define DWIN_HEADER_1        0x5A
#define DWIN_HEADER_2        0xA5
static 	uint8_t rx_buffer[DWIN_RX_BUFFER_SIZE];
static 	uint8_t dwin_rx_copy[DWIN_RX_BUFFER_SIZE];
volatile uint8_t dwin_frame_ready = 0;
volatile uint16_t dwin_rx_length = 0;
volatile uint16_t vp_address = 0;
volatile uint16_t received_value = 0;
volatile uint8_t  vp_updated = 0;
static uint8_t tx_buffer[32];
uint16_t current_value=0;

static const uint16_t data_send_command_flow             = 0x5110;
static const uint16_t data_send_command_required_flow    = 0x5120;



void DWIN_Send(uint8_t *data, uint16_t len)
{
	//    if (huart6.gState == HAL_UART_STATE_READY)
	//    {


//	        HAL_UART_Transmit_DMA(&huart6, data, len);
	//    }
		HAL_UART_Transmit(&huart1, data, len, HAL_MAX_DELAY);
}

void DWIN_WriteVP(uint16_t vp, uint16_t value)
{
    tx_buffer[0] = 0x5A;
    tx_buffer[1] = 0xA5;
    tx_buffer[2] = 0x05;
    tx_buffer[3] = 0x82;
    tx_buffer[4] = (vp >> 8);
    tx_buffer[5] = (vp & 0xFF);
    tx_buffer[6] = (value >> 8);
    tx_buffer[7] = (value & 0xFF);

    DWIN_Send(tx_buffer, 8);
}

void dwin_page_change(int page_no)
{
    tx_buffer[0] = 0x5A;
    tx_buffer[1] = 0xA5;
    tx_buffer[2] = 0x07;
    tx_buffer[3] = 0x82;
    tx_buffer[4] = 0x00;
    tx_buffer[5] = 0x84;
    tx_buffer[6] = 0x5A;
    tx_buffer[7] = 0x01;
    tx_buffer[8] = (page_no >> 8);
    tx_buffer[9] = (page_no & 0xFF);

    DWIN_Send(tx_buffer, 10);
}

void dwin_copy_image(
    uint16_t vp,
    uint16_t image_id,
    uint16_t page,
    uint16_t layer,
    uint16_t src_x,
    uint16_t src_y,
    uint16_t width,
    uint16_t height,
    uint16_t dst_x,
    uint16_t dst_y
)
{
    static uint8_t frame[26];

    frame[0]  = 0x5A;
    frame[1]  = 0xA5;
    frame[2]  = 0x17;        // Data length
    frame[3]  = 0x82;        // Write VP

    frame[4]  = vp >> 8;
    frame[5]  = vp & 0xFF;

    frame[6]  = image_id >> 8;
    frame[7]  = image_id & 0xFF;

    frame[8]  = page >> 8;
    frame[9]  = page & 0xFF;

    frame[10] = layer >> 8;
    frame[11] = layer & 0xFF;


    frame[12] = src_x >> 8;
    frame[13] = src_x & 0xFF;

    frame[14] = src_y >> 8;
    frame[15] = src_y & 0xFF;


    frame[16] = width >> 8;
    frame[17] = width & 0xFF;

    frame[18] = height >> 8;
    frame[19] = height & 0xFF;


    frame[20] = dst_x >> 8;
    frame[21] = dst_x & 0xFF;

    frame[22] = dst_y >> 8;
    frame[23] = dst_y & 0xFF;

    frame[24] = 0xFF;
    frame[25] = 0x00;

    DWIN_Send(frame, sizeof(frame));
}

void dwin_set_flow(uint16_t vp,uint16_t data_update)
{
	DWIN_WriteVP(vp, data_update);
}
void battery_one_level()
{
	dwin_copy_image(0x6020,0x0006,0x0001,0x000b,0x048e,0x019a,0x04b5,0x01e1,0x04a9,0x025e);
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart,
                                uint16_t Size)
{
    if (huart->Instance == USART1)
    {
        memcpy(dwin_rx_copy, rx_buffer, Size);

        dwin_rx_length = Size;
        dwin_frame_ready = 1;

        HAL_UARTEx_ReceiveToIdle_DMA(&huart1,rx_buffer,DWIN_RX_BUFFER_SIZE);
        __HAL_DMA_DISABLE_IT(&hdma_usart1_rx,DMA_IT_HT);
    }
}

void DWIN_ParseFrame(uint8_t *data, uint16_t len)
{
    for (uint16_t i = 0; i + 3 < len; i++)
    {
        if (data[i] == 0x5A && data[i+1] == 0xA5)
        {
            uint8_t payload_len = data[i+2];
            uint16_t frame_len = payload_len + 3;

            if (i + frame_len > len)
                break;  // incomplete frame

            uint8_t cmd = data[i+3];

            if (cmd == 0x83 && payload_len >= 5)
            {
                uint16_t vp =
                    (data[i+4] << 8) |
                     data[i+5];

                uint16_t value =
                    (data[i+frame_len-2] << 8) |
                     data[i+frame_len-1];

                vp_address   = vp;
                received_value = value;
                vp_updated = 1;
            }
            i += frame_len - 1;  // jump to next frame
        }
    }
}

void display_control(void)
{
    static uint32_t last_touch_time = 0;

    if(!vp_updated)
          return;

     vp_updated = 0;

//     if(HAL_GetTick() - last_touch_time < 150)
//         return;
//
//     last_touch_time = HAL_GetTick();

        switch (vp_address)
        {
            case 0x1000:
            {
            	switch(received_value)
            	{
            	case 0x0001:

            		break;

            	case 0x0002:

            		break;
            	}
            	break;

            }

            case 0x2000:
            {
            	current_value=received_value;     //for values
            	break;
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

  HAL_UARTEx_ReceiveToIdle_DMA(&huart1,rx_buffer,DWIN_RX_BUFFER_SIZE);
  __HAL_DMA_DISABLE_IT(&hdma_usart1_rx,DMA_IT_HT);

  dwin_page_change(0);
  HAL_Delay(1000);
  dwin_page_change(9);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  if (dwin_frame_ready)
	  {
	      dwin_frame_ready = 0;
	      DWIN_ParseFrame(dwin_rx_copy, dwin_rx_length);
	  }

	  display_control();
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
