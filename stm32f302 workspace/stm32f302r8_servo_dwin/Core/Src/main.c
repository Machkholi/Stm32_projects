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
uint8_t screen_change_command[]={0x5A, 0xA5, 0x07, 0x82, 0x00, 0x84, 0x5A, 0x01, 0x00, 0x00};
uint8_t ack_cmd[]=               {0x5a,0xa5,0x03,0x82,0x4f,0x4b};
uint8_t show_value[]=            {0x5a,0xa5,0x05,0x82,0x20,0x00,0x00,0x00};

/////////////////////////////////////////basic graphic command/////////////////////
uint8_t show_30[]=    {0x5a,0xa5,0x17,0x82,0x50,0x00,0x00,0x06,0x00,0x01,0x00,0x02,0x01,0x20,0x00,0x31,0x01,0x68,0x00,0x6a,0x01,0x20,0x00,0x31,0xff,0x00};
uint8_t hide_30[]=    {0x5a,0xa5,0x17,0x82,0x50,0x00,0x00,0x06,0x00,0x00,0x00,0x02,0x01,0x20,0x00,0x31,0x01,0x68,0x00,0x6a,0x01,0x20,0x00,0x31,0xff,0x00};
uint8_t show_60[]=    {0x5a,0xa5,0x17,0x82,0x50,0x10,0x00,0x06,0x00,0x01,0x00,0x02,0x01,0x81,0x00,0x31,0x01,0xc9,0x00,0x6a,0x01,0x81,0x00,0x31,0xff,0x00};
uint8_t hide_60[]=    {0x5a,0xa5,0x17,0x82,0x50,0x10,0x00,0x06,0x00,0x00,0x00,0x02,0x01,0x81,0x00,0x31,0x01,0xc9,0x00,0x6a,0x01,0x81,0x00,0x31,0xff,0x00};
uint8_t show_90[]=    {0x5a,0xa5,0x17,0x82,0x50,0x20,0x00,0x06,0x00,0x01,0x00,0x02,0x01,0x20,0x00,0x6d,0x01,0x68,0x00,0xa6,0x01,0x20,0x00,0x6d,0xff,0x00};
uint8_t hide_90[]=    {0x5a,0xa5,0x17,0x82,0x50,0x20,0x00,0x06,0x00,0x00,0x00,0x02,0x01,0x20,0x00,0x6d,0x01,0x68,0x00,0xa6,0x01,0x20,0x00,0x6d,0xff,0x00};
uint8_t show_120[]=   {0x5a,0xa5,0x17,0x82,0x50,0x30,0x00,0x06,0x00,0x01,0x00,0x02,0x01,0x7f,0x00,0x6d,0x01,0xc7,0x00,0xa6,0x01,0x7f,0x00,0x6d,0xff,0x00};
uint8_t hide_120[]=   {0x5a,0xa5,0x17,0x82,0x50,0x30,0x00,0x06,0x00,0x00,0x00,0x02,0x01,0x20,0x00,0xa9,0x01,0x68,0x00,0xe2,0x01,0x20,0x00,0xa9,0xff,0x00};
uint8_t show_150[]=   {0x5a,0xa5,0x17,0x82,0x50,0x40,0x00,0x06,0x00,0x01,0x00,0x02,0x01,0x20,0x00,0xa9,0x01,0x68,0x00,0xe2,0x01,0x20,0x00,0xa9,0xff,0x00};
uint8_t hide_150[]=   {0x5a,0xa5,0x17,0x82,0x50,0x40,0x00,0x06,0x00,0x00,0x00,0x02,0x01,0x20,0x00,0xa9,0x01,0x68,0x00,0xe2,0x01,0x20,0x00,0xa9,0xff,0x00};
uint8_t show_180[]=   {0x5a,0xa5,0x17,0x82,0x50,0x50,0x00,0x06,0x00,0x01,0x00,0x02,0x01,0x81,0x00,0xa9,0x03,0x46,0x00,0xe2,0x01,0x81,0x00,0xa9,0xff,0x00};
uint8_t hide_180[]=   {0x5a,0xa5,0x17,0x82,0x50,0x50,0x00,0x06,0x00,0x00,0x00,0x02,0x01,0x81,0x00,0xa9,0x03,0x46,0x00,0xe2,0x01,0x81,0x00,0xa9,0xff,0x00};
uint8_t show_0[]=     {0x5a,0xa5,0x17,0x82,0x50,0x60,0x00,0x06,0x00,0x01,0x00,0x02,0x01,0x4c,0x00,0xe5,0x01,0x9a,0x01,0x04,0x01,0x4c,0x00,0xe5,0xff,0x00};
uint8_t hide_0[]=     {0x5a,0xa5,0x17,0x82,0x50,0x60,0x00,0x06,0x00,0x00,0x00,0x02,0x01,0x4c,0x00,0xe5,0x01,0x9a,0x01,0x04,0x01,0x4c,0x00,0xe5,0xff,0x00};




uint8_t rxbuffer[10];
uint8_t flag=0;
uint8_t flag2=0;
uint8_t i=25;
uint8_t j=125;
uint8_t VALUE=0;
uint16_t length=0;
uint32_t startTime=0;
volatile uint16_t adc_val=0;
char adc_buffer[9];
uint8_t adc_f=0;
uint32_t timer=0;
uint8_t scale_adc_val=0;
uint32_t tim2=0;
uint8_t count=0;
uint16_t angle=0;
uint16_t number=0;
uint8_t new_value=0;

volatile uint8_t data_ready=0;
#define buffer_size 16


volatile uint8_t value=0;

uint32_t time1=0;
uint32_t time2=0;
uint8_t increasing=0;


void sendCommandToDwin(uint8_t *cmd,uint16_t length)
{
	HAL_UART_Transmit_IT(&huart1, cmd, length);
	HAL_Delay(5);


	__HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
//	HAL_Delay(20);
	HAL_UART_Receive_IT(&huart1, rxbuffer, 10);
//memset(rxbuffer,0x00,9);

}
void press_30()
{
	sendCommandToDwin(show_30,sizeof(show_30));
}

void release_30()
{
	sendCommandToDwin(hide_30,sizeof(hide_30));
}

void press_60()
{
	sendCommandToDwin(show_60,sizeof(show_60));
}
void press_90()
{
	sendCommandToDwin(show_90,sizeof(show_90));
}

void press_120()
{
	sendCommandToDwin(show_120,sizeof(show_120));
}
void press_150()
{
	sendCommandToDwin(show_150,sizeof(show_150));
}
void press_180()
{
	sendCommandToDwin(show_180,sizeof(show_180));
}
void press_0()
{
	sendCommandToDwin(show_0,sizeof(show_0));
}
void release_60()
{
	sendCommandToDwin(hide_60,sizeof(hide_60));
}

void release_90()
{
	sendCommandToDwin(hide_90,sizeof(hide_90));
}

void release_120()
{
	sendCommandToDwin(hide_120,sizeof(hide_120));
}
void release_150()
{
	sendCommandToDwin(hide_150,sizeof(hide_150));
}
void release_180()
{
	sendCommandToDwin(hide_180,sizeof(hide_180));
}
void release_0()
{
	sendCommandToDwin(hide_0,sizeof(hide_0));
}

void page_change(uint8_t page_no)
{

	screen_change_command[9]=page_no;

		sendCommandToDwin(screen_change_command,sizeof(screen_change_command));
}



void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance ==USART1)
	{
	    __HAL_UART_DISABLE_IT(&huart1, UART_IT_RXNE);


	}
}

void servo_value(uint16_t value)
{

	show_value[7]=(uint8_t)value;
	if(HAL_GetTick()-tim2>50)
	{
	sendCommandToDwin(show_value,sizeof(show_value));
	tim2=HAL_GetTick();
	}
}
void button_func()
{
	switch(value)
	{
	case 25:
		press_0();
		release_30();
		release_60();
		release_90();
		release_120();
		release_150();
		release_180();
		HAL_Delay(10);
		servo_value(0);
		break;

	case 42:
		release_0();
		press_30();
		release_60();
		release_90();
		release_120();
		release_150();
		release_180();
		servo_value(30);
		HAL_Delay(10);
		break;
	case 58:
		release_0();
		release_30();
		press_60();
		release_90();
		release_120();
		release_150();
		release_180();
		servo_value(60);
		HAL_Delay(10);
		break;
	case 75:
		release_0();
		release_30();
		release_60();
		press_90();
		release_120();
		release_150();
		release_180();
		servo_value(90);
		break;
	case 91:
		release_0();
		release_30();
		release_60();
		release_90();
		press_120();
		release_150();
		release_180();
		servo_value(120);
		break;
	case 108:
		release_0();
		release_30();
		release_60();
		release_90();
		release_120();
		press_150();
		release_180();
		servo_value(150);
		break;
	case 125:
		release_0();
		release_30();
		release_60();
		release_90();
		release_120();
		release_150();
		press_180();
		servo_value(180);
		break;

	}
}

//void display_value(uint8_t arr[],uint16_t value)
//{
//	arr[7]=(uint8_t)value;
//	sendCommandToDwin(show_value,sizeof(show_value));
//}

void process_received_data(uint8_t *data)
{

			if(((rxbuffer[7]==0x10)&&(rxbuffer[8]==0x00))||((rxbuffer[8]==0x10)&&(rxbuffer[9]==0x00)))
			{


			}
			else if(((rxbuffer[7]==0x10)&&(rxbuffer[8]==0x01))||((rxbuffer[8]==0x10)&&(rxbuffer[9]==0x01)))
			{

value=25;
button_func();

			}
			else if(((rxbuffer[7]==0x10)&&(rxbuffer[8]==0x02))||((rxbuffer[8]==0x10)&&(rxbuffer[9]==0x02)))
			{
value=42;
button_func();

			}
				else if(((rxbuffer[7]==0x10)&&(rxbuffer[8]==0x03))||((rxbuffer[8]==0x10)&&(rxbuffer[9]==0x03)))
				{

value=58;
button_func();


				}
				else if(((rxbuffer[7]==0x10)&&(rxbuffer[8]==0x04))||((rxbuffer[8]==0x10)&&(rxbuffer[9]==0x04)))
				{
					value=75;

					button_func();


				}
				else if(((rxbuffer[7]==0x10)&&(rxbuffer[8]==0x05))||((rxbuffer[8]==0x10)&&(rxbuffer[9]==0x05)))
				{
					value=91;
					button_func();

				}
				else if(((rxbuffer[7]==0x10)&&(rxbuffer[8]==0x06))||((rxbuffer[8]==0x10)&&(rxbuffer[9]==0x06)))
				{
					value=108;
					button_func();


				}
				else if(((rxbuffer[7]==0x10)&&(rxbuffer[8]==0x07))||((rxbuffer[8]==0x10)&&(rxbuffer[9]==0x07)))
				{
					value=125;
					button_func();


				}


}



void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
 if(huart->Instance==USART1)
 {
			 data_ready=1;
			 HAL_UART_Receive_IT(&huart1, rxbuffer, 9);
 }
}

uint32_t MAP(uint32_t au32_IN, uint32_t au32_INmin, uint32_t au32_INmax, uint32_t au32_OUTmin, uint32_t au32_OUTmax)
{
    return ((((au32_IN - au32_INmin)*(au32_OUTmax - au32_OUTmin))/(au32_INmax - au32_INmin)) + au32_OUTmin);
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{

flag=1;
}

void increment_servo()
{
	if((rxbuffer[5]==0x20)&&(rxbuffer[6]==0x00))
	{
	  	  if(HAL_GetTick()-time1>10)
	  	  {
number=rxbuffer[9];
new_value=MAP(number,0,180,25,125);
	  	  TIM2->CCR1=new_value;

	  	  time1=HAL_GetTick();
	  	  }
	}
//	if((rxbuffer[4]==0x20)&&(rxbuffer[5]==0x00))
//	{
//	  	  if(HAL_GetTick()-time1>10)
//	  	  {
//
//	  		number=rxbuffer[8];
//	  		new_value=MAP(number,0,180,25,125);
//	  			  	  TIM2->CCR1=new_value;
//
//	  	  time1=HAL_GetTick();
//	  	  }
//	}
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
  HAL_TIM_PWM_Start_IT(&htim2, TIM_CHANNEL_1);
  HAL_UART_Receive_IT(&huart1, rxbuffer, 9);
  page_change(1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if(data_ready)
	  	  {
	  	  	HAL_Delay(10);

	  	  	 process_received_data(rxbuffer);
	  	  data_ready=0;
	  count++;

	  	  }

	  	  if(flag)
	  	  {
	  		  switch (value)
	  		  {
	  		  case 25:

	  				  		  	  if(HAL_GetTick()-time1>20)
	  				  		  	  {
	  				  		  		  if(increasing)
	  				  		  		  {
	  				  		  			  if(i<118)
	  				  		  			  {
	  				  		  				  i++;
	  							  		  	  TIM2->CCR1=i;
	  					  				  		  	  angle=MAP(i,25,125,0,180);
	  					  				  		  	  servo_value(angle);
	  				  		  			  }
	  				  		  			  else
	  				  		  			  {
	  				  		  				  increasing=0;
	  				  		  			  }
	  				  		  		  }
	  				  		  		  else
	  				  		  		  {
	  				  		  			  if(i>25)
	  				  		  			  {
	  				  		  				  i--;
	  							  		  	  TIM2->CCR1=i;
	  					  				  		  	  angle=MAP(i,25,125,0,180);
	  					  				  		  	  servo_value(angle);
	  				  		  			  }
	  				  		  			  else
	  				  		  			  {
	  				  		  				  increasing=1;
	  				  		  			  }
	  				  		  		  }

	  				  		  	  time1=HAL_GetTick();
	  				  		  	  }

	  		  	  break;
	  		  case 42:
	  		  	  if(HAL_GetTick()-time1>100)
	  		  	  {

	  		  	  TIM2->CCR1=42;

	  		  	  time1=HAL_GetTick();
	  		  	  }
	  		  	  break;
	  		  case 58:
	  		  	  if(HAL_GetTick()-time1>100)
	  		  	  {

	  		  	  TIM2->CCR1=58;

	  		  	  time1=HAL_GetTick();
	  		  	  }
	  		  	  break;

	  		  case 75:
	  		  	  if(HAL_GetTick()-time1>100)
	  		  	  {

	  		  	  TIM2->CCR1=75;

	  		  	  time1=HAL_GetTick();
	  		  	  }
	  		  	  break;

	  		  case 91:
	  		  	  if(HAL_GetTick()-time1>100)
	  		  	  {

	  		  	  TIM2->CCR1=91;

	  		  	  time1=HAL_GetTick();
	  		  	  }
	  		  	  break;
	  		  case 108:
	  		  	  if(HAL_GetTick()-time1>100)
	  		  	  {

	  		  	  TIM2->CCR1=108;

	  		  	  time1=HAL_GetTick();
	  		  	  }
	  		  	  break;
	  		  case 125:
	  		  	  if(HAL_GetTick()-time1>100)
	  		  	  {

	  		  	  TIM2->CCR1=124;

	  		  	  time1=HAL_GetTick();
	  		  	  }
	  		  	  break;

	  		  }

	  		  increment_servo();



	  	  flag=0;
	  	  }

	  	  if(HAL_GetTick()-tim2>500)
	  	  {
	  	  	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_13);
	  	  	tim2=HAL_GetTick();
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL11;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 880-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
