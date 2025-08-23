/*
 * display.c
 *
 *  Created on: Jun 29, 2024
 *      Author: rahul
 */
#include "display.h"
#include "stm32f3xx_hal.h"
#include "stm32f3xx_hal_uart.h"
#include "stm32f3xx_hal_adc.h"

extern UART_HandleTypeDef huart1;
extern ADC_HandleTypeDef hadc1;

//extern uint16_t arr[];
extern void Error_Handler(void);
uint8_t show_value2[]={0x5a,0xa5,0x05,0x82,0x20,0x00,0x00,0x00};
uint8_t show_value1[]={0x5a,0xa5,0x05,0x82,0x20,0x10,0x00,0x00};
uint8_t led1_off[]=   {0x5a,0xa5,0x17,0x82,0x60,0x00,0x00,0x06,0x00,0x01,0x00,0x01,0x01,0x16,0x00,0x66,0x01,0x5e,0x00,0xa6,0x01,0x16,0x00,0x66,0xff,0x00};
uint8_t led1_on[]=    {0x5a,0xa5,0x17,0x82,0x60,0x00,0x00,0x06,0x00,0x00,0x00,0x01,0x01,0x16,0x00,0x66,0x01,0x5e,0x00,0xa6,0x01,0x16,0x00,0x66,0xff,0x00};
uint8_t led2_off[]=   {0x5a,0xa5,0x17,0x82,0x60,0x20,0x00,0x06,0x00,0x01,0x00,0x01,0x01,0x77,0x00,0x5e,0x01,0xbf,0x00,0xa3,0x01,0x77,0x00,0x5e,0xff,0x00};
uint8_t led2_on[]=    {0x5a,0xa5,0x17,0x82,0x60,0x20,0x00,0x06,0x00,0x00,0x00,0x01,0x01,0x77,0x00,0x5e,0x01,0xbf,0x00,0xa3,0x01,0x77,0x00,0x5e,0xff,0x00};
uint8_t animation_start[] =  {0x5a,0xa5,0x05,0x82,0x30,0x10,0x00,0x00};
uint8_t animation_stop[]  =  {0x5a,0xa5,0x05,0x82,0x30,0x10,0x00,0x01};
uint8_t adc_f=0;
uint16_t adc_val_new=0;
uint8_t rx_buffer[10];
uint32_t timer=0;
uint8_t adc_ready=0;
uint16_t adc_val[2];
uint8_t screen_change_command[]= {0x5a,0xa5,0x07,0x82,0x00,0x84,0x5a,0x01,0x00,0x01};
uint8_t data_ready=0;
uint8_t flagp=0;
uint8_t flagm=0;
uint8_t scale_adc_val[2];
uint32_t timer2=0;
uint32_t tim2=0;
uint8_t count=0;
uint32_t timee2=0;
uint8_t start_f=0;



void sendCommandToDwin(uint8_t *cmd,uint16_t length)
{
	HAL_UART_Transmit_IT(&huart1, cmd, length);
	HAL_Delay(5);
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
	HAL_UART_Receive_IT(&huart1, rx_buffer, 10);
}

void start_animation()
{
	sendCommandToDwin(animation_start,sizeof(animation_start));
}
void stop_animation()
{
	sendCommandToDwin(animation_stop,sizeof(animation_stop));
}
void off_led1()
{
	sendCommandToDwin(led1_off,sizeof(led1_off));
}
void on_led1()
{
	sendCommandToDwin(led1_on,sizeof(led1_on));
}

void off_led2()
{
	sendCommandToDwin(led2_off,sizeof(led2_off));
}
void on_led2()
{
	sendCommandToDwin(led2_on,sizeof(led2_on));
}


void page_change(uint8_t data)
{
	screen_change_command[9]=data;
	sendCommandToDwin(screen_change_command,sizeof(screen_change_command));
}

void process_data()
{
		if(((rx_buffer[7]==0x10)&&(rx_buffer[8]==0x00)) ||((rx_buffer[8]==0x10)&&(rx_buffer[9]==0x00)))
		{
			page_change(1);
		}
		else if(((rx_buffer[7]==0x10)&&(rx_buffer[8]==0x02)) ||((rx_buffer[8]==0x10)&&(rx_buffer[9]==0x02)))
		{
			page_change(0);
		}
		else if(((rx_buffer[7]==0x10)&&(rx_buffer[8]==0x03))||((rx_buffer[8]==0x10)&&(rx_buffer[9]==0x03)))
		{

		}
		else if(((rx_buffer[7]==0x00)&&(rx_buffer[8]==0x01))||((rx_buffer[8]==0x00)&&(rx_buffer[9]==0x01)))
		{
			start_f= !start_f;
			if(start_f)
			{
				start_animation();
			}
			else
			{
			stop_animation();
			}
		}

}
//		else if(((rx_buffer[7]==0x10)&&(rx_buffer[8]==0x05))||((rx_buffer[8]==0x10)&&(rx_buffer[9]==0x05)))
//		{
//			start_f= !start_f;
//			if(start_f)
//			{
//				start_animation();
//			}
//			else
//			{
//				stop_animation();
//				show_value1[7]=0;
//
//				if(HAL_GetTick()-tim2>100)
//				{
//				sendCommandToDwin(show_value1,sizeof(show_value1));
//				tim2=HAL_GetTick();
//				}
//			}
//		}



void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	__HAL_UART_DISABLE_IT(&huart1, UART_IT_RXNE);
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance==USART1)
	{
data_ready=1;
	}
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    	adc_val[count]=HAL_ADC_GetValue(&hadc1);
    	adc_f=1;
}
void adc_value(uint8_t counter)
{
	scale_adc_val[counter]=(adc_val[counter]*100)/4095;

	show_value1[7]=scale_adc_val[0];
	show_value2[7]=scale_adc_val[1];

	if(HAL_GetTick()-tim2>100)
	{
	sendCommandToDwin(show_value2,sizeof(show_value2));
	sendCommandToDwin(show_value1,sizeof(show_value1));
	tim2=HAL_GetTick();
	}
}

void channel_select(uint8_t channel)
{
	ADC_ChannelConfTypeDef sConfig = {0};

	switch(channel)
	{
	case 0:
		sConfig.Channel = ADC_CHANNEL_1;
		break;
	case 1:
		sConfig.Channel = ADC_CHANNEL_2;
		break;
	}
//	  sConfig.Channel = channel;
	  sConfig.Rank = ADC_REGULAR_RANK_1;
	  sConfig.SingleDiff = ADC_SINGLE_ENDED;
	  sConfig.SamplingTime = ADC_SAMPLETIME_601CYCLES_5;
	  sConfig.OffsetNumber = ADC_OFFSET_NONE;
	  sConfig.Offset = 0;
	  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }
}

void uart_func()
{
	if(data_ready)
	{
		process_data();
		HAL_Delay(10);
		data_ready=0;
	}
}

void adc_func()
{
	if(adc_f)
	{
		adc_value(count);
		count++;
		if(count>=2)
		{
			count=0;
		}
		adc_f=0;
		channel_select(count);
		HAL_Delay(10);
		HAL_ADC_Start_IT(&hadc1);
	}
}

void blink_led1()
{
	if(HAL_GetTick()-timer2>200)
	{
		flagp= !flagp;
		if(flagp)
		{
			on_led1();
		}
		else
		{
			off_led1();
		}
		timer2=HAL_GetTick();
	}
}

void blink_led2()
{
	if(HAL_GetTick()-timee2>800)
	{
		flagm= !flagm;
		if(flagm)
		{
			on_led2();
		}
		else
		{
			off_led2();
		}
		timee2=HAL_GetTick();
	}
}


