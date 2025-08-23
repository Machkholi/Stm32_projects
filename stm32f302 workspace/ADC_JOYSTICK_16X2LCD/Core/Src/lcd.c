///*
// * lcd.c
// *
// *  Created on: Jun 10, 2024
// *      Author: rahul
//*/
//
////void byte_write(uint8_t x)                                      for 8 bit lcd
////{
////	pinconfig y=(pinconfig)x;
////	HAL_GPIO_WritePin(GPIOB, D0_Pin , y.b0);
////	HAL_GPIO_WritePin(GPIOB, D1_Pin, y.b1);
////	HAL_GPIO_WritePin(GPIOB, D2_Pin, y.b2);
////	HAL_GPIO_WritePin(GPIOB, D3_Pin, y.b3);
////	HAL_GPIO_WritePin(GPIOB, D4_Pin , y.b4);
////	HAL_GPIO_WritePin(GPIOB, D5_Pin, y.b5);
////	HAL_GPIO_WritePin(GPIOB, D6_Pin, y.b6);
////	HAL_GPIO_WritePin(GPIOB, D7_Pin, y.b7);
////
////}
//
//void high_nibble_write(uint8_t x)
//{
//	    pinconfig y= (pinconfig)x;
//		HAL_GPIO_WritePin(GPIOB, D4_Pin , y.b4);
//		HAL_GPIO_WritePin(GPIOB, D5_Pin, y.b5);
//		HAL_GPIO_WritePin(GPIOB, D6_Pin, y.b6);
//		HAL_GPIO_WritePin(GPIOB, D7_Pin, y.b7);
//}
//
//void low_nibble_write(uint8_t x)
//{
//	pinconfig y = (pinconfig)x;
//	HAL_GPIO_WritePin(GPIOB, D4_Pin, y.b0);
//	HAL_GPIO_WritePin(GPIOB, D5_Pin, y.b1);
//	HAL_GPIO_WritePin(GPIOB, D6_Pin, y.b2);
//	HAL_GPIO_WritePin(GPIOB, D7_Pin, y.b3);
//}
//
//void lcd_data(uint8_t data)
//{
//	high_nibble_write(data);
//	HAL_GPIO_WritePin(RS_GPIO_Port, RS_Pin, GPIO_PIN_SET);
//	HAL_GPIO_WritePin(RW_GPIO_Port, RW_Pin, GPIO_PIN_RESET);
//	HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, GPIO_PIN_SET);
//	HAL_Delay(5);
//	HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, GPIO_PIN_RESET);
//	HAL_Delay(5);
//
//	low_nibble_write(data);
//	HAL_GPIO_WritePin(RS_GPIO_Port, RS_Pin, GPIO_PIN_SET);
//	HAL_GPIO_WritePin(RW_GPIO_Port, RW_Pin, GPIO_PIN_RESET);
//	HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, GPIO_PIN_SET);
//	HAL_Delay(5);
//	HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, GPIO_PIN_RESET);
//	HAL_Delay(5);
//}
//
//void lcd_cmd(uint8_t cmd)
//{
//	high_nibble_write(cmd);
//	HAL_GPIO_WritePin(RS_GPIO_Port, RS_Pin, GPIO_PIN_RESET);
//	HAL_GPIO_WritePin(RW_GPIO_Port, RW_Pin, GPIO_PIN_RESET);
//	HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, GPIO_PIN_SET);
//	HAL_Delay(5);
//	HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, GPIO_PIN_RESET);
//	HAL_Delay(5);
//
//	low_nibble_write(cmd);
//	HAL_GPIO_WritePin(RS_GPIO_Port, RS_Pin, GPIO_PIN_RESET);
//	HAL_GPIO_WritePin(RW_GPIO_Port, RW_Pin, GPIO_PIN_RESET);
//	HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, GPIO_PIN_SET);
//	HAL_Delay(5);
//	HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, GPIO_PIN_RESET);
//	HAL_Delay(5);
//}
//
//void lcd_string(uint8_t *ptr)
//{
//	while(*ptr!='\0')
//	{
//		lcd_data(*ptr++);
//	}
//}
//
//void adc_select_ch1()
//{
//	ADC_ChannelConfTypeDef sConfig = {0};
//	  sConfig.Channel = ADC_CHANNEL_1;
//	  sConfig.Rank = 1;
//	  sConfig.SingleDiff = ADC_SINGLE_ENDED;
//	  sConfig.SamplingTime = ADC_SAMPLETIME_601CYCLES_5;
//	  sConfig.OffsetNumber = ADC_OFFSET_NONE;
//	  sConfig.Offset = 0;
//	  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
//	  {
//	    Error_Handler();
//	  }
//	  HAL_ADC_Start(&hadc1);
//	  if(HAL_ADC_PollForConversion(&hadc1, 100)==HAL_OK)
//	  {
//		  xval=HAL_ADC_GetValue(&hadc1);
//	  }
//	  HAL_ADC_Stop(&hadc1);
//	  HAL_Delay(100);
//}
//
//void adc_select_ch2()
//{
//	ADC_ChannelConfTypeDef sConfig = {0};
//	  sConfig.Channel = ADC_CHANNEL_2;
//	  sConfig.Rank = 1;
//	  sConfig.SingleDiff = ADC_SINGLE_ENDED;
//	  sConfig.SamplingTime = ADC_SAMPLETIME_601CYCLES_5;
//	  sConfig.OffsetNumber = ADC_OFFSET_NONE;
//	  sConfig.Offset = 0;
//	  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
//	  {
//	    Error_Handler();
//	  }
//	  HAL_ADC_Start(&hadc1);
//	  if(HAL_ADC_PollForConversion(&hadc1, 100)==HAL_OK)
//	  {
//		  yval=HAL_ADC_GetValue(&hadc1);
//	  }
//	  HAL_ADC_Stop(&hadc1);
//	  HAL_Delay(100);
//}

