/*
 * display.h
 *
 *  Created on: Jun 29, 2024
 *      Author: rahul
 */

#ifndef INC_DISPLAY_H_
#define INC_DISPLAY_H_

#include<stdint.h>

extern uint8_t rx_buffer[10];
extern uint8_t flagm;

void off_led1();
void on_led1();
void off_led2();
void on_led2();
void sendCommandToDwin(uint8_t *cmd,uint16_t length);
void page_change(uint8_t data);
void process_data();
void channel_select(uint8_t channel);
void adc_value(uint8_t counter);
void uart_func();
void adc_func();
void blink_led1();
void blink_led2();
void start_animation();
void stop_animation();

#endif /* INC_DISPLAY_H_ */
