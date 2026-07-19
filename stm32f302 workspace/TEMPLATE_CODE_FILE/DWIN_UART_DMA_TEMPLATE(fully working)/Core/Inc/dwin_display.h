#ifndef INC_DWIN_DISPLAY_H_
#define INC_DWIN_DISPLAY_H_

#include "main.h"      // gives UART_HandleTypeDef and HAL macros/types
#include <stdint.h>    // gives uint8_t/uint16_t
/*
 * dwin_display.h
 *
 *  Created on: 18-Jul-2026
 *      Author: rahul
 */




void DWIN_Init(UART_HandleTypeDef *uart);
void DWIN_Process(void);

void DWIN_WriteVP(uint16_t vp, uint16_t value);
void DWIN_PageChange(uint16_t page_no);

void DWIN_OnRxEvent(UART_HandleTypeDef *huart, uint16_t size);
void DWIN_OnTxCplt(UART_HandleTypeDef *huart);
void DWIN_OnError(UART_HandleTypeDef *huart);

/* Returns 1 when a new VP/value is available, else 0 */
uint8_t DWIN_TryGetLastUpdate(uint16_t *vp, uint16_t *value);

#endif /* INC_DWIN_DISPLAY_H_ */
