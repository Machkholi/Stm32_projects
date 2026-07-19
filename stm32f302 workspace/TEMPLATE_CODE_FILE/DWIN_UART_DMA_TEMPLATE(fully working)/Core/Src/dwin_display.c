/*
 * dwin_display.c
 *
 *  Created on: 18-Jul-2026
 *      Author: rahul
 */

#include "dwin_display.h"
#include <string.h>
#include <stdint.h>

#define DWIN_RX_BUFFER_SIZE       64
#define DWIN_TX_FRAME_MAX_LEN     32
#define DWIN_TX_QUEUE_SIZE        16
#define DWIN_USE_NONBLOCKING_TX   1

static UART_HandleTypeDef *dwin_uart = NULL;

static uint8_t rx_buffer[DWIN_RX_BUFFER_SIZE];
static uint8_t rx_copy[DWIN_RX_BUFFER_SIZE];
static uint8_t parse_copy[DWIN_RX_BUFFER_SIZE];

static volatile uint8_t frame_ready = 0;
static volatile uint16_t rx_length = 0;

static uint8_t tx_buf[32];
static uint8_t tx_queue[DWIN_TX_QUEUE_SIZE][DWIN_TX_FRAME_MAX_LEN];
static uint16_t tx_len[DWIN_TX_QUEUE_SIZE];
static volatile uint8_t tx_head = 0;
static volatile uint8_t tx_tail = 0;
static volatile uint8_t tx_busy = 0;

static volatile uint16_t last_vp = 0;
static volatile uint16_t last_val = 0;
static volatile uint8_t update_pending = 0;

static uint8_t tx_next(uint8_t idx) {
    return (uint8_t)((idx + 1U) % DWIN_TX_QUEUE_SIZE);
}

static uint8_t tx_empty(void) {
    return (tx_head == tx_tail);
}

static uint8_t tx_full(void) {
    return (tx_next(tx_head) == tx_tail);
}

static void tx_start_from_tail(void) {
    if (tx_empty()) {
        tx_busy = 0U;
        return;
    }

    if (HAL_UART_Transmit_DMA(dwin_uart, tx_queue[tx_tail], tx_len[tx_tail]) != HAL_OK) {
        tx_busy = 0U;
    }
}

static void DWIN_Send_Blocking(uint8_t *data, uint16_t len) {
    if (dwin_uart == NULL || data == NULL || len == 0U) return;
    HAL_UART_Transmit(dwin_uart, data, len, HAL_MAX_DELAY);
}

static void DWIN_Send_NonBlocking(uint8_t *data, uint16_t len) {
    uint8_t start_tx = 0U;

    if (dwin_uart == NULL || data == NULL || len == 0U) return;
    if (len > DWIN_TX_FRAME_MAX_LEN) return;

    while (tx_full()) {
        /* no-drop policy: wait */
    }

    __disable_irq();
    memcpy(tx_queue[tx_head], data, len);
    tx_len[tx_head] = len;
    tx_head = tx_next(tx_head);

    if (tx_busy == 0U) {
        tx_busy = 1U;
        start_tx = 1U;
    }
    __enable_irq();

    if (start_tx) {
        tx_start_from_tail();
    }
}

static void DWIN_Send(uint8_t *data, uint16_t len) {
#if DWIN_USE_NONBLOCKING_TX
    DWIN_Send_NonBlocking(data, len);
#else
    DWIN_Send_Blocking(data, len);
#endif
}

static void DWIN_ParseFrame(uint8_t *data, uint16_t len) {
    for (uint16_t i = 0; i + 3 < len; i++) {
        if (data[i] == 0x5A && data[i + 1] == 0xA5) {
            uint8_t payload_len = data[i + 2];
            uint16_t frame_len = (uint16_t)(payload_len + 3U);

            if ((uint16_t)(i + frame_len) > len) break;

            uint8_t cmd = data[i + 3];
            if (cmd == 0x83 && payload_len >= 5U) {
                uint16_t vp = (uint16_t)((data[i + 4] << 8) | data[i + 5]);
                uint16_t val = (uint16_t)((data[i + frame_len - 2] << 8) | data[i + frame_len - 1]);

                last_vp = vp;
                last_val = val;
                update_pending = 1U;
            }

            i = (uint16_t)(i + frame_len - 1U);
        }
    }
}

void DWIN_Init(UART_HandleTypeDef *uart) {
    dwin_uart = uart;

    __disable_irq();
    tx_head = tx_tail = 0;
    tx_busy = 0;
    frame_ready = 0;
    rx_length = 0;
    update_pending = 0;
    __enable_irq();

    HAL_UARTEx_ReceiveToIdle_DMA(dwin_uart, rx_buffer, DWIN_RX_BUFFER_SIZE);
    __HAL_DMA_DISABLE_IT(dwin_uart->hdmarx, DMA_IT_HT);
}

void DWIN_Process(void) {
    if (frame_ready) {
        uint16_t len = 0;

        __disable_irq();
        if (frame_ready) {
            frame_ready = 0;
            len = rx_length;
            if (len > DWIN_RX_BUFFER_SIZE) len = DWIN_RX_BUFFER_SIZE;
            memcpy(parse_copy, rx_copy, len);
        }
        __enable_irq();

        if (len > 0) {
            DWIN_ParseFrame(parse_copy, len);
        }
    }
}

void DWIN_WriteVP(uint16_t vp, uint16_t value) {
    tx_buf[0] = 0x5A;
    tx_buf[1] = 0xA5;
    tx_buf[2] = 0x05;
    tx_buf[3] = 0x82;
    tx_buf[4] = (uint8_t)(vp >> 8);
    tx_buf[5] = (uint8_t)(vp & 0xFF);
    tx_buf[6] = (uint8_t)(value >> 8);
    tx_buf[7] = (uint8_t)(value & 0xFF);

    DWIN_Send(tx_buf, 8);
}

void DWIN_PageChange(uint16_t page_no) {
    tx_buf[0] = 0x5A;
    tx_buf[1] = 0xA5;
    tx_buf[2] = 0x07;
    tx_buf[3] = 0x82;
    tx_buf[4] = 0x00;
    tx_buf[5] = 0x84;
    tx_buf[6] = 0x5A;
    tx_buf[7] = 0x01;
    tx_buf[8] = (uint8_t)(page_no >> 8);
    tx_buf[9] = (uint8_t)(page_no & 0xFF);

    DWIN_Send(tx_buf, 10);
}

void DWIN_OnRxEvent(UART_HandleTypeDef *huart, uint16_t size) {
    if (dwin_uart == NULL || huart->Instance != dwin_uart->Instance) return;

    uint16_t copy_len = (size > DWIN_RX_BUFFER_SIZE) ? DWIN_RX_BUFFER_SIZE : size;
    memcpy(rx_copy, rx_buffer, copy_len);

    rx_length = copy_len;
    frame_ready = 1U;

    HAL_UARTEx_ReceiveToIdle_DMA(dwin_uart, rx_buffer, DWIN_RX_BUFFER_SIZE);
    __HAL_DMA_DISABLE_IT(dwin_uart->hdmarx, DMA_IT_HT);
}

void DWIN_OnTxCplt(UART_HandleTypeDef *huart) {
    if (dwin_uart == NULL || huart->Instance != dwin_uart->Instance) return;

    __disable_irq();
    if (!tx_empty()) {
        tx_tail = tx_next(tx_tail);
    }

    if (tx_empty()) {
        tx_busy = 0U;
        __enable_irq();
        return;
    }
    __enable_irq();

    tx_start_from_tail();
}

void DWIN_OnError(UART_HandleTypeDef *huart) {
    if (dwin_uart == NULL || huart->Instance != dwin_uart->Instance) return;

    __disable_irq();
    if (!tx_empty()) {
        tx_tail = tx_next(tx_tail);
    }

    if (tx_empty()) {
        tx_busy = 0U;
        __enable_irq();
        return;
    }
    __enable_irq();

    tx_start_from_tail();
}

uint8_t DWIN_TryGetLastUpdate(uint16_t *vp, uint16_t *value) {
    if (!update_pending) return 0U;

    __disable_irq();
    if (vp) *vp = last_vp;
    if (value) *value = last_val;
    update_pending = 0U;
    __enable_irq();

    return 1U;
}
