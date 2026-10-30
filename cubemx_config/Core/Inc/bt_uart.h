#ifndef __BT_UART_H
#define __BT_UART_H

#include "main.h"
#include "usart.h"
#include <stdint.h>

#define BT_RX_BUFFER_SIZE    64

typedef struct
{
    uint8_t rx_byte;                         // 单字节接收缓存
    uint8_t rx_buffer[BT_RX_BUFFER_SIZE];    // 一帧数据缓存
    uint8_t rx_index;                        // 当前接收下标
    volatile uint8_t frame_ready;                     // 一帧接收完成标志
} BT_UartRx_t;

extern BT_UartRx_t g_bt_rx;

void BT_UART_Init(void);
void BT_UART_ProcessByte(uint8_t data);
void BT_UART_SendString(char *str);
void BT_UART_ClearFrame(void);

#endif
