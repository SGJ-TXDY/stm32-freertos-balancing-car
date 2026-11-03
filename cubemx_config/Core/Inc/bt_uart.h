#ifndef __BT_UART_H
#define __BT_UART_H

#include "main.h"
#include "usart.h"
#include <stdint.h>

#define BT_RX_BUFFER_SIZE    64

void BT_UART_Init(void);
/* Task that drains the byte ring. Pass xTaskGetCurrentTaskHandle(). */
void BT_UART_BindTask(void *task);
/* Pop one byte queued by the USART2 ISR. Returns 1 if a byte was stored. */
int BT_UART_Pop(uint8_t *byte);
/* Re-arm RX if an overrun or a busy HAL lock left reception stopped. */
void BT_UART_Service(void);
void BT_UART_SendString(char *str);

#endif
