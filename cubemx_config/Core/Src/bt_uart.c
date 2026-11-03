#include "bt_uart.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>

/*
 * USART2 ISR is the only producer; the communication task is the only
 * consumer. Bytes are queued even while a previous command is still being
 * parsed, so a following $STOP# is not dropped. 256 bytes is about 260 ms
 * at 9600 baud.
 */
#define BT_RING_SIZE 256u
#define BT_RING_MASK (BT_RING_SIZE - 1u)

static uint8_t s_ring[BT_RING_SIZE];
static volatile uint16_t s_head;
static volatile uint16_t s_tail;
static volatile TaskHandle_t s_task;
static uint8_t s_rx_byte;

static void bt_push_from_isr(uint8_t data)
{
    uint16_t head = s_head;
    uint16_t next = (uint16_t)((head + 1u) & BT_RING_MASK);
    BaseType_t woken = pdFALSE;

    if (next != s_tail) {
        s_ring[head] = data;
        s_head = next;
    }
    if (s_task != NULL &&
        xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
        vTaskNotifyGiveFromISR(s_task, &woken);
        portYIELD_FROM_ISR(woken);
    }
}

static void bt_arm_rx(void)
{
    (void)HAL_UART_Receive_IT(&huart2, &s_rx_byte, 1);
}

void BT_UART_Init(void)
{
    s_head = 0;
    s_tail = 0;
    s_task = NULL;
    bt_arm_rx();
}

void BT_UART_BindTask(void *task)
{
    s_task = (TaskHandle_t)task;
}

int BT_UART_Pop(uint8_t *byte)
{
    uint16_t tail;
    uint16_t head;

    if (byte == NULL) {
        return 0;
    }
    taskENTER_CRITICAL();
    tail = s_tail;
    head = s_head;
    if (tail == head) {
        taskEXIT_CRITICAL();
        return 0;
    }
    *byte = s_ring[tail];
    s_tail = (uint16_t)((tail + 1u) & BT_RING_MASK);
    taskEXIT_CRITICAL();
    return 1;
}

void BT_UART_Service(void)
{
    /* READY means IT reception was aborted (overrun) or never re-armed. */
    if (huart2.RxState == HAL_UART_STATE_READY) {
        __HAL_UART_CLEAR_OREFLAG(&huart2);
        bt_arm_rx();
    }
}

void BT_UART_SendString(char *str)
{
    if (str == NULL) {
        return;
    }
    HAL_UART_Transmit(&huart2, (uint8_t *)str, (uint16_t)strlen(str), 30);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) {
        bt_push_from_isr(s_rx_byte);
        bt_arm_rx();
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART2) {
        return;
    }
    /* ORE aborts the IT transfer and leaves RxState READY with RXNEIE off.
     * Framing/noise errors keep the transfer running; do not start a second one. */
    if (huart->RxState == HAL_UART_STATE_READY) {
        __HAL_UART_CLEAR_OREFLAG(huart);
        bt_arm_rx();
    }
}
