#include "bt_uart.h"
#include <string.h>

BT_UartRx_t g_bt_rx;

/* 初始化蓝牙串口接收 */
void BT_UART_Init(void)
{
    memset(&g_bt_rx, 0, sizeof(g_bt_rx));

    HAL_UART_Receive_IT(&huart2, &g_bt_rx.rx_byte, 1);
}

/* 清空一帧 */
void BT_UART_ClearFrame(void)
{
    memset(g_bt_rx.rx_buffer, 0, sizeof(g_bt_rx.rx_buffer));
    g_bt_rx.rx_index = 0;
    g_bt_rx.frame_ready = 0;
}

/* 发送字符串到蓝牙串口 */
void BT_UART_SendString(char* str)
{
    if (str == NULL) return;

    HAL_UART_Transmit(&huart2, (uint8_t*)str, strlen(str), 30);
}

/* 处理接收到的单字节 */
void BT_UART_ProcessByte(uint8_t data)
{
    /* 如果已经接收到完整帧，先不覆盖，等主循环处理 */
    if (g_bt_rx.frame_ready)
    {
        return;
    }

    /* 帧头同步：不是 '$' 且当前还没开始接收，则丢弃 */
    if (g_bt_rx.rx_index == 0)
    {
        if (data != '$')
        {
            return;
        }
    }

    if (g_bt_rx.rx_index < (BT_RX_BUFFER_SIZE - 1))
    {
        g_bt_rx.rx_buffer[g_bt_rx.rx_index++] = data;

        /* 收到帧尾 '#'，判定一帧完成 */
        if (data == '#')
        {
            g_bt_rx.rx_buffer[g_bt_rx.rx_index] = '\0';
            g_bt_rx.frame_ready = 1;
        }
    }
    else
    {
        /* 缓冲区溢出，直接清空重来 */
        BT_UART_ClearFrame();
    }
}
