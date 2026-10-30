#include "display.h"
#include "oled.h"
#include <stdio.h>
#include <string.h>

void Display_Init(void)
{
    OLED_Init();
}

/* 开机欢迎页 */
void Display_ShowStartupPage(void)
{
    OLED_Clear();

    OLED_ShowString(0, 0,  "BALANCE CAR");
    OLED_ShowString(0, 16, "OLED READY");
    OLED_ShowString(0, 32, "STM32F103C8T6");

    OLED_Refresh();
}

/* 主页面：显示电压、倾角、速度 */
void Display_ShowMainPage(PtGlobalData_t* data)
{
    char buf[24];

    if (data == NULL)
    {
        return;
    }

    OLED_Clear();

    /* 第1行：标题 */
    OLED_ShowString(0, 0, "BALANCE CAR");

    /* 第2行：电压 */
    memset(buf, 0, sizeof(buf));
    snprintf(buf, sizeof(buf), "BAT:%.1f V", data->BatVolt);
    OLED_ShowString(0, 10, buf);

    /* 第3行：倾角 */
    memset(buf, 0, sizeof(buf));
    snprintf(buf, sizeof(buf), "ANG:%.1f ", data->PitchAngle);
    OLED_ShowString(0, 20, buf);

    /* 第4行：速度 */
    memset(buf, 0, sizeof(buf));
    snprintf(buf, sizeof(buf), "SPD:%.1f RPM", data->CurSpeed);
    OLED_ShowString(0, 30, buf);

    /* 第5行：目标模式 和 目标速度 */
    memset(buf, 0, sizeof(buf));
    snprintf(buf, sizeof(buf), "TM&TS:%d & %d", data->TargetMode, data->TargetSpeed);
    OLED_ShowString(0, 40, buf);

    OLED_Refresh();
}
