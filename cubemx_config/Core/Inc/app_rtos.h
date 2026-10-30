#ifndef APP_RTOS_H
#define APP_RTOS_H

#include "FreeRTOS.h"
#include "task.h"

#define APP_CONTROL_PERIOD_MS 5U
#define APP_I2C_TIMEOUT_MS 2U

void App_RTOS_Start(void);
void App_I2C_Lock(void);
void App_I2C_Unlock(void);

#endif
