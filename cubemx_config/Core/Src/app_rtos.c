#include "app_rtos.h"
#include "semphr.h"
#include "queue.h"
#include "pt_mpu6050.h"
#include "pt_pid.h"
#include "encode.h"
#include "tb6612.h"
#include "bt_uart.h"
#include "oled.h"
#include "usart.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

typedef struct {
    float pitch;
    int encoder_sum;
    uint8_t armed;
} UiSnapshot;

typedef enum { COMMAND_STOP, COMMAND_ARM } ControlCommand;
static SemaphoreHandle_t i2c_mutex;
static QueueHandle_t ui_queue;
static QueueHandle_t command_queue;

/* Only ControlTask feeds this watchdog. SysTick can cut motor power even
 * while that task is stuck in the I2C driver, recovery, or a bus mutex.
 * Task-side motor writes and these fields are protected from SysTick by
 * a short critical section; a trip must be acknowledged before re-arming. */
static volatile uint32_t control_last_output_ms;
static volatile uint8_t control_watchdog_active;
static volatile uint8_t control_watchdog_tripped;

void App_ControlWatchdogTick(void)
{
    if (control_watchdog_active &&
        (uint32_t)(HAL_GetTick() - control_last_output_ms) >= APP_CONTROL_WATCHDOG_MS) {
        control_watchdog_active = 0;
        control_watchdog_tripped = 1;
        Motor_EmergencyStop();
    }
}

void App_I2C_Lock(void)
{
    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
        configASSERT(xSemaphoreTake(i2c_mutex, portMAX_DELAY) == pdTRUE);
    }
}

void App_I2C_Unlock(void)
{
    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
        configASSERT(xSemaphoreGive(i2c_mutex) == pdTRUE);
    }
}

static void ControlTask(void *argument)
{
    TickType_t wake = xTaskGetTickCount();
    TickType_t last_sample = wake;
    const TickType_t period = pdMS_TO_TICKS(APP_CONTROL_PERIOD_MS);
    uint8_t armed = 0;
    uint8_t have_sample = 0;
    UiSnapshot snapshot = {0};
    ControlCommand command;
    (void)argument;

    for (;;) {
        int left, right, fresh, sample_status, reject_arm;
        int pwm = 0;
        TickType_t now;
        /* Only a read that reaches the FIFO tail is fresh. */
        App_I2C_Lock();
        sample_status = DMP_Pitch();
        App_I2C_Unlock();
        fresh = (sample_status == DMP_PITCH_OK);
        if (fresh) {
            last_sample = xTaskGetTickCount();
            have_sample = 1;
        }

        left = -Read_Encoder(MOTOR_LEFT);
        right = Read_Encoder(MOTOR_RIGHT);
        reject_arm = control_watchdog_tripped || sample_status == DMP_PITCH_UNSAFE ||
                     xTaskGetTickCount() - wake >= period;
        if (reject_arm) {
            armed = 0;
        }
        while (xQueueReceive(command_queue, &command, 0) == pdTRUE) {
            if (command == COMMAND_STOP) {
                armed = 0;
            } else if (!reject_arm && have_sample && fresh &&
                       fabs(gGlobalData.PitchAngle - Car_zero) < 8.0) {
                armed = 1;
                PID_Reset();
            }
        }

        if (!have_sample || !isfinite(gGlobalData.PitchAngle) ||
            fabs(gGlobalData.PitchAngle - Car_zero) > 35.0 ||
            xTaskGetTickCount() - last_sample > pdMS_TO_TICKS(25)) {
            armed = 0;
        }

        if (armed) {
            pwm = zhili((float)gGlobalData.PitchAngle,
                        gGlobalData.data.gyro_y) + sudu(left, right);
        }

        /* Check again immediately before writing PWM. A watchdog interrupt
         * during PID computation must not be undone by a late task write.
         * Never feed the watchdog at the start of an I2C operation. */
        taskENTER_CRITICAL();
        now = xTaskGetTickCount();
        if (control_watchdog_tripped || now - wake >= period) {
            armed = 0;
        }
        if (now - wake >= period) {
            wake = now;
        }
        if (armed) {
            control_last_output_ms = HAL_GetTick();
            control_watchdog_active = 1;
            setpwm(pwm, pwm);
        } else {
            control_watchdog_active = 0;
            Motor_EmergencyStop();
        }
        control_watchdog_tripped = 0;
        taskEXIT_CRITICAL();
        if (!armed) {
            PID_Reset();
        }

        snapshot.pitch = (float)gGlobalData.PitchAngle;
        snapshot.encoder_sum = left + right;
        snapshot.armed = armed;
        xQueueOverwrite(ui_queue, &snapshot);

        vTaskDelayUntil(&wake, period);
    }
}

static void UiTask(void *argument)
{
    UiSnapshot snapshot;
    char text[24];
    (void)argument;
    for (;;) {
        if (xQueuePeek(ui_queue, &snapshot, 0) == pdTRUE) {
            OLED_Clear();
            OLED_ShowString(0, 0, "STM32 + FreeRTOS");
            snprintf(text, sizeof(text), "PITCH:%.1f", (double)snapshot.pitch);
            OLED_ShowString(0, 16, text);
            snprintf(text, sizeof(text), "ENC/5ms:%d", snapshot.encoder_sum);
            OLED_ShowString(0, 32, text);
            OLED_ShowString(0, 48, snapshot.armed ? "ARMED" : "STOPPED");
            OLED_Refresh();
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

static int FeedFrame(uint8_t data, char *frame, size_t *len)
{
    if (*len == 0) {
        if (data != '$') {
            return 0;
        }
    }
    if (*len >= (BT_RX_BUFFER_SIZE - 1)) {
        *len = 0;
        if (data != '$') {
            return 0;
        }
    }
    frame[(*len)++] = (char)data;
    if (data == '#') {
        frame[*len] = '\0';
        return 1;
    }
    return 0;
}

static void HandleFrame(char *frame)
{
    ControlCommand command;
    const char *reply = "ERR:COMMAND\r\n";
    if (strcmp(frame, "$ARM#") == 0 || strcmp(frame, "$STOP#") == 0) {
        command = strcmp(frame, "$ARM#") == 0 ? COMMAND_ARM : COMMAND_STOP;
        reply = xQueueSend(command_queue, &command, 0) == pdTRUE ?
                "OK:QUEUED\r\n" : "ERR:BUSY\r\n";
    }
    BT_UART_SendString((char *)reply);
    HAL_UART_Transmit(&huart1, (uint8_t *)frame, (uint16_t)strlen(frame), 20);
    HAL_UART_Transmit(&huart1, (uint8_t *)"\r\n", 2, 20);
}

static void CommunicationTask(void *argument)
{
    char frame[BT_RX_BUFFER_SIZE];
    size_t len = 0;
    uint8_t byte;
    (void)argument;
    BT_UART_BindTask(xTaskGetCurrentTaskHandle());
    for (;;) {
        while (BT_UART_Pop(&byte)) {
            if (FeedFrame(byte, frame, &len)) {
                HandleFrame(frame);
                len = 0;
            }
        }
        /* Wake on the next byte, and also poll so a stopped RX can be re-armed. */
        BT_UART_Service();
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(20));
    }
}

void App_RTOS_Start(void)
{
    i2c_mutex = xSemaphoreCreateMutex();
    ui_queue = xQueueCreate(1, sizeof(UiSnapshot));
    command_queue = xQueueCreate(4, sizeof(ControlCommand));
    configASSERT(i2c_mutex && ui_queue && command_queue);
    configASSERT(xTaskCreate(ControlTask, "control", 384, NULL, 4, NULL) == pdPASS);
    configASSERT(xTaskCreate(UiTask, "ui", 384, NULL, 1, NULL) == pdPASS);
    configASSERT(xTaskCreate(CommunicationTask, "comm", 256, NULL, 2, NULL) == pdPASS);
    vTaskStartScheduler();
    Error_Handler();
}

void vApplicationMallocFailedHook(void)
{
    Error_Handler();
}

void vApplicationStackOverflowHook(TaskHandle_t task, char *name)
{
    (void)task;
    (void)name;
    Error_Handler();
}
