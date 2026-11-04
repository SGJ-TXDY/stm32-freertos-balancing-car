"""Run host regressions against the firmware's actual control/DMP functions.

Requires a host C compiler (cc or gcc). HAL time, I2C and scheduler events are
simulated; these tests do not replace an on-board timing/closed-loop test.
"""
from pathlib import Path
import re
import shutil
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "cubemx_config/Core/Src"
INC = ROOT / "cubemx_config/Core/Inc"


def function(source, name):
    start = re.search(r"^(?:static )?(?:void|int) " + name + r"\(", source, re.M).start()
    brace = source.index("{", start)
    depth = 1
    end = brace + 1
    while depth:
        depth += (source[end] == "{") - (source[end] == "}")
        end += 1
    return source[start:end]


def defines(path, prefix):
    # Existing MPU header uses GBK comments; only ASCII #defines are needed.
    return "\n".join(line for line in path.read_text(encoding="utf-8", errors="replace").splitlines()
                     if line.startswith("#define " + prefix)) + "\n"


COMMON = r"""
#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
typedef struct { double PitchAngle; struct {float gyro_y;} data; } Global;
Global gGlobalData;
"""

DMP_STUBS = r"""
#define INV_WXYZ_QUAT 0x100
#define MPU6050_GYRO_SCALE_2000 16.4f
#define M_PI 3.14159265358979323846
static int queued, reads, fail_read, missing_quat;
int dmp_read_fifo(short *gyro, short *accel, long *quat,
                  unsigned long *timestamp, short *sensors, unsigned char *more)
{
    if (!queued || reads + 1 == fail_read) return -1;
    queued--; reads++;
    quat[0] = 1073741824L; quat[1] = quat[2] = quat[3] = 0;
    gyro[1] = reads * 164;
    *sensors = missing_quat ? 0 : INV_WXYZ_QUAT;
    *more = queued;
    return 0;
}
"""

DMP_TESTS = r"""
static void reset(int packets)
{
    queued = packets; reads = fail_read = missing_quat = 0;
    gGlobalData.PitchAngle = 123; gGlobalData.data.gyro_y = 456;
}
static void unchanged(void)
{
    assert(gGlobalData.PitchAngle == 123);
    assert(gGlobalData.data.gyro_y == 456);
}
int main(void)
{
    reset(0);
    assert(DMP_Pitch() == DMP_PITCH_NO_SAMPLE); unchanged();
    reset(1);
    assert(DMP_Pitch() == DMP_PITCH_OK && reads == 1);
    assert(gGlobalData.PitchAngle == 0);
    assert(fabsf(gGlobalData.data.gyro_y - 10) < 0.001f);
    reset(DMP_FIFO_DRAIN_MAX);
    assert(DMP_Pitch() == DMP_PITCH_OK && queued == 0);
    assert(fabsf(gGlobalData.data.gyro_y - reads * 10) < 0.001f);
    reset(12);
    assert(DMP_Pitch() == DMP_PITCH_UNSAFE && reads == DMP_FIFO_DRAIN_MAX);
    unchanged();
    reset(2); fail_read = 2;
    assert(DMP_Pitch() == DMP_PITCH_UNSAFE); unchanged();
    reset(1); missing_quat = 1;
    assert(DMP_Pitch() == DMP_PITCH_UNSAFE); unchanged();
    reset(DMP_FIFO_DRAIN_MAX + 1);
    assert(DMP_Pitch() == DMP_PITCH_UNSAFE); unchanged();
    assert(DMP_Pitch() == DMP_PITCH_OK && queued == 0);
    puts("PASS: FIFO tail, deep backlog, partial error, empty FIFO and catch-up");
}
"""

CONTROL_STUBS = r"""
#include <setjmp.h>
typedef uint32_t TickType_t;
typedef void *SemaphoreHandle_t;
typedef void *QueueHandle_t;
#define pdMS_TO_TICKS(x) (x)
#define pdTRUE 1
#define MOTOR_LEFT 0
#define MOTOR_RIGHT 1
static float Car_zero;
static uint32_t tick;
static int cycle, scenario, command_given, pwm_value, nonzero_cycles;
static int stop_tick, trip_pending, critical_depth;
static jmp_buf finished;
void App_ControlWatchdogTick(void);
void advance(unsigned ms)
{
    while (ms--) { tick++; App_ControlWatchdogTick(); }
}
uint32_t HAL_GetTick(void) { return tick; }
TickType_t xTaskGetTickCount(void) { return tick; }
void App_I2C_Lock(void)
{
    if (scenario == 4 && cycle == 1) advance(26);
}
void App_I2C_Unlock(void) {}
int DMP_Pitch(void)
{
    if (scenario == 1 && cycle == 1) { advance(26); return DMP_PITCH_NO_SAMPLE; }
    if (scenario == 2 && cycle == 1) return DMP_PITCH_UNSAFE;
    gGlobalData.PitchAngle = 0;
    return DMP_PITCH_OK;
}
int Read_Encoder(int motor) { return 0; }
int xQueueReceive(QueueHandle_t queue, void *command, unsigned wait)
{
    /* Queue ARM before the fault returns too: it must be discarded. */
    if (!command_given && (cycle == 0 || cycle == 1 || cycle == 4)) {
        *(int *)command = 1; command_given = 1; return pdTRUE;
    }
    return 0;
}
void PID_Reset(void) {}
int zhili(float angle, float gyro)
{
    if (scenario == 3 && cycle == 1) advance(10);
    if (scenario == 5 && cycle == 1) advance(5);
    if (scenario == 6 && cycle == 1) trip_pending = 1;
    return 100;
}
int sudu(int left, int right) { return 0; }
void Motor_EmergencyStop(void)
{
    if (pwm_value != 0 && stop_tick < 0) stop_tick = (int)tick;
    pwm_value = 0;
}
void setpwm(int left, int right)
{
    assert(critical_depth == 1);
    assert(left == right && left != 0);
    pwm_value = left; nonzero_cycles |= 1 << cycle;
}
void enter_critical(void)
{
    /* Interrupt immediately before entering the motor-write critical section. */
    if (trip_pending) { trip_pending = 0; advance(10); }
    critical_depth++;
}
#define taskENTER_CRITICAL() enter_critical()
#define taskEXIT_CRITICAL() (critical_depth--)
void xQueueOverwrite(QueueHandle_t queue, const void *snapshot) {}
void vTaskDelayUntil(TickType_t *wake, TickType_t period)
{
    if (++cycle == 6) longjmp(finished, 1);
    command_given = 0;
    *wake += period;
    if ((int32_t)(*wake - tick) > 0) advance(*wake - tick);
}
"""

CONTROL_TESTS = r"""
int main(void)
{
    for (scenario = 0; scenario <= 6; scenario++) {
        tick = cycle = command_given = pwm_value = nonzero_cycles = 0;
        trip_pending = critical_depth = 0; stop_tick = -1;
        control_last_output_ms = 0;
        control_watchdog_active = control_watchdog_tripped = 0;
        if (!setjmp(finished)) ControlTask(NULL);
        assert(critical_depth == 0);
        if (scenario == 0) {
            assert(nonzero_cycles == 63 && stop_tick == -1);
        } else {
            /* No late output, no automatic restart; only a new ARM at cycle 4. */
            assert(nonzero_cycles == ((1 << 0) | (1 << 4) | (1 << 5)));
            if (scenario == 1 || scenario == 4)
                assert(stop_tick == APP_CONTROL_WATCHDOG_MS);
        }
    }
    control_watchdog_active = 0; pwm_value = 0; tick = 123;
    App_ControlWatchdogTick(); assert(pwm_value == 0);
    control_last_output_ms = UINT32_MAX - 4;
    tick = control_last_output_ms; pwm_value = 100;
    control_watchdog_active = 1; control_watchdog_tripped = 0;
    advance(APP_CONTROL_WATCHDOG_MS - 1); assert(pwm_value == 100);
    advance(1); assert(pwm_value == 0 && control_watchdog_tripped);
    puts("PASS: normal control, blocked I2C/mutex, stale data, late output, ISR race, explicit re-arm and tick wrap");
}
"""

MOTOR_STUBS = r"""
#include <assert.h>
#include <stdio.h>
typedef struct {int compare[2];} Timer;
static Timer htim4 = {{100, 200}};
static int gpio_writes;
#define TIM_CHANNEL_3 0
#define TIM_CHANNEL_4 1
#define GPIOB 123
#define AIN1_Pin 1
#define AIN2_Pin 2
#define BIN1_Pin 4
#define BIN2_Pin 8
#define GPIO_PIN_RESET 0
#define __HAL_TIM_SET_COMPARE(timer, channel, value) ((timer)->compare[channel] = (value))
void HAL_GPIO_WritePin(int port, int pins, int level)
{
    assert(port == GPIOB && pins == 15 && level == GPIO_PIN_RESET);
    gpio_writes++;
}
"""

MOTOR_TESTS = r"""
int main(void)
{
    Motor_EmergencyStop();
    assert(htim4.compare[0] == 0 && htim4.compare[1] == 0);
    assert(gpio_writes == 1);
    puts("PASS: emergency stop clears both PWM channels and direction pins");
}
"""


def main():
    compiler = shutil.which("cc") or shutil.which("gcc")
    if not compiler:
        raise SystemExit("Host C compiler not found (install gcc or clang).")
    dmp = (SRC / "pt_mpu6050.c").read_text()
    app = (SRC / "app_rtos.c").read_text()
    constants = defines(INC / "pt_mpu6050.h", "DMP_PITCH_")
    dmp_case = (COMMON + constants + defines(SRC / "pt_mpu6050.c", "DMP_FIFO_")
                + DMP_STUBS + function(dmp, "DMP_Pitch") + DMP_TESTS)
    control_case = (COMMON + constants + defines(INC / "app_rtos.h", "APP_")
                    + CONTROL_STUBS
                    + app[app.index("typedef struct"):app.index("void App_I2C_Lock")]
                    + function(app, "ControlTask") + CONTROL_TESTS)
    motor = (SRC / "tb6612.c").read_text()
    motor_case = (MOTOR_STUBS + function(motor, "Motor_Stop")
                  + function(motor, "Motor_EmergencyStop") + MOTOR_TESTS)
    with tempfile.TemporaryDirectory(prefix="balance-safety-") as folder:
        for name, content in [("dmp", dmp_case), ("control", control_case),
                              ("motor", motor_case)]:
            source = Path(folder) / (name + ".c")
            binary = Path(folder) / name
            source.write_text(content)
            subprocess.run([compiler, "-std=c11", "-O2", "-Wall", "-Wextra",
                            "-Wno-unused-parameter", "-Wno-unused-variable",
                            str(source), "-lm", "-o", str(binary)], check=True)
            subprocess.run([str(binary)], check=True)
    it = (SRC / "stm32f1xx_it.c").read_text()
    systick = function(it, "SysTick_Handler")
    assert systick.index("HAL_IncTick();") < systick.index("App_ControlWatchdogTick();")
    assert systick.index("App_ControlWatchdogTick();") < systick.index("xPortSysTickHandler();")
    print("PASS: watchdog wired into SysTick after HAL tick and before RTOS tick")


if __name__ == "__main__":
    main()
