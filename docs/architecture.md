# 固件架构与维护

## 启动顺序

`main.c` 完成 HAL、72 MHz 时钟树、GPIO、I²C2、TIM1/2/3/4、USART1/2 初始化，启动零占空比 PWM 与编码器计数，再初始化 OLED 和 MPU6050 DMP，开启蓝牙逐字节接收，最后调用 `App_RTOS_Start()`。

`App_RTOS_Start()` 创建 I²C 互斥锁、状态队列、命令队列及三个任务，然后启动调度器。TIM1 保留外设配置，控制工作由 FreeRTOS 任务完成，不启动 TIM1 控制中断。

## 中断与任务边界

- `USART2_IRQHandler → HAL_UART_RxCpltCallback → BT_UART_ProcessByte`：组帧并重新挂接单字节接收。完整帧就绪期间，接收器不覆盖缓冲区；通信任务取走该帧后清除标志。
- `SysTick_Handler`：先增加 HAL tick，调度器启动后再调用 `xPortSysTickHandler`。
- `SVC_Handler / PendSV_Handler`：通过 `FreeRTOSConfig.h` 的映射由官方 Cortex-M3 移植层实现。
- 通信任务把 ARM / STOP 放入命令队列；控制任务拥有电机输出和 PID 状态，其他任务不直接改写控制变量。

蓝牙使用单帧缓冲，请等待上一条指令的回复后再发送下一条。USART2 中断优先级为 5；内核允许调用 FromISR API 的最高逻辑优先级也为 5，SysTick 和 PendSV 使用优先级 15。

## 共享 I²C

运行期的 MPU6050 FIFO 读取整体持有互斥锁，OLED 在每条命令或每个 16 字节数据块传输时持锁。锁采用 FreeRTOS 的优先级继承机制，底层传输超时为 2 ms，避免整个 OLED 帧占用总线。

初始化阶段调度器尚未启动，总线按顺序访问。此时 `App_I2C_Lock / Unlock` 不调用互斥锁 API。

## 姿态与控制

MPU6050 保留 InvenSense DMP 路径：200 Hz 采样及 FIFO，启用六轴四元数、陀螺仪校准、加速度与校准角速度输出。身份安装矩阵对应标量 `136`。

四元数从 Q30 转为浮点后计算 Y 轴俯仰角，角速度按 ±2000 °/s 的量程换算。姿态校验失败不覆盖上一帧；超过 25 ms 无有效姿态则停止电机。传感器安装坐标应与实际车体一致。

控制保持原工程的直立 PD 与速度 PI 输出叠加方式。速度反馈为左右轮每 5 ms 的编码器脉冲之和，显示单位为 `ENC/5ms`。速度 PI 保留一阶低通和积分限幅；停止状态重置其内部状态。

## 内存与构建

FreeRTOS 的 10 KB 堆包含任务栈、任务控制块、队列和互斥锁。GNU 链接脚本另为 newlib 格式化分配 1 KB 堆，为主栈保留至少 1 KB 空间。运行期只有显示任务使用 `snprintf`，FreeRTOS 对象由 `heap_4` 分配。

GNU 构建从 `.uvprojx` 提取 C 源文件清单，替换为 GNU 启动文件并增加 `GCC/syscalls.c`。Keil 使用原有启动汇编及 Arm Compiler 6。

## CubeMX 重新生成时保留的接入点

1. `main.c` 的应用初始化与 `App_RTOS_Start()`。
2. `stm32f1xx_it.c` 的 SysTick 衔接与 SVC / PendSV 归属。
3. `usart.c` 的 TX 复用推挽、RX 输入配置和 USART2 中断优先级。
4. `oled.c` 的短传输与总线锁，以及 `pt_mpu6050.c` 的 DMP 初始化。
5. Keil 工程内的 FreeRTOS 源文件、头文件目录和 `app_rtos.c`。

生成后运行 `python scripts/check_project.py` 和 `python scripts/build.py`，再按硬件调试文档确认运行行为。
