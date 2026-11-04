# 固件架构与维护

## 启动顺序

`main.c` 完成 HAL、72 MHz 时钟树、GPIO、I²C2、TIM1/2/3/4、USART1/2 初始化，启动零占空比 PWM 与编码器计数，再初始化 OLED 和 MPU6050 DMP，开启蓝牙逐字节接收，最后调用 `App_RTOS_Start()`。

`App_RTOS_Start()` 创建 I²C 互斥锁、状态队列、命令队列及三个任务，然后启动调度器。TIM1 只保留 CubeMX 里的 200 Hz 时基配置，计数器和 TIM1_UP 中断都不启动；5 ms 控制在 `ControlTask`。

## 中断与任务边界

- `USART2_IRQHandler` 把字节写入环形缓冲并通知通信任务，通信任务再组帧。接收中断在溢出错误后会重新打开。
- `SysTick_Handler`：先增加 HAL tick，在 `USER CODE` 段调用 `App_ControlWatchdogTick`，然后在调度器启动后调用 `xPortSysTickHandler`。看门检查不调用 RTOS API；未使能电机时不动作。
- `SVC_Handler / PendSV_Handler`：通过 `FreeRTOSConfig.h` 的映射由官方 Cortex-M3 移植层实现。`.ioc` 关闭了这两个处理函数的生成，避免和移植层重复定义。
- 通信任务把 ARM / STOP 放入命令队列；控制任务拥有正常电机输出和 PID 状态，其他任务不直接改写控制变量。SysTick 仅在输出看门超时后执行紧急切断。

蓝牙可以连续发送多条 `$...#` 帧。USART2 中断优先级为 5；内核允许调用 FromISR API 的最高逻辑优先级也为 5，SysTick 和 PendSV 使用优先级 15。

## 共享 I²C

运行期的 MPU6050 FIFO 读取整体持有互斥锁，OLED 在每条命令或每个 16 字节数据块传输时持锁。锁采用 FreeRTOS 的优先级继承机制，传给 HAL 的传输超时为 2 ms，避免整个 OLED 帧占用总线。注意 STM32F1 HAL 的 BUSY 等待仍使用内部 25 ms 超时，不能将整次调用视为最多 2 ms。连续失败达到 3 次时，在已持有的互斥锁内复位 I²C（DeInit、GPIO 上 9 个 SCL 脉冲、重新初始化），并在 USART1 记录错误；这些阻塞操作不妨碍 SysTick 的电机切断检查。

初始化阶段调度器尚未启动，总线按顺序访问。此时 `App_I2C_Lock / Unlock` 不调用互斥锁 API。

## 姿态与控制

MPU6050 保留 InvenSense DMP 路径：200 Hz 采样及 FIFO，启用六轴四元数、陀螺仪校准、加速度与校准角速度输出。身份安装矩阵对应标量 `136`。

四元数从 Q30 转为浮点后计算 Y 轴俯仰角，角速度按 ±2000 °/s 的量程换算。`DMP_Pitch` 最多读取 4 包，仅在有效包的 `more == 0` 时一次性发布姿态与角速度。达到上限仍有积压、缺少四元数或部分读取后出错，返回 `DMP_PITCH_UNSAFE`，保留上一帧但立即解除使能；后续周期继续排空，追上后仍需新的 ARM。首次读取无包时返回 `DMP_PITCH_NO_SAMPLE`，超过 25 ms 无有效姿态则停止电机。传感器安装坐标应与实际车体一致。

电机输出看门使用 HAL 毫秒时基，只在控制任务提交有效 PWM 时更新。超过 `APP_CONTROL_WATCHDOG_MS`（10 ms，即两个正常控制周期）未更新，SysTick 通过 `Motor_EmergencyStop` 清零两个 PWM 比较寄存器并拉低方向引脚，不等待 I²C 或任务恢复。控制任务提交 PWM 前，在短临界区内再次检查超时与故障标志，防止中断切断后又被迟到的控制输出覆盖。故障返回时丢弃该轮排队的 ARM，重新运行需要后续的新 ARM。依赖 SysTick 正常运行；长时间屏蔽中断或 CPU 故障不在此软件保护范围内。

控制保持原工程的直立 PD 与速度 PI 输出叠加方式。速度反馈为左右轮每 5 ms 的编码器脉冲之和，显示单位为 `ENC/5ms`。速度 PI 保留一阶低通和积分限幅；停止状态重置其内部状态。

## 内存与构建

FreeRTOS 的 10 KB 堆包含任务栈、任务控制块、队列和互斥锁。GNU 链接脚本另为 newlib 格式化分配 1 KB 堆，为主栈保留至少 1 KB 空间。运行期只有显示任务使用 `snprintf`，FreeRTOS 对象由 `heap_4` 分配。

GNU 构建从 `.uvprojx` 提取 C 源文件清单，替换为 GNU 启动文件并增加 `GCC/syscalls.c`。Keil 使用原有启动汇编及 Arm Compiler 6。

## CubeMX 重新生成时保留的接入点

1. `main.c` 的应用初始化在 `USER CODE BEGIN 2`，外设初始化列表留给 CubeMX。
2. `stm32f1xx_it.c` 的 SysTick 衔接在 `USER CODE BEGIN SysTick_IRQn 1`。不要重新打开 SVC / PendSV 处理函数生成。
3. `usart.c` 的 TX 复用推挽、RX 输入配置和 USART2 中断优先级。
4. `oled.c` 的短传输与总线锁，以及 `pt_mpu6050.c` 的 DMP 初始化。
5. Keil 工程内的 FreeRTOS 源文件、头文件目录和 `app_rtos.c`。

生成后运行 `python scripts/check_project.py`、`python scripts/test_control_safety.py`（需主机 C 编译器）和 `python scripts/build.py`，再按硬件调试文档确认运行行为。
