# 第三方组件与来源

| 组件 | 来源 | 仓库内的许可信息 |
| --- | --- | --- |
| STM32F1 HAL | 随原始工程提供；STMicroelectronics | `cubemx_config/Drivers/STM32F1xx_HAL_Driver/License.md` |
| CMSIS | 随原始工程提供；Arm | `cubemx_config/Drivers/CMSIS/LICENSE.txt` |
| STM32F1 设备支持与启动文件 | 随原始工程提供；STMicroelectronics | `cubemx_config/Drivers/CMSIS/Device/ST/STM32F1xx/License.md` 及文件头 |
| FreeRTOS Kernel V10.5.1 | [FreeRTOS 官方仓库固定版本](https://github.com/FreeRTOS/FreeRTOS-Kernel/tree/V10.5.1) | `cubemx_config/Middlewares/Third_Party/FreeRTOS/LICENSE.md`，MIT |
| InvenSense MPU / DMP 驱动 | 随原始工程提供 | `inv_mpu*`、`dmpKey.h`、`dmpmap.h` 的原始版权和许可引用 |

FreeRTOS 仅收录本工程使用的内核、头文件、Cortex-M3 GCC 移植层和 `heap_4`。未修改其官方实现。其余应用基于提供的 `pinghengche-master.zip` 整理，保留原有文件的第三方版权信息；图纸来自同批项目材料。

各组件按其原始许可与文件头执行，本说明不改变第三方组件的许可条件。
