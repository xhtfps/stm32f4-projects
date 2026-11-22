# F 题 STM32F407 框架

本工程由 `简易相位差测量仪` 模板独立复制而来，保留原有 800×480
LCD 的绘制函数、左右分栏、字号、菜单选择方式和 FreeRTOS 任务组织，
只把业务内容替换为 F 题“李萨如图形显示控制装置”。

## 当前已经完成

- PB10 / TIM2_CH3 外部输入上升沿捕获；
- PB11 / TIM2_CH4 AD9959实际输出反馈上升沿捕获；
- TIM2 计数时钟 84 MHz、32 位自由运行计数器；
- DMA1_Stream1/Stream6 分别批量读取 CCR3/CCR4 时间戳；
- 一次结果自动覆盖多个输入周期，降低单边沿量化误差；
- 5 点中值滤波和 EMA 平滑；
- PB10同时输出精确频率和按100 Hz四舍五入的题目频点；
- PB11输出反馈频率不量化，保留小数供频差闭环使用；
- 频率窗口最小值、最大值和峰峰抖动 ppm；
- 350 ms 无边沿超时；
- PA1/ADC1 与 PA2/ADC2 规则同步采样、TIM8 TRGO 和 DMA2_Stream0；
- 自适应 `12×fin` 采样率，覆盖 1 kHz～100 kHz 输入；
- 双频单点 DFT：要求2计算 `φy-φx`，要求3计算 `φy-2φx`；
- 输入/输出 Vpp、1.635 V 偏置、削顶和DMA溢出检测；
- LCD 五个分项菜单与要求5三种 PC 视觉目标接口；
- USART6 PC6/PC7、230400 8N1 的要求5 ASCII 从机协议；
- 要求5搜索黄灯、保持蓝灯、完成绿灯和失败红灯状态；
- F407 按基地模板固定引脚，通过GPIO模拟串行时序直接控制AD9959；
- AD9959 支持整数 Hz 接口和要求5使用的毫赫兹命令接口；
- 多通道参数可先缓存，再通过一次 IO_UPDATE 同步生效。

## 测频接线

| 信号 | STM32F407 引脚 | 说明 |
|---|---|---|
| 测频输入 | PB10 / TIM2_CH3 | 只允许 0～3.3 V 数字信号 |
| 地 | GND | 信号源与 F407 共地 |

禁止把负电压、超过 3.3 V 的方波或示波器模拟通道信号直接接到 PB10。
正弦波必须先经过高速比较器/施密特触发器整形，并完成限幅保护。

当前接受范围严格为 1 kHz～100 kHz，输出频率恒为 100 Hz 的整数倍。
例如内部滤波结果为 12 347 Hz 时，应用层和 LCD 显示 12 300 Hz，
同时输出频点序号 123，不会出现带小数的频率。

> 重要：上述有线测频仅用于前期调试、DDS 校准和硬件自检。题目自动
> 模式明确要求装置与信号源之间没有电气连接，所以比赛自动测量时不得
> 把未知信号源接到 PB10，频率搜索必须依靠 PC 摄像头对示波器图形的视觉反馈。

## LCD 菜单

1. 要求2正交：PB10+PA1/PA2，测同频等幅和 `φy-φx`；
2. 要求3倍频：PB10+PA1/PA2，测2倍频等幅和 `φy-2φx`；
3. 要求4幅值：数字键2/4/6/8选择Y轴目标档位；
4. 要求5自动：数字键1/2/3选择对角线、圆、无穷；只用 PC 视觉串口；
5. 调试：回车运行原测频、双ADC偏置/Vpp、DMA和FFT相位测试；数字键
   1/2/3/4独立测试AD9959 CH0的1/10/100/200 kHz输出，数字0关闭DDS；
   5/6调整原始FTW，7/9调整相邻FTW混合比例，8开关FTW抖动。

板载 KEY1～KEY3 对应数字 1～3，WK_UP 对应回车；完整菜单操作建议使用
PS/2 键盘。退格键停止当前操作并返回主菜单。

## 要求5 PC 串口接线

| USB-TTL | STM32F407 | 说明 |
|---|---|---|
| TXD | PC7 / USART6_RX | 交叉连接 |
| RXD | PC6 / USART6_TX | 交叉连接 |
| GND | GND | 必须共地 |

使用 3.3 V USB-TTL，禁止将 5 V TXD 接入 F407。参数为 `230400 8N1`。自动模式
只初始化 `VisionSlave` 并控制 AD9959 CH0，不启动 PB10、PA1、PA2、频率计或
双 ADC。当前工程没有蜂鸣器驱动，完成提示实现为 LCD `AUTO COMPLETE` 加绿灯；
若硬件另接蜂鸣器，应在明确引脚后复用 `DONE` 分支触发。

菜单5的AD9959测试固定使用118 mVpp、0°，避免第一次联调时输出过大。
测试点使用CH0：测DDS模块时应连接CH0的I/V和滤波后单端输出，测完整链路
时连接CH0经OPA1612后的最终输出。AD9959的`IOUT0/IOUT0B`是差分电流输出，
不能在没有I/V转换的情况下直接接示波器电压输入。

FTW抖动以10 ms周期在`base`与`base+1`之间切换，比例为0/16～16/16。
25 MHz系统时钟下，每个比例档位对应约0.364 mHz的平均频率变化。切换过程
只写CFTW0，不改相位和幅度，也不清零相位累加器。

## AD9959 固定 GPIO 接线

当前工程严格沿用基地标准库 `Drive_AD9959.h` 的引脚，不允许自行改线：

| F407 引脚 | AD9959 引脚 | 说明 |
|---|---|---|
| PE5 | CS | 片选，低电平有效 |
| PE3 | SCLK | GPIO模拟串行时钟 |
| PB0 | IO_UPDATE | 将I/O缓冲区参数同步更新到DDS |
| PF2 | RESET | 高电平复位 |
| PE4 | SDIO0 | 单线串行数据 |
| PE2 | SDIO1 | 单线模式下由程序保持低电平 |
| PE0 | SDIO2 | 单线模式下由程序保持低电平 |
| PE1 | SDIO3 | 单线模式下由程序保持低电平 |
| PF4 | PDC/PWR | 程序输出低电平，正常工作 |
| PF5 | PROFILE0 | 当前固定为低电平 |
| PF3 | PROFILE1 | 当前固定为低电平 |
| PF1 | PROFILE2 | 当前固定为低电平 |
| PE6 | PROFILE3 | 当前固定为低电平 |
| 3.3 V / GND | 数字接口电源 / 地 | 必须与 F407 共地 |

由于PE3/PE4不是同一组硬件SPI复用引脚，驱动与基地模板一样使用GPIO模拟
串行时序：SCLK空闲低、MSB先行、AD9959在上升沿采样。SPI3源文件仍可供
其他器件使用，但已从当前Keil目标移除，不参与AD9959控制。

这些引脚与外部SRAM的部分FSMC地址/字节选通信号复用。当前F题工程只初始化
内部SRAM和CCM，LCD使用PE7～PE15及PF0，因此能够使用上述AD9959固定引脚。
后续不得再同时启用占用PE0～PE6、PF1～PF5的外部SRAM。

默认配置假设 AD9959 模块使用 **25 MHz 晶体**，并关闭内部 PLL：

```text
系统时钟 = 25 MHz
FTW物理分辨率 = 25 MHz / 2^32 = 0.005820766 Hz
```

要求2～4已经接入有线PI控制服务，频率命令单位为0.001 Hz，并自动使用
`base/base+1`的16级Σ-Δ平均；25 MHz系统时钟下长期平均分辨率约为
0.364 mHz。要求5的光学锁频由 PC 独立视觉状态机负责，不能调用有线PI。

```c
/* 要求2～4：CH0，100 kHz，471 mVpp，90°。 */
AD9959_SetChannelOutput(0U, 100000U, 471.0f, 90U, 1U);

/* 要求5：CH0，100000.025 Hz，471 mVpp，90.000°。 */
AD9959_SetChannelOutputPrecise(
    0U, 100000025U, 471.0f, 90000U, 1U);
```

`AD9959_DEFAULT_FULL_SCALE_MV=500 mVpp` 只是初始换算参数，必须根据所用
AD9959 模块的实测满量程输出重新标定。模块的 `CLK_MODE_SEL` 等硬件配置也
必须处于与 25 MHz 晶体直驱相符的状态。

旧的F1/UART通信文件仍保留在源码目录中供追溯，但已经从当前Keil目标
移除，F407的所有DDS控制均进入新的固定GPIO驱动。

## 编译

### VS Code

1. 用 VS Code 打开本文件所在的整个文件夹；
2. 按 `Ctrl+Shift+B`；
3. 选择或直接运行默认任务“Keil：编译 F题 F407”。

### Keil

打开：

`Projects/MDK-ARM/atk_f407.uvprojx`

输出文件：

`Output/f407_lissajous.axf`

## 关键代码

- `Drivers/BSP/PHASE/phase_capture.c`：PB10/CH3与PB11/CH4共TIM2双DMA捕获；
- `Drivers/BSP/PHASE/phase_adc.c`：PA1/PA2 + TIM8 + DMA2 同步采样；
- `Drivers/BSP/PHASE/phase_fft.c`：同频/双频单点 DFT；
- `Drivers/BSP/MEASUREMENT/wired_measurement.c`：幅值和相位关系服务；
- `Drivers/BSP/FREQUENCY/frequency_meter.c`：测频换算与滤波；
- `Drivers/BSP/CONTROL/wired_control.c`：要求2～4频率、幅值、相位PI及失锁回退；
- `Drivers/BSP/AD9959/Drive_AD9959.c`：固定GPIO时序、FTW/CPOW/ASF与同步更新；
- `User/f_app.c`：F 题应用状态机；
- `User/freertos_demo.c`：按键、UI、仪器三任务协同；
- `User/user.c`：沿用模板样式的 LCD 页面内容。

## 要求5上板验收

依次验证单点 `SET/APPLIED`、连续十点、192 点搜索、37 度双相位确认、三种目标、
相位扫描、幅度修正和 5 秒保持。记录 PC 窗口中的 `set_to_applied_ms`、
`applied_to_frame_ms`、`frame_process_ms` 和总锁定时间。Keil 编译通过并完成真实
硬件测试前，不应把离线测试结果等同于 10 秒实机验收。
