# PID 串口调参工具

## 运行

```powershell
python -m pip install -r requirements.txt
python pid_tuner.py
```

串口参数默认为 `230400, 8-N-1`，对应 STM32 的 USART1：`PA9=TX`、`PA10=RX`。
K230 视觉链路仍使用 USART6：`PC6=TX`、`PC7=RX`，两路串口相互独立。

## 协议

每条命令以换行结束：

```text
PID GET
PID SET FREQ 1.00 2.00 0.0 0.0
PID SET FREQ_LOW 0.25 0.03 0.0 0.0
PID SET PHASE 0.20 0.14 0.001 0.05
PID SET PHASE_LOW 0.08 0.10 0.001 0.05
PID SET AMP 0.015 0.030 0.0 0.0
```

`FREQ_LOW` 和 `PHASE_LOW` 是 4.5 kHz 端点的基准值。固件在
900 Hz~4.5 kHz 内自动调度：`Kp/Kd` 乘以 `f/4500`，`Ki` 乘以
`(f/4500)^2`，比例最小限制为 0.2。低频环的输出限幅分别为
`FREQ_LOW = +/-3 Hz`、`PHASE_LOW = +/-1.5 Hz`。

四个数依次为 `Kp Ki Kd derivative_filter_alpha`。前三项允许范围为
`0~1000`，微分滤波系数允许范围为 `0~1`。

设备应答使用 `@PID`、`@OK`、`@ERR` 前缀。工具会忽略同一串口上的
VOFA `ADC:` 波形数据。

参数保存在 RAM 中，复位后恢复固件里的默认值。
