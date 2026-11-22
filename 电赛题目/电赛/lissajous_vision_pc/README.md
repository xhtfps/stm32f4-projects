# DS2302A 李萨如图形 PC-F407 视觉闭环

本目录既可离线验证视觉算法，也可作为要求 5 的 PC 主控。正式入口同时打开摄像头
和串口，先以 10/20/30/50/70 kHz 五点粗扫识别 10~100 kHz、步进 10 kHz 的输入，
再完成 37 度双相位确认、相位/幅度修正和 5 秒稳定保持；F407
仅执行 AD9959 CH0 设置并返回 `APPLIED`。

## 已实现功能

- DS2302A 纯波形区自动寻找，以及固定机位的一次性手动四角标定。
- 透视校正为严格的 8×8 分度坐标，标准画布为 800×800 像素，即 100 px/div。
- HSV 与 RGB 黄色优势联合分割；以强黄色为种子沿弱黄色受限生长，
  补全高光和白色栅格交叉处的浅黄色轨迹，同时屏蔽顶部触发标记。
- 对角线、圆/椭圆、∞/8 字形三分类。
- X/Y 跨度、中心偏差、相关系数、相位代理量、直线残差、椭圆拟合、∞解析残差、对称性和模板 Chamfer 距离。
- `score_1x`、`score_2x` 两个可供 STM32 搜索使用的归一化评价值。
- 三帧几何掩膜累积，同时保留原始单帧用于运动评价。
- 视频稳定性、帧相似度和翻转周期的初步估计。
- 单图、目录批处理、视频、USB 摄像头、手机虚拟摄像头、HTTP/RTSP 网络视频源。
- Unicode 中文路径读写，JSON/JSONL/CSV 结果和三联调试图。
- 230400 8N1 串口、严格 ASCII 协议、序号回执和超时重发。
- PC 主控 deadline 为 60 s；达到 deadline 仍未锁定会发送 `FAILED,2` 并停止输出。
- 对角线、圆/椭圆和无穷符号的完整 PC 主控状态机。

## PC-MCP 接口

`mcp_server.py` 是一个 stdio JSON-RPC MCP 服务，不监听网络端口。它复用本目录的
视觉算法和串口链路，可供 MCP 客户端调用 `list_cameras`、`capture_frame`、
`capture_and_analyze`、`watch_camera`、`run_shape`、`set_output`、`stop`。

```powershell
cd lissajous_vision_pc
python mcp_server.py
```

三种目标波形的编号固定为：`1=diagonal`、`2=circle`、`3=infinity`。PC 可向空闲
F407 发送 `$RUN,1\r\n`、`$RUN,2\r\n` 或 `$RUN,3\r\n` 主动启动闭环；F407
收到后仍按原有 `$SET,seq,probe_id,frequency_millihz,phase_millideg,amplitude_mv,target_shape`
协议设置 AD9959 CH0，并回传 `$APPLIED`。DDS 载波保持正弦，三种“波形”指视觉闭环
要锁定的三类李萨如目标及其频率比/相位关系。

## 比赛拍摄约束

正式运行必须满足题目原文：

1. 摄像头只拍示波器屏幕，不得拍信号源的操作和面板。
2. X-Y 模式下关闭所有测量信息窗口；无法关闭的内容用黑色不透明胶带遮挡。
3. 推荐使用黑色挡板，只露出 8×8 波形栅格区。
4. 摄像头、焦距、曝光和示波器亮度固定；启动时做一次手动标定。

`典型照片` 中三张照片仍带菜单和频率框，只用于开发阶段的抗干扰检查，不能作为正式比赛安装方式。正式机位不要长期依赖自动找屏；手动标定更准确也更可复现。

## 安装

已验证环境为 Python 3.13、OpenCV 4.13、NumPy 2.4。进入本目录后执行：

```powershell
python -m pip install -e .
```

也可以不安装，直接在本目录执行 `python -m lissajous_vision ...`。

## 使用 VS Code 启动

在 VS Code 中选择“文件 → 打开文件夹”，打开本目录
`D:\STM32_f4_project\电赛题目\电赛\lissajous_vision_pc`，不要只打开某个
Python 文件。工程已提供 `.vscode/launch.json` 和 Python 3.13 解释器配置。

打开左侧“运行和调试”（快捷键 `Ctrl+Shift+D`），在顶部选择：

- `摄像头1：已标定运行`：正常使用，读取 `configs/my_camera.json`；
- `摄像头1：自动寻找栅格`：没有标定文件时临时使用；
- `重新标定摄像头1`：重新点击四角并覆盖保存标定文件；
- `单图：圆4div`：用标准图片检查安装；
- `运行全部测试`：运行自动回归测试。

选择后按 `F5` 启动，按 `Shift+F5` 停止调试。第一次打开工程时，如果 VS
Code 提示安装推荐扩展，安装 Microsoft Python 和 Python Debugger。

### 串口接线

使用 3.3 V USB-TTL，交叉连接：USB-TTL TXD -> F407 PC7，USB-TTL RXD ->
F407 PC6，并连接 GND。禁止使用 5 V 串口电平。通信参数固定为 `230400 8N1`。

直接双击或运行 `启动摄像头.py`，启动窗口会自动扫描串口；选择连接 F407 的 `COM` 口和波特率后点击“启动视觉闭环”。也可以手动输入端口，例如 `COM5`。

命令行启动时，可在 `configs/pc_fast_runtime.json` 中把 `serial_port` 设置为实际的 `COM5` 等端口。
`auto` 只在 Windows 恰好检测到一个串口时自动选择；存在零个或多个端口时程序会
列出错误并停止。也可临时设置环境变量：

```powershell
$env:LISSAJOUS_SERIAL_PORT='COM5'
```

### 一键闭环

要求 5 的一键入口是工程根目录中的 `启动摄像头.py`。点击编辑器右上角
“运行 Python 文件”，或在 PowerShell 执行：

```powershell
python 启动摄像头.py
```

程序先打开串口和摄像头，随后等待 F407 自动页的数字键 1/2/3。摄像头、串口、
网格模式、帧率、控制超时和门限集中配置在 `configs/pc_fast_runtime.json`。
窗口中按 `A` 自动找网格、按 `F` 使用
`configs/my_camera.json` 固定四角、按 `R` 清除缓存重新识别、按 `S` 保存现场图。
预览采用独立状态栏和 320 像素三联图，依次显示原图四角、透视校正后的 8x8
网格和黄色轨迹掩膜；终端每 10 帧打印一行中文识别摘要。
透视图和掩膜图会按 K230 四边窄带算法标出峰值点：左红、右蓝、上紫、
下青，并显示约分比例、计数置信度、控制阶段、输入频率候选、DDS 参数以及
`SET->APPLIED`、`APPLIED->FRAME`、单帧处理耗时。

推荐上板顺序：单点 SET/APPLIED、五点粗扫、10~100 kHz 十个输入点、双相位确认、1:1/2:1
族验证、三种目标、相位扫描、幅度修正、连续三帧锁定、5 秒保持。没有完成真实
示波器、摄像头、USB-TTL 和 F407 联调前，不应宣称已经满足 10 秒指标。

## 第一步：固定机位标定

从摄像头取一帧进行标定：

```powershell
python -m lissajous_vision calibrate --source 1 --output-config configs/my_camera.json
```

也可以用照片：

```powershell
python -m lissajous_vision calibrate --image "标定照片.jpg" --output-config configs/my_camera.json
```

依次点击波形栅格的左上、右上、右下、左下四角，按 Enter 保存；按 `R` 重选，Esc 取消。此后固定摄像头位置，并使用 `--no-auto-grid` 防止运行时重新误检：

```powershell
python -m lissajous_vision camera --config configs/my_camera.json --no-auto-grid
```

## 单图与批处理

分析一张图，并生成“原图四角 / 校正波形 / 测量掩膜”三联图：

```powershell
python -m lissajous_vision image "..\100Hz\标准波形\圆4div.jpg" `
  --debug-image outputs\circle4_debug.jpg --include-debug
```

批量处理，递归搜索图片并输出 CSV、JSON 和调试图：

```powershell
python -m lissajous_vision batch "..\100Hz" --recursive `
  --csv outputs\standard.csv `
  --json outputs\standard.json `
  --debug-dir outputs\debug
```

## 摄像头、手机和视频

当前默认摄像头设备为 `1`。普通 USB 摄像头或被手机软件注册成 Windows
虚拟摄像头时，可以省略设备编号：

```powershell
python -m lissajous_vision camera --config configs/my_camera.json --no-auto-grid
```

如果需要切换设备，仍可显式传入 `0`、`1` 或 `2`：

```powershell
python -m lissajous_vision camera 0 --config configs/my_camera.json --no-auto-grid
```

如果手机软件提供 HTTP、MJPEG 或 RTSP 地址，直接把地址作为 source：

```powershell
python -m lissajous_vision camera "http://192.168.1.20:8080/video" --config configs/my_camera.json
python -m lissajous_vision camera "rtsp://192.168.1.20:8554/live" --config configs/my_camera.json
```

离线视频：

```powershell
python -m lissajous_vision video demo.mp4 --jsonl --output-video outputs\annotated.mp4
```

窗口内按 `Q`/Esc 退出，按 `R` 清除自动角点并重新找屏，按 `S`
同时保存原始帧和调试图到 `outputs/snapshots`。无图形界面的运行环境加
`--no-show`；需要逐帧给串口桥或上位机消费时加 `--jsonl`。

预览默认使用三个 `500×500` 面板，以适配开启 125%/150% DPI 缩放的
Windows 屏幕。如果右侧 HSV 掩膜仍显示不全，可进一步缩小：

```powershell
python -m lissajous_vision camera --config configs/my_camera.json `
  --no-auto-grid --panel-size 420
```

## 输出字段

关键字段如下：

| 字段 | 含义 |
|---|---|
| `valid` | 本帧结果是否可用 |
| `shape` | `diagonal`、`ellipse`、`infinity` |
| `confidence` | 三分类置信度 |
| `span_x_div`, `span_y_div` | 波形水平/垂直全跨度，单位 div |
| `center_x_div`, `center_y_div` | 相对栅格中心偏移，单位 div；Y 向上为正 |
| `score_1x`, `score_2x` | 1:1 与 2:1 搜索目标评价值，越大越好 |
| `frame_similarity`, `motion_score` | 原始单帧相似度及运动量 |
| `flip_period_ms`, `flip_confidence` | 慢翻转周期估计及可信度 |
| `stable`, `stable_ms` | 测量窗口是否已稳定 |
| `grid_corners_px` | 原图四角，顺序 TL/TR/BR/BL |

离线 CLI 仍可输出 JSON/JSONL；要求 5 一键入口使用上级目录
`STM32视觉闭环快速协议.md` 中的 ASCII 串口协议，不发送逐帧 JSON。

## 已知边界

- 自动找屏是标定辅助，不是正式运行的精度保证；照片含强反光、菜单框或黄色波形贴边时，自动四角可能产生分度误差。
- `phase_abs_deg` 是由中心截距得到的绝对相位代理值，不能确定正负相位。
- 翻转周期至少需要约 12 帧和 0.5 秒历史；静态图片该字段为 0。
- 黄色阈值适用于当前 DS2302A 样本。改变示波器亮度、相机白平衡或增加黄光环境后，应修改配置中的 HSV/RGB 阈值。

## 验证

```powershell
python -m unittest discover -s tests -v
```

当前标准 100 Hz 数据集共 12 张（3 类 × 2/4/6/8 div），分类应为 12/12，跨度误差门限为 ±0.35 div。三张带 UI 的典型照片仅检查分类，应为圆、对角线、∞。
