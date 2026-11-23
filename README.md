# STM32F4 工程集（STM32F407）

STM32F407 的工程模板、基地调试程序、外设实践与历年电赛题目的统一存放处。所有工程均为 Keil MDK 工程，可直接打开编译（`.o/.d/.crf` 等中间产物与 IDE 缓存不入库）。

## 1. 目录一览

| 目录 | 内容 |
| --- | --- |
| `工程模板/` | 基地通用模板：HAL 库 + BSP 驱动 + 分层目录（`_01_App`～`_08_USB`），新工程从这里复制 |
| `基地调试程序/` | 板级调试与自检程序（LED/KEY/串口/LCD/AD9959/ADS1256 等逐项验证） |
| `往年题目/` | 历届电赛题目原文与参考方案 |
| `电赛题目/` | 本届/校赛题目与器件清单 |

## 2. 软硬件环境

| 项 | 要求 |
| --- | --- |
| 开发板 | 正点原子 探索者 STM32F407（`atk_f407` 工程名，板载 TFTLCD、XMRAM） |
| MCU | STM32F407ZGT6（168 MHz / 1 MB Flash / 192 KB SRAM） |
| IDE | Keil MDK-ARM 5.36+，器件包 `Keil.STM32F4xx_DFP` |
| 调试器 | J-Link（推荐）或 ST-Link V2，SWD 四线 |
| 串口 | USART1，115200-8-N-1 |

## 3. 快速开始

1. 打开 Keil，`Project → Open Project`，进入具体工程目录，选择 `*.uvprojx`（如 `工程模板/atk_f407.uvprojx`）。
2. `Project → Rebuild all target files`。
3. 连接调试器，`Flash → Download`。
4. 串口助手（115200）看打印；模板默认会跑 LED 心跳与串口打印自检。

新工程起手式：复制 `工程模板/` 整个目录 → 改名 → 修改 `_01_App/User.c` 与 `main.c` → 保留 `_02_Core/_03_Drive/_04_FWLib` 不动。

## 4. 关键外设实践

- **AD9959**（四通道 DDS）：SPI + 多路 IO 更新，代码在 `_03_Drive/` 下，用于扫频、正弦/方波发生。
- **ADS1256**（24 位 ADC）：SPI 读取，配合基准电压做高精度直流测量。
- **TFTLCD / USB**：`_07_TFT_LCD`、`_08_USB` 为可选模块，不用的外设建议在工程分组中移除。

## 5. 常见问题

- **编译找不到头文件**：`Options → C/C++ → Include Paths` 里补 `_03_Drive`、`_04_FWLib` 子目录；路径避免中文。
- **下载失败 / 芯片被锁**：Keil `Flash → Configure Flash Tools → Debug → Settings` 里把 J-Link 速度降到 1 MHz，必要时用 ST-Link Utility 全片擦除。
- **AD9959 无输出**：先确认 `CS/SCLK/SDIO0-3/IO_UPDATE/P0-P3` 引脚与板级定义一致，再查 SPI 分频（建议 ≤10 MHz）。
- **中文文件名导致 Keil 报错**：目录名含中文时，Keil 5 在部分系统上会无法打开工程，可临时改为英文目录再编译。

## 6. 维护说明

- 被排除文件清单见 [`EXCLUDED_FILES.md`](EXCLUDED_FILES.md)（条目数、总体积与排除原因分布）；逐文件明细由 `tools/audit-git.ps1` 现场生成，不入库。
- 芯片手册等大体积 PDF 不进仓库：手册统一登记在 `notes` 仓的手册索引表，仓内只留链接与版本号。
- 同源旧仓库 `STM32project`（早期综合工程，仅 2 个源文件版本不同）已归档，等价内容在本仓库 `基地调试程序/` 内。
- 第三方库、芯片手册版权归原作者，仅作学习参考。

## 大文件去哪了

本仓已做过体积治理：大文件从当前分支移出，但历史提交完整保留，随时可取回。

- **完整台账**：[`notes/offload.json`](https://github.com/xhtfps/notes/blob/main/offload.json)（repo / path / size / base_commit），恢复命令与口径见索引仓 [`SLIM-PLAN.md`](https://github.com/xhtfps/xhtfps/blob/main/SLIM-PLAN.md)。

clone 后发现某个文件不在，按台账里的 `base_commit` 打开 `https://github.com/xhtfps/<repo>/blob/<base_commit>/<path>` 取回即可，不要重复提交进仓。
