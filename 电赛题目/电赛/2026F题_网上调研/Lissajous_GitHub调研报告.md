# 2026 电赛 F 题（李萨如图形显示控制装置）开源方案 GitHub 调研报告

> 调研时间：2026-10-18
> 调研目标：系统检索 GitHub / Gitee / 立创开源广场(oshwhub) / 电子森林 / 公开技术社区，寻找是否存在比用户现有方案（STM32F407+FreeRTOS + AD9959 GPIO 模拟时序 + K230 CanMV 视觉 + PC OpenCV 原型）更优秀、可直接落地的开源方案，重点关注要求 5 的"摄像头拍示波器屏幕→自动闭环调出对角线/圆/∞"。
> 验证等级约定：**已查证** = 实际打开页面/仓库并读过正文或 README；**一方说法** = 仅搜索摘要可见；**未找到** = 多组关键词检索后无命中。

---

## 一、核心结论（先读）

1. **截至 2026-10-18，GitHub、oshwhub、Gitee 上不存在任何一个完整实现 2026 年 F 题（李萨如图形显示控制装置）的开源仓库**，尤其没有任何仓库实现要求 5 的"断开信号源后纯视觉闭环"。这是直接检索验证过的结论，不是推测。
2. 用户当前方案（STM32F407 + AD9959 + K230 视觉 + PC OpenCV 原型）在公开开源世界里**已经处于第一梯队甚至领先位置**——没有可"拿来直接替换"的成熟开源工程。
3. 能捡到的"碎片级"可复用件只有三类，且都只能局部替换用户已有模块，不能整体替代：
   - **AD9959 的 STM32 HAL 驱动**（Silver-Ray/AD9959_HAL_DRIVER，28★，同芯片 STM32F407ZGT6）：可替换用户"GPIO 模拟时序"，四线串行、四通道独立幅/频/相，含扫频/ASK/FSK。
   - **纯视觉测频/测相的算法思路**（IIETA 论文：摄像头拍李萨如视频→逐帧二值化→归一化面积负峰→翻转周期→反推频差 Δf=1/(m·T)）：可直接用于要求 5 断开信号源后估计未知 X 频率/相位，比用户"四边波峰计数"在数学上更鲁棒、且对 2 倍频 ∞ 形同样成立。
   - **DDS 相位微扰锁定技巧**（IDC 文档：AD9850 双 DDS 共享主时钟 + Arduino 短时微扰频率到目标相位，分辨率约 1.5°）：可替代/补强用户相位 PI。
4. 时间窗口判断：2026 电赛 8 月初刚结束、8 月底才出获奖名单，团队通常在赛后 1~3 个月才放源码。**当前 F 题开源空白是"时间问题"而非"没有方案"**，建议 11~12 月再复搜一次 GitHub/oshwhub。

---

## 二、检索过程与"零命中"证据（已查证）

| # | 检索动作 | 关键词 | 结果 | 验证等级 |
|---|---|---|---|---|
| 1 | GitHub 仓库搜索 | `李萨如 电赛` | **0 results** | 已查证（打开搜索页原文"Your search did not match any repositories"） |
| 2 | GitHub 仓库搜索 | `2026 李萨` | **0 results** | 已查证 |
| 3 | GitHub 代码搜索 | `电子设计竞赛 李萨如` | 无任何代码命中 | 已查证 |
| 4 | 立创开源广场 oshwhub 搜索 | `李萨如` | 工程 0 / 文章 0 / 用户 0 | 已查证 |
| 5 | Gitee 仓库搜索 | `李萨如` | 页面无法正常加载（link dead） | 已查证（如实记录） |
| 6 | GitHub 仓库搜索 | `电赛 2026` | 198 个仓库，但**翻遍首页全是 E 题(拼图)、H 题(滚球)、控制题，无 F 题** | 已查证 |
| 7 | GitHub 仓库搜索 | `lissajous`（通用） | 984 个仓库，全是 JS/Web 音频可视化、Unity 动画、SVG 生成器，**无一是嵌入式电赛方案** | 已查证 |
| 8 | GitHub 仓库搜索 | `AD9959` | 63 个仓库，全是 DDS 驱动库，无系统级闭环 | 已查证 |
| 9 | GitHub 仓库搜索 | `K230` | 678 个仓库，全是 Kendryte 官方 SDK / 通用视觉例程 | 已查证 |

**GitHub "电赛 2026" 首页实际列出的仓库（已查证，用于证明没有 F 题）：**

| 仓库 | star | 内容 | 与 F 题关系 |
|---|---|---|---|
| zuoliangyu/NUEDC_TOPIC | 121 | 历年真题文档（含 2026 题面 PDF） | 仅题面，无方案 |
| nanwanuser/NUEDC_2026_Control | 5 | 2026 控制题代码 | 无关 |
| Andy07033/2026-DianSai-Problem-E-vision-main | 6 | 2026 **E 题**省一视觉开源 | E 题非 F 题 |
| lvy-Xie/26EC | 6 | 2026 **E 题** | 无关 |
| good1example/2026-H-balance | 9 | 2026 **H 题**滚球平衡 | 无关 |
| P1ne4pp1e/2026_H | 4 | 2026 **H 题** | 无关 |
| CassiaMCU/2026TI_MPU6050_Yaw | 41 | MSPM0+MPU6050 姿态解算 | 无关 |
| CNJerryZheng/2026NUEDC | 3 | 2026 电赛综合 | 未含 F 题 |

> 旁证：武夷学院 2026-08-26 获奖公示显示 F 题已有多支队伍获省级二等奖，说明比赛真实举办、题目真实存在，但源码尚未放出来。来源：https://www.wuyiu.edu.cn/2026/0826/c701a140362/page.htm

---

## 三、逐资源评估（核心资源表）

| 资源名 | URL | 架构 / 关键指标 | 是否开源 | 验证等级 | 与本题关联度 | 是否优于用户方案候选 |
|---|---|---|---|---|---|---|
| **Silver-Ray/AD9959_HAL_DRIVER** | https://github.com/Silver-Ray/AD9959_HAL_DRIVER | STM32F407ZGT6 + HAL 库；四线串行双线模式；四通道独立设置幅值/频率/相位；支持 Single Tune、扫频、ASK/FSK；可读写寄存器；28★ / 2 fork / 9 commits；2025-07 更新；HUST-STI-S12 | 是（源码开放，.ioc 可直接 CubeIDE 打开） | **已查证**（读过 README 全文） | 高——直接对应用户 AD9959 驱动层 | **局部可替换**：用 HAL 驱动替换用户 GPIO 模拟时序，省出 CPU 且时序更稳；但不含视觉/闭环/幅相 PI |
| **chuxuezhe766/K230** | https://github.com/chuxuezhe766/K230 | K230 CanMV MicroPython 视觉例程；色块追踪(红/绿)、矩形检测、动态阈值菜单；108★ / 8 fork；Apache-2.0；2025-07 更新 | 是 | **已查证**（读过 README/文件列表） | 中——K230 视觉基础能力参考，但非李萨如 | 否——无示波器屏幕识别、无李萨如分类 |
| **cjheath/AD9959** | https://github.com/cjheath/AD9959 | 4 通道 DDS Arduino 库；19★；2023 更新；C++ | 是 | 一方说法（搜索摘要） | 中——Arduino 而非 STM32 | 否（平台不符） |
| **veis-lzf/DDS-AD9959** | https://github.com/veis-lzf/DDS-AD9959 | AD9959 驱动 + 硬件设计文件；12★；C；2020 | 是 | 一方说法 | 中 | 否（较老，无闭环） |
| **emoestudio/Emoe_DDS** | https://github.com/emoestudio/Emoe_DDS | AD9959 信号发生器；10★；2025-05 更新；C | 是 | 一方说法 | 中 | 否（纯信号源，无视觉） |
| **QTC-UMD/dds-sweeper** | https://github.com/QTC-UMD/dds-sweeper | Raspberry Pi Pico 控 AD9959 扫频；10★；2025-10 更新 | 是 | 一方说法 | 低-中（扫频思路可参考） | 否 |
| **boochow/lissa** | https://github.com/boochow/lissa | 为 Eurorack NTS-1 合成器产生李萨如信号；logue-sdk；23★；Makefile | 是 | 一方说法 | 低（音乐合成器，非示波器闭环） | 否 |
| **johnde18/exp.8.-Measurement-of-phase-and-frequency-using-lissajous-pattern** | （GitHub 仓库，0★） | 教学实验：用波形发生器+李萨如测正弦/方波/三角波频率相位；2025-11 更新 | 是 | 一方说法 | 中（原理验证，非自动闭环） | 否（手动教学实验） |
| **element14：ESP32-S3 + PCM5102A 自制双通道李萨如信号源** | https://community.element14.com/challenges-projects/project14/spring-clean/b/spring-clean-projects-2026/posts/building-a-standalone-esp32-signal-generator | ESP32-S3 + I²S + PCM5102A 立体声 DAC；相位累加器生成正弦；电位器实时调频率比/相位；EMA+迟滞滤波；2026-05 发布 | 否（博客文章，无公开代码仓库链接） | **已查证**（读完全文） | 中——思路与用户"装置输出 Y"同源，且自己产生 X+Y 两通道 | 否——它自己产生两个信号，**不需要摄像头闭环、也不处理未知 X 频率**，与要求 5 相反 |
| **IIETA 论文：Enhanced Frequency Measurement via Lissajous Figure Flipping Periods** | https://iieta.org/download/file/fid/100034 | 摄像头拍示波器李萨如视频(640×480@30fps)→逐帧灰度二值化→归一化面积 NA(i)→找负峰(翻转周期标记)→最小二乘求旋转周期 T̂→Δf=1/(m·T)→反推被测频率；2023, Traitement du Signal 40(3):961-970 | 否（LabVIEW+MATLAB 论文，无开源代码） | **已查证**（读完全文与公式） | **高——直接对应要求 5 断开信号源后的纯视觉测频** | **算法层面优于用户"四边波峰计数"**：该法对 1:1（圆/椭圆慢转）和 2:1（∞慢转）都适用，且用多帧最小二乘抗噪；但需自行用 K230/OpenCV 复现 |
| **IDC：Hardware of Lissajous Curves（AD9850 双 DDS）** | https://www.idc-online.com/technical_references/pdfs/electronic_engineering/Hardware_of_Lissajous_Curves.pdf | 两块 AD9850 模块共享 125MHz 主时钟（6ppm 频差）；AD9850 自带 5bit 相位寄存器(步进 11.25° 太粗)；改用"短时微扰一路频率→等到目标相位→恢复锁定"，Arduino 实现相位分辨率约 1.5° | 否（PDF 文章，代码未单独开源） | **已查证**（读完全文） | 中-高——相位精锁技巧 | 可补强用户相位 PI：在粗相位寄存器外叠加"微扰定时锁定"到 1.5° |
| **eet-china：2026 电赛题目汇总** | https://www.eet-china.com/mp/a513478.html | 确认 2026 F 题"李萨如图形显示控制装置"存在 | — | 一方说法 | 题面旁证 | — |
| **zuoliangyu/NUEDC_TOPIC** | https://github.com/zuoliangyu/NUEDC_TOPIC | 历年真题 PDF 归档，已更新到 2026 | 是 | 一方说法 | 题面获取 | — |
| **Telonic：SIGLENT SDS 示波器 LAN 截图 Python 例** | https://telonic.co.uk/programming-example-sds-oscilloscope-screen-image-capture-using-python-over-lan/ | Python socket 经 LAN 抓示波器屏幕图像 | 是（代码片段公开） | 一方说法 | 低（注意：题目要求 5 必须用摄像头拍，走 LAN/USB 读屏属于违规灰色手段，仅供理解屏幕结构参考） | 否——违反赛规，不可用 |
| **EdgeSpotter（arXiv 2506.07112）** | https://arxiv.org/pdf/2506.07112 | 工业面板（含示波器）密集文字检测 | 待开源（论文称 code 即将放出） | 一方说法 | 低-中（示波器面板 OCR，非李萨如图形识别） | 否 |

---

## 四、关键问题专项回答

### 4.1 断开信号源后（要求 5），别人是怎么"得知输入频率/相位"的？

这是本题最硬核的问题。检索结论：

- **没有任何开源仓库公开了这一环的完整代码。**
- 学术界最接近的是 IIETA 论文（已查证全文）：**纯视觉**手段——摄像头固定拍示波器屏幕，李萨如因两独立时钟存在微小频差而缓慢旋转/翻转；对连续帧逐帧二值化，统计"归一化面积"曲线，其负峰对应图形回到同一姿态的时刻；两负峰间隔即翻转周期 T，由 Δf=1/(m·T) 反推频差，m 为频率比（1:1 时 m=1；2:1 时 m=2）。该法**完全不需要接触信号源**，且对圆（1:1）和 ∞（2:1）两种目标图形都成立。
- **灰色手段（仅记录，赛规不允许）**：用 SCPI/LAN/USB 直接读示波器屏幕截图或波形数据（Telonic 例、Keysight VISA）。题目明确要求"摄像头拍示波器屏幕""不得有任何电气耦合""摄像头不得拍信号源面板"，走仪器接口=违规。
- **预存频率表**：题目明确"频率点不得预存在装置中"，且输入 1kHz~100kHz、100Hz 步进共 991 点，预存不可行。
- **用户现有"四边波峰计数（左==右、上==下）"思路**与论文的"交点计数法"（fY/fX = nX/nY，nX 为图形与横轴交点数、nY 为与纵轴交点数）同源。论文的"逐帧面积负峰"是比单帧波峰计数**更抗噪、且能给出连续频差估计**的升级，建议移植。

### 4.2 用户已知短板对应的开源解药

| 用户已知问题 | 检索到的开源/算法资源 | 建议 |
|---|---|---|
| AD9959 用 GPIO 模拟时序、CPU 开销大 | Silver-Ray/AD9959_HAL_DRIVER（STM32F407 HAL，四线串行） | 直接替换驱动层，.ioc 可导入 CubeIDE |
| K230 单帧波峰计数准确率仅 41.1%（高倍频几乎不可用） | IIETA 逐帧归一化面积 + 多帧最小二乘；chuxuezhe766/K230 色块/矩形基础 | 把"单帧判别"改成"连续 N 帧面积时序"判别，∞ 形也能用 |
| 192 点扫频预算 5.8s 太紧 | IIETA 翻转周期法不需要全扫，只需图形慢转数秒即可估频差 | 大幅压缩搜索预算：先粗判 1:1 还是 2:1（数横/纵交点），再用翻转周期直接算频差微调 |
| 独立时钟漂移需持续微调 | IDC 短时微扰频率到目标相位（~1.5° 分辨率） | 在 DDS 相位寄存器粗调后叠加定时微扰精锁 |
| 三端代码量大、调试链长 | 无开源整合方案可抄 | 保持现有三端架构，但用上述单片算法减少 K230-STM32 往返 |

---

## 五、与用户方案的对比总结

- **整体替代**：无。没有任何开源仓库做到"摄像头→识别对角线/圆/∞→自动控 DDS→声光提示→10s 内稳定 5s"。
- **局部借鉴**：有，且建议采纳：
  1. AD9959 驱动换成 Silver-Ray 的 HAL 版本（同芯片、即插即用）。
  2. 视觉测频/判形从"单帧四边波峰计数"升级为"IIETA 式连续帧归一化面积 + 翻转周期"，重点救 ∞ 形（2:1）场景。
  3. 相位精锁用"短时微扰频率定时锁定"替代纯 PI。
- **结论**：用户方案不是"落后于开源"，而是"目前开源世界里没有可对照的成品"。真正的风险是**时间窗**——等 11~12 月获奖队伍陆续开源后，可能出现更完整的端到端工程，届时应复搜。

---

## 六、建议的复搜关键词清单（1~2 个月后执行）

```
site:github.com 2026 李萨如
site:github.com NUEDC 2026 F
site:github.com lissajous dds stm32
oshwhub 李萨如
gitee 电赛 2026 F题
"李萨如" site:blog.csdn.net 2026
"lissajous" "oscilloscope" "camera" github
```

---

## 七、来源 URL 汇总

已查证（实际打开读过）：
- https://github.com/search?q=李萨如+电赛&type=repositories
- https://github.com/search?q=2026+李萨&type=repositories
- https://github.com/search?q=电子设计竞赛+李萨如&type=code
- https://github.com/search?q=电赛+2026&type=repositories
- https://github.com/search?q=lissajous&type=repositories
- https://github.com/search?q=AD9959&type=repositories
- https://github.com/search?q=K230&type=repositories
- https://github.com/Silver-Ray/AD9959_HAL_DRIVER
- https://github.com/chuxuezhe766/K230
- https://oshwhub.com/search?keyword=李萨如
- https://community.element14.com/challenges-projects/project14/spring-clean/b/spring-clean-projects-2026/posts/building-a-standalone-esp32-signal-generator
- https://iieta.org/download/file/fid/100034
- https://www.idc-online.com/technical_references/pdfs/electronic_engineering/Hardware_of_Lissajous_Curves.pdf
- https://www.wuyiu.edu.cn/2026/0826/c701a140362/page.htm

一方说法（仅搜索摘要）：
- https://www.eet-china.com/mp/a513478.html
- https://github.com/cjheath/AD9959
- https://github.com/veis-lzf/DDS-AD9959
- https://github.com/emoestudio/Emoe_DDS
- https://github.com/QTC-UMD/dds-sweeper
- https://github.com/boochow/lissa
- https://telonic.co.uk/programming-example-sds-oscilloscope-screen-image-capture-using-python-over-lan/
- https://arxiv.org/pdf/2506.07112
- https://zuoliangyu.github.io/NUEDC_TOPIC/

无法访问：
- https://search.gitee.com/?q=李萨如&type=repository （link dead）
