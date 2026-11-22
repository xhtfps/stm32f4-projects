#include "EthMeasureCore.h"
#include <math.h>
#include <string.h>

/*------------------ 宏定义 / 常量配置 ------------------*/
#define ETH_MAX_IFFT_POINTS 1024U          /* IFFT 最大点数 */
#define ETH_MIN_SWEEP_POINTS 8U           /* 最小扫频点数 */
#define ETH_PEAK_GUARD_BINS 2U            /* 峰值搜索时两端跳过的点数（保护带） */
#define ETH_MIN_NVP_X1000 500U            /* NVP (速率因子) 千倍最小值 0.500 */
#define ETH_MAX_NVP_X1000 990U            /* NVP 千倍最大值 0.990 */
#define ETH_CALIB_LENGTH_STEP_CM 50U      /* 校准用电缆长度步进/默认值 (cm) */
#define ETH_SIGNAL_UART_BAUD 115200U      /* 信号源控制串口波特率 */
#define ETH_SIGNAL_OFFSET_HZ 10000U       /* 混频/测量中频偏移 10kHz */
#define ETH_SIGNAL_FRAME_GAP_MS 2U        /* DDS 命令间延时 (ms) */
#define ETH_SIGNAL_RANGE_MV 500.0f        /* DDS 输出幅度 (mV) */
#define ETH_ADC_IF_SAMPLE_RATE_HZ 160000.0f /* ADC 采样率 160kHz */
#define ETH_DEFAULT_FRAMES 4U             /* 默认每频点采集帧数 */
#define ETH_MAX_FRAMES 8U                 /* 最大每频点帧数 */
#define ETH_DEFAULT_FINE_SPAN_CM 100U     /* 精细搜索默认跨度 (cm) */
#define ETH_MAX_FINE_SPAN_CM 500U         /* 精细搜索最大跨度 (cm) */
#define ETH_MIN_REFERENCE_V 0.001f        /* 参考通道最小有效幅度 (V) */
#define ETH_CLIP_LOW_V 0.02f             /* ADC 限幅下限 (V) */
#define ETH_CLIP_HIGH_V 3.28f            /* ADC 限幅上限 (V) */
#define ETH_CAL_OPEN 0x01U               /* 校准标准标志：开路 */
#define ETH_CAL_SHORT 0x02U              /* 校准标准标志：短路 */
#define ETH_CAL_LOAD 0x04U               /* 校准标准标志：负载 */
#define ETH_CAL_ALL (ETH_CAL_OPEN | ETH_CAL_SHORT | ETH_CAL_LOAD) /* 三个标准全部采集 */
#define ETH_MIN_PEAK_SNR 2.5f            /* 峰值信噪比最小阈值 */

/*------------------ 全局默认配置 ------------------*/
EthernetMeasureConfig g_eth_cfg =
{
    3000000U, 170000000U, 1000000U, 20U, 660U, 1024U, 1000U,
    ETH_DEFAULT_FINE_SPAN_CM, ETH_DEFAULT_FRAMES, ETH_WINDOW_KAISER, 60U
};
EthernetMeasureRuntime g_eth_runtime = {0};   /* 运行时状态/结果 */

/*------------------ 静态工作缓冲区 ------------------*/
static float g_trace[ETH_MAX_IFFT_POINTS];      /* 存储时域幅值（对数/线性，用于显示和找峰） */
static complex g_gamma[ETH_MAX_IFFT_POINTS];    /* 当前扫频得到的反射系数 S11 或 修正后数据 */
static complex g_ifft[ETH_MAX_IFFT_POINTS];     /* IFFT 输入/输出缓冲区 */
static complex g_open[ETH_MAX_IFFT_POINTS];     /* 开路标准测量数据 / 最终用于修正的反射跟踪项 */
static complex g_short[ETH_MAX_IFFT_POINTS];    /* 短路标准测量数据 / 最终用于修正的源匹配项 */
static complex g_load[ETH_MAX_IFFT_POINTS];     /* 负载标准测量数据（方向性） */
static float g_adc0[ADCDataLength];             /* ADC 通道0 原始数据（反射） */
static float g_adc1[ADCDataLength];             /* ADC 通道1 原始数据（参考） */
static float g_adc_fs = ETH_ADC_IF_SAMPLE_RATE_HZ; /* 实际 ADC 采样率 */
static uint8_t g_cal_flags = 0U;                /* 已采集的校准标准标志位 */
static uint16_t g_cal_points = 0U;              /* 校准时使用的扫频点数 */
static uint32_t g_cal_start = 0U, g_cal_stop = 0U, g_cal_step = 0U; /* 校准时的频率参数 */

/*------------------ 内部函数声明 ------------------*/
static void EthSignalSource_Init(void);
static void EthSignalSource_SetFrequency(uint32_t freq_hz);
static uint16_t EthPoints(void);
static float EthVelocity(void);
static float EthLengthFromBin(float bin);
static uint8_t EthCaptureRaw(uint16_t count, complex *out);
static uint8_t EthFrame(complex *gamma);
static uint8_t EthPhasor(const float *samples, complex *out);
static uint8_t EthRatio(const complex *response, const complex *reference, complex *out);
static uint8_t EthClipped(const float *a, const float *b);
static void EthCorrect(uint16_t count, const complex *raw, complex *out);
static float EthWindow(uint16_t i, uint16_t n);
static float EthI0(float x);
static uint8_t EthProcess(uint16_t count);
static uint16_t EthPeak(uint16_t n);
static float EthPeakInterp(uint16_t bin, uint16_t n);
static uint8_t EthFineSearch(uint16_t count, float coarse, float *fine, float *metric);
static uint8_t EthCalMatches(uint16_t count);
static uint8_t EthBuildSol(void);
static complex CAdd(complex a, complex b);
static complex CSub(complex a, complex b);
static complex CMul(complex a, complex b);
static uint8_t CDiv(const complex *a, const complex *b, complex *out);

/*======================================================
 * 系统初始化：清屏，配置规范化，初始化信号源和 ADC
 *======================================================*/
void EthApp_Init(void)
{
    LCD_Clear(Black);
    memset(&g_eth_runtime, 0, sizeof(g_eth_runtime));
    EthNormalizeConfig();                     /* 保证所有配置参数合法 */
    EthSignalSource_Init();                   /* 串口、DDS 初始化 */
    User_ADC_Init(1U);                        /* 开启 ADC（双通道同步采集） */
    g_adc_fs = Set_SamplingFre(ETH_ADC_IF_SAMPLE_RATE_HZ); /* 配置采样率并获取实际值 */
    g_eth_runtime.dds_ready = 1U;
    g_eth_runtime.adc_ready = 1U;
    EthRefreshDdsPreview();                   /* 将信号源设置到起始频率 */
    sprintf(g_eth_runtime.last_message, "FDR ready");
}

/*------------------------------------------------------
 * 信号源底层初始化：打开串口，装载 DDS 默认参数，
 * 并将频率设置到当前起始频率。
 *------------------------------------------------------*/
static void EthSignalSource_Init(void)
{
    Init_Uart(ETH_SIGNAL_UART_BAUD);
    DDSDataInit();
    EthSignalSource_SetFrequency(g_eth_cfg.start_freq_hz);
    delay_ms(20U);
}

/*------------------------------------------------------
 * 设置两个 DDS 通道的频率：
 *   ch0  = 扫描频率 f
 *   ch1  = f + 10 kHz（为接收机提供本振/参考）
 * 同时配置扫频参数（虽然当前使用固定频率，但保留了 step 信息）
 *------------------------------------------------------*/
static void EthSignalSource_SetFrequency(uint32_t freq_hz)
{
    uint32_t f1 = freq_hz + ETH_SIGNAL_OFFSET_HZ;

    dds[0].mode=NORMAL; dds[0].output=1U; dds[0].range=ETH_SIGNAL_RANGE_MV;
    dds[0].fre=freq_hz; dds[0].fre_start=freq_hz; dds[0].fre_stop=freq_hz;
    dds[0].step=g_eth_cfg.step_hz; dds[0].step_time=(uint32_t)g_eth_cfg.settle_ms*1000U;

    dds[1].mode=NORMAL; dds[1].output=1U; dds[1].range=ETH_SIGNAL_RANGE_MV;
    dds[1].fre=f1; dds[1].fre_start=f1; dds[1].fre_stop=f1;
    dds[1].step=g_eth_cfg.step_hz; dds[1].step_time=(uint32_t)g_eth_cfg.settle_ms*1000U;

    sendData(dds[0],0U); delay_ms(ETH_SIGNAL_FRAME_GAP_MS);
    sendData(dds[1],1U); delay_ms(ETH_SIGNAL_FRAME_GAP_MS);
    sendData(dds[0],2U); delay_ms(ETH_SIGNAL_FRAME_GAP_MS);
    sendData(dds[1],3U); delay_ms(ETH_SIGNAL_FRAME_GAP_MS);
}

/*------------------------------------------------------
 * 更新 DDS 频率预览：将信号源恢复至起始频率，
 * 一般用于在测量前后消除累积的延迟影响。
 *------------------------------------------------------*/
void EthRefreshDdsPreview(void)
{
    if(g_eth_runtime.dds_ready != 0U)
    {
        EthSignalSource_SetFrequency(g_eth_cfg.start_freq_hz);
        g_eth_runtime.dds_freq_hz=g_eth_cfg.start_freq_hz;
    }
}

/*------------------------------------------------------
 * 配置参数范围检查与规范化
 *------------------------------------------------------*/
void EthNormalizeConfig(void)
{
    if(g_eth_cfg.step_hz==0U) g_eth_cfg.step_hz=1000000U;
    if(g_eth_cfg.start_freq_hz<1000000U) g_eth_cfg.start_freq_hz=1000000U;
    if(g_eth_cfg.stop_freq_hz<=g_eth_cfg.start_freq_hz)
        g_eth_cfg.stop_freq_hz=g_eth_cfg.start_freq_hz+g_eth_cfg.step_hz;
    if(g_eth_cfg.settle_ms==0U) g_eth_cfg.settle_ms=1U;
    if(g_eth_cfg.nvp_x1000<ETH_MIN_NVP_X1000) g_eth_cfg.nvp_x1000=ETH_MIN_NVP_X1000;
    if(g_eth_cfg.nvp_x1000>ETH_MAX_NVP_X1000) g_eth_cfg.nvp_x1000=ETH_MAX_NVP_X1000;
    if(g_eth_cfg.calib_length_cm<ETH_CALIB_LENGTH_STEP_CM)
        g_eth_cfg.calib_length_cm=ETH_CALIB_LENGTH_STEP_CM;
    if(g_eth_cfg.fine_search_span_cm==0U || g_eth_cfg.fine_search_span_cm>ETH_MAX_FINE_SPAN_CM)
        g_eth_cfg.fine_search_span_cm=ETH_DEFAULT_FINE_SPAN_CM;
    if(g_eth_cfg.frames_per_point==0U || g_eth_cfg.frames_per_point>ETH_MAX_FRAMES)
        g_eth_cfg.frames_per_point=ETH_DEFAULT_FRAMES;
    if(g_eth_cfg.window_type>ETH_WINDOW_KAISER) g_eth_cfg.window_type=ETH_WINDOW_KAISER;
    if(g_eth_cfg.kaiser_beta_x10>140U) g_eth_cfg.kaiser_beta_x10=60U;
    if(g_eth_cfg.ifft_points<=256U) g_eth_cfg.ifft_points=256U;
    else if(g_eth_cfg.ifft_points<=512U) g_eth_cfg.ifft_points=512U;
    else g_eth_cfg.ifft_points=1024U;
}

/*------------------------------------------------------
 * 根据当前起止频率和步进计算扫频点数
 *------------------------------------------------------*/
static uint16_t EthPoints(void)
{
    uint32_t n;
    if(g_eth_cfg.stop_freq_hz<g_eth_cfg.start_freq_hz) return 0U;
    n=((g_eth_cfg.stop_freq_hz-g_eth_cfg.start_freq_hz)/g_eth_cfg.step_hz)+1U;
    return (n<=ETH_MAX_IFFT_POINTS)?(uint16_t)n:0U;
}

/*------------------------------------------------------
 * 计算电磁波在电缆中的传播速度 (m/s)：
 * v = c * (NVP/1000) ，c = 3e8 m/s
 *------------------------------------------------------*/
static float EthVelocity(void) {
    return 300000000.0f * ((float)g_eth_cfg.nvp_x1000 / 1000.0f);
}

/*------------------------------------------------------
 * 将 IFFT 的 bin 号转换为故障距离 (m)
 * 原理：f_step 决定最大无模糊距离，bin 距离 = bin * v / (2 * N_ifft * f_step)
 *------------------------------------------------------*/
static float EthLengthFromBin(float bin)
{
    return bin * EthVelocity() / (2.0f * (float)g_eth_cfg.ifft_points * (float)g_eth_cfg.step_hz);
}

/*======================================================
 * 执行一次完整的 FDR 测量
 * 返回 1 表示成功，0 表示失败（错误信息在 last_message 中）
 *======================================================*/
uint8_t EthMeasureOnce(void)
{
    uint16_t count;
    uint16_t i;
    EthNormalizeConfig();
    count = EthPoints();
    if(count < ETH_MIN_SWEEP_POINTS) {
        sprintf(g_eth_runtime.last_message, "Sweep points invalid");
        return 0U;
    }
    g_eth_runtime.sweep_count++;
    g_eth_runtime.last_valid_points=0U;
    g_eth_runtime.last_rejected_frames=0U;
    g_eth_runtime.last_clipped_frames=0U;
    g_eth_runtime.last_peak_metric=0.0f;
    g_eth_runtime.last_peak_snr=0.0f;
    g_eth_runtime.last_phase_metric=0.0f;
    memset(g_trace, 0, sizeof(g_trace));
    memset(g_gamma, 0, sizeof(g_gamma));

    /* 步骤 1：逐频点采集原始反射系数 */
    if(EthCaptureRaw(count, g_gamma) == 0U) {
        EthRefreshDdsPreview();
        return 0U;
    }
    /* 步骤 2：根据校准数据对原始数据进行误差修正 */
    EthCorrect(count, g_gamma, g_gamma);
    /* 求幅值用于显示 */
    for(i=0U; i<count; i++) g_trace[i] = complex_abs_float(g_gamma[i]);
    /* 步骤 3：加窗、IFFT、寻峰、精细搜索 */
    if(EthProcess(count) == 0U) {
        EthRefreshDdsPreview();
        return 0U;
    }
    EthRefreshDdsPreview();
    return 1U;
}

/*------------------------------------------------------
 * 原始数据采集：按频率依次扫描 count 个点，
 * 每个点采集 frames_per_point 帧进行平均，
 * 结果存入 out 复数数组（反射系数 S11）
 *------------------------------------------------------*/
static uint8_t EthCaptureRaw(uint16_t count, complex *out)
{
    uint16_t i, frame, valid;
    float bw = (float)((count-1U) * g_eth_cfg.step_hz);

    /* 记录扫频基本参数 */
    g_eth_runtime.last_sweep_points = count;
    g_eth_runtime.last_resolution_m = (bw > 0.0f) ? EthVelocity() / (2.0f * bw) : 0.0f;
    g_eth_runtime.last_max_range_m = EthVelocity() / (2.0f * (float)g_eth_cfg.step_hz);

    for(i = 0U; i < count; i++)
    {
        complex sum, one;
        uint32_t f = g_eth_cfg.start_freq_hz + (uint32_t)i * g_eth_cfg.step_hz;

        /* 设置频率并等待稳定 */
        EthSignalSource_SetFrequency(f);
        g_eth_runtime.dds_freq_hz = f;
        delay_ms(g_eth_cfg.settle_ms);

        sum.real = 0.0f; sum.imag = 0.0f;
        valid = 0U;

        /* 多次采集取平均，提高信噪比 */
        for(frame = 0U; frame < g_eth_cfg.frames_per_point; frame++)
        {
            if(EthFrame(&one) != 0U) {
                sum = CAdd(sum, one);
                valid++;
            } else {
                g_eth_runtime.last_rejected_frames++;
            }
        }
        if(valid == 0U) {
            g_eth_runtime.adc_capture_ready = 0U;
            sprintf(g_eth_runtime.last_message, "ADC frame invalid");
            return 0U;
        }
        out[i].real = sum.real / (float)valid;
        out[i].imag = sum.imag / (float)valid;
        g_eth_runtime.last_valid_points++;
        g_eth_runtime.adc_capture_ready = 1U;
    }
    return 1U;
}

/*------------------------------------------------------
 * 获取一帧 ADC 数据并计算该频点的反射系数
 * 步骤：ADC采集 → 限幅检查 → 计算反射与参考相量 → 比值得到 Gamma
 *------------------------------------------------------*/
static uint8_t EthFrame(complex *gamma)
{
    complex r, ref;
    if(Get_ACVol(g_adc0, g_adc1) == 0U) {
        sprintf(g_eth_runtime.last_message, "ADC DMA timeout");
        return 0U;
    }
    if(EthClipped(g_adc0, g_adc1) != 0U) {
        g_eth_runtime.last_clipped_frames++;
        return 0U;
    }
    if(EthPhasor(g_adc0, &r) == 0U || EthPhasor(g_adc1, &ref) == 0U ||
       EthRatio(&r, &ref, gamma) == 0U)
        return 0U;

    g_eth_runtime.detector_voltage_v = complex_abs_float(r);
    g_eth_runtime.detector_aux_voltage_v = complex_abs_float(ref);
    return 1U;
}

/*------------------------------------------------------
 * 检查 ADC 数据是否发生削顶/削底（超出线性范围）
 *------------------------------------------------------*/
static uint8_t EthClipped(const float *a, const float *b)
{
    uint16_t i;
    for(i = 0U; i < ADCDataLength; i++)
        if(a[i] <= ETH_CLIP_LOW_V || a[i] >= ETH_CLIP_HIGH_V ||
           b[i] <= ETH_CLIP_LOW_V || b[i] >= ETH_CLIP_HIGH_V)
            return 1U;
    return 0U;
}

/*------------------------------------------------------
 * 数字正交解调：从 ADC 采样中提取中频信号的 I/Q 分量
 * 中频频率为 ETH_SIGNAL_OFFSET_HZ (10kHz)
 * 先减去直流偏置，再与正交本振相乘累加，得到复振幅
 *------------------------------------------------------*/
static uint8_t EthPhasor(const float *x, complex *out)
{
    uint16_t i;
    float mean = 0.0f, ir = 0.0f, iq = 0.0f;
    if(x == NULL || out == NULL) return 0U;

    /* 计算直流偏置 */
    for(i = 0U; i < ADCDataLength; i++) mean += x[i];
    mean /= (float)ADCDataLength;

    /* 数字锁相 / 单频 DFT */
    for(i = 0U; i < ADCDataLength; i++)
    {
        float phase = 2.0f * PI * (float)ETH_SIGNAL_OFFSET_HZ * (float)i / g_adc_fs;
        float y = x[i] - mean;
        ir += y * cosf(phase);
        iq -= y * sinf(phase);   /* 注意这里是负号，取决于参考相位定义 */
    }
    out->real = 2.0f * ir / (float)ADCDataLength;
    out->imag = 2.0f * iq / (float)ADCDataLength;
    return 1U;
}

/*------------------------------------------------------
 * 计算反射系数： Gamma = r / ref
 * 通过最小二乘或简单比值（此处为直接复数除法）
 * 若参考通道幅度过小则返回 0
 *------------------------------------------------------*/
static uint8_t EthRatio(const complex *r, const complex *ref, complex *out)
{
    float p = ref->real * ref->real + ref->imag * ref->imag;
    if(p < ETH_MIN_REFERENCE_V * ETH_MIN_REFERENCE_V) return 0U;
    out->real = (r->real * ref->real + r->imag * ref->imag) / p;
    out->imag = (r->imag * ref->real - r->real * ref->imag) / p;
    return 1U;
}

/*------------------ 基本复数运算 ------------------*/
static complex CAdd(complex a, complex b) {
    complex r; r.real = a.real + b.real; r.imag = a.imag + b.imag;
    return r;
}
static complex CSub(complex a, complex b) {
    complex r; r.real = a.real - b.real; r.imag = a.imag - b.imag;
    return r;
}
static complex CMul(complex a, complex b) {
    complex r;
    r.real = a.real * b.real - a.imag * b.imag;
    r.imag = a.real * b.imag + a.imag * b.real;
    return r;
}
static uint8_t CDiv(const complex *a, const complex *b, complex *out)
{
    float p = b->real * b->real + b->imag * b->imag;
    if(p < 1.0e-12f) return 0U;
    out->real = (a->real * b->real + a->imag * b->imag) / p;
    out->imag = (a->imag * b->real - a->real * b->imag) / p;
    return 1U;
}

/*------------------------------------------------------
 * 判断当前频率设置是否与已保存的校准数据匹配
 *------------------------------------------------------*/
static uint8_t EthCalMatches(uint16_t count)
{
    return (g_cal_points == count &&
            g_cal_start == g_eth_cfg.start_freq_hz &&
            g_cal_stop == g_eth_cfg.stop_freq_hz &&
            g_cal_step == g_eth_cfg.step_hz) ? 1U : 0U;
}

/*------------------------------------------------------
 * 误差修正（SOL 校准应用）
 * 如果三个标准全，则使用单端口误差模型：
 *   Gamma_DUT = (S11M - D) / (R + S * (S11M - D))
 * 其中 D = 负载测量值，S = 等效源匹配，R = 等效反射跟踪。
 * 若仅有基线（负载），则简单减去方向性。
 *------------------------------------------------------*/
static void EthCorrect(uint16_t count, const complex *raw, complex *out)
{
    uint16_t i;
    uint8_t match = EthCalMatches(count);

    for(i = 0U; i < count; i++)
    {
        out[i] = raw[i];
        if(match != 0U && g_eth_runtime.port_cal_valid != 0U)
        {
            complex d = CSub(raw[i], g_load[i]);               /* d = S11M - D */
            complex den = CAdd(g_open[i], CMul(d, g_short[i])); /* den = R + S * d */
            if(CDiv(&d, &den, &out[i]) == 0U)
                out[i] = raw[i];   /* 计算失败则保留原始值 */
        }
        else if(match != 0U && g_eth_runtime.baseline_ready != 0U)
        {
            out[i] = CSub(raw[i], g_load[i]);  /* 仅减去方向性（基线） */
        }
    }
}

/*------------------------------------------------------
 * 零阶修正贝塞尔函数 I0(x) （用于 Kaiser 窗）
 * 采用级数展开，精度 1e-6
 *------------------------------------------------------*/
static float EthI0(float x)
{
    uint8_t k;
    float sum = 1.0f, term = 1.0f, x2 = x * x * 0.25f;
    for(k = 1U; k < 20U; k++) {
        term *= x2 / ((float)k * (float)k);
        sum += term;
        if(term < sum * 1.0e-6f) break;
    }
    return sum;
}

/*------------------------------------------------------
 * 窗函数生成：矩形、汉宁、凯泽（Beta 由配置给出）
 * i：样点索引，n：窗长度
 *------------------------------------------------------*/
static float EthWindow(uint16_t i, uint16_t n)
{
    float r, b;
    if(n <= 1U || g_eth_cfg.window_type == ETH_WINDOW_RECTANGULAR)
        return 1.0f;

    r = 2.0f * (float)i / (float)(n - 1U) - 1.0f;  /* 映射到 [-1, 1] */

    if(g_eth_cfg.window_type == ETH_WINDOW_HANN)
        return 0.5f - 0.5f * cosf(PI * (r + 1.0f));

    /* 凯泽窗 */
    b = (float)g_eth_cfg.kaiser_beta_x10 / 10.0f;
    return EthI0(b * sqrtf(1.0f - r * r)) / EthI0(b);
}

/*------------------------------------------------------
 * 在迹线 g_trace 中寻找最大峰值（跳过前端保护带）
 *------------------------------------------------------*/
static uint16_t EthPeak(uint16_t n)
{
    uint16_t i, peak = 0U;
    float v = 0.0f;
    for(i = ETH_PEAK_GUARD_BINS; i < (uint16_t)(n - 1U); i++)
        if(g_trace[i] > v) { v = g_trace[i]; peak = i; }
    return peak;
}

/*------------------------------------------------------
 * 抛物线插值，获得更精确的峰值位置（浮点 bin）
 *------------------------------------------------------*/
static float EthPeakInterp(uint16_t p, uint16_t n)
{
    float l, c, r, d, o;
    if(p == 0U || p >= n - 1U) return (float)p;

    l = g_trace[p - 1U];
    c = g_trace[p];
    r = g_trace[p + 1U];
    d = l - 2.0f * c + r;
    if(fabsf(d) < 1.0e-9f) return (float)p;

    o = 0.5f * (l - r) / d;
    if(o > 0.5f) o = 0.5f;
    if(o < -0.5f) o = -0.5f;
    return (float)p + o;
}

/*------------------------------------------------------
 * 精细长度搜索：在粗略长度 coarse 附近小范围扫描，
 * 寻找使加窗重建能量最大的长度，相当于基于相位/匹配滤波的细化
 *------------------------------------------------------*/
static uint8_t EthFineSearch(uint16_t count, float coarse, float *fine, float *metric)
{
    float v = EthVelocity();
    float limit = g_eth_runtime.last_max_range_m;
    float span = (float)g_eth_cfg.fine_search_span_cm / 100.0f;
    float start = coarse - span, stop = coarse + span;
    float L, bestL = coarse, best = -1.0f, amp = 0.0f;
    uint16_t i;

    if(start < 0.0f) start = 0.0f;
    if(stop > limit) stop = limit;

    /* 计算原始数据总功率，用于归一化 */
    for(i = 0U; i < count; i++) amp += complex_abs_float(g_gamma[i]);
    if(amp < 1.0e-6f) return 0U;

    /* 以 1 cm 步进扫描 */
    for(L = start; L <= stop + 0.0001f; L += 0.01f)
    {
        complex a = {0.0f, 0.0f};
        float m;

        for(i = 0U; i < count; i++)
        {
            float f = (float)(g_eth_cfg.start_freq_hz + (uint32_t)i * g_eth_cfg.step_hz);
            float ph = 4.0f * PI * f * L / v;   /* 往返相位 */
            float c = cosf(ph), q = sinf(ph);
            float w = EthWindow(i, count);

            /* 复数加权投影：sum( w * Gamma(f) * exp(-j*2*beta*L) ) */
            a.real += w * (g_gamma[i].real * c - g_gamma[i].imag * q);
            a.imag += w * (g_gamma[i].real * q + g_gamma[i].imag * c);
        }
        m = complex_abs_float(a) / amp;
        if(m > best) { best = m; bestL = L; }
    }
    *fine = bestL;
    *metric = best;
    return 1U;
}

/*------------------------------------------------------
 * 核心处理流程：
 * 1. 对频域 g_gamma 加窗并补零至 ifft_points
 * 2. 执行 IFFT 变换到时域
 * 3. 寻找峰值位置、计算 SNR
 * 4. 通过插值得到粗略长度，再进行精细搜索
 *------------------------------------------------------*/
static uint8_t EthProcess(uint16_t count)
{
    uint16_t i, p, noiseN = 0U;
    float noise = 0.0f, coarse, fine, metric;

    /* 加窗并移至 IFFT 缓冲区 */
    for(i = 0U; i < count; i++) {
        float w = EthWindow(i, count);
        g_ifft[i].real = g_gamma[i].real * w;
        g_ifft[i].imag = g_gamma[i].imag * w;
    }
    /* 补零 */
    for(i = count; i < g_eth_cfg.ifft_points; i++) {
        g_ifft[i].real = 0.0f;
        g_ifft[i].imag = 0.0f;
    }

    ifft(g_eth_cfg.ifft_points, g_ifft);           /* 执行逆傅里叶变换 */

    /* 取幅值 */
    for(i = 0U; i < g_eth_cfg.ifft_points; i++)
        g_trace[i] = complex_abs_float(g_ifft[i]);

    p = EthPeak(g_eth_cfg.ifft_points);
    if(p == 0U) {
        sprintf(g_eth_runtime.last_message, "No reflection peak");
        return 0U;
    }

    /* 估算噪声：排除峰值附近区域 */
    for(i = ETH_PEAK_GUARD_BINS; i < (uint16_t)(g_eth_cfg.ifft_points - 1U); i++)
        if(i + 2U < p || i > p + 2U) {
            noise += g_trace[i];
            noiseN++;
        }

    g_eth_runtime.last_peak_metric = g_trace[p];
    g_eth_runtime.last_peak_snr = g_trace[p] / ((noiseN > 0U ? noise / (float)noiseN : 0.0f) + 1.0e-9f);

    if(g_eth_runtime.last_peak_snr < ETH_MIN_PEAK_SNR) {
        sprintf(g_eth_runtime.last_message, "Reflection SNR low");
        return 0U;
    }

    /* 插值峰值得到粗略长度 */
    coarse = EthLengthFromBin(EthPeakInterp(p, g_eth_cfg.ifft_points));

    /* 精细搜索 */
    if(EthFineSearch(count, coarse, &fine, &metric) == 0U) {
        fine = coarse;
        metric = 0.0f;
    }

    g_eth_runtime.last_peak_bin = p;
    g_eth_runtime.last_coarse_length_m = coarse;
    g_eth_runtime.last_length_m = fine;
    g_eth_runtime.last_phase_metric = metric;

    sprintf(g_eth_runtime.last_message, "Peak %u SNR %.1f",
            (unsigned int)p, g_eth_runtime.last_peak_snr);
    return 1U;
}

/*------------------------------------------------------
 * 清除全部端口校准数据
 *------------------------------------------------------*/
void EthClearPortCalibration(void)
{
    memset(g_open, 0, sizeof(g_open));
    memset(g_short, 0, sizeof(g_short));
    memset(g_load, 0, sizeof(g_load));
    g_cal_flags = 0U;
    g_cal_points = 0U;
    g_cal_start = 0U; g_cal_stop = 0U; g_cal_step = 0U;
    g_eth_runtime.baseline_ready = 0U;
    g_eth_runtime.port_cal_valid = 0U;
}

/*------------------------------------------------------
 * 根据 OSL（开路-短路-负载）测量结果求解误差模型参数
 * 假设理想标准：开路=1，短路=-1，负载=0。
 * 解出的 g_short 为等效源匹配 S，g_open 为等效反射跟踪 R
 *------------------------------------------------------*/
static uint8_t EthBuildSol(void)
{
    uint16_t i;
    if(g_cal_flags != ETH_CAL_ALL || EthCalMatches(g_cal_points) == 0U)
        return 0U;

    for(i = 0U; i < g_cal_points; i++)
    {
        /* a = open - load,  b = short - load */
        complex a = CSub(g_open[i], g_load[i]);
        complex b = CSub(g_short[i], g_load[i]);
        complex num = CAdd(a, b);       /* num = a + b */
        complex den = CSub(a, b);       /* den = a - b */
        complex one = {1.0f, 0.0f};
        complex delta;

        /* 求解源匹配 S = (a+b)/(a-b) */
        if(CDiv(&num, &den, &g_short[i]) == 0U) return 0U;

        /* 求解反射跟踪 R = a * (1 - S) */
        delta = CSub(one, g_short[i]);
        g_open[i] = CMul(a, delta);
    }
    g_eth_runtime.port_cal_valid = 1U;
    return 1U;
}

/*------------------------------------------------------
 * 采集指定端口校准标准（开路/短路/负载）
 * 采集满三个标准后自动调用 EthBuildSol 求解误差项
 *------------------------------------------------------*/
uint8_t EthCapturePortCalibration(EthernetPortCalStandard standard)
{
    uint16_t n;
    complex *dst;
    EthNormalizeConfig();
    n = EthPoints();
    if(n < ETH_MIN_SWEEP_POINTS) return 0U;
    if(standard > ETH_PORT_CAL_LOAD) return 0U;

    /* 若已有完整校准且频率参数改变则清空重来 */
    if(g_eth_runtime.port_cal_valid != 0U) EthClearPortCalibration();
    if(g_cal_flags != 0U && EthCalMatches(n) == 0U) EthClearPortCalibration();

    /* 首次采集时记录频率参数 */
    if(g_cal_flags == 0U) {
        g_cal_points = n;
        g_cal_start = g_eth_cfg.start_freq_hz;
        g_cal_stop = g_eth_cfg.stop_freq_hz;
        g_cal_step = g_eth_cfg.step_hz;
    }

    g_eth_runtime.sweep_count++;
    g_eth_runtime.last_rejected_frames = 0U;
    g_eth_runtime.last_clipped_frames = 0U;

    /* 捕获当前标准的原始反射系数 */
    if(EthCaptureRaw(n, g_gamma) == 0U) {
        EthRefreshDdsPreview();
        return 0U;
    }

    dst = (standard == ETH_PORT_CAL_OPEN)  ? g_open :
          (standard == ETH_PORT_CAL_SHORT) ? g_short : g_load;
    memcpy(dst, g_gamma, (uint32_t)n * sizeof(complex));

    /* 更新标志位 */
    if(standard == ETH_PORT_CAL_OPEN)
        g_cal_flags |= ETH_CAL_OPEN;
    else if(standard == ETH_PORT_CAL_SHORT)
        g_cal_flags |= ETH_CAL_SHORT;
    else {
        g_cal_flags |= ETH_CAL_LOAD;
        g_eth_runtime.baseline_ready = 1U;   /* 只要负载就可用作基线 */
    }

    /* 如果三个标准都采集完成，尝试解算完整误差模型 */
    if(g_cal_flags == ETH_CAL_ALL) {
        if(EthBuildSol() == 0U) {
            sprintf(g_eth_runtime.last_message, "SOL solve failed");
            return 0U;
        }
        sprintf(g_eth_runtime.last_message, "SOL calibration ready");
    } else {
        sprintf(g_eth_runtime.last_message, "Port standard captured");
    }

    EthRefreshDdsPreview();
    return 1U;
}

/*------------------------------------------------------
 * NVP 速率因子校准：根据已知长度的电缆测量结果，
 * 自动修正 g_eth_cfg.nvp_x1000 使测量值等于实际值
 *------------------------------------------------------*/
uint8_t EthApplyCalibration(void)
{
    float known, measured, nvp, v;
    measured = g_eth_runtime.last_length_m;
    if(measured <= 0.0f) {
        sprintf(g_eth_runtime.last_message, "NVP calibration failed");
        return 0U;
    }
    known = (float)g_eth_cfg.calib_length_cm / 100.0f;
    nvp = (float)g_eth_cfg.nvp_x1000 * known / measured;
    if(nvp < ETH_MIN_NVP_X1000) nvp = ETH_MIN_NVP_X1000;
    if(nvp > ETH_MAX_NVP_X1000) nvp = ETH_MAX_NVP_X1000;

    g_eth_cfg.nvp_x1000 = (uint16_t)(nvp + 0.5f);
    v = EthVelocity();
    g_eth_runtime.last_length_m = known;
    g_eth_runtime.last_max_range_m = v / (2.0f * (float)g_eth_cfg.step_hz);
    g_eth_runtime.calib_ready = 1U;
    sprintf(g_eth_runtime.last_message, "NVP calibrated %u", g_eth_cfg.nvp_x1000);
    return 1U;
}

/*------------------------------------------------------
 * 单次 ADC 捕获测试：仅采集一帧并计算 Gamma，
 * 用于诊断硬件或信号连接是否正常
 *------------------------------------------------------*/
uint8_t EthProbeAdcOnce(void)
{
    complex g;
    if(EthFrame(&g) == 0U) {
        g_eth_runtime.adc_capture_ready = 0U;
        return 0U;
    }
    g_eth_runtime.adc_capture_ready = 1U;
    sprintf(g_eth_runtime.last_message, "ADC frame OK");
    return 1U;
}
