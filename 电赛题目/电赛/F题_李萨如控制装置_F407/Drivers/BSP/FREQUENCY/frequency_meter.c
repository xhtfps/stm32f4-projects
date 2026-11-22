/**
 * @file    frequency_meter.c
 * @brief   TIM2 输入捕获测频服务实现。
 */

#include "frequency_meter.h"

#include <stddef.h>
#include <string.h>

FrequencyMeter_t g_frequency_meter;

#define FREQUENCY_METER_LOW_BAND_MAX_HZ 4500.0f
#define FREQUENCY_METER_LOW_EMA_ALPHA       0.50f
#define FREQUENCY_METER_LOW_MEDIAN_WINDOW   3U

static HAL_StatusTypeDef FrequencyMeter_InitImpl(
    FrequencyMeter_t *self, const FrequencyMeterConfig_t *config);
static HAL_StatusTypeDef FrequencyMeter_StartImpl(
    FrequencyMeter_t *self);
static HAL_StatusTypeDef FrequencyMeter_StopImpl(
    FrequencyMeter_t *self);
static uint8_t FrequencyMeter_ProcessImpl(FrequencyMeter_t *self);
static uint8_t FrequencyMeter_ProcessFeedback(
    FrequencyMeter_t *self, uint32_t now);
static const FrequencyMeterResult_t *FrequencyMeter_GetResultImpl(
    const FrequencyMeter_t *self);

static const FrequencyMeterOps_t s_frequency_meter_ops =
{
    FrequencyMeter_InitImpl,
    FrequencyMeter_StartImpl,
    FrequencyMeter_StopImpl,
    FrequencyMeter_ProcessImpl,
    FrequencyMeter_GetResultImpl
};

/**
 * @brief 判断中值窗口配置是否合法。
 */
static uint8_t FrequencyMeter_IsWindowValid(uint8_t window)
{
    return ((window >= 1U) &&
            (window <= FREQUENCY_METER_MAX_WINDOW) &&
            ((window & 1U) != 0U)) ? 1U : 0U;
}

/**
 * @brief 用插入排序计算小窗口中值。
 *
 * 窗口最多只有 9 个元素，使用固定栈数组比引入通用排序函数更直观，
 * 而且执行时间上界明确。
 */
static float FrequencyMeter_Median(const float *values, uint8_t count)
{
    float sorted[FREQUENCY_METER_MAX_WINDOW];
    float value;
    uint8_t index;
    uint8_t position;

    if ((values == NULL) || (count == 0U))
    {
        return 0.0f;
    }

    for (index = 0U; index < count; ++index)
    {
        value = values[index];
        position = index;
        while ((position > 0U) &&
               (sorted[position - 1U] > value))
        {
            sorted[position] = sorted[position - 1U];
            --position;
        }
        sorted[position] = value;
    }
    return sorted[count / 2U];
}

static float FrequencyMeter_RecentMedian(
    const float *values,
    uint8_t capacity,
    uint8_t write_index,
    uint8_t count,
    uint8_t requested_count)
{
    float recent[FREQUENCY_METER_MAX_WINDOW];
    uint8_t available;
    uint8_t index;
    uint8_t position;

    if ((values == NULL) || (capacity == 0U) || (count == 0U))
    {
        return 0.0f;
    }

    available = count;
    if (available > requested_count)
    {
        available = requested_count;
    }
    for (index = 0U; index < available; ++index)
    {
        position = (uint8_t)(
            (write_index + capacity - 1U - index) % capacity);
        recent[index] = values[position];
    }
    return FrequencyMeter_Median(recent, available);
}

/**
 * @brief 把连续测量值量化为题目规定的 100 Hz 离散频点。
 *
 * 例如 12347 Hz 四舍五入为 12300 Hz，频点序号为 123。
 * 最终结果同时被限制在 1 kHz～100 kHz，保证应用层和 DDS 搜索
 * 永远不会收到带小数或不在 100 Hz 网格上的频率。
 */
static uint32_t FrequencyMeter_Quantize(
    const FrequencyMeter_t *self, float frequency_hz)
{
    uint32_t frequency;
    uint32_t step;

    step = self->config.frequency_step_hz;
    frequency =
        (uint32_t)((frequency_hz + (float)step * 0.5f) /
                   (float)step) * step;

    if (frequency < self->config.valid_min_hz)
    {
        frequency = self->config.valid_min_hz;
    }
    if (frequency > self->config.valid_max_hz)
    {
        frequency = self->config.valid_max_hz;
    }
    return frequency;
}

/**
 * @brief 由当前滑动窗口计算最小值、最大值和峰峰抖动。
 */
static void FrequencyMeter_UpdateWindowStatistics(
    FrequencyMeter_t *self)
{
    float minimum;
    float maximum;
    uint8_t index;

    if ((self == NULL) || (self->median_count == 0U))
    {
        return;
    }

    minimum = self->median_buffer[0];
    maximum = minimum;
    for (index = 1U; index < self->median_count; ++index)
    {
        if (self->median_buffer[index] < minimum)
        {
            minimum = self->median_buffer[index];
        }
        if (self->median_buffer[index] > maximum)
        {
            maximum = self->median_buffer[index];
        }
    }

    self->result.window_min_hz = minimum;
    self->result.window_max_hz = maximum;
    if (self->result.filtered_frequency_hz > 0.0f)
    {
        self->result.jitter_ppm =
            (maximum - minimum) * 1000000.0f /
            self->result.filtered_frequency_hz;
    }
    else
    {
        self->result.jitter_ppm = 0.0f;
    }
}

/**
 * @brief 处理 PB11/TIM2_CH4 的输出反馈频率。
 *
 * 与 PB10 输入频点不同，本函数绝不执行 100 Hz 量化。返回的
 * feedback_filtered_frequency_hz 可直接用于后续频差环；LCD 是否
 * 取整只属于显示层，不能反向影响该测量值。
 */
static uint8_t FrequencyMeter_ProcessFeedback(
    FrequencyMeter_t *self, uint32_t now)
{
    uint32_t period_ticks;
    uint32_t capture_cycles;
    float raw_frequency;
    float median_frequency;
    float minimum_accepted_hz;
    float maximum_accepted_hz;
    float ema_alpha;
    uint8_t filter_window;
    uint8_t changed = 0U;

    if (PhaseCapture_TakeFeedbackMeasurement(
            self->capture, &period_ticks, &capture_cycles) != 0U)
    {
        if ((period_ticks == 0U) || (capture_cycles == 0U))
        {
            return 0U;
        }

        raw_frequency = (float)(
            ((double)self->capture->config.timer_tick_hz *
             (double)capture_cycles) / (double)period_ticks);
        self->result.feedback_raw_frequency_hz = raw_frequency;
        self->result.feedback_period_ticks = period_ticks;
        self->result.feedback_capture_cycles = capture_cycles;
        self->result.feedback_dma_error_count =
            PhaseCapture_GetFeedbackDmaErrorCount(self->capture);
        self->feedback_last_valid_tick_ms = now;

        /*
         * 要求2/4输出最高100 kHz，要求3输出最高200 kHz。上下限只用于
         * 排除开路噪声，不对合法的精确频率进行100 Hz吸附。
         */
        minimum_accepted_hz =
            (float)self->config.valid_min_hz -
            (float)self->config.frequency_step_hz * 0.5f;
        maximum_accepted_hz =
            (float)self->config.valid_max_hz * 2.0f +
            (float)self->config.frequency_step_hz * 0.5f;
        if ((raw_frequency < minimum_accepted_hz) ||
            (raw_frequency > maximum_accepted_hz))
        {
            self->result.feedback_valid = 0U;
            self->result.feedback_signal_age_ms = 0U;
            return 1U;
        }

        self->feedback_median_buffer[
            self->feedback_median_write_index] = raw_frequency;
        self->feedback_median_write_index =
            (uint8_t)((self->feedback_median_write_index + 1U) %
                      self->config.median_window);
        if (self->feedback_median_count <
            self->config.median_window)
        {
            ++self->feedback_median_count;
        }

        filter_window =
            (raw_frequency < FREQUENCY_METER_LOW_BAND_MAX_HZ) ?
            FREQUENCY_METER_LOW_MEDIAN_WINDOW :
            self->config.median_window;
        ema_alpha =
            (raw_frequency < FREQUENCY_METER_LOW_BAND_MAX_HZ) ?
            FREQUENCY_METER_LOW_EMA_ALPHA : self->config.ema_alpha;
        median_frequency = FrequencyMeter_RecentMedian(
            self->feedback_median_buffer,
            self->config.median_window,
            self->feedback_median_write_index,
            self->feedback_median_count,
            filter_window);
        if (self->result.feedback_sample_count == 0U)
        {
            self->result.feedback_filtered_frequency_hz =
                median_frequency;
        }
        else
        {
            self->result.feedback_filtered_frequency_hz +=
                ema_alpha *
                (median_frequency -
                 self->result.feedback_filtered_frequency_hz);
        }

        ++self->result.feedback_sample_count;
        self->result.feedback_signal_age_ms = 0U;
        self->result.feedback_valid = 1U;
        changed = 1U;
    }
    else
    {
        self->result.feedback_signal_age_ms =
            (uint32_t)(now - self->feedback_last_valid_tick_ms);
        self->result.feedback_dma_error_count =
            PhaseCapture_GetFeedbackDmaErrorCount(self->capture);
        if ((self->result.feedback_signal_age_ms >=
             self->config.signal_timeout_ms) &&
            (self->result.feedback_valid != 0U))
        {
            self->result.feedback_valid = 0U;
            changed = 1U;
        }
    }

    return changed;
}

void FrequencyMeterConfig_Default(FrequencyMeterConfig_t *config)
{
    if (config == NULL)
    {
        return;
    }

    /* 题目信号源严格位于 1 kHz～100 kHz 的 100 Hz 网格。 */
    config->valid_min_hz = 900U;
    config->valid_max_hz = 100000U;
    config->frequency_step_hz = 100U;
    config->ema_alpha = 0.25f;
    config->signal_timeout_ms = 350U;
    config->median_window = 5U;
}

void FrequencyMeter_Construct(FrequencyMeter_t *self)
{
    if ((self == NULL) || (self->ops != NULL))
    {
        return;
    }

    (void)memset(self, 0, sizeof(*self));
    self->capture = &g_phase_capture;
    self->ops = &s_frequency_meter_ops;
}

static HAL_StatusTypeDef FrequencyMeter_InitImpl(
    FrequencyMeter_t *self, const FrequencyMeterConfig_t *config)
{
    PhaseCaptureConfig_t capture_config;

    if ((self == NULL) || (config == NULL) ||
        (config->valid_min_hz == 0U) ||
        (config->valid_max_hz <= config->valid_min_hz) ||
        (config->frequency_step_hz == 0U) ||
        ((config->valid_min_hz % config->frequency_step_hz) != 0U) ||
        ((config->valid_max_hz % config->frequency_step_hz) != 0U) ||
        (config->ema_alpha <= 0.0f) ||
        (config->ema_alpha > 1.0f) ||
        (FrequencyMeter_IsWindowValid(config->median_window) == 0U))
    {
        return HAL_ERROR;
    }

    self->config = *config;
    self->capture = &g_phase_capture;
    (void)memset(&self->result, 0, sizeof(self->result));
    (void)memset(self->median_buffer, 0,
                 sizeof(self->median_buffer));
    (void)memset(self->feedback_median_buffer, 0,
                 sizeof(self->feedback_median_buffer));
    self->median_count = 0U;
    self->median_write_index = 0U;
    self->feedback_median_count = 0U;
    self->feedback_median_write_index = 0U;
    self->running = 0U;
    self->result.status = FREQUENCY_METER_STATUS_STOPPED;

    PhaseCaptureConfig_Default(&capture_config);
    if (PhaseCapture_Init(self->capture, &capture_config) != HAL_OK)
    {
        self->result.status =
            FREQUENCY_METER_STATUS_HARDWARE_ERROR;
        return HAL_ERROR;
    }

    self->initialized = 1U;
    return HAL_OK;
}

static HAL_StatusTypeDef FrequencyMeter_StartImpl(
    FrequencyMeter_t *self)
{
    if ((self == NULL) || (self->initialized == 0U))
    {
        return HAL_ERROR;
    }

    if (PhaseCapture_Start(self->capture) != HAL_OK)
    {
        self->result.status =
            FREQUENCY_METER_STATUS_HARDWARE_ERROR;
        return HAL_ERROR;
    }

    (void)memset(&self->result, 0, sizeof(self->result));
    (void)memset(self->median_buffer, 0,
                 sizeof(self->median_buffer));
    (void)memset(self->feedback_median_buffer, 0,
                 sizeof(self->feedback_median_buffer));
    self->median_count = 0U;
    self->median_write_index = 0U;
    self->feedback_median_count = 0U;
    self->feedback_median_write_index = 0U;
    self->last_valid_tick_ms = HAL_GetTick();
    self->feedback_last_valid_tick_ms =
        self->last_valid_tick_ms;
    self->result.status = FREQUENCY_METER_STATUS_WAIT_SIGNAL;
    self->running = 1U;
    return HAL_OK;
}

static HAL_StatusTypeDef FrequencyMeter_StopImpl(
    FrequencyMeter_t *self)
{
    HAL_StatusTypeDef status;

    if ((self == NULL) || (self->initialized == 0U))
    {
        return HAL_ERROR;
    }

    status = PhaseCapture_Stop(self->capture);
    self->running = 0U;
    self->result.valid = 0U;
    self->result.feedback_valid = 0U;
    self->result.status = FREQUENCY_METER_STATUS_STOPPED;
    return status;
}

static uint8_t FrequencyMeter_ProcessImpl(FrequencyMeter_t *self)
{
    uint32_t period_ticks;
    uint32_t capture_cycles;
    uint32_t now;
    float raw_frequency;
    float median_frequency;
    float ema_alpha;
    uint8_t filter_window;
    uint8_t changed;

    if ((self == NULL) || (self->initialized == 0U) ||
        (self->running == 0U))
    {
        return 0U;
    }

    now = HAL_GetTick();
    changed = FrequencyMeter_ProcessFeedback(self, now);
    if (PhaseCapture_TakeMeasurement(
            self->capture, &period_ticks, &capture_cycles) != 0U)
    {
        if ((period_ticks == 0U) || (capture_cycles == 0U))
        {
            return changed;
        }

        /*
         * period_ticks 是多个输入周期累计的定时器计数。
         * 先用 double 计算可避免 84 MHz × 周期数的 32 位整数溢出。
         */
        raw_frequency = (float)(
            ((double)self->capture->config.timer_tick_hz *
             (double)capture_cycles) / (double)period_ticks);

        self->result.raw_frequency_hz = raw_frequency;
        self->result.period_ticks = period_ticks;
        self->result.capture_cycles = capture_cycles;
        self->result.dma_error_count =
            PhaseCapture_GetDmaErrorCount(self->capture);
        self->last_valid_tick_ms = now;

        /*
         * 在上下限各放宽半个频点，避免 1 kHz/100 kHz 因晶振误差
         * 稍微越界后被误判无效；量化结果随后仍会钳位到严格范围。
         */
        if ((raw_frequency <
             ((float)self->config.valid_min_hz -
              (float)self->config.frequency_step_hz * 0.5f)) ||
            (raw_frequency >
             ((float)self->config.valid_max_hz +
              (float)self->config.frequency_step_hz * 0.5f)))
        {
            self->result.valid = 0U;
            self->result.status =
                FREQUENCY_METER_STATUS_OUT_OF_RANGE;
            self->result.signal_age_ms = 0U;
            return 1U;
        }

        self->median_buffer[self->median_write_index] =
            raw_frequency;
        self->median_write_index =
            (uint8_t)((self->median_write_index + 1U) %
                      self->config.median_window);
        if (self->median_count < self->config.median_window)
        {
            ++self->median_count;
        }

        filter_window =
            (raw_frequency < FREQUENCY_METER_LOW_BAND_MAX_HZ) ?
            FREQUENCY_METER_LOW_MEDIAN_WINDOW :
            self->config.median_window;
        ema_alpha =
            (raw_frequency < FREQUENCY_METER_LOW_BAND_MAX_HZ) ?
            FREQUENCY_METER_LOW_EMA_ALPHA : self->config.ema_alpha;
        median_frequency = FrequencyMeter_RecentMedian(
            self->median_buffer,
            self->config.median_window,
            self->median_write_index,
            self->median_count,
            filter_window);
        if (self->result.sample_count == 0U)
        {
            self->result.filtered_frequency_hz =
                median_frequency;
        }
        else
        {
            self->result.filtered_frequency_hz +=
                ema_alpha *
                (median_frequency -
                 self->result.filtered_frequency_hz);
        }

        ++self->result.sample_count;
        self->result.frequency_hz =
            FrequencyMeter_Quantize(
                self, self->result.filtered_frequency_hz);
        self->result.frequency_100hz =
            self->result.frequency_hz /
            self->config.frequency_step_hz;
        self->result.valid = 1U;
        self->result.status = FREQUENCY_METER_STATUS_VALID;
        self->result.signal_age_ms = 0U;
        FrequencyMeter_UpdateWindowStatistics(self);
        changed = 1U;
    }
    else
    {
        self->result.signal_age_ms =
            (uint32_t)(now - self->last_valid_tick_ms);
        self->result.dma_error_count =
            PhaseCapture_GetDmaErrorCount(self->capture);
        if (self->result.signal_age_ms >=
            self->config.signal_timeout_ms)
        {
            if ((self->result.valid != 0U) ||
                (self->result.status !=
                 FREQUENCY_METER_STATUS_WAIT_SIGNAL))
            {
                changed = 1U;
            }
            self->result.valid = 0U;
            self->result.status =
                FREQUENCY_METER_STATUS_WAIT_SIGNAL;
        }
    }

    return changed;
}

static const FrequencyMeterResult_t *FrequencyMeter_GetResultImpl(
    const FrequencyMeter_t *self)
{
    if ((self == NULL) || (self->initialized == 0U))
    {
        return NULL;
    }
    return &self->result;
}

HAL_StatusTypeDef frequency_meter_init(
    const FrequencyMeterConfig_t *config)
{
    FrequencyMeter_Construct(&g_frequency_meter);
    return g_frequency_meter.ops->init(&g_frequency_meter, config);
}

HAL_StatusTypeDef frequency_meter_start(void)
{
    FrequencyMeter_Construct(&g_frequency_meter);
    return g_frequency_meter.ops->start(&g_frequency_meter);
}

HAL_StatusTypeDef frequency_meter_stop(void)
{
    FrequencyMeter_Construct(&g_frequency_meter);
    return g_frequency_meter.ops->stop(&g_frequency_meter);
}

uint8_t frequency_meter_process(void)
{
    FrequencyMeter_Construct(&g_frequency_meter);
    return g_frequency_meter.ops->process(&g_frequency_meter);
}

const FrequencyMeterResult_t *frequency_meter_get_result(void)
{
    FrequencyMeter_Construct(&g_frequency_meter);
    return g_frequency_meter.ops->get_result(&g_frequency_meter);
}
