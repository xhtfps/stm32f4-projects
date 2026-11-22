#include "phase_adc.h"

#include <stddef.h>
#include <string.h>

ADC_HandleTypeDef hadc_phase_master;
ADC_HandleTypeDef hadc_phase_slave;
DMA_HandleTypeDef hdma_phase_adc;

static TIM_HandleTypeDef s_phase_adc_timer;
static uint32_t s_phase_adc_buffer[PHASE_ADC_SAMPLE_COUNT];
static volatile uint8_t s_phase_adc_ready_mask;
static volatile uint32_t s_phase_adc_overrun_count;
static uint8_t s_phase_adc_initialized;

static HAL_StatusTypeDef PhaseAdc_InitHandle(ADC_HandleTypeDef *hadc,
                                             ADC_TypeDef *instance)
{
    hadc->Instance = instance;
    hadc->Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc->Init.Resolution = ADC_RESOLUTION_12B;
    hadc->Init.ScanConvMode = DISABLE;
    hadc->Init.ContinuousConvMode = DISABLE;
    hadc->Init.DiscontinuousConvMode = DISABLE;
    hadc->Init.NbrOfDiscConversion = 0U;
    hadc->Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
    hadc->Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T8_TRGO;
    hadc->Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc->Init.NbrOfConversion = 1U;
    hadc->Init.DMAContinuousRequests = ENABLE;
    hadc->Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    return HAL_ADC_Init(hadc);
}

void HAL_ADC_MspInit(ADC_HandleTypeDef *hadc)
{
    GPIO_InitTypeDef gpio = {0};

    if (hadc == NULL)
    {
        return;
    }

    __HAL_RCC_ADC1_CLK_ENABLE();
    __HAL_RCC_ADC2_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    gpio.Pin = PHASE_ADC_GPIO_PIN_1 | PHASE_ADC_GPIO_PIN_2;
    gpio.Mode = GPIO_MODE_ANALOG;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(PHASE_ADC_GPIO_PORT, &gpio);
}

static HAL_StatusTypeDef PhaseAdc_InitDma(void)
{
    __HAL_RCC_DMA2_CLK_ENABLE();

    (void)memset(&hdma_phase_adc, 0, sizeof(hdma_phase_adc));
    hdma_phase_adc.Instance = DMA2_Stream0;
    hdma_phase_adc.Init.Channel = DMA_CHANNEL_0;
    hdma_phase_adc.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_phase_adc.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_phase_adc.Init.MemInc = DMA_MINC_ENABLE;
    hdma_phase_adc.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_phase_adc.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    hdma_phase_adc.Init.Mode = DMA_CIRCULAR;
    hdma_phase_adc.Init.Priority = DMA_PRIORITY_VERY_HIGH;
    hdma_phase_adc.Init.FIFOMode = DMA_FIFOMODE_DISABLE;

    if (HAL_DMA_Init(&hdma_phase_adc) != HAL_OK)
    {
        return HAL_ERROR;
    }

    __HAL_LINKDMA(&hadc_phase_master, DMA_Handle, hdma_phase_adc);
    HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 5U, 0U);
    HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
    return HAL_OK;
}

static HAL_StatusTypeDef PhaseAdc_InitTimer(uint32_t sample_rate_hz)
{
    TIM_MasterConfigTypeDef master = {0};
    uint32_t period;

    if ((sample_rate_hz == 0U) ||
        (sample_rate_hz > PHASE_ADC_TIMER_CLOCK_HZ))
    {
        return HAL_ERROR;
    }

    period = PHASE_ADC_TIMER_CLOCK_HZ / sample_rate_hz;
    if (period == 0U)
    {
        return HAL_ERROR;
    }

    __HAL_RCC_TIM8_CLK_ENABLE();
    (void)memset(&s_phase_adc_timer, 0, sizeof(s_phase_adc_timer));
    s_phase_adc_timer.Instance = PHASE_ADC_TIMER;
    s_phase_adc_timer.Init.Prescaler = 0U;
    s_phase_adc_timer.Init.CounterMode = TIM_COUNTERMODE_UP;
    s_phase_adc_timer.Init.Period = period - 1U;
    s_phase_adc_timer.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    s_phase_adc_timer.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    if (HAL_TIM_Base_Init(&s_phase_adc_timer) != HAL_OK)
    {
        return HAL_ERROR;
    }

    master.MasterOutputTrigger = TIM_TRGO_UPDATE;
    master.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    return HAL_TIMEx_MasterConfigSynchronization(&s_phase_adc_timer, &master);
}

HAL_StatusTypeDef PhaseAdc_Init(void)
{
    ADC_ChannelConfTypeDef channel = {0};
    ADC_MultiModeTypeDef multimode = {0};

    if (PhaseAdc_InitHandle(&hadc_phase_master, ADC1) != HAL_OK)
    {
        return HAL_ERROR;
    }
    if (PhaseAdc_InitHandle(&hadc_phase_slave, ADC2) != HAL_OK)
    {
        return HAL_ERROR;
    }

    channel.Rank = 1U;
    channel.SamplingTime = ADC_SAMPLETIME_28CYCLES;
    channel.Offset = 0U;
    channel.Channel = PHASE_ADC_CHANNEL_1;
    if (HAL_ADC_ConfigChannel(&hadc_phase_master, &channel) != HAL_OK)
    {
        return HAL_ERROR;
    }
    channel.Channel = PHASE_ADC_CHANNEL_2;
    if (HAL_ADC_ConfigChannel(&hadc_phase_slave, &channel) != HAL_OK)
    {
        return HAL_ERROR;
    }

    multimode.Mode = ADC_DUALMODE_REGSIMULT;
    multimode.DMAAccessMode = ADC_DMAACCESSMODE_2;
    multimode.TwoSamplingDelay = ADC_TWOSAMPLINGDELAY_5CYCLES;
    if (HAL_ADCEx_MultiModeConfigChannel(&hadc_phase_master, &multimode) != HAL_OK)
    {
        return HAL_ERROR;
    }

    if (PhaseAdc_InitDma() != HAL_OK)
    {
        return HAL_ERROR;
    }

    s_phase_adc_ready_mask = 0U;
    s_phase_adc_overrun_count = 0U;
    s_phase_adc_initialized = 1U;
    return HAL_OK;
}

HAL_StatusTypeDef PhaseAdc_Start(uint32_t sample_rate_hz)
{
    if (s_phase_adc_initialized == 0U)
    {
        return HAL_ERROR;
    }
    if (PhaseAdc_InitTimer(sample_rate_hz) != HAL_OK)
    {
        return HAL_ERROR;
    }

    s_phase_adc_ready_mask = 0U;
    if (HAL_ADC_Start(&hadc_phase_slave) != HAL_OK)
    {
        return HAL_ERROR;
    }
    if (HAL_ADCEx_MultiModeStart_DMA(&hadc_phase_master,
                                     s_phase_adc_buffer,
                                     PHASE_ADC_SAMPLE_COUNT) != HAL_OK)
    {
        (void)HAL_ADC_Stop(&hadc_phase_slave);
        return HAL_ERROR;
    }
    return HAL_TIM_Base_Start(&s_phase_adc_timer);
}

HAL_StatusTypeDef PhaseAdc_Stop(void)
{
    if (s_phase_adc_initialized == 0U)
    {
        return HAL_ERROR;
    }

    (void)HAL_TIM_Base_Stop(&s_phase_adc_timer);
    (void)HAL_ADCEx_MultiModeStop_DMA(&hadc_phase_master);
    (void)HAL_ADC_Stop(&hadc_phase_slave);
    s_phase_adc_ready_mask = 0U;
    return HAL_OK;
}

static void PhaseAdc_MarkReady(uint8_t mask)
{
    if ((s_phase_adc_ready_mask & mask) != 0U)
    {
        ++s_phase_adc_overrun_count;
    }
    s_phase_adc_ready_mask |= mask;
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
    if ((hadc != NULL) && (hadc->Instance == ADC1))
    {
        PhaseAdc_MarkReady(0x01U);
    }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if ((hadc != NULL) && (hadc->Instance == ADC1))
    {
        PhaseAdc_MarkReady(0x02U);
    }
}

HAL_StatusTypeDef PhaseAdc_TakeBlock(int16_t *channel1,
                                     int16_t *channel2,
                                     uint16_t length)
{
    uint8_t ready_mask;
    uint16_t offset;
    uint16_t index;

    if ((channel1 == NULL) || (channel2 == NULL) ||
        (length != PHASE_ADC_BLOCK_COUNT))
    {
        return HAL_ERROR;
    }

    __disable_irq();
    ready_mask = s_phase_adc_ready_mask;
    if ((ready_mask & 0x01U) != 0U)
    {
        s_phase_adc_ready_mask &= (uint8_t)~0x01U;
        offset = 0U;
    }
    else if ((ready_mask & 0x02U) != 0U)
    {
        s_phase_adc_ready_mask &= (uint8_t)~0x02U;
        offset = PHASE_ADC_BLOCK_COUNT;
    }
    else
    {
        __enable_irq();
        return HAL_BUSY;
    }
    __enable_irq();

    for (index = 0U; index < length; ++index)
    {
        uint32_t packed = s_phase_adc_buffer[offset + index];
        channel1[index] = (int16_t)(packed & 0x0FFFU) - 2048;
        channel2[index] = (int16_t)((packed >> 16U) & 0x0FFFU) - 2048;
    }
    return HAL_OK;
}

uint32_t PhaseAdc_GetOverrunCount(void)
{
    return s_phase_adc_overrun_count;
}
