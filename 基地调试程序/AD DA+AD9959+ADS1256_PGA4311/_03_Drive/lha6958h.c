#include "lha6958h.h"
#include "delay.h"

#define LHA_REG_STATUS          0x01U
#define LHA_REG_CONFIG          0x02U
#define LHA_REG_RANGE_CH1_CH2   0x03U
#define LHA_REG_RANGE_CH3_CH4   0x04U
#define LHA_REG_RANGE_CH5_CH6   0x05U
#define LHA_REG_RANGE_CH7_CH8   0x06U
#define LHA_REG_OVERSAMPLING    0x08U
#define LHA_RANGE_CH1_CH2       0x10U /* CH1: +-2.5 V, CH2: +-5 V */
#define LHA_RANGE_CH3_CH4       0x32U /* CH3: +-10 V, CH4: +-12.5 V */
#define LHA_RANGE_CH5_CH8       0x22U /* CH5..CH8: +-10 V */
#define LHA_DELAY_NS            200U

#define LHA_FRSTDATA_PIN GPIO_Pin_6
#define LHA_BUSY_PIN     GPIO_Pin_7
#define LHA_CS_PIN       GPIO_Pin_8
#define LHA_RD_SCLK_PIN  GPIO_Pin_9  /* Module silk: SCLK; parallel-mode function: RD. */
#define LHA_RD_PIN       LHA_RD_SCLK_PIN
#define LHA_WR_PIN       GPIO_Pin_10
#define LHA_CONVST_PIN   GPIO_Pin_11
#define LHA_OS0_PIN      GPIO_Pin_12
#define LHA_OS1_PIN      GPIO_Pin_13
#define LHA_OS2_PIN      GPIO_Pin_8
#define LHA_RESET_PIN    GPIO_Pin_9
#define LHA_DB_G_MASK    (GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | \
                          GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_8 | \
                          GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12 | \
                          GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15)

static uint8_t s_register_mode = 0U;

static void LHA_DWT_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static void LHA_DelayNs(uint32_t ns)
{
    uint32_t start;
    uint32_t cycles;
    cycles = (uint32_t)(((uint64_t)SystemCoreClock * ns + 999999999ULL) / 1000000000ULL);
    if (cycles == 0U) cycles = 1U;
    start = DWT->CYCCNT;
    while ((uint32_t)(DWT->CYCCNT - start) < cycles) __NOP();
}

static uint8_t LHA_TimedOut(uint32_t start, uint32_t timeout_us)
{
    uint32_t cycles;
    cycles = (uint32_t)(((uint64_t)SystemCoreClock * timeout_us) / 1000000ULL);
    return ((uint32_t)(DWT->CYCCNT - start) >= cycles) ? 1U : 0U;
}

static void LHA_BusInput(void)
{
    GPIO_InitTypeDef gpio;
    gpio.GPIO_Pin = LHA_DB_G_MASK;
    gpio.GPIO_Mode = GPIO_Mode_IN;
    gpio.GPIO_Speed = GPIO_Speed_100MHz;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOG, &gpio);
    gpio.GPIO_Pin = GPIO_Pin_0;
    GPIO_Init(GPIOC, &gpio);
}

static void LHA_BusOutput(uint16_t value)
{
    GPIO_InitTypeDef gpio;
    GPIOG->ODR = (GPIOG->ODR & GPIO_Pin_7) |
                 ((uint32_t)value & 0x007FU) |
                 (((uint32_t)value & 0x7F80U) << 1U);
    GPIOC->ODR = (GPIOC->ODR & ~GPIO_Pin_0) | (((uint32_t)value >> 15U) & 0x0001U);
    gpio.GPIO_Pin = LHA_DB_G_MASK;
    gpio.GPIO_Mode = GPIO_Mode_OUT;
    gpio.GPIO_Speed = GPIO_Speed_100MHz;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOG, &gpio);
    gpio.GPIO_Pin = GPIO_Pin_0;
    GPIO_Init(GPIOC, &gpio);
}

static void LHA_GPIO_Init(void)
{
    GPIO_InitTypeDef gpio;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOG, ENABLE);

    gpio.GPIO_Pin = LHA_FRSTDATA_PIN | LHA_BUSY_PIN;
    gpio.GPIO_Mode = GPIO_Mode_IN;
    gpio.GPIO_Speed = GPIO_Speed_100MHz;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOC, &gpio);

    gpio.GPIO_Pin = LHA_CS_PIN | LHA_RD_PIN | LHA_WR_PIN | LHA_CONVST_PIN | LHA_OS0_PIN | LHA_OS1_PIN;
    gpio.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_Init(GPIOC, &gpio);
    gpio.GPIO_Pin = LHA_OS2_PIN | LHA_RESET_PIN;
    GPIO_Init(GPIOA, &gpio);

    GPIO_SetBits(GPIOC, LHA_CS_PIN | LHA_RD_PIN | LHA_WR_PIN | LHA_OS0_PIN | LHA_OS1_PIN);
    GPIO_ResetBits(GPIOC, LHA_CONVST_PIN);
    GPIO_SetBits(GPIOA, LHA_OS2_PIN);
    GPIO_ResetBits(GPIOA, LHA_RESET_PIN);
    LHA_BusInput();
}

static LHA6958H_Status LHA_WaitBusy(BitAction state, uint32_t timeout_us)
{
    uint32_t start;
    start = DWT->CYCCNT;
    while (GPIO_ReadInputDataBit(GPIOC, LHA_BUSY_PIN) != state)
    {
        if (LHA_TimedOut(start, timeout_us) != 0U) return LHA6958H_TIMEOUT;
    }
    return LHA6958H_OK;
}

static void LHA_WriteFrame(uint16_t command)
{
    GPIO_SetBits(GPIOC, LHA_RD_PIN | LHA_WR_PIN | LHA_CS_PIN);
    LHA_BusOutput(command);
    LHA_DelayNs(LHA_DELAY_NS);
    GPIO_ResetBits(GPIOC, LHA_CS_PIN);
    LHA_DelayNs(LHA_DELAY_NS);
    GPIO_ResetBits(GPIOC, LHA_WR_PIN);
    LHA_DelayNs(LHA_DELAY_NS);
    GPIO_SetBits(GPIOC, LHA_WR_PIN);
    LHA_DelayNs(LHA_DELAY_NS);
    GPIO_SetBits(GPIOC, LHA_CS_PIN);
    LHA_DelayNs(LHA_DELAY_NS);
    LHA_BusInput();
}

static LHA6958H_Status LHA_ReadRegisterFrame(uint8_t address, uint8_t *value)
{
    uint16_t response;
    if (value == 0) return LHA6958H_ERROR;
    LHA_WriteFrame((uint16_t)(0x8000U | ((uint16_t)(address & 0x7FU) << 8U)));
    GPIO_ResetBits(GPIOC, LHA_CS_PIN);
    LHA_DelayNs(LHA_DELAY_NS);
    GPIO_ResetBits(GPIOC, LHA_RD_PIN);
    LHA_DelayNs(LHA_DELAY_NS);
    response = (uint16_t)((GPIOG->IDR & 0x007FU) |
                          ((GPIOG->IDR & 0xFF00U) >> 1U) |
                          ((GPIOC->IDR & GPIO_Pin_0) << 15U));
    GPIO_SetBits(GPIOC, LHA_RD_PIN | LHA_CS_PIN);
    LHA_DelayNs(LHA_DELAY_NS);
    if (((response >> 8U) & 0x7FU) != (address & 0x7FU)) return LHA6958H_VERIFY_ERROR;
    *value = (uint8_t)response;
    s_register_mode = 1U;
    return LHA6958H_OK;
}

static LHA6958H_Status LHA_EnterRegisterMode(void)
{
    uint8_t value;
    if (s_register_mode != 0U) return LHA6958H_OK;
    return LHA_ReadRegisterFrame(LHA_REG_STATUS, &value);
}

static void LHA_ExitRegisterMode(void)
{
    if (s_register_mode != 0U)
    {
        LHA_WriteFrame(0x0000U);
        s_register_mode = 0U;
    }
}

LHA6958H_Status LHA6958H_ReadRegister(uint8_t address, uint8_t *value)
{
    LHA6958H_Status status;
    status = LHA_EnterRegisterMode();
    if (status == LHA6958H_OK) status = LHA_ReadRegisterFrame(address, value);
    LHA_ExitRegisterMode();
    return status;
}

LHA6958H_Status LHA6958H_WriteRegister(uint8_t address, uint8_t value)
{
    LHA6958H_Status status;
    uint8_t readback;
    status = LHA_EnterRegisterMode();
    if (status == LHA6958H_OK)
    {
        LHA_WriteFrame((uint16_t)(((uint16_t)(address & 0x7FU) << 8U) | value));
        status = LHA_ReadRegisterFrame(address, &readback);
        if ((status == LHA6958H_OK) && (readback != value)) status = LHA6958H_VERIFY_ERROR;
    }
    LHA_ExitRegisterMode();
    return status;
}

LHA6958H_Status LHA6958H_Init(void)
{
    LHA6958H_Status status;
    uint8_t value;
    SystemCoreClockUpdate();
    LHA_DWT_Init();
    LHA_GPIO_Init();
    delay_ms(10U);
    GPIO_SetBits(GPIOA, LHA_RESET_PIN);
    LHA_DelayNs(10000U);
    GPIO_ResetBits(GPIOA, LHA_RESET_PIN);
    delay_ms(1U);
    s_register_mode = 0U;

    status = LHA_EnterRegisterMode();
    if (status != LHA6958H_OK) return status;
    LHA_WriteFrame((uint16_t)(LHA_REG_CONFIG << 8U));
    LHA_WriteFrame((uint16_t)((LHA_REG_RANGE_CH1_CH2 << 8U) | LHA_RANGE_CH1_CH2));
    LHA_WriteFrame((uint16_t)((LHA_REG_RANGE_CH3_CH4 << 8U) | LHA_RANGE_CH3_CH4));
    LHA_WriteFrame((uint16_t)((LHA_REG_RANGE_CH5_CH6 << 8U) | LHA_RANGE_CH5_CH8));
    LHA_WriteFrame((uint16_t)((LHA_REG_RANGE_CH7_CH8 << 8U) | LHA_RANGE_CH5_CH8));
    LHA_WriteFrame((uint16_t)(LHA_REG_OVERSAMPLING << 8U));

    status = LHA_ReadRegisterFrame(LHA_REG_RANGE_CH1_CH2, &value);
    if ((status != LHA6958H_OK) || (value != LHA_RANGE_CH1_CH2)) status = LHA6958H_VERIFY_ERROR;
    if (status == LHA6958H_OK) { status = LHA_ReadRegisterFrame(LHA_REG_RANGE_CH3_CH4, &value); if (value != LHA_RANGE_CH3_CH4) status = LHA6958H_VERIFY_ERROR; }
    if (status == LHA6958H_OK) { status = LHA_ReadRegisterFrame(LHA_REG_RANGE_CH5_CH6, &value); if (value != LHA_RANGE_CH5_CH8) status = LHA6958H_VERIFY_ERROR; }
    if (status == LHA6958H_OK) { status = LHA_ReadRegisterFrame(LHA_REG_RANGE_CH7_CH8, &value); if (value != LHA_RANGE_CH5_CH8) status = LHA6958H_VERIFY_ERROR; }
    if (status == LHA6958H_OK) { status = LHA_ReadRegisterFrame(LHA_REG_OVERSAMPLING, &value); if (value != 0U) status = LHA6958H_VERIFY_ERROR; }
    LHA_ExitRegisterMode();
    return status;
}

LHA6958H_Status LHA6958H_StartConversion(void)
{
    if (s_register_mode != 0U) LHA_ExitRegisterMode();
    if (GPIO_ReadInputDataBit(GPIOC, LHA_BUSY_PIN) == Bit_SET) return LHA6958H_BUSY;
    GPIO_ResetBits(GPIOC, LHA_CONVST_PIN);
    LHA_DelayNs(LHA_DELAY_NS);
    GPIO_SetBits(GPIOC, LHA_CONVST_PIN);
    LHA_DelayNs(LHA_DELAY_NS);
    GPIO_ResetBits(GPIOC, LHA_CONVST_PIN);
    return LHA_WaitBusy(Bit_SET, 5U);
}

LHA6958H_Status LHA6958H_WaitReady(uint32_t timeout_us)
{
    return LHA_WaitBusy(Bit_RESET, timeout_us);
}

LHA6958H_Status LHA6958H_ReadFrame(LHA6958H_Frame *frame)
{
    uint8_t index;
    uint8_t first_data;
    if (frame == 0) return LHA6958H_ERROR;
    if (GPIO_ReadInputDataBit(GPIOC, LHA_BUSY_PIN) == Bit_SET) return LHA6958H_BUSY;
    LHA_BusInput();
    GPIO_SetBits(GPIOC, LHA_WR_PIN | LHA_RD_PIN);
    GPIO_ResetBits(GPIOC, LHA_CS_PIN);
    LHA_DelayNs(LHA_DELAY_NS);
    for (index = 0U; index < 8U; index++)
    {
        GPIO_ResetBits(GPIOC, LHA_RD_PIN);
        LHA_DelayNs(LHA_DELAY_NS);
        frame->code[index] = (int16_t)((GPIOG->IDR & 0x007FU) |
                                       ((GPIOG->IDR & 0xFF00U) >> 1U) |
                                       ((GPIOC->IDR & GPIO_Pin_0) << 15U));
        first_data = (GPIO_ReadInputDataBit(GPIOC, LHA_FRSTDATA_PIN) == Bit_SET) ? 1U : 0U;
        GPIO_SetBits(GPIOC, LHA_RD_PIN);
        LHA_DelayNs(LHA_DELAY_NS);
        if (((index == 0U) && (first_data == 0U)) || ((index != 0U) && (first_data != 0U)))
        {
            GPIO_SetBits(GPIOC, LHA_CS_PIN);
            return LHA6958H_FRSTDATA_ERROR;
        }
    }
    GPIO_SetBits(GPIOC, LHA_CS_PIN);
    return LHA6958H_OK;
}

LHA6958H_Status LHA6958H_Sample(LHA6958H_Frame *frame, uint32_t timeout_us)
{
    LHA6958H_Status status;
    status = LHA6958H_StartConversion();
    if (status != LHA6958H_OK) return status;
    status = LHA6958H_WaitReady(timeout_us);
    if (status != LHA6958H_OK) return status;
    return LHA6958H_ReadFrame(frame);
}

float LHA6958H_CodeToVoltage(uint8_t channel, int16_t code)
{
    static const float full_scale[8] = {2.5f, 5.0f, 10.0f, 12.5f, 10.0f, 10.0f, 10.0f, 10.0f};
    if ((channel < 1U) || (channel > 8U)) return 0.0f;
    return ((float)code * full_scale[channel - 1U]) / 32768.0f;
}
