#include "./BSP/PS2/ps2.h"

/* 私有宏定义 ---------------------------------------------------------------------------------*/
/**
 * @brief PS2协议参数宏
 */
#define PS2_RX_TIMEOUT_MS      10      /* 接收超时时间(ms)，超过此时间未收到完整帧则复位 */

/* 私有变量定义 -------------------------------------------------------------------------------*/
/**
 * @brief 全局PS2键盘状态机实例
 */
PS2_KeyStateMachine_t g_ps2KeyStateMachine = {0};

/**
 * @brief 全局按键值变量
 */
volatile PS2_KeyValue_t g_ps2CurrentKeyValue = PS2_KEY_VALUE_NULL;

/**
 * @brief 内部接收状态变量
 */
static volatile uint8_t  s_ps2RxBuffer = 0;          /* 接收数据缓冲区 */
static volatile uint8_t  s_ps2RxBitCount = 0;        /* 接收位计数器 */
static volatile bool     s_ps2RxComplete = false;    /* 接收完成标志 */
static volatile bool     s_ps2IsBreakCode = false;   /* 是否为断码 */
static volatile bool     s_ps2IsExtendCode = false;  /* 是否为扩展码 */
static volatile uint16_t s_ps2RxTimeout = 0;         /* 接收超时计数器 */

/* 私有函数声明 -------------------------------------------------------------------------------*/
static void PS2_GPIO_Init(void);
static void PS2_ProcessScanCode(uint8_t scancode);

/* 函数实现 -----------------------------------------------------------------------------------*/

/**
 * @brief PS2键盘硬件初始化
 * @details 初始化PS2接口GPIO（含外部中断）以及状态机
 * @note 定时器TIM4初始化由btim.c的btim_tim4_int_init()负责，需在main.c中调用
 *       必须在系统启动时调用一次
 */
void PS2_Keyboard_Init(void)
{
    /* 初始化GPIO（含外部中断配置） */
    PS2_GPIO_Init();

    /* 初始化状态机 */
    PS2_ClearStateMachine();
}

/**
 * @brief PS2接口GPIO初始化
 * @details 配置PS2_CLK引脚为下降沿触发外部中断，PS2_DATA引脚为上拉输入
 *          使用HAL库API，与项目其他BSP驱动（KEY/LED/EXTI）风格一致
 */
static void PS2_GPIO_Init(void)
{
    GPIO_InitTypeDef gpio_init_struct;

    /* 使能GPIOA时钟 */
    PS2_CLK_GPIO_CLK_ENABLE();
    PS2_DATA_GPIO_CLK_ENABLE();

    /* 配置PS2_CLK（PA12）为下降沿触发外部中断模式 */
    gpio_init_struct.Pin = PS2_CLK_GPIO_PIN;
    gpio_init_struct.Mode = GPIO_MODE_IT_FALLING;
    gpio_init_struct.Pull = GPIO_PULLUP;
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(PS2_CLK_GPIO_PORT, &gpio_init_struct);

    /* 配置PS2_DATA（PA11）为上拉输入模式 */
    gpio_init_struct.Pin = PS2_DATA_GPIO_PIN;
    gpio_init_struct.Mode = GPIO_MODE_INPUT;
    gpio_init_struct.Pull = GPIO_PULLUP;
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(PS2_DATA_GPIO_PORT, &gpio_init_struct);

    /* 配置外部中断NVIC（EXTI15_10_IRQn） */
    HAL_NVIC_SetPriority(EXTI15_10_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

/**
 * @brief 读取PS2键盘原始扫描码
 * @param[out] pScanCode 用于存储读取到的扫描码的指针
 * @return bool 读取成功返回true，无数据返回false
 */
bool PS2_ReadRawScanCode(uint8_t* pScanCode)
{
    if (pScanCode == NULL)
    {
        return false;
    }

    if (s_ps2RxComplete)
    {
        *pScanCode = s_ps2RxBuffer;
        s_ps2RxComplete = false;
        return true;
    }

    return false;
}

/**
 * @brief 键盘状态机扫描函数
 * @param[in,out] pKeyFSM 指向PS2键盘状态机的指针
 * @details 必须在1ms定时器中断中周期性调用
 *          负责：接收超时恢复、扫描码读取、消抖计时、按键模式计时
 */
void PS2_KeyStateMachine_Sweep(PS2_KeyStateMachine_t* pKeyFSM)
{
    uint8_t scancode;

    if (pKeyFSM == NULL)
    {
        return;
    }

    /* ---- 接收超时恢复 ---- */
    /* 如果正在接收数据但长时间未收到新位，则认为接收异常并复位 */
    if (s_ps2RxBitCount > 0 && !s_ps2RxComplete)
    {
        if (++s_ps2RxTimeout >= PS2_RX_TIMEOUT_MS)
        {
            /* 接收超时，复位接收状态机 */
            s_ps2RxBitCount = 0;
            s_ps2RxBuffer = 0;
            s_ps2IsBreakCode = false;
            s_ps2IsExtendCode = false;
            s_ps2RxTimeout = 0;
        }
    }
    else
    {
        s_ps2RxTimeout = 0;
    }

    /* ---- 读取新的扫描码 ---- */
    if (PS2_ReadRawScanCode(&scancode))
    {
        PS2_ProcessScanCode(scancode);
    }

    /* ---- 按键消抖计时 ---- */
    if (pKeyFSM->filter_counter > 0)
    {
        pKeyFSM->filter_counter--;
        if (pKeyFSM->filter_counter == 0)
        {
            /* 消抖完成，更新按键状态 */
            pKeyFSM->last_scancode = pKeyFSM->current_scancode;
        }
    }

    /* ---- 按键模式计时 ---- */
    if (pKeyFSM->key_mode != PS2_KEY_MODE_NONE)
    {
        pKeyFSM->key_counter++;

        switch (pKeyFSM->key_mode)
        {
            case PS2_KEY_MODE_SHORT:
                if (pKeyFSM->key_counter >= PS2_LONG_PRESS_MS)
                {
                    /* 触发长按键 */
                    pKeyFSM->key_mode = PS2_KEY_MODE_LONG;
                    pKeyFSM->key_counter = 0;
                }
                break;

            case PS2_KEY_MODE_LONG:
                if (pKeyFSM->key_counter >= PS2_CONTINUOUS_MS)
                {
                    /* 触发持续按键 */
                    pKeyFSM->key_mode = PS2_KEY_MODE_CONTINUOUS;
                    pKeyFSM->key_counter = 0;
                }
                break;

            case PS2_KEY_MODE_CONTINUOUS:
                if (pKeyFSM->key_counter >= PS2_CONTINUOUS_MS)
                {
                    /* 持续按键重复触发 */
                    pKeyFSM->key_counter = 0;
                }
                break;

            default:
                break;
        }
    }
}

/**
 * @brief PS2键盘按键扫描与解析
 * @details 整合扫描码读取与状态机处理，更新全局按键值
 * @note 建议在主循环中周期性调用（10ms~20ms周期）
 */
void PS2_KeyScan(void)
{
    PS2_KeyValue_t keyValue = PS2_KEY_VALUE_NULL;

    /* 检查短按键 */
    if (PS2_KEY_0_SHORT()) keyValue = PS2_KEY_VALUE_0;
    else if (PS2_KEY_1_SHORT()) keyValue = PS2_KEY_VALUE_1;
    else if (PS2_KEY_2_SHORT()) keyValue = PS2_KEY_VALUE_2;
    else if (PS2_KEY_3_SHORT()) keyValue = PS2_KEY_VALUE_3;
    else if (PS2_KEY_4_SHORT()) keyValue = PS2_KEY_VALUE_4;
    else if (PS2_KEY_5_SHORT()) keyValue = PS2_KEY_VALUE_5;
    else if (PS2_KEY_6_SHORT()) keyValue = PS2_KEY_VALUE_6;
    else if (PS2_KEY_7_SHORT()) keyValue = PS2_KEY_VALUE_7;
    else if (PS2_KEY_8_SHORT()) keyValue = PS2_KEY_VALUE_8;
    else if (PS2_KEY_9_SHORT()) keyValue = PS2_KEY_VALUE_9;
    else if (PS2_KEY_IS_SHORT_PRESSED(&g_ps2KeyStateMachine, PS2_SCANCODE_POINT)) keyValue = PS2_KEY_VALUE_POINT;
    else if (PS2_KEY_IS_SHORT_PRESSED(&g_ps2KeyStateMachine, PS2_SCANCODE_BACKSPACE)) keyValue = PS2_KEY_VALUE_BACKSPACE;
    else if (PS2_KEY_IS_SHORT_PRESSED(&g_ps2KeyStateMachine, PS2_SCANCODE_ENTER)) keyValue = PS2_KEY_VALUE_ENTER;
    else if (PS2_KEY_IS_SHORT_PRESSED(&g_ps2KeyStateMachine, PS2_SCANCODE_ADD)) keyValue = PS2_KEY_VALUE_ADD;
    else if (PS2_KEY_IS_SHORT_PRESSED(&g_ps2KeyStateMachine, PS2_SCANCODE_MINUS)) keyValue = PS2_KEY_VALUE_MINUS;
    else if (PS2_KEY_IS_SHORT_PRESSED(&g_ps2KeyStateMachine, PS2_SCANCODE_MULTIPLY)) keyValue = PS2_KEY_VALUE_MULTIPLY;
    else if (PS2_KEY_IS_SHORT_PRESSED(&g_ps2KeyStateMachine, PS2_SCANCODE_DIVIDE)) keyValue = PS2_KEY_VALUE_DIVIDE;
    else if (PS2_KEY_IS_SHORT_PRESSED(&g_ps2KeyStateMachine, PS2_SCANCODE_NUMLOCK)) keyValue = PS2_KEY_VALUE_NUMLOCK;

    /* 更新全局按键值 */
    if (keyValue != PS2_KEY_VALUE_NULL)
    {
        g_ps2CurrentKeyValue = keyValue;
        /* 清除按键状态，确保每个按键只被处理一次 */
        g_ps2KeyStateMachine.key_mode = PS2_KEY_MODE_NONE;
        g_ps2KeyStateMachine.last_scancode = 0;
    }
}

/**
 * @brief 获取当前按键值
 * @return PS2_KeyValue_t 当前有效按键值，无按键返回PS2_KEY_VALUE_NULL
 * @note 此函数会清除当前按键值，确保每个按键只被处理一次
 */
PS2_KeyValue_t PS2_GetCurrentKeyValue(void)
{
    PS2_KeyValue_t keyValue = g_ps2CurrentKeyValue;
    g_ps2CurrentKeyValue = PS2_KEY_VALUE_NULL;
    return keyValue;
}

/**
 * @brief 清除键盘状态机
 * @details 清除所有按键状态和缓存，用于异常恢复
 */
void PS2_ClearStateMachine(void)
{
    g_ps2KeyStateMachine.state_flag = 0;
    g_ps2KeyStateMachine.current_scancode = 0;
    g_ps2KeyStateMachine.last_scancode = 0;
    g_ps2KeyStateMachine.key_mode = PS2_KEY_MODE_NONE;
    g_ps2KeyStateMachine.key_counter = 0;
    g_ps2KeyStateMachine.filter_counter = 0;

    /* 复位接收状态 */
    s_ps2RxBuffer = 0;
    s_ps2RxBitCount = 0;
    s_ps2RxComplete = false;
    s_ps2IsBreakCode = false;
    s_ps2IsExtendCode = false;
    s_ps2RxTimeout = 0;

    g_ps2CurrentKeyValue = PS2_KEY_VALUE_NULL;
}

/**
 * @brief 处理接收到的PS2扫描码
 * @param[in] scancode 接收到的原始扫描码
 */
static void PS2_ProcessScanCode(uint8_t scancode)
{
    /* 处理断码前缀 */
    if (scancode == PS2_BREAK_CODE)
    {
        s_ps2IsBreakCode = true;
        return;
    }

    /* 处理扩展码前缀 */
    if (scancode == PS2_EXTEND_CODE)
    {
        s_ps2IsExtendCode = true;
        return;
    }

    /* 处理断码（按键释放） */
    if (s_ps2IsBreakCode)
    {
        if (scancode == g_ps2KeyStateMachine.current_scancode)
        {
            g_ps2KeyStateMachine.key_mode = PS2_KEY_MODE_NONE;
            g_ps2KeyStateMachine.key_counter = 0;
            g_ps2KeyStateMachine.last_scancode = 0;
            g_ps2KeyStateMachine.current_scancode = 0;  /* 清除当前扫描码，允许同键再次被识别 */
        }

        s_ps2IsBreakCode = false;
        s_ps2IsExtendCode = false;
        return;
    }

    /* 处理通码（按键按下） */
    if (scancode != g_ps2KeyStateMachine.current_scancode)
    {
        /* 新按键按下 */
        g_ps2KeyStateMachine.current_scancode = scancode;
        g_ps2KeyStateMachine.filter_counter = PS2_DEBOUNCE_MS;
        g_ps2KeyStateMachine.key_mode = PS2_KEY_MODE_SHORT;
        g_ps2KeyStateMachine.key_counter = 0;
    }

    s_ps2IsExtendCode = false;
}

/**
 * @brief EXTI15_10中断服务函数
 * @details PS2_CLK引脚下降沿触发中断，模拟PS/2协议接收数据
 *          PS/2协议帧格式：1位起始位(0) + 8位数据(LSB先) + 1位奇偶校验 + 1位停止位(1)
 *          使用HAL库EXTI标志位操作，避免与现有HAL_GPIO_EXTI_Callback冲突
 */
void EXTI15_10_IRQHandler(void)
{
    if (__HAL_GPIO_EXTI_GET_IT(PS2_CLK_GPIO_PIN) != RESET)
    {
        /* 清除中断标志 */
        __HAL_GPIO_EXTI_CLEAR_IT(PS2_CLK_GPIO_PIN);

        /* 读取数据线电平 */
        uint8_t sda = PS2_DATA_READ();

        /* 复位超时计数器（每收到一位就复位） */
        s_ps2RxTimeout = 0;

        /* ---- 处理起始位 ---- */
        if (s_ps2RxBitCount == 0)
        {
            if (sda == 0)
            {
                /* 有效起始位，准备接收数据 */
                s_ps2RxBitCount++;
                s_ps2RxBuffer = 0;
            }
            return;
        }

        /* ---- 处理8位数据位（LSB first） ---- */
        if (s_ps2RxBitCount <= 8)
        {
            s_ps2RxBuffer >>= 1;
            if (sda)
            {
                s_ps2RxBuffer |= 0x80;
            }
            s_ps2RxBitCount++;
            return;
        }

        /* ---- 处理奇偶校验位 ---- */
        if (s_ps2RxBitCount == 9)
        {
            /* 当前未做奇偶校验检查，仅计数跳过 */
            s_ps2RxBitCount++;
            return;
        }

        /* ---- 处理停止位 ---- */
        if (s_ps2RxBitCount == 10)
        {
            if (sda == 1)
            {
                /* 有效停止位，标记接收完成 */
                s_ps2RxComplete = true;
            }

            /* 复位接收位计数 */
            s_ps2RxBitCount = 0;
            return;
        }

        /* 异常情况：位计数异常，强制复位 */
        s_ps2RxBitCount = 0;
    }
}

/************************ (C) COPYRIGHT 2026 ************************/
/*************************** END OF FILE ****************************/

