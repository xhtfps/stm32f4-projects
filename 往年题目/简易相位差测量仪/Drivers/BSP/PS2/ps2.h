#ifndef __PS2_H
#define __PS2_H

#ifdef __cplusplus
extern "C" {
#endif

#include "./SYSTEM/sys/sys.h"
#include <stdbool.h>

/******************************************************************************************/
/* PS2引脚定义 */

#define PS2_CLK_GPIO_PORT                   GPIOA
#define PS2_CLK_GPIO_PIN                    GPIO_PIN_12
#define PS2_CLK_GPIO_CLK_ENABLE()           do{ __HAL_RCC_GPIOA_CLK_ENABLE(); }while(0)   /* PA口时钟使能 */

#define PS2_DATA_GPIO_PORT                  GPIOA
#define PS2_DATA_GPIO_PIN                   GPIO_PIN_11
#define PS2_DATA_GPIO_CLK_ENABLE()          do{ __HAL_RCC_GPIOA_CLK_ENABLE(); }while(0)   /* PA口时钟使能 */

/* PS2数据读取宏 */
#define PS2_DATA_READ()                     HAL_GPIO_ReadPin(PS2_DATA_GPIO_PORT, PS2_DATA_GPIO_PIN)

/******************************************************************************************/
/* 协议参数定义 */

#define PS2_TIMEOUT_MS          10          /* 单次位传输超时时间(ms) */
#define PS2_DEBOUNCE_MS         20          /* 按键消抖时间(ms) */
#define PS2_LONG_PRESS_MS       500         /* 长按键触发时间(ms) */
#define PS2_CONTINUOUS_MS       100         /* 持续按键重复间隔(ms) */

/* 协议特殊扫描码 */
#define PS2_BREAK_CODE          0xF0        /* 断码前缀 */
#define PS2_EXTEND_CODE         0xE0        /* 扩展码前缀 */

/******************************************************************************************/
/* 类型定义 */

/**
 * @brief 按键模式枚举
 * @details 定义状态机识别的所有按键触发模式
 */
typedef enum
{
    PS2_KEY_MODE_NONE        = 0,    /* 无按键按下 */
    PS2_KEY_MODE_SHORT       = 1,    /* 短按键（按下后松开触发） */
    PS2_KEY_MODE_LONG        = 2,    /* 长按键（按下超过阈值触发） */
    PS2_KEY_MODE_CONTINUOUS  = 3     /* 持续按键（长按后周期性触发） */
} PS2_KeyMode_t;

/**
 * @brief PS2键盘扫描码定义（数字小键盘通码）
 * @details 采用PS/2协议第二套扫描码（通码），扩展键包含0xE0前缀
 */
typedef enum
{
    PS2_SCANCODE_0          = 0x70,
    PS2_SCANCODE_1          = 0x69,
    PS2_SCANCODE_2          = 0x72,
    PS2_SCANCODE_3          = 0x7A,
    PS2_SCANCODE_4          = 0x6B,
    PS2_SCANCODE_5          = 0x73,
    PS2_SCANCODE_6          = 0x74,
    PS2_SCANCODE_7          = 0x6C,
    PS2_SCANCODE_8          = 0x75,
    PS2_SCANCODE_9          = 0x7D,
    PS2_SCANCODE_MAIN_0     = 0x45,
    PS2_SCANCODE_MAIN_1     = 0x16,
    PS2_SCANCODE_MAIN_2     = 0x1E,
    PS2_SCANCODE_MAIN_3     = 0x26,
    PS2_SCANCODE_MAIN_4     = 0x25,
    PS2_SCANCODE_MAIN_5     = 0x2E,
    PS2_SCANCODE_MAIN_6     = 0x36,
    PS2_SCANCODE_MAIN_7     = 0x3D,
    PS2_SCANCODE_MAIN_8     = 0x3E,
    PS2_SCANCODE_MAIN_9     = 0x46,
    PS2_SCANCODE_ENTER      = 0x5A,   /* 数字小键盘Enter为扩展键，完整通码0xE0 0x5A */
    PS2_SCANCODE_ADD        = 0x79,
    PS2_SCANCODE_MINUS      = 0x7B,
    PS2_SCANCODE_MULTIPLY   = 0x7C,
    PS2_SCANCODE_DIVIDE     = 0x4A,
    PS2_SCANCODE_NUMLOCK    = 0x77,
    PS2_SCANCODE_POINT      = 0x71,
    PS2_SCANCODE_BACKSPACE  = 0x66,
    PS2_SCANCODE_EXT_PREFIX = 0xE0    /* 扩展键前缀码 */
} PS2_ScanCode_t;

/**
 * @brief 按键值定义（应用层使用）
 * @details 统一映射所有按键为应用层可直接使用的数值
 */
typedef enum
{
    PS2_KEY_VALUE_0         = 0,
    PS2_KEY_VALUE_1         = 1,
    PS2_KEY_VALUE_2         = 2,
    PS2_KEY_VALUE_3         = 3,
    PS2_KEY_VALUE_4         = 4,
    PS2_KEY_VALUE_5         = 5,
    PS2_KEY_VALUE_6         = 6,
    PS2_KEY_VALUE_7         = 7,
    PS2_KEY_VALUE_8         = 8,
    PS2_KEY_VALUE_9         = 9,
    PS2_KEY_VALUE_POINT     = 10,     /* 小数点 */
    PS2_KEY_VALUE_BACKSPACE = 11,     /* 退格键 */
    PS2_KEY_VALUE_ENTER     = 12,     /* 回车键 */
    PS2_KEY_VALUE_ADD       = 13,     /* 加号 */
    PS2_KEY_VALUE_MINUS     = 14,     /* 减号 */
    PS2_KEY_VALUE_MULTIPLY  = 15,     /* 乘号 */
    PS2_KEY_VALUE_DIVIDE    = 16,     /* 除号 */
    PS2_KEY_VALUE_NUMLOCK   = 17,     /* 数字锁定键 */
    PS2_KEY_VALUE_NULL      = 20      /* 无有效按键 */
} PS2_KeyValue_t;

/**
 * @brief PS2键盘状态机结构体
 * @details 管理PS2键盘的所有状态信息
 */
typedef struct
{
    volatile uint16_t state_flag;       /* 状态机内部状态标志 */
    volatile uint8_t  current_scancode; /* 当前接收到的扫描码 */
    volatile uint8_t  last_scancode;    /* 上一次有效扫描码 */
    volatile PS2_KeyMode_t key_mode;    /* 当前按键触发模式 */
    volatile uint16_t key_counter;      /* 长按键/持续按键计时计数器 */
    volatile uint16_t filter_counter;   /* 按键消抖计数器 */
} PS2_KeyStateMachine_t;

/******************************************************************************************/
/* 外部变量声明 */

extern PS2_KeyStateMachine_t g_ps2KeyStateMachine;
extern volatile PS2_KeyValue_t g_ps2CurrentKeyValue;

/******************************************************************************************/
/* 按键状态查询宏 */

#define PS2_KEY_IS_SHORT_PRESSED(fsm, scancode) \
    (((fsm)->last_scancode == (scancode)) && ((fsm)->key_mode == PS2_KEY_MODE_SHORT))

#define PS2_KEY_IS_LONG_PRESSED(fsm, scancode) \
    (((fsm)->last_scancode == (scancode)) && ((fsm)->key_mode == PS2_KEY_MODE_LONG))

#define PS2_KEY_IS_CONTINUOUS(fsm, scancode) \
    (((fsm)->last_scancode == (scancode)) && ((fsm)->key_mode == PS2_KEY_MODE_CONTINUOUS))

/* 常用按键快捷查询宏 */
#define PS2_KEY_DIGIT_SHORT(keypad, main) \
    (PS2_KEY_IS_SHORT_PRESSED(&g_ps2KeyStateMachine, (keypad)) || \
     PS2_KEY_IS_SHORT_PRESSED(&g_ps2KeyStateMachine, (main)))
#define PS2_KEY_0_SHORT() PS2_KEY_DIGIT_SHORT(PS2_SCANCODE_0, PS2_SCANCODE_MAIN_0)
#define PS2_KEY_1_SHORT() PS2_KEY_DIGIT_SHORT(PS2_SCANCODE_1, PS2_SCANCODE_MAIN_1)
#define PS2_KEY_2_SHORT() PS2_KEY_DIGIT_SHORT(PS2_SCANCODE_2, PS2_SCANCODE_MAIN_2)
#define PS2_KEY_3_SHORT() PS2_KEY_DIGIT_SHORT(PS2_SCANCODE_3, PS2_SCANCODE_MAIN_3)
#define PS2_KEY_4_SHORT() PS2_KEY_DIGIT_SHORT(PS2_SCANCODE_4, PS2_SCANCODE_MAIN_4)
#define PS2_KEY_5_SHORT() PS2_KEY_DIGIT_SHORT(PS2_SCANCODE_5, PS2_SCANCODE_MAIN_5)
#define PS2_KEY_6_SHORT() PS2_KEY_DIGIT_SHORT(PS2_SCANCODE_6, PS2_SCANCODE_MAIN_6)
#define PS2_KEY_7_SHORT() PS2_KEY_DIGIT_SHORT(PS2_SCANCODE_7, PS2_SCANCODE_MAIN_7)
#define PS2_KEY_8_SHORT() PS2_KEY_DIGIT_SHORT(PS2_SCANCODE_8, PS2_SCANCODE_MAIN_8)
#define PS2_KEY_9_SHORT() PS2_KEY_DIGIT_SHORT(PS2_SCANCODE_9, PS2_SCANCODE_MAIN_9)
#define PS2_KEY_ENTER_SHORT()       PS2_KEY_IS_SHORT_PRESSED(&g_ps2KeyStateMachine, PS2_SCANCODE_ENTER)
#define PS2_KEY_BACKSPACE_SHORT()   PS2_KEY_IS_SHORT_PRESSED(&g_ps2KeyStateMachine, PS2_SCANCODE_BACKSPACE)

/******************************************************************************************/
/* 函数声明 */

void PS2_Keyboard_Init(void);
bool PS2_ReadRawScanCode(uint8_t* pScanCode);
void PS2_KeyStateMachine_Sweep(PS2_KeyStateMachine_t* pKeyFSM);
void PS2_KeyScan(void);
PS2_KeyValue_t PS2_GetCurrentKeyValue(void);
void PS2_ClearStateMachine(void);

#ifdef __cplusplus
}
#endif

#endif /* __PS2_H */
