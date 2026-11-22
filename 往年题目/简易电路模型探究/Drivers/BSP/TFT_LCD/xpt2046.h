/**
 * @file Drivers/TFT_LCD/xpt2046.h
 * @brief XPT2046/ADS7843 电阻触摸控制器驱动模块。
 * @details
 * - 属于显示子系统的输入驱动，负责触摸控制器 GPIO/SPI 时序和坐标读取。
 * - 默认使用软件 SPI，管脚与已验证可用的“基地板调试程序”保持一致：
 *   PC13=CS、PB13=CLK、PB14=MISO、PB15=MOSI、PD6=T_PEN/IRQ。
 * - 当前 LVGL 端口默认仍走无 IRQ 轮询，避免引入额外中断依赖。
 * @encoding UTF-8
 */

#ifndef __XPT2046_H
#define __XPT2046_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

/* ========================================================================== */
/* 硬件连接配置                                                               */
/* ========================================================================== */

/**
 * @name XPT2046 硬件引脚定义
 * @{
 */
#define XPT2046_CS_GPIO_PORT              GPIOC   /**< 片选CS GPIO端口 */
#define XPT2046_CS_GPIO_PIN               GPIO_PIN_13  /**< 片选CS引脚 */
#define XPT2046_CLK_GPIO_PORT             GPIOB   /**< 时钟CLK GPIO端口 */
#define XPT2046_CLK_GPIO_PIN              GPIO_PIN_13  /**< 时钟CLK引脚 */
#define XPT2046_MISO_GPIO_PORT            GPIOB   /**< MISO GPIO端口 */
#define XPT2046_MISO_GPIO_PIN             GPIO_PIN_14  /**< MISO引脚 */
#define XPT2046_MOSI_GPIO_PORT            GPIOB   /**< MOSI GPIO端口 */
#define XPT2046_MOSI_GPIO_PIN             GPIO_PIN_15  /**< MOSI引脚 */

#define XPT2046_IRQ_GPIO_PORT             GPIOD   /**< 中断IRQ GPIO端口 */
#define XPT2046_IRQ_GPIO_PIN              GPIO_PIN_6  /**< 中断IRQ引脚 */
/** @} */

/**
 * @brief 是否需要额外拉高与触摸共享 SPI 总线的板载 SD 片选。
 * @note 当前工程使用 USART3(PB10/PB11) 作为机械臂协议串口，其中 PB10 同时是历史
 *       共享 SD 片选脚。若继续由触摸驱动把 PB10 配置成普通 GPIO，会破坏 USART3_TX。
 *       因此这里默认关闭 SD 片选释放，只保留板载 Flash(PB9) 的释放动作。
 */
#define XPT2046_USE_SHARED_SD_CS          0U

/**
 * @brief 是否使用IRQ引脚进行触摸检测
 * @note 参考工程未依赖 T_PEN 中断，当前默认继续关闭 IRQ 判按下逻辑。
 */
#define XPT2046_USE_IRQ_PIN               0U

/* ========================================================================== */
/* 坐标校准配置                                                               */
/* ========================================================================== */

/**
 * @name LCD显示尺寸
 * @{
 */
#define XPT2046_LCD_WIDTH                 800U   /**< LCD宽度（像素） */
#define XPT2046_LCD_HEIGHT                480U   /**< LCD高度（像素） */
/** @} */

/**
 * @name 原始ADC有效范围（需根据实际触摸屏校准）
 * @{
 */
#define XPT2046_RAW_X_MIN                 200U   /**< X坐标原始ADC最小值 */
#define XPT2046_RAW_X_MAX                 3900U  /**< X坐标原始ADC最大值 */
#define XPT2046_RAW_Y_MIN                 200U   /**< Y坐标原始ADC最小值 */
#define XPT2046_RAW_Y_MAX                 3900U  /**< Y坐标原始ADC最大值 */
/** @} */

/**
 * @name 坐标变换选项
 * @{
 */
#define XPT2046_SWAP_XY                   0U     /**< 交换X/Y坐标（1=交换） */
#define XPT2046_INVERT_X                  0U     /**< X轴反向（1=镜像） */
#define XPT2046_INVERT_Y                  0U     /**< Y轴反向（1=镜像） */
/** @} */

/**
 * @brief 兼容历史 Drive_Touch.c 的屏幕走线方向配置
 * @note 参考“基地板调试程序”当前生效的是 else 分支，即中间排线方案。
 */
#define XPT2046_SCREEN_TOUCH              0U

/**
 * @brief 触摸点信息结构体
 * @details 存储从XPT2046读取到的原始数据和处理后的像素坐标。
 */
typedef struct
{
    uint16_t raw_x;       /**< X方向原始ADC值（未经过滤波和映射） */
    uint16_t raw_y;       /**< Y方向原始ADC值（未经过滤波和映射） */
    uint16_t x;           /**< 映射到LCD像素坐标后的X坐标（范围0~LCD_WIDTH-1） */
    uint16_t y;           /**< 映射到LCD像素坐标后的Y坐标（范围0~LCD_HEIGHT-1） */
    uint16_t pressure;    /**< 压力参考值，未按下时为0（实际为Z1测量值） */
    uint8_t pressed;      /**< 1表示当前检测到按下，0表示未按下 */
} Xpt2046Point;

/**
 * @brief XPT2046设备结构体（不透明指针前向声明）
 */
typedef struct Xpt2046Device Xpt2046Device;

/**
 * @brief XPT2046设备操作函数表（虚函数表）
 * @details 通过函数指针实现面向对象风格的设备操作。
 */
typedef struct
{
    void (*init)(Xpt2046Device *self);                                 /**< 设备初始化函数指针 */
    uint8_t (*read_point)(Xpt2046Device *self, Xpt2046Point *point);   /**< 读取触摸点函数指针 */
    uint8_t (*is_pressed)(Xpt2046Device *self);                        /**< 检测按下状态函数指针 */
} Xpt2046DeviceOps;

/**
 * @brief XPT2046设备实例结构体
 * @details 包含初始化标志和操作函数表指针。
 */
struct Xpt2046Device
{
    uint8_t initialized;                 /**< 初始化标志（1=已初始化） */
    const Xpt2046DeviceOps *ops;        /**< 操作函数表指针（指向常量函数表） */
};

/**
 * @brief 全局XPT2046设备实例
 * @note 由驱动模块内部定义，外部可直接使用。
 */
extern Xpt2046Device g_xpt2046_touch;

/*============================== 外部API函数声明 ==============================*/

/**
 * @brief 初始化XPT2046触摸控制器（全局接口）
 * @note 调用前需确保硬件连接正确。该函数会初始化GPIO并设置内部状态。
 */
void XPT2046_Init(void);

/**
 * @brief 释放触摸和同组SPI外设片选，不启用触摸读取。
 * @note 用于禁用LVGL触摸输入后，仍保证PB9/PB10/PB12等片选处于非选中状态。
 */
void XPT2046_ReleaseBus(void);

/**
 * @brief 读取当前触摸点坐标（全局接口）
 * @param point 指向Xpt2046Point结构体的指针，用于存储读取到的触摸信息
 * @return 1: 成功读取到有效的触摸点, 0: 未触摸或数据无效
 * @note 该函数内部会进行软件滤波、坐标映射和有效性检查。
 */
uint8_t XPT2046_ReadPoint(Xpt2046Point *point);

/**
 * @brief 读取触摸坐标，兼容App_Touch风格接口。
 * @param x 输出X坐标；无触摸时为0xFFFF
 * @param y 输出Y坐标；无触摸时为0xFFFF
 */
void XPT2046_TouchRead(uint16_t *x, uint16_t *y);

/**
 * @brief 兼容旧触摸应用层的TouchRead命名。
 */
void TouchRead(uint16_t *x, uint16_t *y);

/**
 * @brief 检测触摸屏是否被按下（全局接口）
 * @return 1: 按下, 0: 未按下
 * @note 根据配置使用IRQ引脚或压力测量来判断。
 */
uint8_t XPT2046_IsPressed(void);

#ifdef __cplusplus
}
#endif

#endif /* __XPT2046_H */
