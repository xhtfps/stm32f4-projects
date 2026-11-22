/**
 * @file Drivers/TFT_LCD/xpt2046.c
 * @brief XPT2046/ADS7843 电阻触摸控制器驱动模块。
 * @encoding UTF-8
 * 
 * 本模块提供了XPT2046触摸控制器的底层驱动接口，包括初始化、读取触摸点坐标、
 * 检测触摸压力等功能。支持通过SPI协议与控制器通信，并提供了软件滤波、坐标映射、
 * 坐标变换等处理。
 */

#include "xpt2046.h"
#include "./SYSTEM/delay/delay.h"
#include <string.h>

/*============================== 宏定义 ==============================*/
/**
 * @name XPT2046命令字定义
 * @{
 */
#define XPT2046_CMD_READ_X                0xD0U   /**< 读取X坐标命令 */
#define XPT2046_CMD_READ_Y                0x90U   /**< 读取Y坐标命令 */
#define XPT2046_CMD_READ_Z1               0xB0U   /**< 读取Z1压力相关值命令 */
#define XPT2046_CMD_READ_Z2               0xC0U   /**< 读取Z2压力相关值命令 */
/** @} */

#define XPT2046_SAMPLE_COUNT              15U     /**< 每次读取的采样次数（兼容Drive_Touch算法） */
#define XPT2046_LOST_SAMPLE_COUNT         5U      /**< 排序后两端舍弃的采样个数 */
#define XPT2046_PRESSURE_MIN              20U     /**< 最小有效压力阈值（用于判断是否按下） */
#define XPT2046_RAW_VALID_MIN             32U     /**< 原始ADC有效最小值 */
#define XPT2046_RAW_VALID_MAX             4060U   /**< 原始ADC有效最大值 */
#define XPT2046_RAW_SCALE                 2048U   /**< 旧Drive_Touch坐标换算使用的ADC满量程 */
#define XPT2046_NO_TOUCH_COORD            0xFFFFU

#define XPT2046_SHARED_FLASH_CS_GPIO_PORT GPIOB
#define XPT2046_SHARED_FLASH_CS_GPIO_PIN  GPIO_PIN_9
#define XPT2046_SHARED_SD_CS_GPIO_PORT    GPIOB
#define XPT2046_SHARED_SD_CS_GPIO_PIN     GPIO_PIN_10

/*============================== GPIO操作宏 ==============================*/
/**
 * @name 引脚操作宏定义
 * @{
 */
#define XPT2046_CS_HIGH()                 HAL_GPIO_WritePin(XPT2046_CS_GPIO_PORT, XPT2046_CS_GPIO_PIN, GPIO_PIN_SET)   /**< 片选CS置高（取消选中） */
#define XPT2046_CS_LOW()                  HAL_GPIO_WritePin(XPT2046_CS_GPIO_PORT, XPT2046_CS_GPIO_PIN, GPIO_PIN_RESET)  /**< 片选CS置低（选中） */
#define XPT2046_CLK_HIGH()                HAL_GPIO_WritePin(XPT2046_CLK_GPIO_PORT, XPT2046_CLK_GPIO_PIN, GPIO_PIN_SET)  /**< 时钟线CLK置高 */
#define XPT2046_CLK_LOW()                 HAL_GPIO_WritePin(XPT2046_CLK_GPIO_PORT, XPT2046_CLK_GPIO_PIN, GPIO_PIN_RESET) /**< 时钟线CLK置低 */
#define XPT2046_MOSI_HIGH()               HAL_GPIO_WritePin(XPT2046_MOSI_GPIO_PORT, XPT2046_MOSI_GPIO_PIN, GPIO_PIN_SET) /**< MOSI置高 */
#define XPT2046_MOSI_LOW()                HAL_GPIO_WritePin(XPT2046_MOSI_GPIO_PORT, XPT2046_MOSI_GPIO_PIN, GPIO_PIN_RESET)/**< MOSI置低 */
#define XPT2046_MISO_READ()               HAL_GPIO_ReadPin(XPT2046_MISO_GPIO_PORT, XPT2046_MISO_GPIO_PIN)               /**< 读取MISO引脚电平 */
/** @} */

/*============================== 内部函数声明 ==============================*/
static void Xpt2046Device_Construct(Xpt2046Device *self);
static void Xpt2046Device_Init(Xpt2046Device *self);
static uint8_t Xpt2046Device_ReadPoint(Xpt2046Device *self, Xpt2046Point *point);
static uint8_t Xpt2046Device_IsPressed(Xpt2046Device *self);
static void Xpt2046Device_ReleaseSharedSpiBus(void);
static void Xpt2046Device_GpioInit(void);
static void Xpt2046Device_WriteByte(uint8_t data);
static uint16_t Xpt2046Device_ReadAd(uint8_t command);
static uint16_t Xpt2046Device_ReadAverage(uint8_t command);
static uint8_t Xpt2046Device_IsRawValid(uint16_t x, uint16_t y);
static uint16_t Xpt2046Device_CorrectCenter(uint16_t value, uint16_t center, uint16_t max);

/**
 * @brief XPT2046设备操作函数表（虚函数表）
 */
static const Xpt2046DeviceOps s_xpt2046_ops =
{
    Xpt2046Device_Init,       /**< 初始化函数指针 */
    Xpt2046Device_ReadPoint,  /**< 读取触摸点函数指针 */
    Xpt2046Device_IsPressed   /**< 检测是否按下函数指针 */
};

/**
 * @brief 全局XPT2046设备实例
 */
Xpt2046Device g_xpt2046_touch;

volatile uint16_t g_xpt2046_debug_z1;
volatile uint16_t g_xpt2046_debug_z2;
volatile uint16_t g_xpt2046_debug_raw_x;
volatile uint16_t g_xpt2046_debug_raw_y;
volatile uint8_t g_xpt2046_debug_rx_hi;
volatile uint8_t g_xpt2046_debug_rx_lo;
volatile uint8_t g_xpt2046_debug_pin_cs;
volatile uint8_t g_xpt2046_debug_pin_clk;
volatile uint8_t g_xpt2046_debug_pin_mosi;
volatile uint8_t g_xpt2046_debug_pin_miso;
volatile uint8_t g_xpt2046_debug_pressed;
volatile uint8_t g_xpt2046_debug_fail_reason;

/*============================== 函数实现 ==============================*/

/**
 * @brief 构造XPT2046设备对象（初始化对象成员）
 * @param self 指向Xpt2046Device对象的指针
 * @note 该函数设置对象的操作函数表，并将initialized标志清零。
 *       如果self为空或ops已存在，则直接返回。
 */
static void Xpt2046Device_Construct(Xpt2046Device *self)
{
    if ((self == NULL) || (self->ops != NULL))
    {
        return;
    }

    self->initialized = 0U;      /* 设备尚未初始化 */
    self->ops = &s_xpt2046_ops;  /* 绑定操作函数表 */
}

/**
 * @brief 释放与触摸共用SPI线的其它器件片选。
 * @note 触摸与板载Flash/SD可能共用CLK/MISO/MOSI，如果其它片选悬空为低，
 *       MISO会被占用，XPT2046读数会一直为0。
 */
static void Xpt2046Device_ReleaseSharedSpiBus(void)
{
    GPIO_InitTypeDef gpio_init_struct;
    uint16_t shared_cs_pins;

    __HAL_RCC_GPIOB_CLK_ENABLE();

    shared_cs_pins = XPT2046_SHARED_FLASH_CS_GPIO_PIN;
#if (XPT2046_USE_SHARED_SD_CS != 0U)
    shared_cs_pins |= XPT2046_SHARED_SD_CS_GPIO_PIN;
#endif

    HAL_GPIO_WritePin(XPT2046_SHARED_FLASH_CS_GPIO_PORT,
                      shared_cs_pins,
                      GPIO_PIN_SET);

    gpio_init_struct.Mode = GPIO_MODE_OUTPUT_PP;
    gpio_init_struct.Pull = GPIO_PULLUP;
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;

    gpio_init_struct.Pin = shared_cs_pins;
    HAL_GPIO_Init(GPIOB, &gpio_init_struct);
}

/**
 * @brief 初始化XPT2046所使用的GPIO引脚
 * @note 配置CS、CLK、MOSI为推挽输出，MISO为输入，并设置默认电平。
 *       若使能了IRQ引脚（XPT2046_USE_IRQ_PIN），则将其配置为输入。
 */
static void Xpt2046Device_GpioInit(void)
{
    GPIO_InitTypeDef gpio_init_struct;

    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    Xpt2046Device_ReleaseSharedSpiBus();

    /* 参考可用工程：CLK/MOSI 在 GPIOB，CS 单独位于 GPIOC13。 */
    gpio_init_struct.Pin = XPT2046_CLK_GPIO_PIN | XPT2046_MOSI_GPIO_PIN;
    gpio_init_struct.Mode = GPIO_MODE_OUTPUT_PP;
    gpio_init_struct.Pull = GPIO_NOPULL;
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &gpio_init_struct);

    gpio_init_struct.Pin = XPT2046_CS_GPIO_PIN;
    HAL_GPIO_Init(XPT2046_CS_GPIO_PORT, &gpio_init_struct);

    /* 配置MISO为输入 */
    gpio_init_struct.Pin = XPT2046_MISO_GPIO_PIN;
    gpio_init_struct.Mode = GPIO_MODE_INPUT;
    gpio_init_struct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(XPT2046_MISO_GPIO_PORT, &gpio_init_struct);

#if XPT2046_USE_IRQ_PIN
    gpio_init_struct.Pin = XPT2046_IRQ_GPIO_PIN;
    gpio_init_struct.Mode = GPIO_MODE_INPUT;
    gpio_init_struct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(XPT2046_IRQ_GPIO_PORT, &gpio_init_struct);
#endif

    /* 完全对齐历史 Drive_Touch.c 的上电空闲态。 */
    XPT2046_CLK_HIGH();
    XPT2046_CS_HIGH();
    XPT2046_MOSI_HIGH();

    XPT2046_CS_HIGH();
}

/**
 * @brief 初始化XPT2046设备
 * @param self 指向Xpt2046Device对象的指针
 * @note 该函数初始化硬件GPIO，并设置initialized标志。
 */
static void Xpt2046Device_Init(Xpt2046Device *self)
{
    if (self == NULL)
    {
        return;
    }

    Xpt2046Device_GpioInit();   /* 配置引脚 */
    self->initialized = 1U;     /* 标记已初始化 */
}

/**
 * @brief SPI单字节传输（软件模拟）
 * @param data 要发送的数据（8位）
 * @return 接收到的数据（8位）
 * @note 使用GPIO模拟SPI时序，时钟极性为低电平空闲，上升沿采样。
 *       每发送一位同时接收一位，先发送高位。
 */
static void Xpt2046Device_WriteByte(uint8_t data)
{
    uint8_t index;

    for (index = 0U; index < 8U; index++)
    {
        if ((data & 0x80U) != 0U)
        {
            XPT2046_MOSI_HIGH();
        }
        else
        {
            XPT2046_MOSI_LOW();
        }

        data <<= 1;
        delay_us(1U);
        XPT2046_CLK_LOW();
        delay_us(1U);
        XPT2046_CLK_HIGH();
        delay_us(1U);
    }
}

/**
 * @brief 读取指定命令的ADC原始值（12位）
 * @param command 读取命令（例如XPT2046_CMD_READ_X）
 * @return 12位ADC原始值（0~4095），已右移3位对齐
 * @note 通信时序：片选有效后发送命令字，然后连续读取两个字节组成12位数据。
 *       最终结果右移3位，得到0~4095的有效值。
 */
static uint16_t Xpt2046Device_ReadAd(uint8_t command)
{
    uint8_t index;
    uint16_t value = 0U;

    XPT2046_CLK_LOW();                         /* 先拉低时钟 */
    XPT2046_CS_LOW();                         /* 选中设备 */
    g_xpt2046_debug_pin_cs = (uint8_t)HAL_GPIO_ReadPin(XPT2046_CS_GPIO_PORT, XPT2046_CS_GPIO_PIN);
    g_xpt2046_debug_pin_clk = (uint8_t)HAL_GPIO_ReadPin(XPT2046_CLK_GPIO_PORT, XPT2046_CLK_GPIO_PIN);
    g_xpt2046_debug_pin_mosi = (uint8_t)HAL_GPIO_ReadPin(XPT2046_MOSI_GPIO_PORT, XPT2046_MOSI_GPIO_PIN);
    g_xpt2046_debug_pin_miso = (uint8_t)HAL_GPIO_ReadPin(XPT2046_MISO_GPIO_PORT, XPT2046_MISO_GPIO_PIN);
    Xpt2046Device_WriteByte(command);         /* 发送命令字 */
    delay_us(6U);                             /* ADS7846最长转换时间 */

    XPT2046_CLK_HIGH();                       /* 给1个时钟，清除BUSY */
    delay_us(1U);
    XPT2046_CLK_LOW();
    delay_us(1U);

    for (index = 0U; index < 16U; index++)
    {
        value <<= 1;
        XPT2046_CLK_LOW();
        delay_us(1U);
        XPT2046_CLK_HIGH();
        delay_us(1U);
        if (XPT2046_MISO_READ() == GPIO_PIN_SET)
        {
            value++;
        }
        delay_us(1U);
    }

    XPT2046_CS_HIGH();
    delay_us(1U);

    g_xpt2046_debug_rx_hi = (uint8_t)(value >> 8);
    g_xpt2046_debug_rx_lo = (uint8_t)(value & 0xFFU);
    value >>= 4;                              /* 旧Drive_Touch逻辑：高12位有效 */
    return (uint16_t)(value & 0x0FFFU);
}

/**
 * @brief 读取多次ADC值并滤波（去极值平均）
 * @param command 读取命令
 * @return 滤波后的ADC值
 * @note 采样XPT2046_SAMPLE_COUNT次，去掉最大值和最小值，再取平均值。
 *       有助于消除偶然噪声。
 */
static uint16_t Xpt2046Device_ReadAverage(uint8_t command)
{
    uint8_t index;
    uint8_t sort_index;
    uint16_t samples[XPT2046_SAMPLE_COUNT];
    uint16_t temp;
    uint32_t sum = 0U;

    for (index = 0U; index < XPT2046_SAMPLE_COUNT; index++)
    {
        samples[index] = Xpt2046Device_ReadAd(command);
    }

    for (index = 0U; index < (XPT2046_SAMPLE_COUNT - 1U); index++)
    {
        for (sort_index = (uint8_t)(index + 1U); sort_index < XPT2046_SAMPLE_COUNT; sort_index++)
        {
            if (samples[index] > samples[sort_index])
            {
                temp = samples[index];
                samples[index] = samples[sort_index];
                samples[sort_index] = temp;
            }
        }
    }

    for (index = XPT2046_LOST_SAMPLE_COUNT; index < (XPT2046_SAMPLE_COUNT - XPT2046_LOST_SAMPLE_COUNT); index++)
    {
        sum += samples[index];
    }

    return (uint16_t)(sum / (XPT2046_SAMPLE_COUNT - (2U * XPT2046_LOST_SAMPLE_COUNT)));
}

static uint8_t Xpt2046Device_IsRawValid(uint16_t x, uint16_t y)
{
    if ((x == 0U) || (x > XPT2046_LCD_WIDTH) ||
        (y == 0U) || (y > XPT2046_LCD_HEIGHT))
    {
        g_xpt2046_debug_fail_reason = 2U;
        return 0U;
    }

    return 1U;
}

static uint16_t Xpt2046Device_CorrectCenter(uint16_t value, uint16_t center, uint16_t max)
{
    uint32_t corrected;

    if (value > center)
    {
        corrected = (uint32_t)value + ((uint32_t)value - center) / 20U;
    }
    else
    {
        corrected = (uint32_t)value - (center - (uint32_t)value) / 20U;
    }

    if (corrected > max)
    {
        return (uint16_t)(max + 1U);
    }

    return (uint16_t)corrected;
}

/**
 * @brief 检测触摸屏是否被按下
 * @param self 指向Xpt2046Device对象的指针
 * @return 1: 按下, 0: 未按下
 * @note 如果定义了XPT2046_USE_IRQ_PIN，则直接读取IRQ引脚电平判断。
 *       否则通过读取Z1/Z2压力值并比较阈值来判断。
 */
static uint8_t Xpt2046Device_IsPressed(Xpt2046Device *self)
{
    uint16_t z1;
    uint16_t z2;

    if (self == NULL)
    {
        return 0U;
    }

    if (self->initialized == 0U)
    {
        self->ops->init(self);   /* 未初始化则先初始化 */
    }

#if XPT2046_USE_IRQ_PIN
    /* 使用IRQ引脚：低电平表示触摸按下 */
    return (HAL_GPIO_ReadPin(XPT2046_IRQ_GPIO_PORT, XPT2046_IRQ_GPIO_PIN) == GPIO_PIN_RESET) ? 1U : 0U;
#else
    /* 无IRQ引脚，读取Z1、Z2并判断压力 */
    z1 = Xpt2046Device_ReadAd(XPT2046_CMD_READ_Z1);
    z2 = Xpt2046Device_ReadAd(XPT2046_CMD_READ_Z2);
    /* 压力条件：Z1 > 阈值 且 Z2 > Z1（典型触摸特性） */
    g_xpt2046_debug_z1 = z1;
    g_xpt2046_debug_z2 = z2;

    if ((z1 > XPT2046_PRESSURE_MIN) && (z1 < XPT2046_RAW_VALID_MAX))
    {
        g_xpt2046_debug_pressed = 1U;
        return 1U;
    }

    g_xpt2046_debug_pressed = 0U;
    return 0U;
#endif
}

/**
 * @brief 读取触摸点坐标及压力信息
 * @param self  指向Xpt2046Device对象的指针
 * @param point 指向Xpt2046Point结构体的指针，用于存储读取到的触摸信息
 * @return 1: 成功读取到有效的触摸点, 0: 未触摸或数据无效
 * @note 该函数会先判断触摸是否按下，然后读取X/Y原始值，进行有效性检查、
 *       可选坐标交换、映射、反转等处理，最后填充point结构。
 */
static uint8_t Xpt2046Device_ReadPoint(Xpt2046Device *self, Xpt2046Point *point)
{
    uint16_t raw_x;
    uint16_t raw_y;
    uint16_t mapped_x;
    uint16_t mapped_y;

    if ((self == NULL) || (point == NULL))
    {
        return 0U;
    }

    if (self->initialized == 0U)
    {
        self->ops->init(self);
    }

    memset(point, 0, sizeof(*point));   /* 清空输出结构 */

    if (self->ops->is_pressed(self) == 0U)
    {
        g_xpt2046_debug_fail_reason = 1U;
        g_xpt2046_debug_pressed = 0U;
        return 0U;
    }

    /* 兼容Drive_Touch.c的读法：15次采样、排序、丢极值后再映射坐标。 */
    raw_x = Xpt2046Device_ReadAverage(XPT2046_CMD_READ_X);
    raw_y = Xpt2046Device_ReadAverage(XPT2046_CMD_READ_Y);
    g_xpt2046_debug_raw_x = raw_x;
    g_xpt2046_debug_raw_y = raw_y;

#if XPT2046_SWAP_XY
    /* 如果需要交换XY坐标（例如LCD安装方向不同） */
    {
        uint16_t raw_tmp = raw_x;
        raw_x = raw_y;
        raw_y = raw_tmp;
    }
#endif

#if XPT2046_SCREEN_TOUCH == 1U
    mapped_x = (uint16_t)(XPT2046_LCD_WIDTH -
                          (((uint32_t)raw_x * XPT2046_LCD_WIDTH) / XPT2046_RAW_SCALE));
    mapped_y = (uint16_t)(XPT2046_LCD_HEIGHT -
                          (((uint32_t)raw_y * XPT2046_LCD_HEIGHT) / XPT2046_RAW_SCALE));

    if (mapped_x > (XPT2046_LCD_WIDTH / 2U))
    {
        mapped_x = (uint16_t)(mapped_x + (mapped_x - (XPT2046_LCD_WIDTH / 2U)) / 20U);
    }
    else
    {
        mapped_x = (uint16_t)(mapped_x - ((XPT2046_LCD_WIDTH / 2U) - mapped_x) / 10U);
    }

    if (mapped_y > (XPT2046_LCD_HEIGHT / 2U))
    {
        mapped_y = (uint16_t)(mapped_y + (mapped_y - (XPT2046_LCD_HEIGHT / 2U)) / 10U);
    }
    else
    {
        mapped_y = (uint16_t)(mapped_y - ((XPT2046_LCD_HEIGHT / 2U) - mapped_y) / 5U);
    }
#else
    mapped_x = (uint16_t)(((uint32_t)raw_x * XPT2046_LCD_WIDTH) / XPT2046_RAW_SCALE);
    mapped_y = (uint16_t)(XPT2046_LCD_HEIGHT -
                          (((uint32_t)raw_y * XPT2046_LCD_HEIGHT) / XPT2046_RAW_SCALE));

    mapped_x = Xpt2046Device_CorrectCenter(mapped_x, XPT2046_LCD_WIDTH / 2U, XPT2046_LCD_WIDTH);
    mapped_y = Xpt2046Device_CorrectCenter(mapped_y, XPT2046_LCD_HEIGHT / 2U, XPT2046_LCD_HEIGHT);
#endif

#if XPT2046_INVERT_X
    /* X轴反向（镜像） */
    mapped_x = (uint16_t)(XPT2046_LCD_WIDTH - 1U - mapped_x);
#endif

#if XPT2046_INVERT_Y
    /* Y轴反向（镜像） */
    mapped_y = (uint16_t)(XPT2046_LCD_HEIGHT - 1U - mapped_y);
#endif

    if (Xpt2046Device_IsRawValid(mapped_x, mapped_y) == 0U)
    {
        g_xpt2046_debug_pressed = 0U;
        return 0U;
    }

    /* 填充结果结构体 */
    point->raw_x = raw_x;
    point->raw_y = raw_y;
    point->x = mapped_x;
    point->y = mapped_y;
    point->pressure = g_xpt2046_debug_z1;
    point->pressed = 1U;
    g_xpt2046_debug_pressed = 1U;
    g_xpt2046_debug_fail_reason = 0U;

    return 1U;
}

/*============================== 外部API函数 ==============================*/

/**
 * @brief 初始化XPT2046触摸控制器（全局接口）
 * @note 该函数构造并初始化全局设备对象g_xpt2046_touch。
 *       用户在使用触摸功能前应先调用此函数。
 */
void XPT2046_Init(void)
{
    Xpt2046Device_Construct(&g_xpt2046_touch);
    g_xpt2046_touch.ops->init(&g_xpt2046_touch);
}

void XPT2046_ReleaseBus(void)
{
    GPIO_InitTypeDef gpio_init_struct;

    Xpt2046Device_ReleaseSharedSpiBus();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    gpio_init_struct.Pin = XPT2046_CS_GPIO_PIN;
    gpio_init_struct.Mode = GPIO_MODE_OUTPUT_PP;
    gpio_init_struct.Pull = GPIO_NOPULL;
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(XPT2046_CS_GPIO_PORT, &gpio_init_struct);

    XPT2046_CS_HIGH();
}

/**
 * @brief 读取当前触摸点坐标（全局接口）
 * @param point 指向Xpt2046Point结构体的指针，用于存储读取到的触摸信息
 * @return 1: 成功读取到有效的触摸点, 0: 未触摸或数据无效
 * @note 该函数会确保设备对象已构造，然后调用内部读取函数。
 */
uint8_t XPT2046_ReadPoint(Xpt2046Point *point)
{
    Xpt2046Device_Construct(&g_xpt2046_touch);
    return g_xpt2046_touch.ops->read_point(&g_xpt2046_touch, point);
}

void XPT2046_TouchRead(uint16_t *x, uint16_t *y)
{
    Xpt2046Point point;

    if ((x == NULL) || (y == NULL))
    {
        return;
    }

    if (XPT2046_ReadPoint(&point) != 0U)
    {
        *x = point.x;
        *y = point.y;
    }
    else
    {
        *x = XPT2046_NO_TOUCH_COORD;
        *y = XPT2046_NO_TOUCH_COORD;
    }
}

void TouchRead(uint16_t *x, uint16_t *y)
{
    XPT2046_TouchRead(x, y);
}

/**
 * @brief 检测触摸屏是否被按下（全局接口）
 * @return 1: 按下, 0: 未按下
 * @note 该函数会确保设备对象已构造，然后调用内部检测函数。
 */
uint8_t XPT2046_IsPressed(void)
{
    Xpt2046Device_Construct(&g_xpt2046_touch);
    return g_xpt2046_touch.ops->is_pressed(&g_xpt2046_touch);
}
