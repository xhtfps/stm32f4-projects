/**
 * @file Drivers/TFT_LCD/spi.c
 * @brief TFT 字库 Flash 使用的 SPI1 总线对象模块。
 * @details
 * - 属于显示子系统，分别负责 LCD、字库 Flash、字体数据或文字渲染中的明确职责。
 * - 模块间通过对象接口或兼容 API 协作，避免 UI 层直接操作显示控制器寄存器。
 * - 字体/字模数据文件以数组尺寸、字号和索引关系一致性为维护重点。
 * - 本文件已移除原子/ST 示例模板，统一使用项目自定义 UTF-8 文件模板。
 * @encoding UTF-8
 */

#include "spi.h"

static void TftSpiBus_Init(TftSpiBus *self);
static void TftSpiBus_SetSpeed(TftSpiBus *self, uint32_t baudrate_prescaler);
static uint8_t TftSpiBus_Transfer(TftSpiBus *self, uint8_t tx_data);
static void TftSpiBus_GpioInit(void);
static void TftSpiBus_Construct(TftSpiBus *self);

static const TftSpiBusOps s_tft_spi_bus_ops =
{
    TftSpiBus_Init,
    TftSpiBus_SetSpeed,
    TftSpiBus_Transfer
};

SPI_HandleTypeDef g_spi1_handler;
TftSpiBus g_tft_spi1_bus;

static void TftSpiBus_Construct(TftSpiBus *self)
{
    if ((self == NULL) || (self->ops != NULL))
    {
        return;
    }

    self->handle = &g_spi1_handler;
    self->instance = SPI1;
    self->default_prescaler = SPI_BAUDRATEPRESCALER_256;
    self->ops = &s_tft_spi_bus_ops;
}

/**
 * @brief  初始化 SPI1 的 GPIO 引脚
 * @note   静态函数，仅供本文件内部调用。配置引脚为：PB3=SCK，PA6=MISO，PA7=MOSI
 * @param  无
 * @retval 无
 */
static void TftSpiBus_GpioInit(void)
{
    GPIO_InitTypeDef gpio_init_struct;

    // 使能 GPIOA 和 GPIOB 的外设时钟
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    // 通用 GPIO 配置
    gpio_init_struct.Mode = GPIO_MODE_AF_PP;          // 复用推挽输出模式（SPI 引脚必需）
    gpio_init_struct.Pull = GPIO_PULLUP;              // 开启上拉电阻，保证空闲时电平稳定
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_VERY_HIGH; // 超高翻转速度，适应高速 SPI 通信
    gpio_init_struct.Alternate = GPIO_AF5_SPI1;       // 将引脚复用映射到 SPI1 外设

    // 配置 PA6 (MISO) 和 PA7 (MOSI) 
    gpio_init_struct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    HAL_GPIO_Init(GPIOA, &gpio_init_struct);

    // 配置 PB3 (SCK)
    gpio_init_struct.Pin = GPIO_PIN_3;
    HAL_GPIO_Init(GPIOB, &gpio_init_struct);
}

/**
 * @brief  初始化 SPI1 外设
 * @details 配置为单主机、全双工模式，8位数据位，模式 3 (CPOL=1, CPHA=2)
 * @param  无
 * @retval 无
 */
static void TftSpiBus_Init(TftSpiBus *self)
{
    if ((self == NULL) || (self->handle == NULL))
    {
        return;
    }

    TftSpiBus_GpioInit();           // 初始化底层的 GPIO 引脚
    __HAL_RCC_SPI1_CLK_ENABLE(); // 使能 SPI1 外设时钟

    self->handle->Instance = self->instance;                           // 选择 SPI1 寄存器基地址
    self->handle->Init.Mode = SPI_MODE_MASTER;                         // 设置为主机模式 (Master)
    self->handle->Init.Direction = SPI_DIRECTION_2LINES;               // 双线全双工通信 (MISO + MOSI)
    self->handle->Init.DataSize = SPI_DATASIZE_8BIT;                   // 帧数据长度为 8 位
    self->handle->Init.CLKPolarity = SPI_POLARITY_HIGH;                // 时钟极性 CPOL=1：空闲时 SCLK 为高电平
    self->handle->Init.CLKPhase = SPI_PHASE_2EDGE;                     // 时钟相位 CPHA=1：在 SCLK 的第二个跳变沿捕捉数据
    self->handle->Init.NSS = SPI_NSS_SOFT;                             // NSS（片选）由软件控制，不使用硬件 NSS 引脚
    self->handle->Init.BaudRatePrescaler = self->default_prescaler;    // 默认最慢速度，确保安全
    self->handle->Init.FirstBit = SPI_FIRSTBIT_MSB;                    // 数据传输高位在前 (Most Significant Bit)
    self->handle->Init.TIMode = SPI_TIMODE_DISABLE;                    // 关闭 TI 模式（使用 Motorola SPI 模式）
    self->handle->Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;    // 关闭硬件 CRC 校验
    self->handle->Init.CRCPolynomial = 7;                              // CRC 多项式寄存器（未开启 CRC 时此值无效）

    // 依照上述参数初始化 SPI1
    HAL_SPI_Init(self->handle);
    
    // 启动时发送一个空字节 (0xFF) 以激活 SPI 总线时钟，并清除可能存在的初始杂波
    (void)self->ops->transfer(self, 0xFF);
}

/**
 * @brief  动态修改 SPI1 的通讯速率（分频值）
 * @note   通过直接操作底层寄存器来实现快速切换，不影响现有代码的调用结构
 * @param  SPI_BaudRatePrescaler: 新的分频系数（例如 SPI_BAUDRATEPRESCALER_4）
 * @retval 无
 */
static void TftSpiBus_SetSpeed(TftSpiBus *self, uint32_t baudrate_prescaler)
{
    if ((self == NULL) || (self->handle == NULL))
    {
        return;
    }

    __HAL_SPI_DISABLE(self->handle); // 必须先关闭 SPI 外设才能修改控制寄存器 CR1
    
    // 修改 CR1 寄存器中的 BR[2:0] 位（波特率控制位）
    // MODIFY_REG 宏的作用：先清除旧的 BR 位，再写入新的分频值
    MODIFY_REG(self->handle->Instance->CR1, SPI_CR1_BR, baudrate_prescaler);
    
    __HAL_SPI_ENABLE(self->handle);  // 重新使能 SPI 外设
}

/**
 * @brief  SPI1 读写一个字节
 * @details SPI 是同步通信，发送一个字节的同时必然会收到一个字节
 * @param  TxData: 要发送的数据字节
 * @retval 接收到的数据字节；若通信超时或失败，则返回 0xFF
 */
static uint8_t TftSpiBus_Transfer(TftSpiBus *self, uint8_t tx_data)
{
    uint8_t rx_data = 0xFF; // 定义接收缓存，默认初始化为 0xFF

    if ((self == NULL) || (self->handle == NULL))
    {
        return rx_data;
    }

    // 调用 HAL 库的全双工阻塞式读写函数，设置超时时间为最大值 (HAL_MAX_DELAY)
    if (HAL_SPI_TransmitReceive(self->handle, &tx_data, &rx_data, 1, HAL_MAX_DELAY) != HAL_OK)
    {
        return 0xFF; // 如果通信失败，返回 0xFF
    }

    return rx_data; // 返回成功接收到的数据
}

void SPI1_Init(void)
{
    TftSpiBus_Construct(&g_tft_spi1_bus);
    g_tft_spi1_bus.ops->init(&g_tft_spi1_bus);
}

void SPI1_SetSpeed(uint8_t SPI_BaudRatePrescaler)
{
    TftSpiBus_Construct(&g_tft_spi1_bus);
    g_tft_spi1_bus.ops->set_speed(&g_tft_spi1_bus, SPI_BaudRatePrescaler);
}

uint8_t SPI1_ReadWriteByte(uint8_t TxData)
{
    TftSpiBus_Construct(&g_tft_spi1_bus);
    return g_tft_spi1_bus.ops->transfer(&g_tft_spi1_bus, TxData);
}
