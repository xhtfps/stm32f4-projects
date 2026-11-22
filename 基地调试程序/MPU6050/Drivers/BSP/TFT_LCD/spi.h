/**
 * @file Drivers/TFT_LCD/spi.h
 * @brief TFT 字库 Flash 使用的 SPI1 总线对象模块。
 * @details
 * - 属于显示子系统，分别负责 LCD、字库 Flash、字体数据或文字渲染中的明确职责。
 * - 模块间通过对象接口或兼容 API 协作，避免 UI 层直接操作显示控制器寄存器。
 * - 字体/字模数据文件以数组尺寸、字号和索引关系一致性为维护重点。
 * - 本文件已移除原子/ST 示例模板，统一使用项目自定义 UTF-8 文件模板。
 * @encoding UTF-8
 */

#ifndef __SPI_H
#define __SPI_H

#include "stm32f4xx_hal.h" // 包含 STM32F4 系列的 HAL 库核心头文件

/* * ==================================================================================
 * SPI1 分频兼容宏
 * ==================================================================================
 * 目的：将 HAL 库的原生分频宏映射为特定命名的宏。
 * 作用：供旧版 W25Q64 等外设驱动直接调用，在不破坏历史项目接口的同时，无缝适配 HAL 库。
 */
#define SPI1_BAUDRATEPRESCALER_2    SPI_BAUDRATEPRESCALER_2   // 2 分频
#define SPI1_BAUDRATEPRESCALER_4    SPI_BAUDRATEPRESCALER_4   // 4 分频
#define SPI1_BAUDRATEPRESCALER_8    SPI_BAUDRATEPRESCALER_8   // 8 分频
#define SPI1_BAUDRATEPRESCALER_16   SPI_BAUDRATEPRESCALER_16  // 16 分频
#define SPI1_BAUDRATEPRESCALER_32   SPI_BAUDRATEPRESCALER_32  // 32 分频
#define SPI1_BAUDRATEPRESCALER_64   SPI_BAUDRATEPRESCALER_64  // 64 分频
#define SPI1_BAUDRATEPRESCALER_128  SPI_BAUDRATEPRESCALER_128 // 128 分频
#define SPI1_BAUDRATEPRESCALER_256  SPI_BAUDRATEPRESCALER_256 // 256 分频

/* ==================================================================================
 * SPI1 总线对象接口
 * ================================================================================== */

typedef struct TftSpiBus TftSpiBus;

/**
 * @brief TFT 字库 Flash 使用的 SPI 总线操作表。
 */
typedef struct
{
    void (*init)(TftSpiBus *self);                              /**< 初始化 SPI 外设和 GPIO 复用。 */
    void (*set_speed)(TftSpiBus *self, uint32_t baudrate_prescaler); /**< 修改 SPI 波特率分频。 */
    uint8_t (*transfer)(TftSpiBus *self, uint8_t tx_data);       /**< 全双工收发一个字节。 */
} TftSpiBusOps;

/**
 * @brief SPI1 总线对象。
 */
struct TftSpiBus
{
    SPI_HandleTypeDef *handle; /**< HAL SPI 句柄。 */
    SPI_TypeDef *instance;     /**< SPI 外设实例。 */
    uint32_t default_prescaler; /**< 默认波特率分频值。 */
    const TftSpiBusOps *ops;   /**< SPI 总线方法表。 */
};

extern TftSpiBus g_tft_spi1_bus;

/* ==================================================================================
 * 外部全局变量声明
 * ================================================================================== */
// 声明在 spi.c 中定义的 SPI1 句柄，允许其他源文件（如 main.c 或 w25q64.c）引用或检查 SPI 状态
extern SPI_HandleTypeDef g_spi1_handler;

/* ==================================================================================
 * 导出函数原型声明 (API 接口)
 * ================================================================================== */

/**
 * @brief  初始化 SPI1 外设及其关联的 GPIO 引脚
 * @note   配置为主机全双工模式，模式 3 (CPOL=1, CPHA=2)，初始默认最慢通信速率 (256分频)
 * @retval 无
 */
void SPI1_Init(void);

/**
 * @brief  动态修改 SPI1 的通信波特率分频系数
 * @param  SPI_BaudRatePrescaler: 传入上面定义的兼容宏（如 SPI1_BAUDRATEPRESCALER_4）
 * @retval 无
 */
void SPI1_SetSpeed(uint8_t SPI_BaudRatePrescaler);

/**
 * @brief  SPI1 读写一个字节（全双工同步传输）
 * @param  TxData: 要发送的数据字节
 * @retval 接收到的数据字节；若传输失败则返回 0xFF
 */
uint8_t SPI1_ReadWriteByte(uint8_t TxData);

#endif /* __SPI_H */
