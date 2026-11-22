/**
 * @file Drivers/TFT_LCD/W25Q64.h
 * @brief W25Q64 SPI Flash 设备对象驱动模块。
 * @details
 * - 属于显示子系统，分别负责 LCD、字库 Flash、字体数据或文字渲染中的明确职责。
 * - 模块间通过对象接口或兼容 API 协作，避免 UI 层直接操作显示控制器寄存器。
 * - 字体/字模数据文件以数组尺寸、字号和索引关系一致性为维护重点。
 * - 本文件已移除原子/ST 示例模板，统一使用项目自定义 UTF-8 文件模板。
 * @encoding UTF-8
 */

#ifndef __W25Q64_H
#define __W25Q64_H

#include "stm32f4xx_hal.h"

typedef struct TftSpiBus TftSpiBus;

/* ==================================================================================
 * 数据类型别名兼容（针对旧标准库驱动）
 * ==================================================================================
 * 作用：如果未定义过旧款数据类型，则在此处建立 stdint.h 标准类型到 u8/u16/u32 的映射，
 * 以确保旧代码和第三方字库/液晶屏驱动能无缝编译。
 */
#ifndef __TFT_LCD_LEGACY_TYPES
#define __TFT_LCD_LEGACY_TYPES
typedef uint8_t  u8;
typedef uint16_t u16;
typedef uint32_t u32;
#endif

/* ==================================================================================
 * 华邦 (Winbond) 串行 Flash 芯片 JEDEC ID 定义
 * ================================================================================== */
#define W25Q80  0XEF13 // W25Q80  ID ( 8M-bit  / 1M-Byte)
#define W25Q16  0XEF14 // W25Q16  ID (16M-bit  / 2M-Byte)
#define W25Q32  0XEF15 // W25Q32  ID (32M-bit  / 4M-Byte)
#define W25Q64  0XEF16 // W25Q64  ID (64M-bit  / 8M-Byte)
#define W25Q128 0XEF17 // W25Q128 ID (128M-bit / 16M-Byte)

/* ==================================================================================
 * W25Q64 硬件片选引脚 (CS) 及控制宏定义
 * ================================================================================== */
#define W25Q64_CS_GPIO_PORT      GPIOB
#define W25Q64_CS_GPIO_PIN       GPIO_PIN_9

// 片选拉高：取消选择 Flash 芯片，释放 SPI 总线
#define W25Q64_CS_H              HAL_GPIO_WritePin(W25Q64_CS_GPIO_PORT, W25Q64_CS_GPIO_PIN, GPIO_PIN_SET)
// 片选拉低：激活并选中 Flash 芯片，准备发起通信指令
#define W25Q64_CS_L              HAL_GPIO_WritePin(W25Q64_CS_GPIO_PORT, W25Q64_CS_GPIO_PIN, GPIO_PIN_RESET)

/* ==================================================================================
 * W25QXX 指令集定义 (标准 SPI 指令)
 * ================================================================================== */
#define W25X_WriteEnable         0x06 // 写使能锁置位
#define W25X_WriteDisable        0x04 // 写使能锁复位 (禁止写)
#define W25X_ReadStatusReg       0x05 // 读取状态寄存器-1
#define W25X_WriteStatusReg      0x01 // 写入状态寄存器-1
#define W25X_ReadData            0x03 // 读取常规数据
#define W25X_FastReadData        0x0B // 快速读取数据 (需要更高的 SPI 频率)
#define W25X_FastReadDual        0x3B // 双路快速读取数据 (Dual SPI)
#define W25X_PageProgram         0x02 // 页编程 (最大单次写 256 字节)
#define W25X_BlockErase          0xD8 // 块擦除 (通常为 64KB 大小)
#define W25X_SectorErase         0x20 // 扇区擦除 (通常为 4KB 大小)
#define W25X_ChipErase           0xC7 // 芯片全擦除 (全片回 0xFF)
#define W25X_PowerDown           0xB9 // 进入低功耗休眠模式
#define W25X_ReleasePowerDown    0xAB // 释放休眠模式并读取设备旧 ID
#define W25X_DeviceID            0xAB // 读取芯片旧版本设备 ID
#define W25X_ManufactDeviceID    0x90 // 读取制造商与设备复合 ID
#define W25X_JedecDeviceID       0x9F // 读取符合 JEDEC 标准的 24 位系统 ID

/* ==================================================================================
 * 外部导出函数原型声明 (API 接口)
 * ================================================================================== */

typedef struct W25q64Device W25q64Device;

/**
 * @brief W25Q64 Flash 设备操作表。
 * @details 把 SPI 指令、写使能、忙等待、擦除和读写流程封装成设备方法。
 */
typedef struct
{
    void (*init)(W25q64Device *self);                                                            /**< 初始化 CS 引脚、SPI 总线并读取芯片型号。 */
    uint16_t (*read_id)(W25q64Device *self);                                                      /**< 读取 JEDEC 制造商/容量复合 ID。 */
    uint8_t (*read_status)(W25q64Device *self);                                                   /**< 读取状态寄存器，bit0 为忙标志。 */
    void (*write_status)(W25q64Device *self, uint8_t status);                                     /**< 写状态寄存器。 */
    void (*write_enable)(W25q64Device *self);                                                     /**< 置位写使能锁存器。 */
    void (*write_disable)(W25q64Device *self);                                                    /**< 清除写使能锁存器。 */
    void (*read)(W25q64Device *self, uint8_t *buffer, uint32_t read_addr, uint16_t length);        /**< 从任意地址连续读取数据。 */
    void (*write_no_check)(W25q64Device *self, uint8_t *buffer, uint32_t write_addr, uint16_t length); /**< 在已擦除区域执行跨页写入。 */
    void (*write)(W25q64Device *self, uint8_t *buffer, uint32_t write_addr, uint16_t length);      /**< 自动处理扇区擦除/合并的高层写入。 */
    void (*erase_chip)(W25q64Device *self);                                                       /**< 擦除整片 Flash。 */
    void (*erase_sector)(W25q64Device *self, uint32_t sector_index);                              /**< 擦除一个 4KB 扇区。 */
    void (*wait_busy)(W25q64Device *self);                                                        /**< 阻塞等待芯片退出忙状态。 */
    void (*power_down)(W25q64Device *self);                                                       /**< 进入深度掉电模式。 */
    void (*wakeup)(W25q64Device *self);                                                           /**< 从深度掉电模式唤醒。 */
} W25q64DeviceOps;

/**
 * @brief W25Q64 Flash 设备对象。
 * @note sector_buffer 由实现文件提供静态缓冲，用于读改写一个 4KB 扇区。
 */
struct W25q64Device
{
    TftSpiBus *spi;              /**< 依赖注入的 SPI1 总线对象。 */
    uint16_t type;               /**< 读取到的 JEDEC 型号，例如 W25Q64。 */
    uint8_t *sector_buffer;      /**< 扇区读改写临时缓冲区。 */
    uint16_t page_size;          /**< 页编程粒度，W25Q64 通常为 256 字节。 */
    uint16_t sector_size;        /**< 扇区擦除粒度，W25Q64 通常为 4096 字节。 */
    const W25q64DeviceOps *ops;  /**< 对象方法表。 */
};

extern W25q64Device g_w25q64_device;
extern uint16_t W25Q64_TYPE;

/**
 * @brief  初始化 W25Q64 片选引脚，使能 SPI1 主机，并读取检测芯片 ID
 */
void W25Q64_Init(void);

/**
 * @brief  读取 W25Q64 的制造商和芯片设备物理 ID
 * @retval 16位复合ID（例如华邦 W25Q64 正常返回：0xEF16）
 */
uint16_t W25Q64_ReadID(void);                  

/**
 * @brief  读取 W25Q64 的状态寄存器-1
 * @retval 状态寄存器的字节数据。最低位 bit0 为 BUSY 标志
 */
uint8_t W25Q64_ReadSR(void);               

/**
 * @brief  向状态寄存器-1 写入配置数据
 * @param  sr: 准备写入状态寄存器的配置字节
 */
void W25Q64_Write_SR(uint8_t sr);           

/**
 * @brief  向芯片发送写使能操作指令 (解锁内部擦写锁定)
 */
void W25Q64_Write_Enable(void);         

/**
 * @brief  向芯片发送写禁止指令 (锁定芯片保护数据)
 */
void W25Q64_Write_Disable(void);        

/**
 * @brief  不进行擦除检测的跨页连续写入函数
 * @note   调用前必须确保目标存储地址原本已经被擦除干净（即全为 0xFF）
 * @param  pBuffer: 待写入本地数据的指针
 * @param  WriteAddr: Flash 的绝对目标起始写入地址
 * @param  NumByteToWrite: 待写的总字节数
 */
void W25Q64_Write_NoCheck(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite);

/**
 * @brief  从 Flash 的指定物理地址开始，连续读取任意长度的数据
 * @param  pBuffer: 本地存储接收数据的缓冲区指针
 * @param  ReadAddr: Flash 内部的绝对起始读取地址
 * @param  NumByteToRead: 待读取的总字节数
 */
void W25Q64_Read(uint8_t* pBuffer, uint32_t ReadAddr, uint16_t NumByteToRead);   

/**
 * @brief  高层万能写函数（自带擦除逻辑和跨页/跨扇区计算）
 * @details 如果写入点有旧数据，会自动搬移整个 4KB 扇区到 RAM，执行擦除后再合并重写。
 * @param  pBuffer: 本地待写入数据的指针
 * @param  WriteAddr: Flash 内部的绝对目标物理地址
 * @param  NumByteToWrite: 待写入的总字节数
 */
void W25Q64_Write(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite);

/**
 * @brief  擦除整颗 Flash 芯片的所有物理空间
 * @note   非常耗时，期间应尽量避免断电
 */
void W25Q64_Erase_Chip(void);           

/**
 * @brief  按照扇区逻辑序号擦除一个 4KB 大小的物理扇区
 * @param  Dst_Addr: 目标物理扇区序号 (0 ~ 2047)
 */
void W25Q64_Erase_Sector(uint32_t Dst_Addr);    

/**
 * @brief  死循环阻塞检测，直到 Flash 芯片退出 BUSY（忙）状态
 */
void W25Q64_Wait_Busy(void);            

/**
 * @brief  使芯片进入深度待机睡眠（超低功耗模式）
 */
void W25Q64_PowerDown(void);            

/**
 * @brief  唤醒处于睡眠模式中的 Flash 芯片，使其恢复正常工作
 */
void W25Q64_WAKEUP(void);               

#endif /* __W25Q64_H */
