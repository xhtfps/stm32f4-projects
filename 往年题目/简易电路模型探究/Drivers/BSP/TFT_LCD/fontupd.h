/**
 * @file Drivers/TFT_LCD/fontupd.h
 * @brief 外部字库存储服务初始化与元信息读取模块。
 * @details
 * - 属于显示子系统，分别负责 LCD、字库 Flash、字体数据或文字渲染中的明确职责。
 * - 模块间通过对象接口或兼容 API 协作，避免 UI 层直接操作显示控制器寄存器。
 * - 字体/字模数据文件以数组尺寸、字号和索引关系一致性为维护重点。
 * - 本文件已移除原子/ST 示例模板，统一使用项目自定义 UTF-8 文件模板。
 * @encoding UTF-8
 */

#ifndef __FONTUPD_H__
#define __FONTUPD_H__
#include "stm32f4xx_hal.h"

typedef struct W25q64Device W25q64Device;

#ifndef __TFT_LCD_LEGACY_TYPES
#define __TFT_LCD_LEGACY_TYPES
typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;
#endif

/* 字库信息结构体，用来保存字库起始地址和大小。 */
__packed typedef struct
{
	u8 fontok;				//字库存在标志，0XAA，字库正常；其他，字库不存在
	u32 ugbkaddr; 			//unigbk的地址
	u32 ugbksize;			//unigbk的大小
	u32 f12addr;			//gbk12地址
	u32 gbk12size;			//gbk12的大小
	u32 f16addr;			//gbk16地址
	u32 gbk16size;			//gbk16的大小
	u32 f24addr;			//gbk24地址
	u32 gkb24size;			//gbk24的大小
} _font_info;

typedef struct FontStorageService FontStorageService;

/**
 * @brief 外部字库存储服务操作表。
 */
typedef struct
{
	u8 (*init)(FontStorageService *self); /**< 初始化 Flash 字库并读取字库元信息。 */
} FontStorageServiceOps;

/**
 * @brief 外部字库存储服务对象。
 * @details 将字库元信息地址、重试次数和 W25Q64 依赖收敛在一起，渲染层只读取 ftinfo 结果。
 */
struct FontStorageService
{
	W25q64Device *flash;              /**< 存放字库的 SPI Flash 设备。 */
	_font_info *info;                 /**< 字库元信息输出结构。 */
	uint32_t info_addr;               /**< 字库元信息在 Flash 中的固定地址。 */
	uint16_t retry_count;             /**< 初始化读取/校验重试次数。 */
	const FontStorageServiceOps *ops; /**< 字库存储服务方法表。 */
};

extern FontStorageService g_font_storage;

extern _font_info ftinfo;	//字库信息结构体

u32 fupd_prog(u16 x,u16 y,u8 size,u32 fsize,u32 pos);	//显示更新进度
u8 updata_fontx(u16 x,u16 y,u8 size,u8 *fxpath,u8 fx);	//更新指定字库
u8 update_font(u16 x,u16 y,u8 size,u8* src);			//更新全部字库
u8 font_init(void);										//初始化字库
#endif





















