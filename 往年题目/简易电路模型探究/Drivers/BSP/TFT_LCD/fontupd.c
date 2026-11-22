/**
 * @file Drivers/TFT_LCD/fontupd.c
 * @brief 外部字库存储服务初始化与元信息读取模块。
 * @details
 * - 属于显示子系统，分别负责 LCD、字库 Flash、字体数据或文字渲染中的明确职责。
 * - 模块间通过对象接口或兼容 API 协作，避免 UI 层直接操作显示控制器寄存器。
 * - 字体/字模数据文件以数组尺寸、字号和索引关系一致性为维护重点。
 * - 本文件已移除原子/ST 示例模板，统一使用项目自定义 UTF-8 文件模板。
 * @encoding UTF-8
 */

#include "fontupd.h"
#include "w25q64.h"
#include "tft_lcd.h"
#include "string.h"

// 字库区域占用的总扇区数大小。
#define FONTSECSIZE	 	791
// 字库存放起始地址。
#define FONTINFOADDR 	1024*1024*12 					//STM32F4是从12M地址以后开始存放字库

// 用来保存字库基本信息。
_font_info ftinfo;

static void FontStorageService_Construct(FontStorageService *self);
static u8 FontStorageService_Init(FontStorageService *self);

static const FontStorageServiceOps s_font_storage_ops =
{
	FontStorageService_Init,
};

FontStorageService g_font_storage;

static void FontStorageService_Construct(FontStorageService *self)
{
	if ((self != NULL) && (self->ops == NULL))
	{
		self->flash = &g_w25q64_device;
		self->info = &ftinfo;
		self->info_addr = FONTINFOADDR;
		self->retry_count = 10U;
		self->ops = &s_font_storage_ops;
	}
}


// 初始化字体，返回 0 表示字库正常。
static u8 FontStorageService_Init(FontStorageService *self)
{
	u16 t=0;

	if ((self == NULL) || (self->info == NULL))
	{
		return 1;
	}

	W25Q64_Init();
	self->flash = &g_w25q64_device;
	while(t<self->retry_count)//连续读取10次,都是错误,说明确实是有问题,得更新字库了
	{
		t++;
		W25Q64_Read((u8*)self->info, self->info_addr, (u16)sizeof(*self->info));//读出ftinfo结构体数据
		if(self->info->fontok==0XAA)
			break;
		HAL_Delay(20);
	}
	if(self->info->fontok!=0XAA)
		return 1;
	return 0;
}

u8 font_init(void)
{
	FontStorageService_Construct(&g_font_storage);
	return g_font_storage.ops->init(&g_font_storage);
}





























