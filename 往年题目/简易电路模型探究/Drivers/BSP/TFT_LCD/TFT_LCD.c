/**
 * @file Drivers/TFT_LCD/TFT_LCD.c
 * @brief TFT LCD 控制器驱动、基础绘图和显示状态对象模块。
 * @details
 * - 属于显示子系统，分别负责 LCD、字库 Flash、字体数据或文字渲染中的明确职责。
 * - 模块间通过对象接口或兼容 API 协作，避免 UI 层直接操作显示控制器寄存器。
 * - 字体/字模数据文件以数组尺寸、字号和索引关系一致性为维护重点。
 * - 本文件已移除原子/ST 示例模板，统一使用项目自定义 UTF-8 文件模板。
 * @encoding UTF-8
 */

#include "TFT_LCD.h"
#include "fonts.h"

/* 汉字库开关配置 */
#if _USE_HZK16==1
#elif _USE_HZK16==2
/* 外部Flash中16点阵汉字库起始地址 */
#define HZK_Address ((uint32_t)0X80BE804)
#endif

/**
 * @brief FSMC 16位并口LCD寄存器/显存结构体映射
 * @note FSMC Bank1 连续地址：低地址=寄存器，高地址=显存数据
 */
typedef struct
{
	__IO uint16_t LCD_REG;  /* LCD 寄存器地址 */
	__IO uint16_t LCD_RAM;  /* LCD 显存数据地址 */
} LCD_TypeDef;

/* FSMC Bank1 起始地址 0x60000000 */
#define LCD_BASE           ((uint32_t)(0x60000000))
#define LCD                ((LCD_TypeDef *) LCD_BASE)

/* 当前使用字体指针，默认加载系统默认字体 */
static sFONT *LCD_Currentfonts = &LCD_DEFAULT_FONT;

/* 全局画笔颜色：字体颜色、背景色 */
__IO uint16_t TextColor = 0x0000, BackColor = 0xFFFF;
/* FSMC(SRAM) 句柄，用于LCD并口通信 */
SRAM_HandleTypeDef g_lcd_sram_handler;

/* 函数前置声明：LCD设备对象私有方法 */
static void TftLcdDevice_Construct(TftLcdDevice *self);
static void TftLcdDevice_Init(TftLcdDevice *self);
static void TftLcdDevice_SetColors(TftLcdDevice *self, uint16_t text_color, uint16_t back_color);
static void TftLcdDevice_GetColors(TftLcdDevice *self, __IO uint16_t *text_color, __IO uint16_t *back_color);
static void TftLcdDevice_SetTextColor(TftLcdDevice *self, uint16_t color);
static void TftLcdDevice_SetBackColor(TftLcdDevice *self, uint16_t color);
static void TftLcdDevice_SetFont(TftLcdDevice *self, sFONT *fonts, uint8_t mode);
static sFONT *TftLcdDevice_GetFont(TftLcdDevice *self);
static void TftLcdDevice_Clear(TftLcdDevice *self, uint16_t color);
static void TftLcdDevice_DrawPoint(TftLcdDevice *self, uint16_t x, uint16_t y, uint16_t color);
static void TftLcdDevice_WriteReg(TftLcdDevice *self, uint16_t reg, uint16_t value);
static uint16_t TftLcdDevice_ReadReg(TftLcdDevice *self, uint16_t reg);

/**
 * @brief LCD设备操作函数接口表(面向对象抽象)
 * @note 统一接口层，隔离底层差异，上层仅调用接口不直接操作硬件
 */
static const TftLcdDeviceOps s_tft_lcd_ops =
{
	TftLcdDevice_Init,
	TftLcdDevice_SetColors,
	TftLcdDevice_GetColors,
	TftLcdDevice_SetTextColor,
	TftLcdDevice_SetBackColor,
	TftLcdDevice_SetFont,
	TftLcdDevice_GetFont,
	TftLcdDevice_Clear,
	TftLcdDevice_DrawPoint,
	TftLcdDevice_WriteReg,
	TftLcdDevice_ReadReg,
};

/* 全局LCD设备实例 */
TftLcdDevice g_tft_lcd;

/* 底层内联接口：LCD寄存器/显存基础操作 */
__inline void LCD_WriteRAM_Prepare(void);
__inline void LCD_REG_Select(uint16_t REG_Code);
__inline void LCD_WriteRAM(uint16_t RGB_Code);

/**
 * @brief LCD设备构造函数，初始化设备成员与函数接口
 * @param self LCD设备对象指针
 * @note 仅首次调用时完成初始化，防止重复赋值
 */
static void TftLcdDevice_Construct(TftLcdDevice *self)
{
	if (self->ops == NULL)
	{
		self->sram_handle = &g_lcd_sram_handler;  /* 绑定FSMC句柄 */
		self->font = LCD_Currentfonts;             /* 绑定当前字体 */
		self->text_color = TextColor;              /* 同步字体颜色 */
		self->back_color = BackColor;              /* 同步背景颜色 */
		self->ops = &s_tft_lcd_ops;                /* 绑定操作接口表 */
	}
}

/**
 * @brief LCD硬件初始化
 * @param self LCD设备对象指针
 * @note 流程：GPIO配置 → FSMC配置 → 延时 → 屏控制器寄存器初始化
 *       多型号LCD通过 __TFT_ID 宏区分不同初始化序列
 */
static void TftLcdDevice_Init(TftLcdDevice *self)
{
	(void)self;
	/* 1. 配置LCD控制、数据GPIO引脚 */
	LCD_CtrlLinesConfig();
	/* 2. 配置FSMC并口时序与模式 */
	LCD_FSMCConfig();
	HAL_Delay(50); 

	/* 不同驱动IC的LCD初始化寄存器序列（时序/电压/Gamma/扫描方向等） */
#if __TFT_ID==0xB505
	/* B505 屏初始化序列 */
	LCD_WriteReg(LCD_REG_229,0x8000); 
	LCD_WriteReg(LCD_REG_0,  0x0001); 
	LCD_WriteReg(LCD_REG_1,  0x0000);   
	LCD_WriteReg(LCD_REG_2,  0x0700); 
	LCD_WriteReg(LCD_REG_3,  0x1018); 
	LCD_WriteReg(LCD_REG_4,  0x0000); 
	LCD_WriteReg(LCD_REG_8,  0x0202); 
	LCD_WriteReg(LCD_REG_9,  0x0000); 
	LCD_WriteReg(LCD_REG_10, 0x0000); 
	LCD_WriteReg(LCD_REG_12, 0x0000); 
	LCD_WriteReg(LCD_REG_13, 0x0000); 
	LCD_WriteReg(LCD_REG_15, 0x0000); 
	
	LCD_WriteReg(LCD_REG_16, 0x0000); 
	LCD_WriteReg(LCD_REG_17, 0x0000); 
	LCD_WriteReg(LCD_REG_18, 0x0000); 
	LCD_WriteReg(LCD_REG_19, 0x0000); 
	HAL_Delay(100);                 
	LCD_WriteReg(LCD_REG_16, 0x17B0); 
	LCD_WriteReg(LCD_REG_17, 0x0137); 
	HAL_Delay(10);                  
	LCD_WriteReg(LCD_REG_18, 0x0139); 
	HAL_Delay(10);                  
	LCD_WriteReg(LCD_REG_19, 0x1d00); 
	LCD_WriteReg(LCD_REG_41, 0x0013); 
	HAL_Delay(10);                  
	LCD_WriteReg(LCD_REG_32, 0x0000); 
	LCD_WriteReg(LCD_REG_33, 0x0000); 
	
	LCD_WriteReg(LCD_REG_48, 0x0006);
	LCD_WriteReg(LCD_REG_49, 0x0101);
	LCD_WriteReg(LCD_REG_50, 0x0003);
	LCD_WriteReg(LCD_REG_53, 0x0106);
	LCD_WriteReg(LCD_REG_54, 0x0b02);
	LCD_WriteReg(LCD_REG_55, 0x0302);
	LCD_WriteReg(LCD_REG_56, 0x0707);
	LCD_WriteReg(LCD_REG_57, 0x0007);
	LCD_WriteReg(LCD_REG_60, 0x0600);
	LCD_WriteReg(LCD_REG_61, 0x020b);
	
	LCD_WriteReg(LCD_REG_80, 0x0000); 
	LCD_WriteReg(LCD_REG_81, 0x00EF); 
	LCD_WriteReg(LCD_REG_82, 0x0000); 
	LCD_WriteReg(LCD_REG_83, 0x013F); 
	LCD_WriteReg(LCD_REG_96,  0x2700); 
	LCD_WriteReg(LCD_REG_97,  0x0001); 
	LCD_WriteReg(LCD_REG_106, 0x0000); 
	
	LCD_WriteReg(LCD_REG_128, 0x0000);
	LCD_WriteReg(LCD_REG_129, 0x0000);
	LCD_WriteReg(LCD_REG_130, 0x0000);
	LCD_WriteReg(LCD_REG_131, 0x0000);
	LCD_WriteReg(LCD_REG_132, 0x0000);
	LCD_WriteReg(LCD_REG_133, 0x0000);
	
	LCD_WriteReg(LCD_REG_144, 0x0010);
	LCD_WriteReg(LCD_REG_146, 0x0000);
	LCD_WriteReg(LCD_REG_147, 0x0003);
	LCD_WriteReg(LCD_REG_149, 0x0110);
	LCD_WriteReg(LCD_REG_151, 0x0000);
	LCD_WriteReg(LCD_REG_152, 0x0000);
	
	LCD_WriteReg(LCD_REG_3, 0x1000);	    
	LCD_WriteReg(LCD_REG_7, 0x0173); 
#elif __TFT_ID==0x4531
	/* 4531 屏初始化序列 */
	LCD_WriteReg(0X00,0X0001);
	HAL_Delay(10);
	LCD_WriteReg(0X10,0X1628);
	LCD_WriteReg(0X12,0X000e);
	LCD_WriteReg(0X13,0X0A39);
	HAL_Delay(10);
	LCD_WriteReg(0X11,0X0040);
	LCD_WriteReg(0X15,0X0050);
	HAL_Delay(10);
	LCD_WriteReg(0X12,0X001e);
	HAL_Delay(10);
	LCD_WriteReg(0X10,0X1620);
	LCD_WriteReg(0X13,0X2A39);
	HAL_Delay(10);
	LCD_WriteReg(0X01,0X0100);
	LCD_WriteReg(0X02,0X0300);
	LCD_WriteReg(0X03,0X1030);
	LCD_WriteReg(0X08,0X0202);
	LCD_WriteReg(0X0A,0X0008);
	LCD_WriteReg(0X30,0X0000);
	LCD_WriteReg(0X31,0X0402);
	LCD_WriteReg(0X32,0X0106);
	LCD_WriteReg(0X33,0X0503);
	LCD_WriteReg(0X34,0X0104);
	LCD_WriteReg(0X35,0X0301);
	LCD_WriteReg(0X36,0X0707);
	LCD_WriteReg(0X37,0X0305);
	LCD_WriteReg(0X38,0X0208);
	LCD_WriteReg(0X39,0X0F0B);
	LCD_WriteReg(0X41,0X0002);
	LCD_WriteReg(0X60,0X2700);
	LCD_WriteReg(0X61,0X0001);
	LCD_WriteReg(0X90,0X0210);
	LCD_WriteReg(0X92,0X010A);
	LCD_WriteReg(0X93,0X0004);
	LCD_WriteReg(0XA0,0X0100);
	LCD_WriteReg(0X07,0X0001);
	LCD_WriteReg(0X07,0X0021);
	LCD_WriteReg(0X07,0X0023);
	LCD_WriteReg(0X07,0X0033);
	LCD_WriteReg(0X07,0X0133);
	LCD_WriteReg(0XA0,0X0000);
#elif __TFT_ID==0x9325
	/* 9325 屏初始化序列 */
	LCD_WriteReg(0x00e7,0x0010);
	LCD_WriteReg(0x0000,0x0001);
	LCD_WriteReg(0x0001,0x0100);
	LCD_WriteReg(0x0002,0x0700);
	
	LCD_WriteReg(0x0003,(1<<12)|(3<<4)|(0<<3) );
	LCD_WriteReg(0x0004,0x0000);
	LCD_WriteReg(0x0008,0x0207);
	LCD_WriteReg(0x0009,0x0000);
	LCD_WriteReg(0x000a,0x0000);
	LCD_WriteReg(0x000c,0x0001);
	LCD_WriteReg(0x000d,0x0000);
	LCD_WriteReg(0x000f,0x0000);
	
	LCD_WriteReg(0x0010,0x0000);
	LCD_WriteReg(0x0011,0x0007);
	LCD_WriteReg(0x0012,0x0000);
	LCD_WriteReg(0x0013,0x0000);
	HAL_Delay(50);
	LCD_WriteReg(0x0010,0x1590);
	LCD_WriteReg(0x0011,0x0227);
	HAL_Delay(50);
	LCD_WriteReg(0x0012,0x009c);
	HAL_Delay(50);
	LCD_WriteReg(0x0013,0x1900);
	LCD_WriteReg(0x0029,0x0023);
	LCD_WriteReg(0x002b,0x000e);
	HAL_Delay(50);
	LCD_WriteReg(0x0020,0x0000);
	LCD_WriteReg(0x0021,0x013f);
	HAL_Delay(50);
	
	LCD_WriteReg(0x0030,0x0007);
	LCD_WriteReg(0x0031,0x0707);
	LCD_WriteReg(0x0032,0x0006);
	LCD_WriteReg(0x0035,0x0704);
	LCD_WriteReg(0x0036,0x1f04);
	LCD_WriteReg(0x0037,0x0004);
	LCD_WriteReg(0x0038,0x0000);
	LCD_WriteReg(0x0039,0x0706);
	LCD_WriteReg(0x003c,0x0701);
	LCD_WriteReg(0x003d,0x000f);
	HAL_Delay(50);
	LCD_WriteReg(0x0050,0x0000); 
	LCD_WriteReg(0x0051,0x00ef); 
	LCD_WriteReg(0x0052,0x0000); 
	LCD_WriteReg(0x0053,0x013f); 
	LCD_WriteReg(0x0060,0xa700);
	LCD_WriteReg(0x0061,0x0001);
	LCD_WriteReg(0x006a,0x0000);
	LCD_WriteReg(0x0080,0x0000);
	LCD_WriteReg(0x0081,0x0000);
	LCD_WriteReg(0x0082,0x0000);
	LCD_WriteReg(0x0083,0x0000);
	LCD_WriteReg(0x0084,0x0000);
	LCD_WriteReg(0x0085,0x0000);
	LCD_WriteReg(0x0090,0x0010);
	LCD_WriteReg(0x0092,0x0000);
	LCD_WriteReg(0x0093,0x0003);
	LCD_WriteReg(0x0095,0x0110);
	LCD_WriteReg(0x0097,0x0000);
	LCD_WriteReg(0x0098,0x0000);

	LCD_WriteReg(0x0007,0x0133);
	LCD_WriteReg(0x0020,0x0000);
	LCD_WriteReg(0x0021,0x013f);
#elif __TFT_ID==0x1936
	/* SSD1963(0x1936) 大屏控制器初始化序列 */
	LCD_REG_Select(0x00E2);	  
	LCD_WriteRAM(0x0023);	    
	LCD_WriteRAM(0x0002);
	LCD_WriteRAM(0x0004);
	LCD_REG_Select(0x00E0);  
	LCD_WriteRAM(0x0001);
	HAL_Delay(1);
	LCD_REG_Select(0x00E0);
	LCD_WriteRAM(0x0003);
	HAL_Delay(5);
	LCD_REG_Select(0x0001);  
	HAL_Delay(5);
	LCD_REG_Select(0x00E6);	
	LCD_WriteRAM(0x0003);
	LCD_WriteRAM(0x00ff);
	LCD_WriteRAM(0x00ff);
	
	LCD_REG_Select(0x00B0);	
	LCD_WriteRAM(0x0027);
	LCD_WriteRAM(0x0000);
	LCD_WriteRAM((SSD1963_HDP>>8)&0X00FF);  
	LCD_WriteRAM(SSD1963_HDP&0X00FF);
	LCD_WriteRAM((SSD1963_VDP>>8)&0X00FF);  
	LCD_WriteRAM(SSD1963_VDP&0X00FF);
	LCD_WriteRAM(0x0000);
	LCD_REG_Select(0x00B4);	
	LCD_WriteRAM((SSD1963_HT>>8)&0X00FF);  
	LCD_WriteRAM(SSD1963_HT&0X00FF);
	LCD_WriteRAM((SSD1963_HPS>>8)&0X00FF);  
	LCD_WriteRAM(SSD1963_HPS&0X00FF);
	LCD_WriteRAM(SSD1963_HPW);			   
	LCD_WriteRAM((SSD1963_LPS>>8)&0X00FF);  
	LCD_WriteRAM(SSD1963_LPS&0X00FF);
	LCD_WriteRAM(0x0000);
	LCD_REG_Select(0x00B6);	
	LCD_WriteRAM((SSD1963_VT>>8)&0X00FF);   
	LCD_WriteRAM(SSD1963_VT&0X00FF);
	LCD_WriteRAM((SSD1963_VPS>>8)&0X00FF);  
	LCD_WriteRAM(SSD1963_VPS&0X00FF);
	LCD_WriteRAM(SSD1963_VPW);			   
	LCD_WriteRAM((SSD1963_FPS>>8)&0X00FF);  
	LCD_WriteRAM(SSD1963_FPS&0X00FF);
	LCD_REG_Select(0x00BA);
	LCD_WriteRAM(0x000F);    
	LCD_REG_Select(0x00B8);
	LCD_WriteRAM(0x0007);    
	LCD_WriteRAM(0x0001);    
	LCD_REG_Select(0x0036); 
	LCD_WriteRAM(0x0000);
	LCD_REG_Select(0x00F0); 
	LCD_WriteRAM(0x0003);
	HAL_Delay(5);
	
	LCD_REG_Select(0xBC);
	LCD_WriteRAM(0x40);
	LCD_WriteRAM(0x80);
	LCD_WriteRAM(0x40);
	LCD_WriteRAM(0x01);
	HAL_Delay(10);
	LCD_REG_Select(0x0029); 
	
	LCD_REG_Select(0xBE); 
	LCD_WriteRAM(0x0E); 
	LCD_WriteRAM(0xFF); 
	LCD_WriteRAM(0x09); 
	LCD_WriteRAM(0xFF); 
	LCD_WriteRAM(0x00); 
	LCD_WriteRAM(0x00); 
	LCD_REG_Select(0xD4);
	
	LCD_WriteRAM(0x00); 
	LCD_WriteRAM(0x1C); 
	LCD_WriteRAM(0x20); 
	LCD_WriteRAM(0x00); 
	LCD_WriteRAM(0x46); 
	LCD_WriteRAM(0x50); 
	LCD_WriteRAM(0x00); 
	LCD_WriteRAM(0xA8); 
	LCD_WriteRAM(0xC0); 
	LCD_REG_Select(0x00d0);  
	LCD_WriteRAM(0x000d);
	/* 设置全屏显示窗口 */
	LCD_SetDisplayWindow(0,0,799,479);
#endif
	/* 同步设备内部状态 */
	self->sram_handle = &g_lcd_sram_handler;
	self->font = LCD_Currentfonts;
	self->text_color = TextColor;
	self->back_color = BackColor;
}

/**
 * @brief LCD对外初始化接口
 */
void TFT_LCD_Init(void)
{
	TftLcdDevice_Construct(&g_tft_lcd);  /* 构建设备 */
	g_tft_lcd.ops->init(&g_tft_lcd);     /* 执行硬件初始化 */
}

/**
 * @brief 同时设置字体颜色与背景色（内部接口）
 * @param self LCD设备对象
 * @param text_color 字体RGB565颜色
 * @param back_color 背景RGB565颜色
 */
static void TftLcdDevice_SetColors(TftLcdDevice *self, uint16_t text_color, uint16_t back_color)
{
	TextColor = text_color;
	BackColor = back_color;
	self->text_color = text_color;
	self->back_color = back_color;
}

/**
 * @brief 对外接口：同时设置字体与背景色
 * @param _TextColor 字体颜色
 * @param _BackColor 背景颜色
 */
void LCD_SetColors(__IO uint16_t _TextColor, __IO uint16_t _BackColor)
{
	TftLcdDevice_Construct(&g_tft_lcd);
	g_tft_lcd.ops->set_colors(&g_tft_lcd, _TextColor, _BackColor);
}

/**
 * @brief 获取当前画笔颜色（内部接口）
 * @param self LCD设备对象
 * @param text_color 输出：当前字体颜色
 * @param back_color 输出：当前背景颜色
 */
static void TftLcdDevice_GetColors(TftLcdDevice *self, __IO uint16_t *text_color, __IO uint16_t *back_color)
{
	self->text_color = TextColor;
	self->back_color = BackColor;
	*text_color = self->text_color;
	*back_color = self->back_color;
}

/**
 * @brief 对外接口：获取当前字体、背景颜色
 * @param _TextColor 颜色接收指针
 * @param _BackColor 颜色接收指针
 */
void LCD_GetColors(__IO uint16_t *_TextColor, __IO uint16_t *_BackColor)
{
	TftLcdDevice_Construct(&g_tft_lcd);
	g_tft_lcd.ops->get_colors(&g_tft_lcd, _TextColor, _BackColor);
}

/**
 * @brief 设置字体颜色（内部接口）
 * @param self LCD设备对象
 * @param color RGB565颜色值
 */
static void TftLcdDevice_SetTextColor(TftLcdDevice *self, uint16_t color)
{
	TextColor = color;
	self->text_color = color;
}

/**
 * @brief 对外接口：设置字体颜色
 * @param Color RGB565颜色值
 */
void LCD_SetTextColor(__IO uint16_t Color)
{
	TftLcdDevice_Construct(&g_tft_lcd);
	g_tft_lcd.ops->set_text_color(&g_tft_lcd, Color);
}

/**
 * @brief 设置背景颜色（内部接口）
 * @param self LCD设备对象
 * @param color RGB565颜色值
 */
static void TftLcdDevice_SetBackColor(TftLcdDevice *self, uint16_t color)
{
	BackColor = color;
	self->back_color = color;
}

/**
 * @brief 对外接口：设置背景颜色
 * @param Color RGB565颜色值
 */
void LCD_SetBackColor(__IO uint16_t Color)
{
	TftLcdDevice_Construct(&g_tft_lcd);
	g_tft_lcd.ops->set_back_color(&g_tft_lcd, Color);
}

/**
 * @brief 全屏清屏（内部接口）
 * @param self LCD设备对象
 * @param color 填充颜色
 */
static void TftLcdDevice_Clear(TftLcdDevice *self, uint16_t color)
{
	uint32_t index = 0;
	(void)self;
	LCD_SetCursor(0x00, 0x00);   /* 光标定位到原点 */
	LCD_WriteRAM_Prepare();       /* 准备连续写显存 */
	/* 逐点填充整屏像素 */
	for(index = 0; index < LCD_Pixel_Max; index++)
		LCD->LCD_RAM = color;
}

/**
 * @brief 对外接口：全屏清屏
 * @param Color 清屏填充色
 */
void LCD_Clear(uint16_t Color)
{
	TftLcdDevice_Construct(&g_tft_lcd);
	g_tft_lcd.ops->clear(&g_tft_lcd, Color);
}

/**
 * @brief 区域清屏：指定矩形区域填充颜色
 * @param Xs 起始X坐标
 * @param Ys 起始Y坐标
 * @param Xe 结束X坐标
 * @param Ye 结束Y坐标
 * @param Color 填充颜色
 */
void LCD_Appoint_Clear(uint16_t Xs,uint16_t Ys,uint16_t Xe,uint16_t Ye,uint16_t Color)
{
	uint32_t i,j;
	/* 遍历矩形区域所有像素 */
	for(i=0; i<Xe-Xs; i++)
	{
		for (j = 0; j < Ye-Ys; j++)
		{
			LCD_SetCursor(Xs+i,Ys+j);
			LCD_WriteRAM_Prepare();
			LCD->LCD_RAM = Color;
		}
	}
}

/**
 * @brief 设置当前使用字体与显示模式（内部接口）
 * @param self LCD设备对象
 * @param fonts 字体结构体指针
 * @param mode 显示模式(0:透明 1:覆盖背景)
 */
static void TftLcdDevice_SetFont(TftLcdDevice *self, sFONT *fonts, uint8_t mode)
{
	LCD_Currentfonts = fonts;
	LCD_Currentfonts->Mode = mode;
	self->font = LCD_Currentfonts;
}

/**
 * @brief 对外接口：切换字体与显示模式
 * @param fonts 字体指针
 * @param Mode 显示模式
 */
void LCD_SetFont(sFONT *fonts,uint8_t Mode)
{
	TftLcdDevice_Construct(&g_tft_lcd);
	g_tft_lcd.ops->set_font(&g_tft_lcd, fonts, Mode);
}

/**
 * @brief 获取当前字体指针（内部接口）
 * @param self LCD设备对象
 * @return 当前字体结构体指针
 */
static sFONT *TftLcdDevice_GetFont(TftLcdDevice *self)
{
	self->font = LCD_Currentfonts;
	return self->font;
}

/**
 * @brief 对外接口：获取当前字体
 * @return 当前字体指针
 */
sFONT *LCD_GetFont(void)
{
	TftLcdDevice_Construct(&g_tft_lcd);
	return g_tft_lcd.ops->get_font(&g_tft_lcd);
}

/**
 * @brief 显示单个ASCII字符
 * @param x 字符左上角X坐标
 * @param y 字符左上角Y坐标
 * @param Char 待显示ASCII字符
 */
void LCD_ShowChar(uint16_t x,uint16_t y,uint8_t Char)
{
	uint16_t temp,dy,dx,K;
	/* 过滤控制字符(0~31)，只显示可打印字符 */
	if(Char > 31)
		Char = Char - 32;
	else  
		Char = 0;

	/* 逐行扫描字模点阵 */
	for(dy=0,K=0; dy<LCD_Currentfonts->Height; dy++)
	{
		temp = LCD_Currentfonts->table[Char][K++];
#if   __FONTS==1
		/* 字体类型1渲染逻辑 */
		for(dx=0; dx<LCD_Currentfonts->Height/2; dx++)
		{
			if(temp & 0x01)
			{
				LCD_SetCursor(x+dx,y+dy);
				LCD_WriteRAM_Prepare();
				LCD_WriteRAM(TextColor);
			}
			else if(LCD_Currentfonts->Mode)
			{
				LCD_SetCursor(x+dx,y+dy);
				LCD_WriteRAM_Prepare();
				LCD_WriteRAM(BackColor);
			}
			temp >>= 1;
		}
#elif __FONTS==2
		/* 字体类型2渲染逻辑 */
		for(dx=0; dx<8; dx++)
		{
			if(temp & 0x01)
			{
				LCD_SetCursor(x+dx,y+dy);
				LCD_WriteRAM_Prepare();
				LCD_WriteRAM(TextColor);
			}
			else if(LCD_Currentfonts->Mode)
			{
				LCD_SetCursor(x+dx,y+dy);
				LCD_WriteRAM_Prepare();
				LCD_WriteRAM(BackColor);
			}
			temp >>= 1;
		}
		temp = LCD_Currentfonts->table[Char][K++];
		for(; dx<LCD_Currentfonts->Width; dx++)
		{
			if(temp & 0x01)
			{
				LCD_SetCursor(x+dx,y+dy);
				LCD_WriteRAM_Prepare();
				LCD_WriteRAM(TextColor);
			}
			else if(LCD_Currentfonts->Mode)
			{
				LCD_SetCursor(x+dx,y+dy);
				LCD_WriteRAM_Prepare();
				LCD_WriteRAM(BackColor);
			}
			temp >>= 1;
		}
#endif
	}
}

#if _USE_HZK16
/**
 * @brief 16点阵汉字显示底层函数
 * @param _Xpos 汉字起始X
 * @param _Ypos 汉字起始Y
 * @param c 汉字点阵数据缓冲区
 * @param _width 汉字宽度
 */
#if   __FONTS==1
static void TFT_DispChar(uint16_t _Xpos, uint16_t _Ypos, const uint8_t *c, uint8_t _width)
{
	uint32_t height = 0, i = 0;
	uint16_t X_add = _Xpos;
	uint16_t Y_add = _Ypos;
	LCD_SetCursor(_Xpos, _Ypos);

	/* 模式1：绘制背景+前景 */
	if (LCD_Currentfonts->Mode)
	{
		for (height = 0; height < 16; height++)  
		{
			X_add = _Xpos;
			for (i = 0; i < _width; i++)	  
			{
				LCD_SetCursor(X_add, Y_add);
				LCD_WriteRAM_Prepare();
				if ((c[height * (_width / 8) + i / 8] & (0x80 >> (i % 8))) == 0x00)
					LCD_WriteRAM(BackColor);
				else
					LCD_WriteRAM(TextColor);
				X_add++;
			}
			Y_add++;
		}
	}
	/* 模式0：仅绘制前景(透明背景) */
	else
	{
		for (height = 0; height < 16; height++)   
		{
			X_add = _Xpos;
			for (i = 0; i < _width; i++)	   
			{
				if ((c[height * (_width / 8) + i / 8] & (0x80 >> (i % 8))) != 0x00)
				{
					LCD_SetCursor(X_add, Y_add);
					LCD_WriteRAM_Prepare();
					LCD->LCD_RAM = TextColor;
				}
				X_add++;
			}
			Y_add++;
		}
	}
}
#elif __FONTS==2
static void TFT_DispChar(uint16_t _Xpos, uint16_t _Ypos, const uint8_t *c, uint8_t _width)
{
	uint32_t height = 0, i = 0;
	uint16_t X_add = _Xpos;
	uint16_t Y_add = _Ypos;
	LCD_SetCursor(_Xpos, _Ypos);

	if (LCD_Currentfonts->Mode)
	{
		for (height = 0; height < 16; height++)  
		{
			X_add = _Xpos;
			for (i = 0; i < _width; i++)	  
			{
				/* 2倍放大绘制 */
				if ((c[height * (_width / 8) + i / 8] & (0x80 >> (i % 8))) == 0x00)
				{
					LCD_SetCursor(X_add, Y_add);
					LCD_WriteRAM_Prepare();
					LCD->LCD_RAM = BackColor;
					LCD_SetCursor(X_add+1, Y_add);
					LCD_WriteRAM_Prepare();
					LCD->LCD_RAM = BackColor;
					LCD_SetCursor(X_add, Y_add+1);
					LCD_WriteRAM_Prepare();
					LCD->LCD_RAM = BackColor;
					LCD_SetCursor(X_add+1, Y_add+1);
					LCD_WriteRAM_Prepare();
					LCD->LCD_RAM = BackColor;
				}
				else
				{
					LCD_SetCursor(X_add, Y_add);
					LCD_WriteRAM_Prepare();
					LCD->LCD_RAM = TextColor;
					LCD_SetCursor(X_add+1, Y_add);
					LCD_WriteRAM_Prepare();
					LCD->LCD_RAM = TextColor;
					LCD_SetCursor(X_add, Y_add+1);
					LCD_WriteRAM_Prepare();
					LCD->LCD_RAM = TextColor;
					LCD_SetCursor(X_add+1, Y_add+1);
					LCD_WriteRAM_Prepare();
					LCD->LCD_RAM = TextColor;
				}
				X_add += 2;
			}
			Y_add += 2;
		}
	}
	else
	{
		for (height = 0; height < 16; height++)   
		{
			X_add = _Xpos;
			for (i = 0; i < _width; i++)	   
			{
				if ((c[height * (_width / 8) + i / 8] & (0x80 >> (i % 8))) != 0x00)
				{
					LCD_SetCursor(X_add, Y_add);
					LCD_WriteRAM_Prepare();
					LCD->LCD_RAM = TextColor;
					LCD_SetCursor(X_add+1, Y_add);
					LCD_WriteRAM_Prepare();
					LCD->LCD_RAM = TextColor;
					LCD_SetCursor(X_add, Y_add+1);
					LCD_WriteRAM_Prepare();
					LCD->LCD_RAM = TextColor;
					LCD_SetCursor(X_add+1, Y_add+1);
					LCD_WriteRAM_Prepare();
					LCD->LCD_RAM = TextColor;
				}
				X_add += 2;
			}
			Y_add += 2;
		}
	}
}
#endif
#endif

/**
 * @brief 字符串显示（支持ASCII + GB2312汉字自动识别）
 * @param Line 行坐标(Y)
 * @param Column 列坐标(X)
 * @param ptr 字符串指针
 * @return 0 正常结束
 * @note 自动换行、越界归零；汉字从外部Flash/SRAM读取16点阵字模
 */
uint32_t LCD_DisplayStringLine(uint16_t Line,uint16_t Column, uint8_t *ptr)
{
#if _USE_HZK16
	uint32_t address;
	uint16_t HZK16[16];  /* 汉字点阵缓存 */
#endif
#if _USE_HZK16==2
	uint32_t *HZK = (uint32_t*)HZK16;
#endif
	uint16_t refcolumn = Column;
	uint16_t refLine = Line;
#if _USE_HZK16
	uint8_t code1,code2;
#endif

	/* 行越界则回到首行 */
	if(refLine >= LCD_Pixel_Height)
	{
		refLine = 0;
		LCD_Currentfonts->Mode = 1;
	}

	/* 遍历整个字符串 */
	while (*ptr != '\0')
	{
#if _USE_HZK16
		code1 = *ptr;
		/* 单字节：ASCII字符 */
		if(code1 < 0x80)	
		{
#endif
			/* 列越界：自动换行 */
			if(refcolumn >= LCD_Pixel_Width)
			{
				refcolumn = 0;
				refLine += LCD_Currentfonts->Height;
				if(refLine >= LCD_Pixel_Height)
				{
					refLine = 0;
					LCD_Currentfonts->Mode = 1;
				}
			}
			LCD_ShowChar(refcolumn, refLine, *ptr);
			refcolumn += LCD_Currentfonts->Width;
#if  _USE_HZK16
		}
		/* 双字节：GB2312汉字 */
		else
		{
			code2 = *++ptr;
			if(code2 == '\0')
				return code1;
			
			/* GB2312 汉字库地址换算 */
			address = ((code1 - 0xa1) * 94 + (code2 - 0xa1)) << 5;
#if _USE_HZK16==1
			/* 从内部SRAM读取字模 */
			SRAM_ReadBuffer(HZK16, address, 16);
#elif _USE_HZK16==2
			/* 累加外部Flash基地址，读取字模 */
			address += HZK_Address;
			for(code1=0; code1<8; code1++)
			{
				HZK[code1] = (*(__IO uint32_t*)address);
				address += 4;
			}
#endif
			/* 汉字列越界换行判断 */
			if(refcolumn >= LCD_Pixel_Width - LCD_Currentfonts->Height)
			{
				refcolumn = 0;
				refLine += LCD_Currentfonts->Height;
				if(refLine >= LCD_Pixel_Height)
				{
					refLine = 0;
					LCD_Currentfonts->Mode = 1;
				}
			}
			TFT_DispChar(refcolumn, refLine, (const uint8_t *)HZK16, 16);
			refcolumn += LCD_Currentfonts->Width * 2;
		}
#endif
		ptr++;
	}
	return 0;
}

#if _USE_0X
/**
 * @brief 字符显示辅助函数（自动换行）
 * @param refLine 当前行指针
 * @param refcolumn 当前列指针
 * @param Char 待显示字符
 */
static void	Display_Control(uint8_t *refLine,uint16_t *refcolumn,uint8_t Char)
{
	if(*refcolumn >= LCD_Pixel_Width - LCD_Currentfonts->Width)
	{
		*refcolumn = 0;
		*refLine += LCD_Currentfonts->Height;
		if(*refLine >= LCD_Pixel_Height)
		{
			*refLine = 0;
			LCD_Currentfonts->Mode = 1;
		}
	}
	LCD_ShowChar(*refcolumn,*refLine,Char);
	*refcolumn += LCD_Currentfonts->Width;
}

/**
 * @brief 以十六进制形式显示字节数组
 * @param Line 起始行
 * @param Column 起始列
 * @param X 字节数组
 * @param N 数组长度
 */
void LCD_Display0x(uint16_t Line,uint16_t Column, uint8_t *X,uint8_t N)
{
	uint16_t refcolumn = Column;
	uint8_t  i=0,temp=0, refLine = Line;
	for(i=0; i<N; i++)
	{
		/* 显示高4位 */
		temp = X[i] / 16;
		if(temp < 10)
			temp += '0';
		else
			temp += 55;
		Display_Control(&refLine,&refcolumn,temp);

		/* 显示低4位 */
		temp = X[i] % 16;
		if(temp < 10)
			temp += '0';
		else
			temp += 55;
		Display_Control(&refLine,&refcolumn,temp);

		/* 加空格分隔 */
		Display_Control(&refLine,&refcolumn,' ');
	}
}
#endif

/**
 * @brief 显示十进制整数
 * @param Line 行坐标
 * @param Column 列坐标
 * @param num 待显示无符号整数
 */
void LCD_DisplayNum(uint16_t Line,uint16_t Column, uint32_t num)
{
	uint32_t temp = num;
	uint16_t refcolumn = Column;
	uint16_t  refLine = Line,len = 0;

	/* 计算数字位数 */
	do
	{
		temp = temp / 10;
		len++;
	}
	while(temp);
	len--;
	
	/* 从右往左逐位显示 */
	refcolumn += len * LCD_Currentfonts->Width;
	do
	{
		temp = num % 10;
		LCD_ShowChar(refcolumn,refLine,temp + '0');
		refcolumn -= LCD_Currentfonts->Width;
		num = num / 10;
	}
	while(len--);
}

#if _USE_Float
/**
 * @brief 显示浮点数（保留5位小数）
 * @param Line 行坐标
 * @param Column 列坐标
 * @param num 待显示浮点数值
 */
void LCD_Display_FloatNum(uint16_t Line,uint16_t Column, double num)
{
	uint64_t Temp;
	uint32_t Temp1;
	uint8_t len = 0;
	/* 负数处理 */
	if(num < 0)
	{
		num = -num;
		LCD_DisplayStringLine(Line,Column,"-");
		len++;
	}
	/* 放大100000倍转为整数处理 */
	Temp = (uint64_t)(num * 100000);
	/* 整数部分 */
	Temp1 = (uint32_t)(Temp / 100000);
	LCD_DisplayNum(Line,Column + Column(len),Temp1);

	/* 计算整数位长度 */
	do
	{
		Temp1 = Temp1 / 10;
		len++;
	}
	while(Temp1);

	/* 小数部分 */
	Temp1 = (uint32_t)(Temp % 100000);
	/* 去除末尾无效0 */
	while(Temp1 % 10 == 0)
		Temp1 = Temp1 / 10;

	/* 非零小数则显示小数点+小数 */
	if(Temp1 != 0)
	{
		LCD_DisplayStringLine(Line,Column + Column(len),".");
		len++;
		LCD_DisplayNum(Line,Column + Column(len),Temp1);
	}
}
#endif

#if _USE_GRAPH
/**
 * @brief 画单个像素点（使用当前画笔颜色）
 * @param x X坐标
 * @param y Y坐标
 */
void PutPixel(int16_t x, int16_t y)
{
	LCD_SetCursor(x,y);
	LCD_WriteRAM_Prepare();
	LCD_WriteRAM(TextColor);
}

/**
 * @brief 批量写入位图数据
 * @param ptr 位图数据缓冲区
 * @param SIZE 像素个数
 */
void LCD_WeBMP_SIZE(uint16_t *ptr,uint32_t SIZE)
{
	uint32_t index;
	LCD_WriteRAM_Prepare(); 
	for (index = 0; index < SIZE; index++)
		LCD_WriteRAM(*ptr++);
}

/**
 * @brief 绘制水平/垂直直线
 * @param Xpos 起点X
 * @param Ypos 起点Y
 * @param Length 线段长度
 * @param Direction 方向 0=水平 1=垂直
 */
void LCD_DrawLine(uint16_t Xpos, uint16_t Ypos, uint16_t Length, uint8_t Direction)
{
	uint32_t i = 0;
	LCD_SetCursor(Xpos, Ypos);
	if(Direction == 0)
	{
		/* 水平线 */
		LCD_WriteRAM_Prepare(); 
		for(i = 0; i < Length; i++)
			LCD_WriteRAM(TextColor);
	}
	else
	{
		/* 垂直线 */
		for(i = 0; i < Length; i++)
		{
			LCD_WriteRAM_Prepare(); 
			LCD_WriteRAM(TextColor);
			Ypos++;
			LCD_SetCursor(Xpos, Ypos);
		}
	}
}

/* 求绝对值宏 */
#define ABS(X)  ((X) > 0 ? (X) : -(X))

// 画线函数：Bresenham 算法绘制任意两点之间直线
void LCD_DrawuniLine(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
	// 坐标差值、当前坐标、步进增量、算法中间变量
	int16_t deltax = 0, deltay = 0, x = 0, y = 0, xinc1 = 0, xinc2 = 0,
	        yinc1 = 0, yinc2 = 0, den = 0, num = 0, numadd = 0, numpixels = 0,
	        curpixel = 0;

	deltax = ABS(x2 - x1);        // X轴坐标差值绝对值
	deltay = ABS(y2 - y1);        // Y轴坐标差值绝对值
	x = x1;                       // 起点X坐标
	y = y1;                       // 起点Y坐标

	// 判断X方向步进方向：右移+1 / 左移-1
	if (x2 >= x1)                 
	{
		xinc1 = 1;
		xinc2 = 1;
	}
	else                          
	{
		xinc1 = -1;
		xinc2 = -1;
	}

	// 判断Y方向步进方向：下移+1 / 上移-1
	if (y2 >= y1)                 
	{
		yinc1 = 1;
		yinc2 = 1;
	}
	else                          
	{
		yinc1 = -1;
		yinc2 = -1;
	}

	// 斜率绝对值 ≤1：以X为主轴步进
	if (deltax >= deltay)         
	{
		xinc1 = 0;                  // 斜向步进X增量置0
		yinc2 = 0;                  // 主步进Y增量置0
		den = deltax;               // 分母 = X差值
		num = deltax / 2;           // 误差项初始值
		numadd = deltay;            // 误差累加步长
		numpixels = deltax;         // 总像素点数
	}
	// 斜率绝对值 >1：以Y为主轴步进
	else                          
	{
		xinc2 = 0;                  // 主步进X增量置0
		yinc1 = 0;                  // 斜向步进Y增量置0
		den = deltay;               // 分母 = Y差值
		num = deltay / 2;           // 误差项初始值
		numadd = deltax;            // 误差累加步长
		numpixels = deltay;         // 总像素点数
	}

	// 逐像素打点绘制直线
	for (curpixel = 0; curpixel <= numpixels; curpixel++)
	{
		PutPixel(x, y);              // 在当前坐标画点
		num += numadd;               // 累加误差值
		// 误差溢出，需要斜向走一步
		if (num >= den)              
		{
			num -= den;               // 误差回滚
			x += xinc1;               // X斜向步进
			y += yinc1;               // Y斜向步进
		}
		x += xinc2;                   // 主方向X步进
		y += yinc2;                   // 主方向Y步进
	}
}

/**
 * @brief  绘制空心矩形（简易版，参数定义不标准）
 * @param  Xpos,Ypos: 矩形左上角起点坐标
 * @param  Height: 矩形高度
 * @param  Width: 矩形宽度
 */
void LCD_DrawRect(uint16_t Xpos, uint16_t Ypos, uint16_t Height, uint16_t Width)
{
	// 四条边分别调用画线接口
	LCD_DrawLine(Xpos,      Ypos,       Width,0);
	LCD_DrawLine(Xpos,      Ypos+Height,Width,0);
	LCD_DrawLine(Xpos,      Ypos,       Height,1);
	LCD_DrawLine(Xpos+Width,Ypos,       Height,1);
}

/**
 * @brief  绘制实心矩形（带颜色）
 * @param  x1,y1: 矩形左上角
 * @param  x2,y2: 矩形右下角
 * @param  color: 填充颜色
 */
void LCD_DrawRectS(u16 x1,u16 y1,u16 x2,u16 y2,u16 color)
{
	u16 temp=TextColor;        // 暂存原有画笔颜色
	uint32_t x;

	if ((x2 < x1) || (y2 < y1))
	{
		return;
	}

	TextColor=color;           // 设置为目标填充颜色
	// 使用更宽的循环变量，避免x1为0时uint16_t递减下溢而无法结束。
	for (x = x1; x <= x2; x++)
		LCD_DrawuniLine((u16)x,y1,(u16)x,y2);
	TextColor=temp;            // 恢复原有画笔颜色
}

/**
 * @brief  绘制空心圆（Bresenham 圆算法，八分圆对称打点）
 * @param  Xpos,Ypos: 圆心坐标
 * @param  Radius: 圆半径
 */
void LCD_DrawCircle(uint16_t Xpos, uint16_t Ypos, uint16_t Radius)
{
	int32_t  D;    // 判别式
	uint32_t  CurX;// 当前X偏移
	uint32_t  CurY;// 当前Y偏移

	D = 3 - (Radius << 1);  // 初始化圆判别式
	CurX = 0;
	CurY = Radius;

	// 遍历1/4圆，利用八分对称性一次性打点8个对称点
	while (CurX <= CurY)
	{
		// 八个对称像素点依次设置光标并写入颜色
		LCD_SetCursor(Xpos + CurX, Ypos + CurY);
		LCD_WriteRAM_Prepare(); 
		LCD_WriteRAM(TextColor);

		LCD_SetCursor(Xpos + CurX, Ypos - CurY);
		LCD_WriteRAM_Prepare(); 
		LCD_WriteRAM(TextColor);

		LCD_SetCursor(Xpos - CurX, Ypos + CurY);
		LCD_WriteRAM_Prepare(); 
		LCD_WriteRAM(TextColor);

		LCD_SetCursor(Xpos - CurX, Ypos - CurY);
		LCD_WriteRAM_Prepare(); 
		LCD_WriteRAM(TextColor);

		LCD_SetCursor(Xpos + CurY, Ypos + CurX);
		LCD_WriteRAM_Prepare(); 
		LCD_WriteRAM(TextColor);

		LCD_SetCursor(Xpos + CurY, Ypos - CurX);
		LCD_WriteRAM_Prepare(); 
		LCD_WriteRAM(TextColor);

		LCD_SetCursor(Xpos - CurY, Ypos + CurX);
		LCD_WriteRAM_Prepare(); 
		LCD_WriteRAM(TextColor);

		LCD_SetCursor(Xpos - CurY, Ypos - CurX);
		LCD_WriteRAM_Prepare(); 
		LCD_WriteRAM(TextColor);

		// 更新判别式与坐标
		if (D < 0)
			D += (CurX << 2) + 6;
		else
		{
			D += ((CurX - CurY) << 2) + 10;
			CurY--;
		}
		CurX++;
	}
}

/**
 * @brief  绘制实心圆（多层圆环叠加）
 * @param  x,y: 圆心
 * @param  size: 外圆半径
 * @param  color: 填充颜色
 */
void LCD_DrawCircleS(uint16_t x,uint16_t y,uint16_t size,uint16_t color)
{
	uint16_t temp=TextColor;
	TextColor =color;
	// 由外向内逐层画圆，叠加成实心效果
	for(; size>0; size--)
		LCD_DrawCircle(x,y,size);
	TextColor=temp;
}

#endif

// ====================== 驱动分支：屏ID = 0x1936 专用接口 ======================
#if __TFT_ID==0x1936
/**
 * @brief  绘制4倍放大点（2x2像素块）
 * @param  X,Y: 基准坐标
 * @param  Col: 颜色值
 */
void LCD_DrawPoint_4(uint16_t X,uint16_t Y,uint16_t Col)
{
	// 设置窗口列地址
	LCD_REG_Select(0x002A);
	LCD_WriteRAM(X>>8);
	LCD_WriteRAM(X&0x00ff);
	LCD_WriteRAM(0x03);
	LCD_WriteRAM(0x1F);
	// 设置窗口行地址
	LCD_REG_Select(0x002b);
	LCD_WriteRAM(Y>>8);
	LCD_WriteRAM(Y&0x00ff);
	LCD_WriteRAM(0x01);
	LCD_WriteRAM(0xdf);
	// 写入颜色，填充2个像素
	LCD_REG_Select(0x002c);
	LCD->LCD_RAM = Col;
	LCD->LCD_RAM = Col;

	Y++; // Y下移一行
	// 重新设置行列地址，绘制第二行2个像素
	LCD_REG_Select(0x002A);
	LCD_WriteRAM(X>>8);
	LCD_WriteRAM(X&0x00ff);
	LCD_WriteRAM(0x03);
	LCD_WriteRAM(0x1F);
	LCD_REG_Select(0x002b);
	LCD_WriteRAM(Y>>8);
	LCD_WriteRAM(Y&0x00ff);
	LCD_WriteRAM(0x01);
	LCD_WriteRAM(0xdf);
	LCD_REG_Select(0x002c);
	LCD->LCD_RAM = Col;
	LCD->LCD_RAM = Col;
}

/**
 * @brief  底层打点接口（0x1936屏），面向抽象设备结构体
 * @param  self: TFT设备实例
 * @param  x,y: 坐标
 * @param  color: 颜色
 */
static void TftLcdDevice_DrawPoint(TftLcdDevice *self, uint16_t x, uint16_t y, uint16_t color)
{
	(void)self; // 消除未使用参数警告
	// 配置列地址
	LCD_REG_Select(0x002A);
	LCD_WriteRAM(x>>8);
	LCD_WriteRAM(x&0x00ff);
	LCD_WriteRAM(0x03);
	LCD_WriteRAM(0x1F);
	// 配置行地址
	LCD_REG_Select(0x002b);
	LCD_WriteRAM(y>>8);
	LCD_WriteRAM(y&0x00ff);
	LCD_WriteRAM(0x01);
	LCD_WriteRAM(0xdf);
	// 写入像素颜色
	LCD_REG_Select(0x002c);
	LCD->LCD_RAM = color;
}

// 对外统一打点API
void LCD_DrawPoint(uint16_t X,uint16_t Y,uint16_t Col)
{
	TftLcdDevice_Construct(&g_tft_lcd);          // 初始化TFT设备对象
	g_tft_lcd.ops->draw_point(&g_tft_lcd, X, Y, Col); // 调用底层打点
}
#else
// ====================== 其他型号TFT屏 底层打点接口 ======================
static void TftLcdDevice_DrawPoint(TftLcdDevice *self, uint16_t x, uint16_t y, uint16_t color)
{
	(void)self;
	LCD->LCD_REG = LCD_REG_32;    // 选择X坐标寄存器
	LCD->LCD_RAM = x;             // 写入X坐标

	LCD->LCD_REG = LCD_REG_33;    // 选择Y坐标寄存器
	LCD->LCD_RAM = y;             // 写入Y坐标

	LCD->LCD_REG = LCD_REG_34;    // 选择颜色寄存器
	LCD->LCD_RAM = color;         // 写入像素颜色
}

// 对外统一打点API
void LCD_DrawPoint(uint16_t X,uint16_t Y,uint16_t Col)
{
	TftLcdDevice_Construct(&g_tft_lcd);
	g_tft_lcd.ops->draw_point(&g_tft_lcd, X, Y, Col);
}
#endif

// ====================== 0x1936 屏 窗口设置接口 ======================
#if __TFT_ID==0x1936
/**
 * @brief  设置TFT显示窗口区域
 * @param  SC: 起始列
 * @param  SP: 起始行
 * @param  EC: 结束列
 * @param  EP: 结束行
 */
void LCD_SetDisplayWindow(uint16_t SC,uint16_t SP,uint16_t EC,uint16_t EP)
{
	// 设置列地址范围
	LCD_REG_Select(0x002A);
	LCD_WriteRAM(SC>>8);
	LCD_WriteRAM(SC&0x00ff);
	LCD_WriteRAM((uint8_t)(EC>>8));
	LCD_WriteRAM((uint8_t)(EC&0x00ff));
	// 设置行地址范围
	LCD_REG_Select(0x002b);
	LCD_WriteRAM(SP>>8);
	LCD_WriteRAM(SP&0x00ff);
	LCD_WriteRAM((uint8_t)(EP>>8));
	LCD_WriteRAM((uint8_t)(EP&0x00ff));
}
#endif

// ====================== 光标设置接口（分屏型号） ======================
#if __TFT_ID==0x1936
/**
 * @brief  设置光标坐标（0x1936屏）
 * @param  Xpos,Ypos: 目标坐标
 */
void LCD_SetCursor(uint16_t Xpos, uint16_t Ypos)
{
	// 配置列地址
	LCD_REG_Select(0x002A);
	LCD_WriteRAM(Xpos>>8);
	LCD_WriteRAM(Xpos&0x00ff);
	LCD_WriteRAM(0x03);
	LCD_WriteRAM(0x1F);
	// 配置行地址
	LCD_REG_Select(0x002b);
	LCD_WriteRAM(Ypos>>8);
	LCD_WriteRAM(Ypos&0x00ff);
	LCD_WriteRAM(0x01);
	LCD_WriteRAM(0xdf);
}
#else
/**
 * @brief  设置光标坐标（其他型号屏，Y轴坐标翻转）
 * @param  Xpos: X坐标
 * @param  Ypos: Y坐标
 */
void LCD_SetCursor(uint16_t Xpos, uint8_t Ypos)
{
	LCD_WriteReg(LCD_REG_32,240-Ypos);  // Y坐标镜像翻转
	LCD_WriteReg(LCD_REG_33,Xpos);      // 设置X坐标
}
#endif

// ====================== 背光控制接口 ======================
#if _USE_DisplayOn
/**
 * @brief  开启LCD背光（PD3输出高电平）
 */
void LCD_DisplayOn(void)
{
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_SET);
}

/**
 * @brief  关闭LCD背光（PD3输出低电平）
 */
void LCD_DisplayOff(void)
{
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_RESET);
}
#endif

/**
 * @brief  底层写寄存器（抽象设备接口）
 * @param  self: TFT设备实例
 * @param  reg: 寄存器地址
 * @param  value: 写入值
 */
static void TftLcdDevice_WriteReg(TftLcdDevice *self, uint16_t reg, uint16_t value)
{
	(void)self;
	LCD->LCD_REG = reg;   // 选择寄存器
	LCD->LCD_RAM = value; // 写入数据
}

/**
 * @brief  对外通用写寄存器API
 * @param  LCD_Reg: 寄存器地址
 * @param  LCD_RegValue: 写入数据
 */
void LCD_WriteReg(uint16_t LCD_Reg, uint16_t LCD_RegValue)
{
	TftLcdDevice_Construct(&g_tft_lcd);
	g_tft_lcd.ops->write_reg(&g_tft_lcd, LCD_Reg, LCD_RegValue);
}

/**
 * @brief  底层读寄存器（抽象设备接口）
 * @param  self: TFT设备实例
 * @param  reg: 寄存器地址
 * @retval 寄存器读取值
 */
static uint16_t TftLcdDevice_ReadReg(TftLcdDevice *self, uint16_t reg)
{
	(void)self;
	LCD->LCD_REG = reg;
	return (LCD->LCD_RAM);
}

/**
 * @brief  对外通用读寄存器API
 * @param  LCD_Reg: 寄存器地址
 * @retval 读取到的寄存器值
 */
uint16_t LCD_ReadReg(uint16_t LCD_Reg)
{
	TftLcdDevice_Construct(&g_tft_lcd);
	return g_tft_lcd.ops->read_reg(&g_tft_lcd, LCD_Reg);
}

/**
 * @brief  准备写入GRAM（切换到颜色数据寄存器）
 */
void LCD_WriteRAM_Prepare(void)
{
#if __TFT_ID==0x1936
	LCD_REG_Select(0x002c);
#else
	LCD->LCD_REG = LCD_REG_34;
#endif
}

/**
 * @brief  选择指定LCD寄存器
 * @param  REG_Code: 寄存器地址
 */
void LCD_REG_Select(uint16_t REG_Code)
{
	LCD->LCD_REG = REG_Code;
}

/**
 * @brief  向GRAM写入颜色数据
 * @param  RGB_Code: 16位RGB颜色值
 */
void LCD_WriteRAM(uint16_t RGB_Code)
{
	LCD->LCD_RAM = RGB_Code;
}

/**
 * @brief  配置LCD FSMC 相关GPIO
 * @note   包含FSMC数据线、控制线、背光引脚初始化
 */
void LCD_CtrlLinesConfig(void)
{
	GPIO_InitTypeDef gpio_init_struct;

	// 使能 GPIOD/GPIOE/GPIOF/GPIOG 时钟（FSMC+背光）
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOE_CLK_ENABLE();
	__HAL_RCC_GPIOF_CLK_ENABLE();
	__HAL_RCC_GPIOG_CLK_ENABLE();

	// FSMC引脚配置：复用推挽、高速、无上下拉
	gpio_init_struct.Mode = GPIO_MODE_AF_PP;
	gpio_init_struct.Pull = GPIO_NOPULL;
	gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;
	gpio_init_struct.Alternate = GPIO_AF12_FSMC;

	// GPIOD: FSMC 低8位数据+部分控制线
	gpio_init_struct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_4 | GPIO_PIN_5 |
	                       GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 |
	                       GPIO_PIN_14 | GPIO_PIN_15;
	HAL_GPIO_Init(GPIOD, &gpio_init_struct);

	// GPIOE: FSMC 高8位数据线
	gpio_init_struct.Pin = GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 |
	                       GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 |
	                       GPIO_PIN_14 | GPIO_PIN_15;
	HAL_GPIO_Init(GPIOE, &gpio_init_struct);

	// GPIOF0: FSMC 地址线（区分寄存器/数据）
	gpio_init_struct.Pin = GPIO_PIN_0;
	HAL_GPIO_Init(GPIOF, &gpio_init_struct);

	// PD3: LCD 背光引脚，配置为通用推挽输出
	gpio_init_struct.Pin = GPIO_PIN_3;
	gpio_init_struct.Mode = GPIO_MODE_OUTPUT_PP;
	gpio_init_struct.Pull = GPIO_NOPULL;
	gpio_init_struct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOD, &gpio_init_struct);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_SET); // 默认开启背光
}

/**
 * @brief  配置FSMC外设，用于16位并口驱动TFT屏
 * @note   使用 FSMC Bank1，模拟SRAM模式访问LCD
 */
void LCD_FSMCConfig(void)
{
	FSMC_NORSRAM_TimingTypeDef timing;

	__HAL_RCC_FSMC_CLK_ENABLE();  // 使能FSMC时钟

	// FSMC 句柄初始化
	g_lcd_sram_handler.Instance = FSMC_NORSRAM_DEVICE;
	g_lcd_sram_handler.Extended = FSMC_NORSRAM_EXTENDED_DEVICE;
	g_lcd_sram_handler.Init.NSBank = FSMC_NORSRAM_BANK1;
	g_lcd_sram_handler.Init.DataAddressMux = FSMC_DATA_ADDRESS_MUX_DISABLE;  // 不使用地址数据复用
	g_lcd_sram_handler.Init.MemoryType = FSMC_MEMORY_TYPE_SRAM;              // 模拟SRAM设备
	g_lcd_sram_handler.Init.MemoryDataWidth = FSMC_NORSRAM_MEM_BUS_WIDTH_16;   // 16位数据宽度
	g_lcd_sram_handler.Init.BurstAccessMode = FSMC_BURST_ACCESS_MODE_DISABLE;
	g_lcd_sram_handler.Init.WaitSignalPolarity = FSMC_WAIT_SIGNAL_POLARITY_LOW;
	g_lcd_sram_handler.Init.WaitSignalActive = FSMC_WAIT_TIMING_BEFORE_WS;
	g_lcd_sram_handler.Init.WriteOperation = FSMC_WRITE_OPERATION_ENABLE;    // 允许写操作
	g_lcd_sram_handler.Init.WaitSignal = FSMC_WAIT_SIGNAL_DISABLE;
	g_lcd_sram_handler.Init.ExtendedMode = FSMC_EXTENDED_MODE_DISABLE;
	g_lcd_sram_handler.Init.AsynchronousWait = FSMC_ASYNCHRONOUS_WAIT_DISABLE;
	g_lcd_sram_handler.Init.WriteBurst = FSMC_WRITE_BURST_DISABLE;
	g_lcd_sram_handler.Init.ContinuousClock = FSMC_CONTINUOUS_CLOCK_SYNC_ONLY;
	g_lcd_sram_handler.Init.PageSize = FSMC_PAGE_SIZE_NONE;

	// FSMC 时序参数配置
	timing.AddressSetupTime = 4;      // 地址建立时间
	timing.AddressHoldTime = 0;       // 地址保持时间
	timing.DataSetupTime = 8;         // 数据建立时间
	timing.BusTurnAroundDuration = 0;
	timing.CLKDivision = 0;
	timing.DataLatency = 0;
	timing.AccessMode = FSMC_ACCESS_MODE_A; // 模式A

	// 初始化FSMC外设
	HAL_SRAM_Init(&g_lcd_sram_handler, &timing, &timing);
}
