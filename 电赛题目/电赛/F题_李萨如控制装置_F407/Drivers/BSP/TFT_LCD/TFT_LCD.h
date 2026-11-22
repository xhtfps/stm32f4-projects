/**
 * @file Drivers/TFT_LCD/TFT_LCD.h
 * @brief TFT LCD 控制器驱动、基础绘图和显示状态对象模块。
 * @details
 * - 属于显示子系统，分别负责 LCD、字库 Flash、字体数据或文字渲染中的明确职责。
 * - 模块间通过对象接口或兼容 API 协作，避免 UI 层直接操作显示控制器寄存器。
 * - 字体/字模数据文件以数组尺寸、字号和索引关系一致性为维护重点。
 * - 本文件已移除原子/ST 示例模板，统一使用项目自定义 UTF-8 文件模板。
 * @encoding UTF-8
 */

#ifndef __TFT_LCD_H
#define __TFT_LCD_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include "fonts.h"

/**
 * @brief 兼容旧版驱动数据类型别名
 * @note 为保留历史代码兼容性，重定义常用基础数据类型
 */
#ifndef __TFT_LCD_LEGACY_TYPES
#define __TFT_LCD_LEGACY_TYPES
typedef uint8_t  u8;    // 8位无符号整型
typedef uint16_t u16;   // 16位无符号整型
typedef uint32_t u32;   // 32位无符号整型
#endif

/**
 * @brief 坐标点结构体
 * @param X 横坐标
 * @param Y 纵坐标
 */
typedef struct
{
	int16_t X;
	int16_t Y;
} Point, * pPoint;

// 前置声明TFT设备结构体
typedef struct TftLcdDevice TftLcdDevice;

/**
 * @brief TFT LCD 控制器对象操作函数表（面向对象接口）
 * @details 将LCD所有操作封装为函数指针，实现分层解耦，上层应用无需关心底层硬件差异
 */
typedef struct
{
	/**
	 * @brief 初始化 LCD 控制器和 FSMC 接口
	 * @param self TFT设备实例指针
	 */
	void (*init)(TftLcdDevice *self);

	/**
	 * @brief 同时设置前景色与背景色
	 * @param self TFT设备实例指针
	 * @param text_color 文字/绘图前景色(RGB565)
	 * @param back_color 背景填充色(RGB565)
	 */
	void (*set_colors)(TftLcdDevice *self, uint16_t text_color, uint16_t back_color);

	/**
	 * @brief 读取当前前景色和背景色
	 * @param self TFT设备实例指针
	 * @param text_color 存储前景色的指针
	 * @param back_color 存储背景色的指针
	 */
	void (*get_colors)(TftLcdDevice *self, __IO uint16_t *text_color, __IO uint16_t *back_color);

	/**
	 * @brief 单独设置文字/绘图前景色
	 * @param self TFT设备实例指针
	 * @param color 目标颜色(RGB565)
	 */
	void (*set_text_color)(TftLcdDevice *self, uint16_t color);

	/**
	 * @brief 单独设置背景填充色
	 * @param self TFT设备实例指针
	 * @param color 目标颜色(RGB565)
	 */
	void (*set_back_color)(TftLcdDevice *self, uint16_t color);

	/**
	 * @brief 设置当前使用的字体与工作模式
	 * @param self TFT设备实例指针
	 * @param fonts 字体结构体指针
	 * @param mode 字体兼容/渲染模式
	 */
	void (*set_font)(TftLcdDevice *self, sFONT *fonts, uint8_t mode);

	/**
	 * @brief 获取当前生效的字体对象
	 * @param self TFT设备实例指针
	 * @retval 当前字体结构体指针
	 */
	sFONT *(*get_font)(TftLcdDevice *self);

	/**
	 * @brief 全屏清屏，用指定颜色填充整个屏幕
	 * @param self TFT设备实例指针
	 * @param color 填充颜色(RGB565)
	 */
	void (*clear)(TftLcdDevice *self, uint16_t color);

	/**
	 * @brief 绘制单个像素点
	 * @param self TFT设备实例指针
	 * @param x 像素横坐标
	 * @param y 像素纵坐标
	 * @param color 像素颜色(RGB565)
	 */
	void (*draw_point)(TftLcdDevice *self, uint16_t x, uint16_t y, uint16_t color);

	/**
	 * @brief 向LCD控制器指定寄存器写入数据
	 * @param self TFT设备实例指针
	 * @param reg 寄存器地址
	 * @param value 待写入的数据
	 */
	void (*write_reg)(TftLcdDevice *self, uint16_t reg, uint16_t value);

	/**
	 * @brief 从LCD控制器指定寄存器读取数据
	 * @param self TFT设备实例指针
	 * @param reg 寄存器地址
	 * @retval 寄存器读取到的数据
	 */
	uint16_t (*read_reg)(TftLcdDevice *self, uint16_t reg);
} TftLcdDeviceOps;

/**
 * @brief TFT LCD 控制器设备主体结构体
 * @details 统一维护屏幕运行状态、硬件句柄与操作接口，作为整个驱动的核心实例
 */
struct TftLcdDevice
{
	SRAM_HandleTypeDef *sram_handle;  /**< FSMC模拟SRAM的HAL句柄，用于LCD并口通信 */
	sFONT *font;                      /**< 当前正在使用的字体结构体指针 */
	uint16_t text_color;              /**< 当前绘图/文字前景色 */
	uint16_t back_color;              /**< 当前绘图/文字背景色 */
	const TftLcdDeviceOps *ops;       /**< 绑定的操作函数表指针 */
};

// 全局TFT设备实例，项目统一使用此对象操作屏幕
extern TftLcdDevice g_tft_lcd;

/************************* 功能模块开关宏定义 *************************/
#define	_USE_GRAPH		1	    /**< 1:启用图形绘制功能(直线/矩形/圆形等) 0:关闭 */
#define _USE_DisplayOn	1	    /**< 1:启用LCD背光控制接口 0:关闭 */
#define _USE_HZK16 0	    /**< 本工程仅编入ASCII字库；外部中文字库未接入 */
#define _USE_0X  1	        /**< 1:启用十六进制数据显示接口 0:关闭 */
#define _USE_Float  1	    /**< 1:启用浮点数显示接口 0:关闭 */
#define __CONFIG_FONTS 0    /**< 字体布局模式选择 0:动态适配字体尺寸 1:固定偏移计算 */

/************************* 屏幕型号标识 *************************/
#define __TFT_ID   0x1936   /**< 当前驱动屏幕ID，0x1936对应SSD1963控制器 */

/************************* LCD控制器寄存器地址定义 *************************/
// 通用寄存器编号映射，统一寄存器名称与物理地址
#define LCD_REG_0             0x00
#define LCD_REG_1             0x01
#define LCD_REG_2             0x02
#define LCD_REG_3             0x03
#define LCD_REG_4             0x04
#define LCD_REG_5             0x05
#define LCD_REG_6             0x06
#define LCD_REG_7             0x07
#define LCD_REG_8             0x08
#define LCD_REG_9             0x09
#define LCD_REG_10            0x0A
#define LCD_REG_12            0x0C
#define LCD_REG_13            0x0D
#define LCD_REG_14            0x0E
#define LCD_REG_15            0x0F
#define LCD_REG_16            0x10
#define LCD_REG_17            0x11
#define LCD_REG_18            0x12
#define LCD_REG_19            0x13
#define LCD_REG_20            0x14
#define LCD_REG_21            0x15
#define LCD_REG_22            0x16
#define LCD_REG_23            0x17
#define LCD_REG_24            0x18
#define LCD_REG_25            0x19
#define LCD_REG_26            0x1A
#define LCD_REG_27            0x1B
#define LCD_REG_28            0x1C
#define LCD_REG_29            0x1D
#define LCD_REG_30            0x1E
#define LCD_REG_31            0x1F
#define LCD_REG_32            0x20    /**< X坐标寄存器 */
#define LCD_REG_33            0x21    /**< Y坐标寄存器 */
#define LCD_REG_34            0x22    /**< GRAM颜色数据寄存器 */
#define LCD_REG_36            0x24
#define LCD_REG_37            0x25
#define LCD_REG_40            0x28
#define LCD_REG_41            0x29
#define LCD_REG_43            0x2B
#define LCD_REG_45            0x2D
#define LCD_REG_48            0x30
#define LCD_REG_49            0x31
#define LCD_REG_50            0x32
#define LCD_REG_51            0x33
#define LCD_REG_52            0x34
#define LCD_REG_53            0x35
#define LCD_REG_54            0x36
#define LCD_REG_55            0x37
#define LCD_REG_56            0x38
#define LCD_REG_57            0x39
#define LCD_REG_58            0x3A
#define LCD_REG_59            0x3B
#define LCD_REG_60            0x3C
#define LCD_REG_61            0x3D
#define LCD_REG_62            0x3E
#define LCD_REG_63            0x3F
#define LCD_REG_64            0x40
#define LCD_REG_65            0x41
#define LCD_REG_66            0x42
#define LCD_REG_67            0x43
#define LCD_REG_68            0x44
#define LCD_REG_69            0x45
#define LCD_REG_70            0x46
#define LCD_REG_71            0x47
#define LCD_REG_72            0x48
#define LCD_REG_73            0x49
#define LCD_REG_74            0x4A
#define LCD_REG_75            0x4B
#define LCD_REG_76            0x4C
#define LCD_REG_77            0x4D
#define LCD_REG_78            0x4E
#define LCD_REG_79            0x4F
#define LCD_REG_80            0x50
#define LCD_REG_81            0x51
#define LCD_REG_82            0x52
#define LCD_REG_83            0x53
#define LCD_REG_96            0x60
#define LCD_REG_97            0x61
#define LCD_REG_106           0x6A
#define LCD_REG_118           0x76
#define LCD_REG_128           0x80
#define LCD_REG_129           0x81
#define LCD_REG_130           0x82
#define LCD_REG_131           0x83
#define LCD_REG_132           0x84
#define LCD_REG_133           0x85
#define LCD_REG_134           0x86
#define LCD_REG_135           0x87
#define LCD_REG_136           0x88
#define LCD_REG_137           0x89
#define LCD_REG_139           0x8B
#define LCD_REG_140           0x8C
#define LCD_REG_141           0x8D
#define LCD_REG_143           0x8F
#define LCD_REG_144           0x90
#define LCD_REG_145           0x91
#define LCD_REG_146           0x92
#define LCD_REG_147           0x93
#define LCD_REG_148           0x94
#define LCD_REG_149           0x95
#define LCD_REG_150           0x96
#define LCD_REG_151           0x97
#define LCD_REG_152           0x98
#define LCD_REG_153           0x99
#define LCD_REG_154           0x9A
#define LCD_REG_157           0x9D
#define LCD_REG_192           0xC0
#define LCD_REG_193           0xC1
#define LCD_REG_229           0xE5

/**
 * @brief SSD1963 屏幕时序参数（分辨率800*480）
 * @note H:水平方向 V:垂直方向 DP:有效区域 TP:总周期 PS:同步起始 PW:脉冲宽度
 */
#if __TFT_ID==0x1936
#define  SSD1963_HDP 799    // 水平有效像素宽度
#define  SSD1963_HT  928    // 水平总周期像素数
#define  SSD1963_HPS 46     // 水平同步信号起始位置
#define  SSD1963_LPS 15     // 水平数据起始偏移
#define  SSD1963_HPW 48     // 水平同步脉冲宽度
#define  SSD1963_VDP 479    // 垂直有效像素高度
#define  SSD1963_VT  525    // 垂直总周期行数
#define  SSD1963_VPS 16     // 垂直同步信号起始位置
#define  SSD1963_FPS 8      // 垂直数据起始偏移
#define  SSD1963_VPW 16     // 垂直同步脉冲宽度
#endif

/************************* 标准颜色定义(RGB565格式) *************************/
#define White          0xFFFF  // 白色
#define Black          0x0000  // 黑色
#define Grey           0xF7DE  // 浅灰色
#define Blue           0x001F  // 纯蓝色
#define Blue2          0x051F  // 浅蓝
#define Red            0xF800  // 纯红色
#define Magenta        0xF81F  // 品红色
#define Green          0x07E0  // 纯绿色
#define Cyan           0x7FFF  // 青色
#define Yellow         0xFFE0  // 黄色
#define BROWN 		   0XBC40  // 棕色
#define BRRED 		   0XFC70  // 砖红
#define GRAY  		   0X8430  // 灰色

#define DARKBLUE       0X01CF  // 深蓝
#define LIGHTBLUE      0X7D7C  // 浅蓝
#define GRAYBLUE       0X5458  // 灰蓝

#define LIGHTGREEN     0X841F  // 浅绿
#define LIGHTGRAY      0XEF5B  // 亮灰
#define LGRAY 		   0XC618  // 淡灰

#define LGRAYBLUE      0XA651  // 浅灰蓝
#define LBBLUE         0X2B12  // 暗青蓝

/************************* 屏幕基础参数与工具宏 *************************/
#define LCD_DEFAULT_FONT         Font32x16  // 系统默认字体：32*16点阵字体
#define LCD_Pixel_Width          0x0320     // 屏幕水平总像素 800
#define LCD_Pixel_Height         0x01E0     // 屏幕垂直总像素 480
#define LCD_Pixel_Max            0x5DC00    // 屏幕总像素点数
#define MAX_Line  (LCD_Pixel_Height/(((sFONT *)LCD_GetFont())->Height))   // 屏幕最大文本行数
#define MAX_Column (LCD_Pixel_Width/(((sFONT *)LCD_GetFont())->Width))    // 屏幕最大文本列数

/**
 * @brief RGB888转RGB565合成宏
 * @param R 红色分量(0~255)
 * @param G 绿色分量(0~255)
 * @param B 蓝色分量(0~255)
 * @retval 16位RGB565颜色值
 */
#define ASSEMBLE_RGB(R, G, B)    ((((R)& 0xF8) << 8) | (((G) & 0xFC) << 3) | (((B) & 0xF8) >> 3))

/**
 * @brief 文本行/列转像素坐标计算宏
 * @note __CONFIG_FONTS=1 使用固定偏移；否则根据当前字体尺寸动态计算
 */
#if __CONFIG_FONTS==1
#define Line(x)  ((x)<<4)        // 文本行转为Y像素坐标(固定16像素行高)
#define Column(x)((x)<<3)        // 文本列转为X像素坐标(固定8像素列宽)
#else
#define Line(x)  ((x) * (((sFONT *)LCD_GetFont())->Height))  // 按字体高度计算行像素
#define Column(x)((x) * (((sFONT *)LCD_GetFont())->Width))  // 按字体宽度计算列像素
#endif

/************************* 对外API函数声明 *************************/
/**
 * @brief TFT LCD 整体初始化
 * @note 依次完成GPIO、FSMC、LCD控制器、背光、显示参数配置
 */
void TFT_LCD_Init(void);

/**
 * @brief 同时设置前景色和背景色
 * @param _TextColor 前景色
 * @param _BackColor 背景色
 */
void LCD_SetColors(__IO uint16_t _TextColor, __IO uint16_t _BackColor);

/**
 * @brief 获取当前前景色和背景色
 * @param _TextColor 存储前景色的指针
 * @param _BackColor 存储背景色的指针
 */
void LCD_GetColors(__IO uint16_t *_TextColor, __IO uint16_t *_BackColor);

/**
 * @brief 设置绘图/文字前景色
 * @param Color 目标颜色
 */
void LCD_SetTextColor(__IO uint16_t Color);

/**
 * @brief 设置绘图/文字背景色
 * @param Color 目标颜色
 */
void LCD_SetBackColor(__IO uint16_t Color);

/**
 * @brief 设置当前使用字体与渲染模式
 * @param fonts 字体结构体指针
 * @param Mode 字体工作模式
 */
void LCD_SetFont(sFONT *fonts,uint8_t Mode);

/**
 * @brief 获取当前使用的字体指针
 * @retval 字体结构体地址
 */
sFONT *LCD_GetFont(void);

/**
 * @brief 清空指定文本行
 * @param Line 待清空的文本行号
 */
void LCD_ClearLine(uint16_t Line);

/**
 * @brief 全屏清屏，用指定颜色填充
 * @param Color 填充颜色
 */
void LCD_Clear(uint16_t Color);

/**
 * @brief 区域清屏，填充指定矩形区域
 * @param Xs,Ys 区域左上角坐标
 * @param Xe,Ye 区域右下角坐标
 * @param Color 填充颜色
 */
void LCD_Appoint_Clear(uint16_t Xs,uint16_t Ys,uint16_t Xe,uint16_t Ye,uint16_t Color);

/**
 * @brief 在指定坐标显示单个ASCII字符
 * @param x,y 字符左上角坐标
 * @param Char 待显示字符
 */
void LCD_ShowChar(uint16_t x,uint16_t y,uint8_t Char);

/**
 * @brief 在指定文本行列显示字符串
 * @param Line 文本行号
 * @param Column 文本列号
 * @param ptr 字符串指针
 * @retval 字符串长度
 */
uint32_t LCD_DisplayStringLine(uint16_t Line,uint16_t Column, uint8_t *ptr);

/**
 * @brief 显示十进制整数
 * @param Line 文本行号
 * @param Column 文本列号
 * @param num 待显示整数
 */
void LCD_DisplayNum(uint16_t Line,uint16_t Column, uint32_t num);

#if _USE_Float
/**
 * @brief 显示浮点数
 * @param Line 文本行号
 * @param Column 文本列号
 * @param num 待显示浮点数
 */
void LCD_Display_FloatNum(uint16_t Line,uint16_t Column, double num);
#endif

#if _USE_0X
/**
 * @brief 以十六进制格式显示数据
 * @param Line 文本行号
 * @param Column 文本列号
 * @param X 数据缓冲区指针
 * @param N 数据长度
 */
void LCD_Display0x(uint16_t Line,uint16_t Column, uint8_t *X,uint8_t N);
#endif

#if _USE_GRAPH
/**
 * @brief 使用当前前景色在指定坐标画点
 * @param x,y 像素坐标
 */
void PutPixel(int16_t x, int16_t y);

/**
 * @brief 批量写入BMP图像数据
 * @param ptr 图像数据缓冲区
 * @param SIZE 数据字节长度
 */
void LCD_WeBMP_SIZE(uint16_t *ptr,uint32_t SIZE);

/**
 * @brief 绘制水平/垂直直线（简易版）
 * @param Xpos,Ypos 起点坐标
 * @param Length 线段长度
 * @param Direction 方向 0:水平 1:垂直
 */
void LCD_DrawLine(uint16_t Xpos, uint16_t Ypos, uint16_t Length, uint8_t Direction);

/**
 * @brief 绘制任意两点间直线（Bresenham算法）
 * @param x1,y1 起点坐标
 * @param x2,y2 终点坐标
 */
void LCD_DrawuniLine(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);

/**
 * @brief 绘制空心矩形
 * @param Xpos,Ypos 矩形左上角
 * @param Height 矩形高度
 * @param Width 矩形宽度
 */
void LCD_DrawRect(uint16_t Xpos, uint16_t Ypos, uint16_t Height, uint16_t Width);

/**
 * @brief 绘制实心矩形（带自定义颜色）
 * @param x1,y1 左上角坐标
 * @param x2,y2 右下角坐标
 * @param color 填充颜色
 */
void LCD_DrawRectS(uint16_t x1,uint16_t y1,uint16_t y2,uint16_t x2,uint16_t color);

/**
 * @brief 绘制空心圆
 * @param Xpos,Ypos 圆心坐标
 * @param Radius 圆半径
 */
void LCD_DrawCircle(uint16_t Xpos, uint16_t Ypos, uint16_t Radius);

/**
 * @brief 绘制实心圆
 * @param x,y 圆心坐标
 * @param size 圆半径
 * @param color 填充颜色
 */
void LCD_DrawCircleS(uint16_t x,uint16_t y,uint16_t size,uint16_t color);
#endif

/**
 * @brief 通用画点接口，指定坐标与颜色
 * @param X,Y 像素坐标
 * @param Col 像素颜色
 */
void LCD_DrawPoint(uint16_t X,uint16_t Y,uint16_t Col);

#if __TFT_ID==0x1936
/**
 * @brief 绘制2*2放大像素点（SSD1963专用）
 * @param X,Y 基准坐标
 * @param Col 像素颜色
 */
void LCD_DrawPoint_4(uint16_t X,uint16_t Y,uint16_t Col);

/**
 * @brief 设置LCD显示窗口区域（SSD1963专用）
 * @param SC,SP 窗口起始列、行
 * @param EC,EP 窗口结束列、行
 */
void LCD_SetDisplayWindow(uint16_t SC,uint16_t SP,uint16_t EC,uint16_t EP);

/**
 * @brief 设置GRAM操作光标坐标（SSD1963专用）
 * @param Xpos,Ypos 光标坐标
 */
void LCD_SetCursor(uint16_t Xpos, uint16_t Ypos);
#else
/**
 * @brief 设置GRAM操作光标坐标（其他型号屏）
 * @param Xpos 横坐标
 * @param Ypos 纵坐标
 */
void LCD_SetCursor(uint16_t Xpos, uint8_t Ypos);
#endif

/**
 * @brief 向LCD指定寄存器写入数据
 * @param LCD_Reg 寄存器地址
 * @param LCD_RegValue 待写入数据
 */
void LCD_WriteReg(uint16_t LCD_Reg, uint16_t LCD_RegValue);

/**
 * @brief 读取LCD指定寄存器数据
 * @param LCD_Reg 寄存器地址
 * @retval 寄存器读取值
 */
uint16_t LCD_ReadReg(uint16_t LCD_Reg);

#if _USE_DisplayOn
/**
 * @brief 开启LCD背光
 */
void LCD_DisplayOn(void);

/**
 * @brief 关闭LCD背光
 */
void LCD_DisplayOff(void);
#endif

/**
 * @brief 配置LCD FSMC相关GPIO引脚
 * @note 包含数据线、控制线、背光引脚初始化
 */
void LCD_CtrlLinesConfig(void);

/**
 * @brief 配置FSMC外设参数，模拟SRAM驱动LCD并口
 */
void LCD_FSMCConfig(void);

// FSMC-SRAM 全局句柄，供底层驱动使用
extern SRAM_HandleTypeDef g_lcd_sram_handler;

#ifdef __cplusplus
}
#endif

// 全局前景色、背景色变量
extern __IO uint16_t TextColor,BackColor;

/**
 * @brief 准备写入GRAM：切换至颜色数据寄存器
 */
void LCD_WriteRAM_Prepare(void);

/**
 * @brief 向GRAM写入颜色数据
 * @param RGB_Code RGB565颜色值
 */
void LCD_WriteRAM(uint16_t RGB_Code);

#endif
