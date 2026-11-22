/**
 * @file Drivers/TFT_LCD/Character.h
 * @brief 内置中文字模渲染器模块。
 * @details
 * - 属于显示子系统，分别负责 LCD、字库 Flash、字体数据或文字渲染中的明确职责。
 * - 模块间通过对象接口或兼容 API 协作，避免 UI 层直接操作显示控制器寄存器。
 * - 字体/字模数据文件以数组尺寸、字号和索引关系一致性为维护重点。
 * - 本文件已移除原子/ST 示例模板，统一使用项目自定义 UTF-8 文件模板。
 * @encoding UTF-8
 */

#ifndef __CHARACTER_H
#define __CHARACTER_H

#include "TFT_LCD.h"

typedef struct LcdBuiltinFontRenderer LcdBuiltinFontRenderer;

/**
 * @brief LCD 内置中文字模渲染器操作表。
 */
typedef struct
{
	void (*show_48)(LcdBuiltinFontRenderer *self, uint16_t y, uint16_t x, uint8_t font[2], uint8_t size);        /**< 绘制单个 48 点阵汉字。 */
	void (*show_string_48)(LcdBuiltinFontRenderer *self, uint16_t y, uint16_t x, uint8_t *str, uint8_t size);    /**< 绘制 48 点阵字符串。 */
	void (*show_32)(LcdBuiltinFontRenderer *self, uint16_t y, uint16_t x, uint8_t font[2], uint8_t size);        /**< 绘制单个 32 点阵汉字。 */
	void (*show_string_32)(LcdBuiltinFontRenderer *self, uint16_t y, uint16_t x, uint8_t *str, uint8_t size);    /**< 绘制 32 点阵字符串。 */
	void (*show_16)(LcdBuiltinFontRenderer *self, uint16_t y, uint16_t x, uint8_t font[2], uint8_t size);        /**< 绘制单个 16 点阵汉字。 */
	void (*show_string_16)(LcdBuiltinFontRenderer *self, uint16_t y, uint16_t x, uint8_t *str, uint8_t size);    /**< 绘制 16 点阵字符串。 */
} LcdBuiltinFontRendererOps;

/**
 * @brief LCD 内置中文字模渲染器对象。
 */
struct LcdBuiltinFontRenderer
{
	const LcdBuiltinFontRendererOps *ops; /**< 内置字模渲染方法表。 */
};

extern LcdBuiltinFontRenderer g_lcd_builtin_font_renderer;

/* LCD 内置中文字模显示接口。 */
void LCD_GB4848(uint16_t y, uint16_t x, uint8_t font[2], uint8_t size);
void Show_Str48(uint16_t y, uint16_t x, uint8_t *str, uint8_t size);
void LCD_GB3232(uint16_t y, uint16_t x, uint8_t font[2], uint8_t size);
void Show_Str32(uint16_t y, uint16_t x, uint8_t *str, uint8_t size);
void LCD_GB1616(uint16_t y, uint16_t x, uint8_t font[2], uint8_t size);
void Show_Str(uint16_t y, uint16_t x, uint8_t *str, uint8_t size);

#endif
