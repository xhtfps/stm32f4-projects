/**
 * @file Drivers/TFT_LCD/text.h
 * @brief 基于外部字库的 GBK 文字渲染服务模块。
 * @details
 * - 属于显示子系统，分别负责 LCD、字库 Flash、字体数据或文字渲染中的明确职责。
 * - 模块间通过对象接口或兼容 API 协作，避免 UI 层直接操作显示控制器寄存器。
 * - 字体/字模数据文件以数组尺寸、字号和索引关系一致性为维护重点。
 * - 本文件已移除原子/ST 示例模板，统一使用项目自定义 UTF-8 文件模板。
 * @encoding UTF-8
 */

#ifndef __TEXT_H__
#define __TEXT_H__

#include <stm32f4xx.h>
#include "fontupd.h"

typedef struct GbkTextRenderer GbkTextRenderer;

/**
 * @brief GBK 文本渲染器操作表。
 */
typedef struct
{
    void (*get_matrix)(GbkTextRenderer *self, unsigned char *code, unsigned char *mat, u8 size);                 /**< 根据 GBK 编码读取点阵。 */
    void (*show_font)(GbkTextRenderer *self, u16 x, u16 y, u8 *font, u8 size, u8 mode);                          /**< 绘制单个 GBK 汉字。 */
    void (*show_string)(GbkTextRenderer *self, u16 x, u16 y, u16 width, u16 height, u8 *str, u8 size, u8 mode);   /**< 在矩形区域内绘制混合字符串。 */
    void (*show_string_mid)(GbkTextRenderer *self, u16 x, u16 y, u8 *str, u8 size, u8 len);                      /**< 居中绘制字符串。 */
} GbkTextRendererOps;

/**
 * @brief GBK 文本渲染器对象。
 * @note 当前无额外状态，字库地址来自 fontupd 模块的 ftinfo。
 */
struct GbkTextRenderer
{
    const GbkTextRendererOps *ops; /**< 渲染器方法表。 */
};

extern GbkTextRenderer g_gbk_text_renderer;

/* ==================================================================================
 * 汉字与字符串显示驱动接口
 * ================================================================================== */

/**
 * @brief  获取汉字的点阵码
 * @note   内部通过解析 GBK 编码，计算出在外部 Flash（如 W25Q64）中的地址并读取点阵数据
 * @param  code: 汉字内码指针（指向当前汉字的 GBK 编码，占用 2 字节）
 * @param  mat:  用于存储读取出来的点阵数据的缓冲区指针
 * @param  size: 字体大小/字号（支持 12、16、24、32 等点阵）
 * @retval 无
 */
void Get_HzMat(unsigned char *code, unsigned char *mat, u8 size);

/**
 * @brief  在 LCD 指定位置显示一个汉字
 * @param  x:    起始横坐标 (X 轴)
 * @param  y:    起始纵坐标 (Y 轴)
 * @param  font: 汉字点阵数据缓冲区的指针，或者直接传入汉字字符串
 * @param  size: 字体大小/字号（如 12、16、24、32 等）
 * @param  mode: 显示模式。
 * 0: 非叠加模式（点阵有像素的点显示前景色，无像素的点用背景色覆盖）
 * 1: 叠加模式（只绘制前景色的点，不破坏原本的 LCD 背景，适合图片/UI背景）
 * @retval 无
 */
void LCD_ShowGBKFont(u16 x, u16 y, u8 *font, u8 size, u8 mode);

/**
 * @brief  在 LCD 指定区域内显示一个字符串（支持中英文混排）
 * @note   会自动判断是 ASCII 字符还是 GBK 汉字，并具备自动换行或超出区域截断的处理机制
 * @param  x:      起始区域横坐标
 * @param  y:      起始区域纵坐标
 * @param  width:  显示区域的总宽度
 * @param  height: 显示区域的总高度
 * @param  str:    要显示的字符串指针（以 '\0' 结尾）
 * @param  size:   字体大小/字号（如 12、16、24、32 等）
 * @param  mode:   显示模式：0 (非叠加), 1 (叠加)
 * @retval 无
 */
void LCD_ShowGBKString(u16 x, u16 y, u16 width, u16 height, u8 *str, u8 size, u8 mode);

/**
 * @brief  在指定宽度的范围内，将字符串居中显示
 * @details 内部会先计算整个字符串所占的总像素宽度，然后调整起始 X 坐标使其居中
 * @param  x:    显示区域的中心参考点或起始横坐标（取决于具体驱动实现）
 * @param  y:    显示区域的起始纵坐标
 * @param  str:  要显示的字符串指针
 * @param  size: 字体大小/字号
 * @param  len:  限制居中区域的逻辑长度或像素限制宽度
 * @retval 无
 */
void LCD_ShowGBKStringMid(u16 x, u16 y, u8 *str, u8 size, u8 len);

#endif /* __TEXT_H__ */
