/**
 * @file Drivers/TFT_LCD/text.c
 * @brief 基于外部字库的 GBK 文字渲染服务模块。
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
#include "text.h"
#include "string.h"

static void GbkTextRenderer_Construct(GbkTextRenderer *self);
static void GbkTextRenderer_GetMatrix(GbkTextRenderer *self, unsigned char *code, unsigned char *mat, u8 size);
static void GbkTextRenderer_ShowFont(GbkTextRenderer *self, u16 x, u16 y, u8 *font, u8 size, u8 mode);
static void GbkTextRenderer_ShowString(GbkTextRenderer *self, u16 x, u16 y, u16 width, u16 height, u8 *str, u8 size, u8 mode);
static void GbkTextRenderer_ShowStringMid(GbkTextRenderer *self, u16 x, u16 y, u8 *str, u8 size, u8 len);

static const GbkTextRendererOps s_gbk_text_renderer_ops =
{
    GbkTextRenderer_GetMatrix,
    GbkTextRenderer_ShowFont,
    GbkTextRenderer_ShowString,
    GbkTextRenderer_ShowStringMid,
};

GbkTextRenderer g_gbk_text_renderer;

static void GbkTextRenderer_Construct(GbkTextRenderer *self)
{
    if (self->ops == NULL)
    {
        self->ops = &s_gbk_text_renderer_ops;
    }
}

/**
 * @brief  从 W25Q64 外部字库中读取指定 GBK 汉字的点阵数据
 * @param  code: 汉字内码指针（GBK编码占用2个字节，code指向第一个字节）
 * @param  mat:  用于存储读取出来的点阵数据的缓冲区指针
 * @param  size: 字体大小/字号（支持 16、24、32 点阵）
 * @retval 无
 */
static void GbkTextRenderer_GetMatrix(GbkTextRenderer *self, unsigned char *code, unsigned char *mat, u8 size)
{
    unsigned char qh, ql;
    unsigned char i;
    unsigned char dx, dy, sx, sy, src_byte;
    unsigned char source[72];
    unsigned long foffset; // 点阵在当前字库文件中的相对偏移地址
    u8 source_csize;
    (void)self;
    
    // 计算当前字号下一个汉字所占用的字节大小 (csize)
    // 算法：(向上取整的每行字节数) * 字体高度。例如：16字号 = (16/8)*16 = 32字节；24字号 = (24/8)*24 = 72字节
    u8 csize = (size / 8 + ((size % 8) ? 1 : 0)) * (size);
    source_csize = (size == 32U) ? 72U : csize;
    
    qh = *code;       // 获取 GBK 编码的高位字节（通常代表“区”）
    ql = *(++code);   // 获取 GBK 编码的低位字节（通常代表“位”）
    
    // GBK 编码合法性检查
    // GBK 标准范围：高位(qh) 0x81~0xFE，低位(ql) 0x40~0xFE (剔除 0x7F)
    if (qh < 0x81 || ql < 0x40 || ql == 0xff || qh == 0xff)
    {
        // 如果不是合法的汉字编码，将点阵缓冲区清零（显示为空白），然后直接退出
        for (i = 0; i < csize; i++)
            *mat++ = 0x00;
        return;
    }
    
    // 计算当前汉字在 GBK 字符集中的绝对二维索引
    // 为什么要剔除 0x7F？因为在 GBK 编码中 0x7F 是控制字符，不是汉字编码，所以后面的编码要向前移一位
    if (ql < 0x7f)
        ql -= 0x40;   // 0x40 之前的区间偏移
    else
        ql -= 0x41;   // 剔除 0x7F 后的区间偏移
        
    qh -= 0x81;       // 高位字节减去基准值 0x81，转换为从 0 开始的索引

    // 计算该汉字点阵在字库中的相对字节偏移量
    // 每个区（即每个高位）固定包含 190 个低位编码点
    foffset = ((unsigned long)190 * qh + ql) * source_csize;
    
    /* 根据请求的字号，加上该字库在 Flash 中的物理基地址，并调用外部 SPI 接口读取点阵数据。
     * 这里是渲染层和外部字库存储层的唯一耦合点：渲染器只计算偏移，不关心 W25Q64 指令细节。
     */
    switch (size)
    {
        case 32:
            /* The original font metadata has 12/16/24-point addresses only.
               Scale the 24-point GBK glyph to 32 points in RAM. */
            W25Q64_Read(source, foffset + ftinfo.f24addr, source_csize);
            memset(mat, 0, csize);
            for (dx = 0U; dx < 32U; dx++)
            {
                sx = (unsigned char)(((unsigned int)dx * 24U) / 32U);
                for (dy = 0U; dy < 32U; dy++)
                {
                    sy = (unsigned char)(((unsigned int)dy * 24U) / 32U);
                    src_byte = source[sx * 3U + (sy / 8U)];
                    if ((src_byte & (unsigned char)(0x80U >> (sy % 8U))) != 0U)
                        mat[dx * 4U + (dy / 8U)] |= (unsigned char)(0x80U >> (dy % 8U));
                }
            }
            break;
            
        case 16:
            W25Q64_Read(mat, foffset + ftinfo.f16addr, csize);
            break;
            
        case 24:
            W25Q64_Read(mat, foffset + ftinfo.f24addr, csize);
            break;
    }
}

static void GbkTextRenderer_ShowFont(GbkTextRenderer *self, u16 x, u16 y, u8 *font, u8 size, u8 mode)
{
    u8 temp, t, t1;
    u16 y0 = y;
    u8 dzk[128];
    u8 csize = (size / 8 + ((size % 8) ? 1 : 0)) * size;

    if (size != 16 && size != 24 && size != 32)
        return;

    /* 点阵按列扫描绘制：每读取一个字节，从最高位开始决定是否点亮像素。
     * mode=1 时同步绘制背景色，mode=0 时保留原背景形成透明效果。
     */
    self->ops->get_matrix(self, font, dzk, size);
    for (t = 0; t < csize; t++)
    {
        temp = dzk[t];
        for (t1 = 0; t1 < 8; t1++)
        {
            if (temp & 0x80)
            {
                PutPixel(x, y);
            }
            else if (mode == 1)
            {
                LCD_SetCursor(x, y);
                LCD_WriteRAM_Prepare();
                LCD_WriteRAM(BackColor);
            }
            temp <<= 1;
            y++;
            if ((y - y0) == size)
            {
                y = y0;
                x++;
                break;
            }
        }
    }
}

static void GbkTextRenderer_ShowString(GbkTextRenderer *self, u16 x, u16 y, u16 width, u16 height, u8 *str, u8 size, u8 mode)
{
    u16 x0 = x;
    u16 y0 = y;
    u8 bHz = 0;

    /* 字符串中可能混合 ASCII 和 GBK 双字节汉字。
     * bHz 用来记录“已遇到汉字高字节，等待低字节组合成完整 GBK 编码”的状态。
     */
    while (*str != '\0')
    {
        if (!bHz)
        {
            if ((signed char)*str < 0)
            {
                bHz = 1;
            }
            else
            {
                if (*str == '\n')
                {
                    y += size;
                    x = x0;
                    str++;
                    continue;
                }
                if (x > (u16)(x0 + width - size / 2U))
                {
                    y += size;
                    x = x0;
                }
                if (y > (u16)(y0 + height - size))
                    break;

                LCD_ShowChar(x, y, *str);
                str++;
                x += size / 2U;
            }
        }
        else
        {
            bHz = 0;
            if (x > (u16)(x0 + width - size))
            {
                y += size;
                x = x0;
            }
            if (y > (u16)(y0 + height - size))
                break;

            self->ops->show_font(self, x, y, str, size, mode);
            str += 2;
            x += size;
        }
    }
}

static void GbkTextRenderer_ShowStringMid(GbkTextRenderer *self, u16 x, u16 y, u8 *str, u8 size, u8 len)
{
    u16 strlenth;

    strlenth = strlen((const char *)str) * size / 2U;
    if (strlenth > len)
    {
        self->ops->show_string(self, x, y, LCD_Pixel_Width, LCD_Pixel_Height, str, size, 1);
    }
    else
    {
        self->ops->show_string(self, (u16)(x + (len - strlenth) / 2U), y, LCD_Pixel_Width, LCD_Pixel_Height, str, size, 1);
    }
}

void Get_HzMat(unsigned char *code, unsigned char *mat, u8 size)
{
    GbkTextRenderer_Construct(&g_gbk_text_renderer);
    g_gbk_text_renderer.ops->get_matrix(&g_gbk_text_renderer, code, mat, size);
}

void LCD_ShowGBKFont(u16 x, u16 y, u8 *font, u8 size, u8 mode)
{
    GbkTextRenderer_Construct(&g_gbk_text_renderer);
    g_gbk_text_renderer.ops->show_font(&g_gbk_text_renderer, x, y, font, size, mode);
}

void LCD_ShowGBKString(u16 x, u16 y, u16 width, u16 height, u8 *str, u8 size, u8 mode)
{
    GbkTextRenderer_Construct(&g_gbk_text_renderer);
    g_gbk_text_renderer.ops->show_string(&g_gbk_text_renderer, x, y, width, height, str, size, mode);
}

void LCD_ShowGBKStringMid(u16 x, u16 y, u8 *str, u8 size, u8 len)
{
    GbkTextRenderer_Construct(&g_gbk_text_renderer);
    g_gbk_text_renderer.ops->show_string_mid(&g_gbk_text_renderer, x, y, str, size, len);
}
