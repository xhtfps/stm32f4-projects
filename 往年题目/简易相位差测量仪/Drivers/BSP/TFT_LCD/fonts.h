/**
 * @file Drivers/TFT_LCD/fonts.h
 * @brief ASCII/基础字体点阵数据表模块。
 * @details
 * - 属于显示子系统，分别负责 LCD、字库 Flash、字体数据或文字渲染中的明确职责。
 * - 模块间通过对象接口或兼容 API 协作，避免 UI 层直接操作显示控制器寄存器。
 * - 字体/字模数据文件以数组尺寸、字号和索引关系一致性为维护重点。
 * - 本文件已移除原子/ST 示例模板，统一使用项目自定义 UTF-8 文件模板。
 * @encoding UTF-8
 */

#ifndef __FONTS_H__
#define __FONTS_H__

#include <stdint.h>

#define __FONTS 2

typedef struct _tFont
{
    const unsigned char (*table)[64];
    uint8_t Height;
    uint8_t Width;
    uint8_t Mode;
} sFONT;

extern sFONT Font16x08;
#if __FONTS == 2
extern sFONT Font32x16;
extern sFONT Font16x08x;
extern sFONT Font24x12;
#endif

extern const unsigned char Font_1608[95][64];
extern const unsigned char Font_1608x[95][64];
extern const unsigned char Font_3216[95][64];
extern const unsigned char Font_2412[195][64];
extern const unsigned char asc2_1608[95][16];
extern const unsigned char asc2_2412[95][36];
extern const unsigned char asc2_3216[95][64];

#endif
