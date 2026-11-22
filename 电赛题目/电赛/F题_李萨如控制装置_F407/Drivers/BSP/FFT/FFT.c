/**
 * @file    FFT.c
 * @brief   STM32F407 浮点基 2 FFT/逆 FFT 实现。
 *
 * 算法在原数组上完成变换，不申请堆内存，适合当前 Keil ARMCC5 + FreeRTOS
 * 工程。相较旧实现，去掉了循环内 pow()/sin()/cos() 调用，并增加了长度和
 * 空指针保护。
 */

#include "FFT.h"
#include <math.h>

/**
 * @brief  交换两个复数变量。
 * @param  left  指向第一个复数的指针。
 * @param  right 指向第二个复数的指针。
 */
static void FFT_Swap(complex *left, complex *right)
{
    complex temporary;

    temporary = *left;
    *left = *right;
    *right = temporary;
}

/**
 * @brief  检测输入长度是否为 2 的整数次幂（基 2 FFT 要求）。
 * @param  length 待检测长度。
 * @return 1：是 2 的幂；0：不是（或 length == 0）。
 */
uint8_t FFT_IsValidLength(uint16_t length)
{
    if (length == 0U)
    {
        return 0U;
    }

    /* 不断移除最低位的 1，若最后结果为 1 说明仅有一个 1 位 */
    while ((length & 1U) == 0U)
    {
        length >>= 1U;
    }

    return (length == 1U) ? 1U : 0U;
}

/**
 * @brief  计算复数的模（幅度）。
 * @param  value 输入复数。
 * @return 模值（float）。
 */
float complex_abs_float(complex value)
{
    return sqrtf((value.real * value.real) +
                 (value.imag * value.imag));
}

/**
 * @brief  计算复数数组的共轭。
 * @param  length  数组长度。
 * @param  input   输入复数数组。
 * @param  output  输出复数数组（可与 input 相同，实现原地共轭）。
 * @note   若任一指针为 NULL，直接返回。
 */
void conjugate_complex(uint16_t length, complex input[], complex output[])
{
    uint16_t index;

    if ((input == NULL) || (output == NULL))
    {
        return;
    }

    for (index = 0U; index < length; index++)
    {
        output[index].real = input[index].real;
        output[index].imag = -input[index].imag;  /* 共轭：虚部取反 */
    }
}

/**
 * @brief  计算复数数组各元素的模，存入浮点数组。
 * @param  data   输入复数数组。
 * @param  output 输出浮点数组。
 * @param  length 数组长度。
 */
void c_abs(complex data[], float output[], uint16_t length)
{
    uint16_t index;

    if ((data == NULL) || (output == NULL))
    {
        return;
    }

    for (index = 0U; index < length; index++)
    {
        output[index] = complex_abs_float(data[index]);
    }
}

/* ──────── 基本复数运算（加法、减法、乘法、除法）──────── */

/**
 * @brief  复数加法：result = a + b。
 */
void c_plus(complex a, complex b, complex *result)
{
    if (result == NULL)
    {
        return;
    }

    result->real = a.real + b.real;
    result->imag = a.imag + b.imag;
}

/**
 * @brief  复数减法：result = a - b。
 */
void c_sub(complex a, complex b, complex *result)
{
    if (result == NULL)
    {
        return;
    }

    result->real = a.real - b.real;
    result->imag = a.imag - b.imag;
}

/**
 * @brief  复数乘法：result = a * b。
 */
void c_mul(complex a, complex b, complex *result)
{
    if (result == NULL)
    {
        return;
    }

    result->real = (a.real * b.real) - (a.imag * b.imag);
    result->imag = (a.real * b.imag) + (a.imag * b.real);
}

/**
 * @brief  复数除法：result = a / b。
 * @note   若分母为 0，结果置零，避免除零错误。
 */
void c_div(complex a, complex b, complex *result)
{
    float denominator;

    if (result == NULL)
    {
        return;
    }

    denominator = (b.real * b.real) + (b.imag * b.imag);
    if (denominator == 0.0f)
    {
        result->real = 0.0f;
        result->imag = 0.0f;
        return;
    }

    result->real = ((a.real * b.real) + (a.imag * b.imag)) / denominator;
    result->imag = ((a.imag * b.real) - (a.real * b.imag)) / denominator;
}

/**
 * @brief  生成旋转因子 W_N^i。
 * @param  n        DFT 点数。
 * @param  index    i（指数）。
 * @param  wn       输出旋转因子（复数指针）。
 * @param  inverse  非 0 表示逆变换（IFFT），虚部取反。
 * @note   直接调用 sinf/cosf，适用于少量旋转因子计算，不在主 FFT 循环内使用。
 */
void Wn_i(uint16_t n, uint16_t index, complex *wn, char inverse)
{
    float angle;

    if ((n == 0U) || (wn == NULL))
    {
        return;
    }

    angle = (2.0f * FFT_PI * (float)index) / (float)n;
    wn->real = cosf(angle);
    wn->imag = (inverse != 0) ? -sinf(angle) : sinf(angle);
}

/* ──────────────── 基 2 时域抽取 FFT ──────────────── */

/**
 * @brief  原位浮点基 2 FFT（时域抽取算法）。
 * @param  length  FFT 点数，必须为 2 的幂且 ≤ 硬件可承受。
 * @param  data    输入/输出复数数组（FFT 完成后为频域结果）。
 *
 * 算法步骤：
 *   1. 位逆序重排（Bit-reversal permutation）
 *   2. 逐级蝶形运算（每一级使用递推计算旋转因子，避免重复 sin/cos）
 *
 * @note   若 data 为 NULL 或 length 非法，直接返回不做任何操作。
 */
void fft(uint16_t length, complex data[])
{
    uint32_t index;
    uint32_t reversed;        /* 位逆序索引 */
    uint32_t bit;
    uint32_t block;           /* 当前级的分块起始 */
    uint32_t offset;          /* 块内偏移 */
    uint32_t half_length;     /* 当前级蝶形跨度的一半 */
    uint32_t stage_length;    /* 当前级的子序列长度 */
    complex even;             /* 蝶形中“偶”点 */
    complex odd;              /* 蝶形中“奇”点 */
    float twiddle_real;       /* 当前旋转因子实部 */
    float twiddle_imag;       /* 当前旋转因子虚部 */
    float step_real;          /* 当前级旋转因子步进 (W_N^1) 实部 */
    float step_imag;          /* 当前级旋转因子步进虚部 */
    float temporary_real;     /* 临时变量 */
    float temporary_imag;
    float angle;              /* 步进角度 */

    /* 参数有效性检查 */
    if ((data == NULL) || (FFT_IsValidLength(length) == 0U))
    {
        return;
    }

    /* ---------- 1. 位逆序重排 ---------- */
    reversed = 0U;
    for (index = 1U; index < (uint32_t)length; index++)
    {
        bit = (uint32_t)length >> 1U;         /* 最高位 */
        while ((reversed & bit) != 0U)        /* 找到 reversed 中最低为 0 的位 */
        {
            reversed ^= bit;                  /* 将该位置 0 */
            bit >>= 1U;
        }
        reversed ^= bit;                      /* 将该位置 1 */

        /* 仅当 index < reversed 时交换，避免重复交换 */
        if (index < reversed)
        {
            FFT_Swap(&data[index], &data[reversed]);
        }
    }

    /* ---------- 2. 逐级蝶形运算 ---------- */
    /* stage_length 从 2 开始，每次加倍，直到等于 length */
    for (stage_length = 2U;
         stage_length <= (uint32_t)length;
         stage_length <<= 1U)
    {
        half_length = stage_length >> 1U;

        /* 计算当前级的旋转因子步进 W_N^1 = e^{-j*2π/stage_length} */
        angle = (-2.0f * FFT_PI) / (float)stage_length;
        step_real = cosf(angle);
        step_imag = sinf(angle);

        /* 遍历每个分块 */
        for (block = 0U; block < (uint32_t)length; block += stage_length)
        {
            /* 旋转因子从 W_N^0 = 1 开始，逐步乘上 step */
            twiddle_real = 1.0f;
            twiddle_imag = 0.0f;

            /* 块内蝶形 */
            for (offset = 0U; offset < half_length; offset++)
            {
                even = data[block + offset];                   /* 偶序号点 */
                odd  = data[block + offset + half_length];    /* 奇序号点 */

                /* 计算 twiddle * odd */
                temporary_real = (twiddle_real * odd.real) -
                                  (twiddle_imag * odd.imag);
                temporary_imag = (twiddle_real * odd.imag) +
                                  (twiddle_imag * odd.real);

                /* 蝶形：上部 = even + temp，下部 = even - temp */
                data[block + offset].real = even.real + temporary_real;
                data[block + offset].imag = even.imag + temporary_imag;
                data[block + offset + half_length].real = even.real -
                                                           temporary_real;
                data[block + offset + half_length].imag = even.imag -
                                                           temporary_imag;

                /* 递推下一个旋转因子：twiddle *= step */
                temporary_real = (twiddle_real * step_real) -
                                  (twiddle_imag * step_imag);
                twiddle_imag = (twiddle_real * step_imag) +
                               (twiddle_imag * step_real);
                twiddle_real = temporary_real;
            }
        }
    }
}

/**
 * @brief  原位浮点基 2 IFFT（逆 FFT）。
 * @param  length 点数（2 的幂）。
 * @param  data   输入/输出复数数组（IFFT 后为时域结果）。
 *
 * 实现方法：
 *   1. 对输入取共轭
 *   2. 调用 fft()
 *   3. 对结果取共轭
 *   4. 每个元素除以 length
 * 等效于标准逆变换公式，避免了重新编写蝶形。
 */
void ifft(uint16_t length, complex data[])
{
    uint16_t index;

    if ((data == NULL) || (FFT_IsValidLength(length) == 0U))
    {
        return;
    }

    /* 共轭 -> FFT -> 共轭 */
    conjugate_complex(length, data, data);
    fft(length, data);
    conjugate_complex(length, data, data);

    /* 缩放 1/N */
    for (index = 0U; index < length; index++)
    {
        data[index].real /= (float)length;
        data[index].imag /= (float)length;
    }
}

/**
 * @brief  谐波分析：从幅度谱中提取主要谐波分量。
 *
 * 将 FFT 的幅度谱（通常为前 half_length 个点）筛选出幅值超过阈值的谐波，
 * 并记录谐波次数和幅值到输出数组。
 *
 * @param  magnitude  输入幅度谱数组（长度为 length，仅使用前半部分）。
 * @param  harm       输出谐波信息数组，格式：
 *                    harm[0]               : 直流分量幅值（已除以 length/2）
 *                    harm[2*k]             : 第 k 次谐波幅值 (k = 1,2,...)
 *                    harm[2*k+1]           : 第 k 次谐波的索引（即频率编号）
 *                    harm 大小由 FFT_HARMONIC_OUTPUT_SIZE 定义。
 * @param  length     原始 FFT 长度。
 *
 * @note   阈值 FFT_HARMONIC_THRESHOLD、最大谐波数 FFT_HARMONIC_COUNT 等在头文件中定义。
 */
void FFT_Harmonic(float harm[], uint16_t magnitude[], uint16_t length)
{
    uint16_t index;
    uint16_t harmonic_index;   /* 当前提取到的谐波序号 */
    uint16_t half_length;

    if ((harm == NULL) || (magnitude == NULL) || (length < 2U))
    {
        return;
    }

    /* 初始化输出为 0 */
    for (index = 0U; index < FFT_HARMONIC_OUTPUT_SIZE; index++)
    {
        harm[index] = 0.0f;
    }

    half_length = length / 2U;

    /* 直流分量：harm[0] 存储直流幅值（归一化到长度的一半） */
    harm[0] = (float)magnitude[0] / (float)length / 2.0f;

    harmonic_index = 1U;  /* 谐波从 1 开始 */

    /* 遍历 1 到 half_length-1 的幅度值 */
    for (index = 1U;
         (index < half_length) && (harmonic_index <= FFT_HARMONIC_COUNT);
         index++)
    {
        /* 若幅度超过阈值，记录该次谐波 */
        if (magnitude[index] > FFT_HARMONIC_THRESHOLD)
        {
            harm[2U * harmonic_index] = (float)magnitude[index];      /* 幅值 */
            harm[(2U * harmonic_index) + 1U] = (float)index;          /* 谐波次数（索引） */
            harmonic_index++;
        }
    }
}
