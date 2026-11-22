/**
 * @file    FFT.h
 * @brief   STM32F407 可用的浮点基 2 FFT/逆 FFT 接口。
 *
 * 提供基于浮点运算的基 2 快速傅里叶变换及其配套辅助函数。
 * 适用于 Keil ARMCC5 + FreeRTOS 工程，全部操作在原数组上完成，无需堆内存。
 */

#ifndef __BSP_FFT_H
#define __BSP_FFT_H

#include <stddef.h>
#include <stdint.h>

/**
 * @def FFT_PI
 * @brief 圆周率 π（float 精度），用于旋转因子角度计算。
 */
#define FFT_PI                         3.14159265358979323846f

/**
 * @def FFT_HARMONIC_COUNT
 * @brief FFT_Harmonic() 最多可提取的谐波个数（不含直流）。
 *        谐波次数从 1 开始，最多支持 7 次。
 */
#define FFT_HARMONIC_COUNT             7U

/**
 * @def FFT_HARMONIC_OUTPUT_SIZE
 * @brief 谐波输出数组 harm[] 所需的最小 float 元素个数。
 *
 * 格式：
 *   - harm[0]           : 直流分量（已缩放）
 *   - harm[2*k]         : 第 k 次谐波幅值 (k = 1..7)
 *   - harm[2*k+1]       : 第 k 次谐波对应的频率索引
 * 因此总大小为 2 + 2*FFT_HARMONIC_COUNT = 16。
 */
#define FFT_HARMONIC_OUTPUT_SIZE      (2U + (FFT_HARMONIC_COUNT * 2U))

/**
 * @def FFT_HARMONIC_THRESHOLD
 * @brief 谐波提取的幅值阈值（基于 FFT 幅度谱的原始整数值）。
 *        只有当幅度大于此值时，才会被记录为有效谐波。
 */
#define FFT_HARMONIC_THRESHOLD         4500U

/**
 * @def PI
 * @brief 兼容旧代码中直接使用 PI 的情况，实际值同 FFT_PI。
 *        新代码建议直接使用 FFT_PI。
 */
#ifndef PI
#define PI                             FFT_PI
#endif

/**
 * @brief 复数数据结构体。
 * @note 实部与虚部均为单精度浮点。
 */
typedef struct
{
    float real; /**< 实部 */
    float imag; /**< 虚部 */
} complex;

/**
 * @brief  检测输入长度是否为基 2 FFT 的有效长度（1, 2, 4, 8...）。
 * @param  length 待检测长度。
 * @retval 1      有效（2 的幂次）。
 * @retval 0      无效（0 或非 2 的幂次）。
 */
uint8_t FFT_IsValidLength(uint16_t length);

/**
 * @brief  原地浮点基 2 FFT（时域抽取算法）。
 * @param  length FFT 点数，必须为 2 的幂次。
 * @param  data   复数数组，输入为时域信号，输出为频域结果（原地变换）。
 * @note   若指针为 NULL 或长度非法，函数直接返回。
 */
void fft(uint16_t length, complex data[]);

/**
 * @brief  原地浮点基 2 IFFT（逆 FFT）。
 * @param  length 点数（2 的幂次）。
 * @param  data   复数数组，输入为频域数据，输出为时域结果（原地变换）。
 * @note   内部通过共轭 -> FFT -> 共轭 -> 缩放实现，无需额外蝶形代码。
 */
void ifft(uint16_t length, complex data[]);

/**
 * @brief  计算复数数组的共轭。
 * @param  length  数组长度。
 * @param  input   输入复数数组。
 * @param  output  输出复数数组（可与 input 相同实现原地共轭）。
 * @note   若任一指针为 NULL，函数直接返回。
 */
void conjugate_complex(uint16_t length, complex input[], complex output[]);

/**
 * @brief  计算复数数组各元素的模值。
 * @param  data   输入复数数组。
 * @param  output 输出浮点数组（存放模值）。
 * @param  length 数组长度。
 */
void c_abs(complex data[], float output[], uint16_t length);

/**
 * @brief  复数加法：result = a + b。
 * @param  a, b   操作数。
 * @param  result 结果指针（必须非空）。
 */
void c_plus(complex a, complex b, complex *result);

/**
 * @brief  复数减法：result = a - b。
 * @param  a, b   操作数。
 * @param  result 结果指针（必须非空）。
 */
void c_sub(complex a, complex b, complex *result);

/**
 * @brief  复数乘法：result = a * b。
 * @param  a, b   操作数。
 * @param  result 结果指针（必须非空）。
 */
void c_mul(complex a, complex b, complex *result);

/**
 * @brief  复数除法：result = a / b。
 * @param  a, b   操作数。
 * @param  result 结果指针（必须非空）。
 * @note   若分母为零，结果置零。
 */
void c_div(complex a, complex b, complex *result);

/**
 * @brief  生成旋转因子 W_N^i = e^(±j*2π*i/N)。
 * @param  n        DFT 点数。
 * @param  index    i（指数）。
 * @param  wn       输出旋转因子（指针必须非空）。
 * @param  inverse  非 0 时生成 IFFT 用的共轭旋转因子（虚部取反）。
 */
void Wn_i(uint16_t n, uint16_t index, complex *wn, char inverse);

/**
 * @brief  计算单个复数的模（与 c_abs 数组版本对应）。
 * @param  value 输入复数。
 * @return 模值（float）。
 */
float complex_abs_float(complex value);

/**
 * @brief  从 FFT 幅度谱中提取直流分量和主要谐波。
 *
 * 遍历幅度谱的前半部分（1 至 length/2-1），将幅值超过
 * FFT_HARMONIC_THRESHOLD 且次数最低的 FFT_HARMONIC_COUNT 个
 * 谐波记录到 harm 数组中。直流分量单独存入 harm[0]。
 *
 * @param  harm      输出数组，长度至少为 FFT_HARMONIC_OUTPUT_SIZE。
 *                   格式见 FFT_HARMONIC_OUTPUT_SIZE 说明。
 * @param  magnitude 输入幅度谱数组（通常为 FFT 结果取模后的前 length 点）。
 * @param  length    原始 FFT 长度。
 */
void FFT_Harmonic(float harm[],
                  uint16_t magnitude[],
                  uint16_t length);

#endif /* __BSP_FFT_H */
