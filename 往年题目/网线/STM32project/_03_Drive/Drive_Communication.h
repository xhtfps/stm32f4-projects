#ifndef DRIVE_COMMUNICATION_H
#define DRIVE_COMMUNICATION_H

#include "stdint.h"
#include "User_header.h"

// 布尔值宏定义
#ifndef True
#define True 1
#endif

#ifndef False
#define False  0
#endif

/************************ 通信协议模式指令定义 ************************/
#define NORMAL         0x0001  // 普通单点频输出模式
#define SWEEP          0x0002  // 线性扫频输出模式
#define MODULATION     0x0004  // 数字调制输出模式
#define INPHASE        0x0008  // 双通道同相输出模式

// 串口收发缓存字节长度
#define RX_LENGTH      48
#define TX_LENGTH      48

/**
 * @brief 串口通信数据结构体
 * rxBuffer：接收数据缓冲区
 * txBuffer：发送数据缓冲区
 * flag：接收完成标志位
 */
typedef struct communicationDataStruct
{
    u8 rxBuffer[RX_LENGTH];
    u8 txBuffer[TX_LENGTH];
    u8 flag;
} communicationDataStruct;

/**
 * @brief AD9958调制类型枚举
 */
typedef enum
{
    AD9958_DISABLE = 0,     // 关闭调制功能
    AD9958_AMPLITUDE,       // AM幅度调制
    AD9958_FREQUENCY,       // FM频率调制
    AD9958_PHASE,           // PM相位调制
} AFP_TypeDef;

/**
 * @brief 数字调制阶数选择枚举
 */
typedef enum
{
    TWO_LEVEL_MODULATION = 0x00,    // 2阶调制(2ASK/2FSK/2PSK)
    FOUR_LEVEL_MODULATION,          // 4阶调制
    EIGHT_LEVEL_MODULATION,         // 8阶调制
    SIXTEEN_LEVEL_MODULATION,       // 16阶调制
} ModulationLevel_TypeDef;

/**
 * @brief DDS通道完整参数配置结构体
 */
typedef struct DDSDataStruct
{
    u8  mode;              // 工作模式：NORMAL/SWEEP/MODULATION/INPHASE
    u8  output;            // 通道输出使能 1开启/0关闭

    float range;           // 输出信号幅度
    u32 phase;             // 信号初始相位
    u32 fre;               // 普通单点模式输出频率

    u32 fre_start;         // 扫频模式：起始频率
    u32 fre_stop;          // 扫频模式：终止频率
    u32 step;              // 扫频频率步进值
    u32 step_time;         // 扫频步进保持时间，单位us

    u8 modulation_mode;    // 调制类型，对应AFP_TypeDef
    u8 modulation_levels;  // 数字调制阶数，对应ModulationLevel_TypeDef

    float ask[16];         // ASK调制各阶幅度系数表
    float fsk[16];         // FSK调制各阶频偏表
    float psk[16];         // PSK调制各阶相位偏移表
} DDSDataStruct;

/************************ 对外函数声明 ************************/
/**
 * @brief 打包DDS参数并通过串口下发至下位机
 * @param data 单路DDS配置结构体
 * @param ch DDS通道编号 0/1
 */
void sendData(DDSDataStruct data, u8 ch);

/**
 * @brief 初始化USART1串口(PA9 TX/PA10 RX)
 * @param bound 串口波特率
 */
void Init_Uart(u32 bound);

/**
 * @brief DDS两路通道参数默认初始化，并自动下发配置
 */
void DDSDataInit(void);

//void Refresh(DDSDataStruct data,u8 choose,u8 ch,u8 inphase);

/************************ 全局外部变量声明 ************************/
extern DDSDataStruct dds[2];
extern communicationDataStruct communicationData;

#endif

/******************************************(C)  COPYRIGHT 2017 吴远 *********************************************************/
