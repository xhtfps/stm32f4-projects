#include "Drive_Communication.h"
#include "TFT_LCD.h"
#include <string.h>

#define DDS_PROTOCOL_LENGTH 38U
#include "delay.h"    // 补充delay_ms依赖头文件

/************************ 全局变量定义 ************************/
// 串口通信收发缓存结构体
communicationDataStruct communicationData = {{0},{0}};

// 两路DDS参数存储数组
DDSDataStruct dds[2]; 

/**
 * @brief CRC16校验计算函数 标准MODBUS多项式 0xA001
 * @param Datas 待校验数据缓冲区指针
 * @param Length 待校验数据长度
 * @retval u16 16位CRC校验值
 */
static u16 crc_16(u8 *Datas, u16 Length)
{
    u8 j;
    u16 temp = 0xFFFF, i;

    for(i = 0; i < Length; ++i)
    {
        temp ^= Datas[i];

        for(j = 0; j < 8; ++j)
        {
            if(temp & 1)
            {
                temp >>= 1;
                temp ^= 0xA001;
            }
            else
            {
                temp >>= 1;
            }
        }
    }

    return temp;
}

/**
 * @brief USART1单字节阻塞发送一组数据
 * @param sendData 待发送数据缓冲区
 * @param length 发送字节长度
 * @retval 无
 */
static void usartSendData(u8 *sendData, u16 length)
{
    u16 i;
    
    for(i = 0 ; i < length ; i++)
    {
        USART_SendData(USART1, sendData[i]);
        // 等待发送数据寄存器空
        while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
    }
    // 等待全部数据发送完成
    while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
}

/**
 * @brief 打包DDS参数并通过串口发送完整协议帧
 * @param data 单路DDS配置参数结构体
 * @param ch DDS通道号 0/1
 * @retval 无
 */
void sendData(DDSDataStruct data, u8 ch)     
{
    u16 crcValue;
    // 幅值放大1000倍转为整数存储
    u32 range = (u32)(data.range * 1000);
    // 相位放大100倍转为整数存储
    u32 phase = (u32)(data.phase * 100);

    // 清空发送缓存
    memset(communicationData.txBuffer, 0, sizeof(communicationData.txBuffer));
    
    // 帧头
    communicationData.txBuffer[0]  = 0xAA;
    communicationData.txBuffer[1]  = 0xA5;

    // 工作模式 高字节 低字节
    communicationData.txBuffer[2]  = (u8)(((u16)data.mode >> 8) & 0xFFU);
    communicationData.txBuffer[3]  = (u8)(data.mode & 0xFF);  

    // 通道号
    communicationData.txBuffer[4]  = ch;
    // 输出使能
    communicationData.txBuffer[5]  = data.output;  

    // 输出频率 4字节大端
    communicationData.txBuffer[6]  = (u8)(data.fre >> 24);
    communicationData.txBuffer[7]  = (u8)((data.fre >> 16) & 0xFF);
    communicationData.txBuffer[8]  = (u8)((data.fre >> 8) & 0xFF);
    communicationData.txBuffer[9]  = (u8)(data.fre & 0xFF);

    // 幅值参数
    communicationData.txBuffer[10] = (u8)(range >> 24);
    communicationData.txBuffer[11] = (u8)((range >> 16) & 0xFF);
    communicationData.txBuffer[12] = (u8)((range >> 8) & 0xFF);
    communicationData.txBuffer[13] = (u8)(range & 0xFF);  

    // 相位参数
    communicationData.txBuffer[14] = (u8)(phase >> 24);
    communicationData.txBuffer[15] = (u8)((phase >> 16) & 0xFF);
    communicationData.txBuffer[16] = (u8)((phase >> 8) & 0xFF);
    communicationData.txBuffer[17] = (u8)(phase & 0xFF);  

    // 扫频步进频率
    communicationData.txBuffer[18] = (u8)(data.step >> 24);
    communicationData.txBuffer[19] = (u8)((data.step >> 16) & 0xFF);
    communicationData.txBuffer[20] = (u8)((data.step >> 8) & 0xFF);
    communicationData.txBuffer[21] = (u8)(data.step & 0xFF);  

    // 步进延时 单位us
    communicationData.txBuffer[22] = (u8)(data.step_time >> 24);
    communicationData.txBuffer[23] = (u8)((data.step_time >> 16) & 0xFF);
    communicationData.txBuffer[24] = (u8)((data.step_time >> 8) & 0xFF);
    communicationData.txBuffer[25] = (u8)(data.step_time & 0xFF);  

    // 扫频起始频率
    communicationData.txBuffer[26] = (u8)(data.fre_start >> 24);
    communicationData.txBuffer[27] = (u8)((data.fre_start >> 16) & 0xFF);
    communicationData.txBuffer[28] = (u8)((data.fre_start >> 8) & 0xFF);
    communicationData.txBuffer[29] = (u8)(data.fre_start & 0xFF);  

    // 扫频终止频率
    communicationData.txBuffer[30] = (u8)(data.fre_stop >> 24);
    communicationData.txBuffer[31] = (u8)((data.fre_stop >> 16) & 0xFF);
    communicationData.txBuffer[32] = (u8)((data.fre_stop >> 8) & 0xFF);
    communicationData.txBuffer[33] = (u8)(data.fre_stop & 0xFF);  
    communicationData.txBuffer[34] = 1U;

    // 计算CRC校验 校验范围：帧头至扫频终止频率
    crcValue = crc_16(&communicationData.txBuffer[0], DDS_PROTOCOL_LENGTH - 3U);
    communicationData.txBuffer[DDS_PROTOCOL_LENGTH - 3U] = crcValue >> 8;
    communicationData.txBuffer[DDS_PROTOCOL_LENGTH - 2U] = crcValue & 0xFF;
    
    // 帧尾标识
    communicationData.txBuffer[DDS_PROTOCOL_LENGTH - 1U] = 0x55;
    
    // 发送完整串口帧
    usartSendData(communicationData.txBuffer, DDS_PROTOCOL_LENGTH);
}

/**
 * @brief 初始化USART1 PA9(TX) PA10(RX)
 * @param bound 波特率
 * @retval 无
 */
void Init_Uart(u32 bound)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
	
    // 开启GPIOA、USART1时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
 
	// PA9 PA10复用映射为USART1
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);
	
	// GPIO初始化配置
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        // 复用功能模式
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	// IO速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      // 推挽输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        // 上拉
	GPIO_Init(GPIOA, &GPIO_InitStructure);

    // USART1参数配置
	USART_InitStructure.USART_BaudRate = bound;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART1, &USART_InitStructure);
	
    // 使能串口外设
    USART_Cmd(USART1, ENABLE);
}

/**
 * @brief DDS两路参数默认初始化，并下发配置
 * @retval 无
 */
void DDSDataInit()
{
    // 输出幅值 500mV (即0.5V)
    dds[0].range = dds[1].range = 500.0f;
	
    // 单点输出频率 1kHz
    dds[0].fre = dds[1].fre = 1000;
	
    // 相位偏移
    dds[0].phase = 0;
	dds[1].phase = 90;
		
    // 扫频步进 500kHz
    dds[0].step = dds[1].step = 500000;
	
    // 步进稳定延时 10000us
    dds[0].step_time = dds[1].step_time = 10000;
	
    // 扫频起始1MHz
	dds[0].fre_start = dds[1].fre_start = 1000000;
 
    // 扫频终止40MHz
	dds[0].fre_stop = dds[1].fre_stop = 40000000;
	
    // 默认正常单点输出模式
	dds[0].mode = dds[1].mode = NORMAL;
	
    // 通道输出使能
	dds[0].output = 1;
	dds[1].output = 1;
    // 分别下发通道0、通道1参数
    sendData(dds[0], 0); 
	delay_ms(100);
    sendData(dds[1], 1); 
}
