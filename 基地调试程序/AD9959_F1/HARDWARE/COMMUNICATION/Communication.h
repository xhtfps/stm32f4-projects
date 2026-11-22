#ifndef _COMMUNICATION_H
#define _COMMUNICATION_H

#include "stdint.h"
#include "stm32f10x.h"

#ifndef True
#define True 1
#endif

#ifndef False
#define False  0
#endif

/* 通信协议指令 */
#define    NORMAL         0x0001 //普通模式
#define    SWEEP          0x0002 //扫频模式
#define		 MODULATION			0x0004 //调制模式
#define		 INPHASE				0x0008 //同相输出模式


#define    RX_LENGTH      38
#define    TX_LENGTH      38
#define    DDS_UPDATE_BATCH_FLAG   1
#define    DDS_UPDATE_SINGLE_FLAG  2

typedef struct
{
    u8 rxBuffer[RX_LENGTH];
    u8 txBuffer[TX_LENGTH];
    u8 flag;					  //接收置位
}communicationDataStruct;

typedef struct DDSDataStruct
{
	u8  mode;      //输出模式
	u8  output;    //输出开关

	float range;   //幅度
	u32 phase;   //相位 	
	u32 fre;       //普通模式的频率

	u32 fre_start; //扫频模式的起始频率
	u32 fre_stop;	 //扫频模式的截止频率
	u32 step;      //步进
	u32 step_time; //步进时间(s)
	
	u8 flag;
}DDSDataStruct; 

extern DDSDataStruct dds[4];
extern communicationDataStruct communicationData; 

void rxDataprocess(void);


#endif


