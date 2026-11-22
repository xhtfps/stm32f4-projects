#ifndef __APP_CODE_H
#define __APP_CODE_H

#include "User_header.h"


#define SendOut PAout(1)

//#define SCIRXST  1 //   定义表示接收  不定义表示发送

#ifdef SCIRXST

extern u16 Get_Data;
extern u8  Get_Flag;

void Show_Data(void);

#else

extern u16 Send_Flag;
extern u8	Send_Data;
extern u8 dat_buf[10];

void DataSend(void);
void Code_Data(void);
void Show_Send(void);
#endif

void Code_main(void);


#endif
