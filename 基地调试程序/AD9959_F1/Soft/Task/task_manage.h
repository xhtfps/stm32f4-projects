#ifndef __task_manage_H
#define __task_manage_H
#include "stm32f10x.h"
#include "lcd.h"
#include "key.h"

#define Interface 3//界面总数
#define StrMax    10//缓存数据大小

extern u32 SysTimer;
extern u32 SinFre[5];
extern u32 SinAmp[5];
extern u32 SinPhr[5];
extern float SweepData[100];

u8 Task_Delay(u32 delay_time, u8* delay_ID);
u8 TaskCycleDelay(u32 delay_time, u8* Last_delay_ID, u8* Self_delay_ID);
void Main_Menu(u32 Key_Value, u8* Task_ID);
void LCD_Show_ModeCEInfo(u16 x0, u8 start_info, u8 current_deal_info);
void Set_PointFre(u32 Key_Value, u8* Task_ID);
void Sweep_Fre(u32 Key_Value, u8* Task_ID);
void Move_Pha(u32 Key_Value, u8* Task_ID);
void Jump_Fre(u32 Key_Value, u8* Task_ID);

void Task0_Fre(u32 fre0,u32 fre1,u32 fre2,u32 fre3);
void Task1_Amp(u32 Amp0,u32 Amp1,u32 Amp2,u32 Amp3);
void Task2_Pha(u32 Pha0,u32 Pha1,u32 Pha2,u32 Pha3);
void Task3_SweepFre(u32 SweepMinFre,u32 SweepMaxFre,u16 SweepTime);


#endif
