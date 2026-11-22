/* ****************************
 * Project description:
 *
 * Author: 创新基地 -> 2025 Lzj
 *
 * Creation Date: 2025/7/1 - 1
 * ****************************/
#include "task_manage.h"
#include "delay.h"
#include "key.h"
#include "AD9959.h"
#include "math.h"
#include <stdio.h>
#include <ctype.h>
#include <cstring>


#define OUT_KEY  GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_2)//读取按键0
#define FLASH_SAVE_ADDR  0x0801F000  				//设置FLASH 保存地址(必须为偶数)

u8 Task_Index = 0;//界面切换
u8 Param_Mode = 0;//参数类型切换
u8 fre_buf[StrMax]; //参数转换字符缓存
u8 display[StrMax]; //参数显示缓存
u8 P_Index = 0; //编辑位置
u8 Task_First=1;//第一次进入标记
u8 _return=0;
u32 SinFre[5] = {5000, 5000, 5000, 5000};
u32 SinAmp[5] = {1023, 1023, 1023, 1023};
u32 SinPhr[5] = {0, 4095, 4095*3, 4095*2};
u32 SinFre0[4] = {1000};
//扫频参数

u32 SweepStepFre = 100;
float SweepData[100]={0};



u8 Task_Delay(u32 delay_time, u8* delay_ID)
{
	static u8 Time_Get = 0;
	static u32 Time_Triger;
    if(Time_Get == 0)
    {
      Time_Triger = SysTimer + delay_time;
      Time_Get = 1;
    }
    if(SysTimer >= Time_Triger)
    { 
      Time_Get = 0;
			*delay_ID = 1;		//	后续代码已被执行一遍
			return 1;
    }
		return 0;
}


u8 TaskCycleDelay(u32 delay_time, u8* Last_delay_ID, u8* Self_delay_ID)
{
	static u8 Time_Get = 0;
	static u32 Time_Triger;
	
	if(!(*Last_delay_ID))
		return 0;
	if(Time_Get == 0)
	{
		Time_Triger = SysTimer + delay_time;
		Time_Get = 1;
	}
	if(SysTimer >= Time_Triger)
	{ 
		Time_Get = 0;
		*Last_delay_ID = 0;
		*Self_delay_ID = 1;		//	后续代码已被执行一遍
		return 1;
	}
	return 0;
}




void Task0_Fre(u32 fre0,u32 fre1,u32 fre2,u32 fre3)//正弦波 (10M) 0-100 000 000
{
	
	u8 Showchar[StrMax];
	Param_Mode = Param_Mode%4;			//设置通道0-3

		//先对右值数据限制再进行赋值
		if(fre0>500000000)
			fre0 = 500000000;
		if(fre1>500000000)
			fre1 = 500000000;
		if(fre2>500000000)
			fre2 = 500000000;
		if(fre3>500000000)
			fre3 = 500000000;
		
		//一次性给四个通道赋值				改等式2右值，改成串口接收到的数据
		SinFre[0] = fre0;
		SinFre[1] = fre1;
		SinFre[2] = fre2;
		SinFre[3] = fre3;
		
		//处理数据写入频率数据
		Write_frequence(0,SinFre[0]); 
		Write_frequence(1,SinFre[1]);
		Write_frequence(2,SinFre[2]);
		Write_frequence(3,SinFre[3]);
	
		_return=1;

}

void Task1_Amp(u32 Amp0,u32 Amp1,u32 Amp2,u32 Amp3)
{
		//加上限制条件
		if(Amp0>576)
			Amp0 = 576;
		if(Amp1>576)
			Amp1 = 576;
		if(Amp2>576)
			Amp2 = 576;
		if(Amp3>576)
			Amp3 = 576;
		//给各个通道赋值
		SinAmp[0] = Amp0;
		SinAmp[1] = Amp1;
		SinAmp[2] = Amp2;
		SinAmp[3] = Amp3;

	//处理幅值相位数据写入
		Write_Amplitude(3, SinAmp[3]);
		Write_Amplitude(0, SinAmp[0]);
		Write_Amplitude(1, SinAmp[1]);
		Write_Amplitude(2, SinAmp[2]);
	
	_return=1;
}

void Task2_Pha(u32 Pha0,u32 Pha1,u32 Pha2,u32 Pha3)
{
			if(Pha0 > 360)
				Pha0 = 360;
			if(Pha1 > 360)
				Pha1 = 360;
			if(Pha2 > 360)
				Pha2 = 360;
			if(Pha3 > 360)
				Pha3 = 360;
			
			SinPhr[0] = Pha0;	
			SinPhr[1] = Pha1;	
			SinPhr[2] = Pha2;		
			SinPhr[3] = Pha3;	

			Write_Phase(4, SinPhr[0]);//根据通道0先同步相位
			Write_Phase(0, SinPhr[0]);
			Write_Phase(1, SinPhr[1]);
			Write_Phase(2, SinPhr[2]);
			Write_Phase(3, SinPhr[3]);
		
}

void Task3_SweepFre(u32 SweepMinFre,u32 SweepMaxFre,u16 SweepTime)//扫频
{
		float ex,P,T,Frecurrent,temp;
		u16 i=0;
		
		if(SweepTime > 100)
			SweepTime =100;
		Timerx_Init(625*SweepTime-1,1151);	//定时器4用于控制扫频时间
	
		temp=log10((SweepMaxFre/SweepMinFre)/1);
		P=pow(10,temp);
		
		for(i=0;i<100;i++)
		{
																			//计算T
			SweepData[i]=pow(P,(0.01*i+log10(SweepMinFre)/log10(P)));
		}

		_return=1;

}




