/**
  ******************************************************************************
 //-----------------------------------------------------------------
// 程序描述:
// 　在Keil uVision开发平台下基于凌智核心开发板的AD7606_ADC模块程序（串行）
// 作　　者: 凌智电子
// 开始日期: 2017-07
// 完成日期: 2017-07
// 修改日期: 2017-07
// 版　　本: V1.0
// 　- V1.0: 通过程序使电压在串口显示
// 开发和调试工具: STM32F103核心板, 2.8寸液晶, 串口调试工具
 
连线：

//控制线 
rst 	 		  PB8
convstB 	  PB9
convstA 	  PB10
STby	      PB11
OS12 	      PB13
OS11 	      PB14
OS10 	      PB15

frstdata 	  PA4
busy 	  		PA5
cs 	  			PA6
rd 	  		  PA7

//数据线
DoutA(DB7)      PC0

//外接(串行模式选择以下两位)

SER             3.3V
D15             0 V


注意：
       (1) 串行：SER = 1；D15 = 0；
			 (2) 通过J3跳帽选择输入电压的范围，修改程序第124行中sprintf里面的(float)(DB_data[i]*5000.0/32768)的5000.0，在5000.0和10000.0根据跳帽改变。
			 (3) J2跳帽这里连接3.3V，具体选择看引脚功能。
//-----------------------------------------------------------------
  ******************************************************************************
  */

//-----------------------------------------------------------------
// 头文件包含
//-----------------------------------------------------------------
#include <stm32f10x.h>
#include <stdio.h>
#include "Delay.h"
#include "lze_lcd.h"
#include "AD7606.h"
#include "key.h"
#include "usb_lib.h"
#include "usb_desc.h"
#include "hw_config.h"
#include "usb_pwr.h"
#include "PeripheralInit.h"

//-----------------------------------------------------------------
// 全局变量
//-----------------------------------------------------------------
// 接收成功标志
extern u8 Resive_Flag;
// 接收缓冲
extern uint8_t USB_Rx_Buffer[VIRTUAL_COM_PORT_DATA_SIZE];

u8 IO_TAG=1;

//-----------------------------------------------------------------
// void USB_Send_string(u8* data_buffer)
//-----------------------------------------------------------------
//
// 函数功能:	USB发送字符串
// 入口参数:  发送字符串首地址
// 返 回 值:  无
// 注意事项:
//-----------------------------------------------------------------

void USB_Send_string(u8* data_buffer)
{  
  // 拷贝用户数据到端点1的发送缓冲区
  UserToPMABufferCopy(data_buffer, ENDP1_TXADDR, 64); 
	// 设置端点1发送数据长度
  SetEPTxCount(ENDP1, 64); 
	// 设置端点1的发送状态为Valid
  SetEPTxValid(ENDP1);		
}

//-----------------------------------------------------------------
// 主程序
//-----------------------------------------------------------------
int main(void)
{
	u8 dis_buf[40];
	u8 i;
	uint8_t  temp;
	s16 DB_data[8] = {0};
	
	PeripheralInit();													// 外设初始化
	LCD_SetBackColor(LCD_COLOR_BLACK);	  		// 设置背景色
	LCD_Clear(LCD_COLOR_BLACK);
	LCD_WriteString(0,20,LCD_COLOR_YELLOW,LCD_COLOR_BLACK,"STM32 AD7606 Channels Test:");
	LCD_WriteString(0, 40,LCD_COLOR_YELLOW,LCD_COLOR_BLACK," Channels Sampling:  ");
	while(1)
	{
		AD7606_startconvst();
		Delay_ns(1);
		temp = GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_5);			 // 读取 BUSY的状态 
	  while((temp == Bit_SET))				//当busy为低电平时，数据转换完毕，此时可以读取数据 
		{
			Delay_ns(10);
			temp = GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_5);		// 读取 BUSY的状态 	 
		}
		AD7606_read_data(DB_data); //读取数据放至数组DB_data[]
		for(i=0;i<8;i++)
		{
			sprintf((char*)dis_buf,"CH%1d:%8.1f mv  0x%04x %6d", i+1, (float)(DB_data[i]*10000.0/32768), (u16)(DB_data[i]^0x8000), (u16)(DB_data[i]^0x8000));
			LCD_WriteString(0,60+(i)*20,LCD_COLOR_RED,LCD_COLOR_BLACK,dis_buf );			//液晶显示
		}
		for(i=0;i<8;i++)
		{
			sprintf((char*)dis_buf,"CH%1d:%8.1f mv  0x%04x %6d\r\n", i+1, (float)(DB_data[i]*10000.0/32768), (u16)(DB_data[i]^0x8000), (u16)(DB_data[i]^0x8000));
			USB_Send_string(dis_buf);                  //在串口显示结果
			Delay_5ms(10);
		}	
		Delay_5ms(100);
	}									
}

//-----------------------------------------------------------------
// End Of File
//-----------------------------------------------------------------
