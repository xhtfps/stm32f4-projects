/* ****************************
 * Project description:
 *
 * Author: 创新基地 -> 2025 Lzj
 *
 * Creation Date: 2025/7/1 - 1
 * ****************************/
#include "stm32_config.h"
#include "stdio.h"
#include "led.h"
#include "lcd.h"
#include "AD9959.h"
#include "key.h"
#include "timer.h"
#include "task_manage.h"

extern u8 _return;

u8 SweepFlag = 0;		//是否开启扫频

int main(void)
{
	uint16_t  i=0;
	//USARTx_Init(115200);
	USART3_Init(115200);
	delay_init(72);		//初始化延时函数
	Init_AD9959();		//初始化引脚以及初始化9959参数
	LED_Init();         //初始化LED
	time1Init();
	//开启扫频
	if(SweepFlag==1)
	{
		Task3_SweepFre(1000,1000000,10);			//初始化扫频使用的指数码表
		TIM_Cmd(TIM4, ENABLE); //开启扫频
	}
//	//不开启扫频
	if(SweepFlag==0)
	{
		Task0_Fre(1000000,1000000,1000000,1000000);
		Task1_Amp(200,300,400,500);
		Task2_Pha(0,90,180,270);
	}
		
  delay_ms(300);

	while(1)
	{

	}	
}

