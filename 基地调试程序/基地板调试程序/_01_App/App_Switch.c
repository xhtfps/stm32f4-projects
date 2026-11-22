/*
*********************************************************************************************************
                                               _04_OS
    File			 : App_OSC.c
    By  			 : Muhe
    platform   : STM32F407ZG
	Data   		 : 2018/7/16
    function 	 : 用于启动和关闭任务
*********************************************************************************************************
*/
#include "App_Switch.h"



void Switch_main(void)
{
	GPIO_Key_Init();
	while(1)
	{
		if(Key_Now_Get(KEY1,KEY_MODE_SHORT))
		{
			OSC_Flag=0;
			OSTaskSuspend(3);
			OSTaskRecovery(4);
		}
		else if(Key_Now_Get(KEY2,KEY_MODE_SHORT))
		{
			AFG_Flag=0;
			OSTaskSuspend(4);
			OSTaskRecovery(3);
		}
		OSTimeDly(333);
	}
}

