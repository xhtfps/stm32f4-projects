#include "App_DAC.h"


float v;
u16 value;
float step =0.5;

void DAC_main()
{
	u8 key;
	float vol;
	Refresh();
	while(1)
	{
		key = KEY_Scan(0);
		Key_Handle(key);
		vol = TouchKey_Scan();
		Touch_Handle(vol);
		OSTimeDly(33);
	}
}

void Touch_Handle(float vol)
{
	if(vol != 0)
	{
		if(vol < 3.3f)
			v = vol;
		DAC_Out();
		Refresh();
	}
	else
		return;
	Refresh();
}

void DAC_Out() // DAC输出
{
	u16 value;
	// DAC1输出
	value = v / 3.3f *4095;
	DAC_SetChannel1Data(DAC_Align_12b_R, value);
}

void Refresh()	// 刷新电压值
{
	LCD_Clear(White);
	OS_Num_Show(30,90,32,1,v,"输出PA4 DAC1:  %.2fV");
	OS_String_Show(270,330,16,1,"THE QUICK BROWN FOX JUMPS OVER A LAZY DOG");
	OS_String_Show(270,360,16,1,"the quick brown fox jumps over a lazy dog");
	OS_String_Show(270,390,16,1,"那只敏捷的棕毛狐狸跃过那只懒狗。");
	OS_String_Show(270,420,16,1,"!\"#$%&'()*+,-./0123456789:;<=>?@[\\]^_`{|}~");
	TouchKey_Draw();//画小键盘
}








