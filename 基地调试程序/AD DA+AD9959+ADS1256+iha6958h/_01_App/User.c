/* ****************************
 * Project description:
 *
 * A Project empty template file
 *
 * Author: 创新基地 -> 2019 Mao
 *
 * Creation Date: 2021/09/05
 *
 * UpDate:
 * 1-> 加入了多个驱动程序
 * 3-> 新增部分注释
 * Update date: 2021/11/03
 * ****************************/

/* ***************************** Include & Define Part     	*****************************
 * 头文件声明及宏定义区
 * */
#include "User.h"
#include "User_header.h"
#include "lha6958h.h"

void AD9959_Drawselect(void);
void AD9959_Selet_Clear(void);
void AD9959_Changeval(uint8_t KeyNum);
uint8_t AD9959_NumScan(uint32_t Num);
void AD9959_Show(void);
void AD9959_Fre_Show(uint16_t x,uint16_t y,uint32_t fre);
float Read_Key(float Num);
void DAC_ValRefresh(void);
void DAC_Select(void);
float AD9959_SelectNum(void);
void AD9959_PopNum(float Num);
void Refresh(void);

/* ***************************** Variable definition Part   *****************************
 * 变量定义区
 * */

// ADC//DAC
float adc_val;
u16 adc;
u16 value;
float step = 0.5f;
uint8_t ADCstate=0;
float DAC_val[2];

float DAC1_val;
float DAC2_val;
//ADS1256
int ADS1256_ad;
float ADS1256_val;

//AD9959
uint32_t channel_fre[4]={50000,50000,50000,50000};
float channel_Amp[4] = {200.0f, 200.0f, 200.0f, 200.0f};
uint16_t channel_Pha[4]={0,0,0,0};

uint8_t state_channel;    //通道标志
uint8_t state_Allkindval; //参数选择


// 菜单序号变量
uint8_t MenuSign = 0;

/* ***************************** Main Part                  *****************************
 * 主函数区
 * */
void User_main(void)
{
	// 初始化全部
	Init_All();

	// 显示主界面
	Disp_Main();

	while (1)
	{
		switch (MenuSign)
		{
		case 0:
			if (Ps2KeyValue != KeyValue_Null) // 在未选中菜单时有按键按下
				Change_Menu(Ps2KeyValue);	  // 根据按键改变菜单界面
			;
			break;
		case 1:
			MenuHaddler_1();
			break;
		case 2:
			MenuHaddler_2();
			break;
		case 3:
			MenuHaddler_3();
			break;
		case 4:
			MenuHaddler_4();
			break;
//		case 5:
//			MenuHaddler_5();
//			break;
		default:
			break;
		}

		delay_ms(10);
	}
}

/* ***************************** Initialization Part        *****************************
 * 初始化函数区
 * */

// 初始化全部 参数：无
void Init_All()
{
	LCD_Clear(Black);

	 //AD DA初始化
	 ADC1_Init();    //PA1
	 ADC2_Init();    //PA2
	 ADC3_Init();    //PA3
	 DAC1_Init();    //PA4
	 DAC2_Init();    //PA5
}

/* ***************************** Display Part               *****************************
 * 显示函数区
 * */

// 显示界面框架 参数：无
void Disp_Main()
{
	uint8_t count;

	// Show title
	OS_String_Show(400 - 32 * 2, 16, 32, 0, TitleStr);
	
	// Draw line
	LCD_Appoint_Clear(0, 64, 800, 64 + 8, White);
	LCD_Appoint_Clear(0, 480 - 32 - 8, 800, 480 - 32, White);
	LCD_Appoint_Clear(250, 64 + 8, 250 + 2, 480 - 32 - 8, White);
	//LCD_Appoint_Clear( 300 + 2 , 96 + 64 * 4 , 800 , 96 + 64 * 4 + 2 , White );

	// Show model version str
	OS_String_Show(32, 480 - 16 - 8, 16, 0, ModelVerStr);
	// Show user version str
	OS_String_Show(632, 480 - 16 - 8, 16, 0, UserVerStr);

	// Disp menu
	for (count = 1; count < MenuChoiceNum + 1; count++)
		OS_String_Show(32, 32 + 64 * count, 32, 0, "—");
	for (count = 0; count < MenuChoiceNum; count++)
	{
		switch (count)
		{
		case 0:
			OS_String_Show(80, 96, 32, 0, Menu1Choice1);
			break;
		case 1:
			OS_String_Show(80, 96 + 64, 32, 0, Menu1Choice2);
			break;
		case 2:
			OS_String_Show(80, 96 + 64 * 2, 32, 0, Menu1Choice3);
			break;
		case 3:
			OS_String_Show(80, 96 + 64 * 3, 32, 0, Menu1Choice4);
			break;
//		case 4:
//			OS_String_Show(80, 96 + 64 * 4, 32, 0, Menu1Choice5);
//			break;
		default:
			break;
		}
	}
	OS_String_Show(400-3*24,480-24,24,1,"湖南理工大学");
}

// 输出数值 参数：1>位置；2>数值；3>输出格式,如"数值%0.0f"
void Show_Val(uint8_t location, float value, char *str)
{

	if (location > 0 && location <= 10)
		OS_Num_Show(250 + 64, 96 + 32 * (location - 1), 32, 1, value, str);
	else if (location > 10 && location <= 20)
		OS_Num_Show(500 + 64, 96 + 32 * (location - 11), 32, 1, value, str);
	else
		OS_String_Show(250 + 64, 96, 32, 1, "ERROR");
}

// 切换菜单页 参数：1>菜单序号(1~5)
void Change_Menu(uint8_t menu_sign)
{
	uint8_t count;

	// 清除显示区域
	LCD_Appoint_Clear(250 + 2, 64 + 8, 800, 480 - 32 - 8, Black);

	for (count = 1; count < MenuChoiceNum + 1; count++)
		OS_String_Show(32, 32 + 64 * count, 32, 1, "—");

	if (menu_sign > 0 && menu_sign <= MenuChoiceNum)
		OS_String_Show(32, 32 + 64 * menu_sign, 32, 1, "◆");
	else
		menu_sign = 0;

	Ps2KeyValue = KeyValue_Null;
	MenuSign = menu_sign;
}

/* ***************************** Menu Handler Part     	 	 	*****************************
 * 菜单执行函数区
 */

// 菜单1执行函数 参数：无
void MenuHaddler_1()
{
	Ps2KeyValue = KeyValue_Null;
	//ADC数值进行显示
	Refresh();
	DAC_Select();
	OS_Num_Show(250+2+16+24,64+8+16+32+16,32,1,adc_val,"PA1 ADC1:  %.6fV");
	OS_Num_Show(250+2+16+24,64+8+16+32+16+32+16,32,1,adc_val,"PA2 ADC2:  %.6fV");
	OS_Num_Show(250+2+16+24,64+8+16+32+16+32+16+32+16,32,1,adc_val,"PA3 ADC3:  %.6fV");
	while (Ps2KeyValue != KeyValue_Back)
	{
		float Temp;
		//控制ADC转化速率
//		PGA2310_SetAv(31.5, 31.5);
//		OS_Num_Show(10 + 10 * 16 + 5 * 16 + 3 * 16 + 7 * 16 + 16 * 10, 480 - 40 + 5, 24, 1, Ps2KeyValue, "%0.0f");
		OS_String_Show(250+2+16,64+8+16,32,1,"AD");

		adc_val = ADC1_GetVoltage();
		OS_Num_Show(250+2+16+24,64+8+16+32+16,32,1,adc_val,"PA1 ADC1:  %.6fV");

		adc_val = ADC3_GetVoltage();
		OS_Num_Show(250+2+16+24,64+8+16+32+16+32+16+32+16,32,1,adc_val,"PA3 ADC3:  %.6fV");	

		adc_val = ADC2_GetVoltage();
		OS_Num_Show(250+2+16+24,64+8+16+32+16+32+16,32,1,adc_val,"PA2 ADC2:  %.6fV");

		if(Ps2KeyValue != KeyValue_Null)
		{
			if(Ps2KeyValue<=KeyValue_Point)
			{
				Temp=Read_Key(DAC_val[ADCstate]);
				if(Temp>3.3f)Temp=3.3f;
				if(Temp<0.0f)Temp=0.0f;
				DAC_val[ADCstate]=Temp;
				DAC_ValRefresh();
			}
			else if(Ps2KeyValue==KeyValue_Add)
			{
				DAC_val[ADCstate]+=0.5f;
				if(DAC_val[ADCstate]>3.3f)DAC_val[ADCstate]=3.3f;
				Ps2KeyValue = KeyValue_Null;
				DAC_ValRefresh();
			}
			else if(Ps2KeyValue==KeyValue_Minus)
			{
				DAC_val[ADCstate]-=0.5f;
				if(DAC_val[ADCstate]<0.0f)DAC_val[ADCstate]=0.0f;
				Ps2KeyValue = KeyValue_Null;
				DAC_ValRefresh();
			}
			else if(Ps2KeyValue==KeyValue_NumLock)
			{
				//DAC选择更换
				ADCstate++;
				ADCstate%=2;
				DAC_Select();
				Ps2KeyValue = KeyValue_Null;
			}
		}
		//DAC输出
		DAC1_Out(DAC_val[0]);
		DAC2_Out(DAC_val[1]);
		delay_ms(100);
	}

	Change_Menu(0);
}

// 菜单2执行函数 参数：无
void MenuHaddler_2()
{
	Ps2KeyValue = KeyValue_Null;
	Init_AD9959();             
	AD9959_Drawselect();
	AD9959_Show();
	while (Ps2KeyValue != KeyValue_Back)
	{
		float Temp;
		uint8_t index;
		//选择通道的数值进行更新
		if(Ps2KeyValue==KeyValue_NumLock)
		{
			state_Allkindval++;
			state_Allkindval%=3;
			Ps2KeyValue=KeyValue_Null;
			AD9959_Drawselect();
		}
		//选择的通道进行更新
		else if(Ps2KeyValue==KeyValue_Div)	
		{
			state_channel++;
			state_channel%=4;
			Ps2KeyValue=KeyValue_Null;
			AD9959_Drawselect();
		}
		//按键为加或减时，对数值进行改变
		else if(Ps2KeyValue==KeyValue_Add||Ps2KeyValue==KeyValue_Minus)
		{
			AD9959_Changeval(Ps2KeyValue);
			Ps2KeyValue=KeyValue_Null;
		}
		//按键为0-9或小数点时，进行输入
		else if(Ps2KeyValue<=KeyValue_Point)
		{
			Temp=AD9959_SelectNum();
			Temp=Read_Key(Temp);
			index=AD9959_NumScan(Temp);
			if(index>=8)
			{
				LCD_Appoint_Clear(16, 96 + 64 * 4, 250 + 2+32, 480 - 32 - 8, Black);
				LCD_Appoint_Clear(250, 64 + 8, 250 + 2, 480 - 32 - 8, White);
			}
			AD9959_PopNum(Temp);
		}
		//AD9959界面进行显示
		AD9959_Show();
		delay_ms(10);
	}
	Change_Menu(0);
}

// 菜单3执行函数 参数：无
void MenuHaddler_3()
{
	//ADS1256初始化
	ADS1256_Init();
	Ps2KeyValue = KeyValue_Null;
	while (Ps2KeyValue != KeyValue_Back)
	{
		//不断更行获得ADS1256各通道的值，并进行更新
		ADS1256_ad = Moving_Average_Filter(ADS1256_MUXP_AIN0 | ADS1256_MUXN_AINCOM,5);
		ADS1256_val = Get_Val(ADS1256_ad);
		OS_String_Show(250+2+16,64+8+16+(32+16)*0,32,1,"ADS1256");
		OS_Num_Show(250+2+16,64+8+16+(32+16)*1,32,1,ADS1256_val,"通道0:  %.2fV ");
		ADS1256_ad = Moving_Average_Filter(ADS1256_MUXP_AIN1 | ADS1256_MUXN_AINCOM,5);
		ADS1256_val = Get_Val(ADS1256_ad);
		OS_Num_Show(250+2+16,64+8+16+(32+16)*2,32,1,ADS1256_val,"通道1:  %.2fV ");
		ADS1256_ad = Moving_Average_Filter(ADS1256_MUXP_AIN2 | ADS1256_MUXN_AINCOM,5);
		ADS1256_val = Get_Val(ADS1256_ad);
		OS_Num_Show(250+2+16,64+8+16+(32+16)*3,32,1,ADS1256_val,"通道2:  %.2fV ");
		ADS1256_ad = Moving_Average_Filter(ADS1256_MUXP_AIN3 | ADS1256_MUXN_AINCOM,5);
		ADS1256_val = Get_Val(ADS1256_ad);
		OS_Num_Show(250+2+16,64+8+16+(32+16)*4,32,1,ADS1256_val,"通道3:  %.2fV ");
		ADS1256_ad = Moving_Average_Filter(ADS1256_MUXP_AIN4 | ADS1256_MUXN_AINCOM,5);
		ADS1256_val = Get_Val(ADS1256_ad);
		OS_Num_Show(250+2+16,64+8+16+(32+16)*5,32,1,ADS1256_val,"通道4:  %.2fV ");	
		ADS1256_ad = Moving_Average_Filter(ADS1256_MUXP_AIN5 | ADS1256_MUXN_AINCOM,5);
		ADS1256_val = Get_Val(ADS1256_ad);
		OS_Num_Show(250+2+16,64+8+16+(32+16)*6,32,1,ADS1256_val,"通道5:  %.2fV ");
		ADS1256_ad = Moving_Average_Filter(ADS1256_MUXP_AIN6 | ADS1256_MUXN_AINCOM,5);
		ADS1256_val = Get_Val(ADS1256_ad);
		OS_Num_Show(250+2+16+16*16,64+8+16+(32+16)*1,32,1,ADS1256_val,"通道6:  %.2fV ");
		ADS1256_ad = Moving_Average_Filter(ADS1256_MUXP_AIN7 | ADS1256_MUXN_AINCOM,5);		
		ADS1256_val = Get_Val(ADS1256_ad);
		OS_Num_Show(250+2+16+16*16,64+8+16+(32+16)*2,32,1,ADS1256_val,"通道7:  %.2fV ");
		delay_ms(10);
	}
	Change_Menu(0);
}


// 菜单4执行函数 参数：无
void MenuHaddler_4()
{
//   static char *code_format[8] =
//   {
//       "通道1 采样值: %0.0f", "通道2 采样值: %0.0f", "通道3 采样值: %0.0f", "通道4 采样值: %0.0f",
//       "通道5 采样值: %0.0f", "通道6 采样值: %0.0f", "通道7 采样值: %0.0f", "通道8 采样值: %0.0f"
//   };
//   LHA6958H_Frame frame;
//   LHA6958H_Status status;
//   uint8_t channel;
//   uint16_t x;
//   uint16_t y;

//   Ps2KeyValue = KeyValue_Null;
//   status = LHA6958H_Init();
//   OS_String_Show(270, 88, 32, 1, "LHA6958H 八路ADC采集");
//   OS_String_Show(270, 128, 16, 1, "确认键:重新初始化   返回键:退出页面");

//   while (Ps2KeyValue != KeyValue_Back)
//   {
//       if (Ps2KeyValue == KeyValue_Enter)
//       {
//           Ps2KeyValue = KeyValue_Null;
//           status = LHA6958H_Init();
//       }

//       if (status == LHA6958H_OK)
//           status = LHA6958H_Sample(&frame, 1000U);

//       if (status == LHA6958H_OK)
//       {
//           OS_String_Show(270, 160, 24, 1, "初始化/采样：正常  ");
//           for (channel = 0U; channel < 8U; channel++)
//           {
//               x = (channel < 4U) ? 270U : 540U;
//               y = (uint16_t)(200U + ((channel & 0x03U) * 48U));
//               OS_Num_Show(x, y, 24, 1, (float)frame.code[channel], code_format[channel]);
//               OS_Num_Show(x, (uint16_t)(y + 24U), 16, 1,
//                           LHA6958H_CodeToVoltage((uint8_t)(channel + 1U), frame.code[channel]),
//                           "%+.5f 伏      ");
//           }
//       }
//       else
//       {
//           OS_Num_Show(270, 160, 24, 1, (float)status, "采集故障代码: %0.0f      ");
//           OS_String_Show(270, 192, 16, 1, "0:正常 2:读取超时 3:BUSY异常 4:首数据错误 5:校验失败");
//       }
//       delay_ms(100);
//   }
//   Change_Menu(0);
}

// 菜单5执行函数 参数：无
void MenuHaddler_5()
{
	Ps2KeyValue = KeyValue_Null;
	while (Ps2KeyValue != KeyValue_Back)
	{
	}

	Change_Menu(0);
}

/* ***************************** Custom Function Part       *****************************
 * 其他自定义函数区
 */

// 从PS2输入数字 参数：1>未按键时返回的数值 返回：键入的数值
float PS2_ReadNum(float num)
{
	uint8_t count = 0;
	uint8_t dec_sign = 0;
	float temp_num = 0;

	if (Ps2KeyValue <= KeyValue_Point)
	{
		TIM_Cmd(TIM6, DISABLE);

		LCD_Appoint_Clear(16, 96 + 64 * 4, 250 + 1, 480 - 32 - 8, Black);
		OS_Rect_Draw(16, 96 + 64 * 4, 250, 96 + 64 * 5-16, 1, White);

		while (Ps2KeyValue <= KeyValue_Point)
		{
			if (dec_sign == 0)
			{
				if (Ps2KeyValue == KeyValue_Point)
				{
					dec_sign = 1;
					count = 0;
				}
				else
					temp_num = temp_num * 10 + Ps2KeyValue;
			}
			else
			{
				if (Ps2KeyValue != KeyValue_Point)
					temp_num = temp_num + (float)Ps2KeyValue / pow(10, count);
			}

			OS_Num_Show(16 + 16, 96 + 64 * 4 + 16, 32, 1, temp_num, "-> %0.2f");

			count++;
			Ps2KeyValue = KeyValue_Null;
			while (Ps2KeyValue == KeyValue_Null);
		}

		Ps2KeyValue = KeyValue_Null;
		LCD_Appoint_Clear(16, 96 + 64 * 4, 250 + 1, 480 - 32 - 8, Black);
	}
	else
		temp_num = num;

	Ps2KeyValue = KeyValue_Null;

	return temp_num;
}

//DA数值显示区域刷新
void Refresh(void)
{
	OS_Rect_Draw(250+2+16+13*16,64+8+16+32+16,800,64+8+16+32+16+32,32,Black);
	OS_String_Show(250+2+16,64+8+16+32+16+32+16+32+16+(16+32)*1,32,1,"DA");
	OS_Num_Show(250+2+16+24,64+8+16+32+16+32+16+32+16+(16+32)*2,32,1,DAC_val[0],"PA4 DAC1:  %.2fV     ");
	OS_Num_Show(250+2+16+24,64+8+16+32+16+32+16+32+16+(16+32)*3,32,1,DAC_val[1],"PA5 DAC2:  %.2fV     ");	
}

//AD9959画选择光标
void AD9959_Drawselect(void)
{
	AD9959_Selet_Clear();
	if(state_channel==0)
	{
		switch(state_Allkindval)
		{
			case 0:
				OS_String_Show(250+2+8,64+8+16+(32+16)*1+(16+16)*1,24,1,"》");
			break;
			case 1:
				OS_String_Show(250+2+8,64+8+16+(32+16)*1+(16+16)*2,24,1,"》");
			break;
			case 2:
				OS_String_Show(250+2+8,64+8+16+(32+16)*1+(16+16)*3,24,1,"》");
			break;
			default:
				break;
		}
	}
	else if(state_channel==1)
	{
		switch(state_Allkindval)
		{
			case 0:
				OS_String_Show(250+2+(800-250-2-16)/2+8,64+8+16+(32+16)*1+(16+16)*1,24,1,"》");
			break;
			case 1:
				OS_String_Show(250+2+(800-250-2-16)/2+8,64+8+16+(32+16)*1+(16+16)*2,24,1,"》");
			break;
			case 2:
				OS_String_Show(250+2+(800-250-2-16)/2+8,64+8+16+(32+16)*1+(16+16)*3,24,1,"》");
			break;
			default:
				break;
		}
	}
	else if(state_channel==2)
	{
		switch(state_Allkindval)
		{
			case 0:
				OS_String_Show(250+2+8,64+8+16+(32+16)*1+(16+16)*5+16,24,1,"》");
			break;
			case 1:
				OS_String_Show(250+2+8,64+8+16+(32+16)*1+(16+16)*6+16,24,1,"》");
			break;
			case 2:
				OS_String_Show(250+2+8,64+8+16+(32+16)*1+(16+16)*7+16,24,1,"》");
			break;
			default:
				break;
		}
	}
	else if(state_channel==3)
	{
		switch(state_Allkindval)
		{
			case 0:
				OS_String_Show(250+2+(800-250-2-16)/2+8,64+8+16+(32+16)*1+(16+16)*5+16,24,1,"》");
			break;
			case 1:
				OS_String_Show(250+2+(800-250-2-16)/2+8,64+8+16+(32+16)*1+(16+16)*6+16,24,1,"》");
			break;
			case 2:
				OS_String_Show(250+2+(800-250-2-16)/2+8,64+8+16+(32+16)*1+(16+16)*7+16,24,1,"》");
			break;
			default:
				break;
		}
	}
}
//AD9959光标清除
void AD9959_Selet_Clear(void)
{
	OS_Rect_Draw(250+2+8,64+8+16+(32+16)*1+(16+16)*1,250+2+8+16,64+8+16+(32+16)*1+(16+16)*7+16+24,24,Black);
	OS_Rect_Draw(250+2+8+(800-250-2-16)/2,64+8+16+(32+16)*1+(16+16)*1,250+2+8+16+(800-250-2-16)/2,64+8+16+(32+16)*1+(16+16)*7+16+24,24,Black);
}

//键盘对AD9959对应数值进行加减
void AD9959_Changeval(uint8_t KeyNum)
{
	uint32_t FreStep = 1;
	//数值减少
	if(KeyNum==KeyValue_Minus)
	{
		switch(state_Allkindval)
		{
			case 0:
				if(channel_fre[state_channel] >= 1000000)FreStep = 1000000;
				else if(channel_fre[state_channel] >= 1000)FreStep = 1000;
				else FreStep = 1;
				if(channel_fre[state_channel] <= FreStep){channel_fre[state_channel]=0;}
				else{channel_fre[state_channel]-=FreStep;}
				Write_frequence(state_channel,channel_fre[state_channel]);
				break;
			case 1:
				if(channel_Amp[state_channel] <= 10.0f){channel_Amp[state_channel]=0.0f;}
				else{channel_Amp[state_channel]-=10.0f;}
				Write_Amplitude(state_channel,channel_Amp[state_channel]);
				break;
			case 2:
				if(channel_Pha[state_channel] <= 10){channel_Pha[state_channel]=0;}
				else{channel_Pha[state_channel]-=10;}
				Write_Phase(state_channel,channel_Pha[state_channel]);
				break;
			default:
				break;
		}
	}
	//数值增加
	if(KeyNum==KeyValue_Add)
	{
		switch(state_Allkindval)
		{
			case 0:
				if(channel_fre[state_channel] >= 1000000)FreStep = 1000000;
				else if(channel_fre[state_channel] >= 1000)FreStep = 1000;
				else FreStep = 1;
				channel_fre[state_channel]+=FreStep;
				if(channel_fre[state_channel]>500000000){channel_fre[state_channel]=500000000;}
				Write_frequence(state_channel,channel_fre[state_channel]);
				break;
			case 1:
				channel_Amp[state_channel]+=10;
				if(channel_Amp[state_channel]>576.0f){channel_Amp[state_channel]=576.0f;}
				Write_Amplitude(state_channel,channel_Amp[state_channel]);
				break;
			case 2:
				channel_Pha[state_channel]+=10;
				if(channel_Pha[state_channel]>360){channel_Pha[state_channel]=360;}
				Write_Phase(state_channel,channel_Pha[state_channel]);
				break;
			default:
				break;
		}
	}
	//AD9959光标对应位进行清除
}
//AD9959频率位数判断
uint8_t AD9959_NumScan(uint32_t Num)
{
	uint8_t count=0;
	if(Num==0){return 1;}
	while(Num!=0)
	{
		Num/=10;
		count++;
	}
	return count;
}
//AD9959频率显示
void AD9959_Fre_Show(uint16_t x,uint16_t y,uint32_t fre)
{
	if(fre>1000000)
	{
		OS_Num_Show(x,y,24,1,(float)fre/1000000,"频率：%.3fMHz      ");
	}
	else if(fre>1000)
	{
		OS_Num_Show(x,y,24,1,(float)fre/1000,"频率：%.3fKHz      ");
	}
	else
	{
		OS_Num_Show(x,y,24,1,fre,"频率：%0.0fHz      ");
	}
}

//AD9959显示
void AD9959_Show(void)
{
	OS_String_Show(250+2+16+16,64+8+16+(32+16)*0,32,1,"AD9959");
	OS_String_Show(250+2+16+16,64+8+16+(32+16)*1,24,1,"通道0");
	AD9959_Fre_Show(250+2+16+16,64+8+16+(32+16)*1+(16+16)*1,channel_fre[0]);
	OS_Num_Show(250+2+16+16,64+8+16+(32+16)*1+(16+16)*2,24,1,channel_Amp[0],"峰峰值：%.2fmV      ");
	OS_Num_Show(250+2+16+16,64+8+16+(32+16)*1+(16+16)*3,24,1,channel_Pha[0],"相位：%0.0f        ");	
	

	OS_String_Show(250+2+16+16+(800-250-2-16)/2,64+8+16+(32+16)*1,24,1,"通道1");
	AD9959_Fre_Show(250+2+16+16+(800-250-2-16)/2,64+8+16+(32+16)*1+(16+16)*1,channel_fre[1]);
	OS_Num_Show(250+2+16+16+(800-250-2-16)/2,64+8+16+(32+16)*1+(16+16)*2,24,1,channel_Amp[1],"峰峰值：%.2fmV      ");
	OS_Num_Show(250+2+16+16+(800-250-2-16)/2,64+8+16+(32+16)*1+(16+16)*3,24,1,channel_Pha[1],"相位：%0.0f      ");	
	
	OS_String_Show(250+2+16+16,64+8+16+(32+16)*1+(16+16)*4+16,24,1,"通道2");
	AD9959_Fre_Show(250+2+16+16,64+8+16+(32+16)*1+(16+16)*5+16,channel_fre[2]);
	OS_Num_Show(250+2+16+16,64+8+16+(32+16)*1+(16+16)*6+16,24,1,channel_Amp[2],"峰峰值：%.2fmV      ");
	OS_Num_Show(250+2+16+16,64+8+16+(32+16)*1+(16+16)*7+16,24,1,channel_Pha[2],"相位：%0.0f");	

	OS_String_Show(250+2+16+16+(800-250-2-16)/2,64+8+16+(32+16)*1+(16+16)*4+16,24,1,"通道3");
	AD9959_Fre_Show(250+2+16+16+(800-250-2-16)/2,64+8+16+(32+16)*1+(16+16)*5+16,channel_fre[3]);
	OS_Num_Show(250+2+16+16+(800-250-2-16)/2,64+8+16+(32+16)*1+(16+16)*6+16,24,1,channel_Amp[3],"峰峰值：%.2fmV      ");
	OS_Num_Show(250+2+16+16+(800-250-2-16)/2,64+8+16+(32+16)*1+(16+16)*7+16,24,1,channel_Pha[3],"相位：%0.0f       ");	
}
//获得AD9959当前光标指向的值
float AD9959_SelectNum(void)
{
	switch(state_Allkindval)
	{
		case 0:
			return channel_fre[state_channel];
		case 1:
			return channel_Amp[state_channel];
		case 2:
			return channel_Pha[state_channel];
		default:
			break;
	}
	return 0;
}
//AD9959将键盘输入的值写入当前光标指向的值
void AD9959_PopNum(float Num)
{
	switch(state_Allkindval)
	{
		case 0:
			if(Num>500000000.0f)Num=500000000.0f;
			if(Num<0.0f)Num=0.0f;
			channel_fre[state_channel]=Num;
			Write_frequence(state_channel,channel_fre[state_channel]);
			break;
		case 1:
			if(Num>576.0f)Num=576.0f;
			if(Num<0.0f)Num=0.0f;
			channel_Amp[state_channel]=Num;
			Write_Amplitude(state_channel,channel_Amp[state_channel]);
			break;
		case 2:
			if(Num>360.0f)Num=360.0f;
			if(Num<0.0f)Num=0.0f;
			channel_Pha[state_channel]=Num;
			Write_Phase(state_channel,channel_Pha[state_channel]);
			break;
		default:
			break;
	}
}
//按键获取
float Read_Key(float Num)
{
		float Num1=0;
		if(Ps2KeyValue<=KeyValue_Point)
		while(Ps2KeyValue!=KeyValue_Enter)
		{			
			if(Ps2KeyValue<=KeyValue_Point)
			{
				Num1=PS2_ReadNum(0);
				Ps2KeyValue = KeyValue_Null;
				return Num1;
			}	
			delay_ms(10);
		}	
		else 
		{
			return Num;
		}
		return Num;
}
//DAC值光标对应位值刷新
void DAC_ValRefresh(void)
{
	if(ADCstate==0)
	{
		OS_Num_Show(250+2+16+24+16*11,64+8+16+32+16+32+16+32+16+(16+32)*2,32,1,DAC_val[0],"%.2fV       ");
	}
	else if(ADCstate==1)
	{
		OS_Num_Show(250+2+16+24+16*11,64+8+16+32+16+32+16+32+16+(16+32)*3,32,1,DAC_val[1],"%.2fV       ");
	}
}
//DAC光标
void DAC_Select(void)
{
	//对光标进行清除
	OS_Rect_Draw(250+2+16,64+8+16+32+16+32+16+32+16+(16+32)*2,250+2+16+24,64+8+16+32+16+32+16+32+16+(16+32)*3+24,24,Black);
	if(ADCstate==0)
	{
		OS_String_Show(250+2+16,64+8+16+32+16+32+16+32+16+(16+32)*2,24,1,"》");
	}
	else if(ADCstate==1)
	{
		OS_String_Show(250+2+16,64+8+16+32+16+32+16+32+16+(16+32)*3,24,1,"》");
	}
}

/* ***************************** IRQHandler Part    	     	*****************************
 * 中断执行函数区
 */

/* ***************************** 						END 	   	     	*****************************/
