#include "App_ADC.h"

float val;
u16 adc;

void ADC_main()
{
	while(1)
	{
		adc=ADC_Average_Get(ADC_Channel_1,128);
		val=adc*3.3/4096;
		OS_Num_Show(30,150,32,1,val,"PA1 ADC1:  %.6fV");
		adc=ADC_Average_Get(ADC_Channel_2,128);
		val=adc*3.3/4096;
		OS_Num_Show(30,200,32,1,val,"PA2 ADC2:  %.6fV");
		LCD_DrawRectS(250,250,260,300,WHITE);
		LCD_DrawRectS(260,250,270,300,BLACK);
		LCD_DrawRectS(270,250,280,300,BLUE);
		LCD_DrawRectS(280,250,290,300,BRED);
		LCD_DrawRectS(290,250,300,300,GRED);
		LCD_DrawRectS(300,250,310,300,GBLUE);
		LCD_DrawRectS(310,250,320,300,RED);
		LCD_DrawRectS(320,250,330,300,MAGENTA);
		LCD_DrawRectS(330,250,340,300,GREEN);
		LCD_DrawRectS(340,250,350,300,CYAN);
		LCD_DrawRectS(350,250,360,300,YELLOW);
		OSTimeDly(33);
	}
}



