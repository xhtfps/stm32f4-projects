#include "App_GPIO.h"

u8 mode_flag = 0;
extern float v;

void GPIOTEST_Init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(((uint32_t)0x0000007f),ENABLE);//使能GPIOA时钟
	
	GPIO_InitStructure.GPIO_Pin = ((uint16_t)0x0700);
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输入模式
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//PC13作输出只能2MHz
	
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;//上拉
	GPIO_Init(GPIOA,&GPIO_InitStructure);//初始化GPIOE
	
	GPIO_InitStructure.GPIO_Pin = ((uint16_t)0x0de3);
	GPIO_Init(GPIOB,&GPIO_InitStructure);//初始化GPIOE
	
	GPIO_InitStructure.GPIO_Pin = ((uint16_t)0x3fc1);
	GPIO_Init(GPIOC,&GPIO_InitStructure);//初始化GPIOE
	
	GPIO_InitStructure.GPIO_Pin = ((uint16_t)0x3800);
	GPIO_Init(GPIOD,&GPIO_InitStructure);//初始化GPIOE
	
	GPIO_InitStructure.GPIO_Pin = ((uint16_t)0x007f);
	GPIO_Init(GPIOE,&GPIO_InitStructure);//初始化GPIOE
	
	GPIO_InitStructure.GPIO_Pin = ((uint16_t)0xfffe);
	GPIO_Init(GPIOF,&GPIO_InitStructure);//初始化GPIOE
	
	GPIO_InitStructure.GPIO_Pin = ((uint16_t)0xffff);
	GPIO_Init(GPIOG,&GPIO_InitStructure);//初始化GPIOE
	
	GPIO_SetBits(GPIOA,(uint16_t)0x071c);
	GPIO_SetBits(GPIOB,(uint16_t)0x0de3);
	GPIO_SetBits(GPIOC,(uint16_t)0x3fc1);
	GPIO_SetBits(GPIOD,(uint16_t)0x3800);
	GPIO_SetBits(GPIOE,(uint16_t)0x007f);
	GPIO_SetBits(GPIOF,(uint16_t)0xfffe);
	GPIO_SetBits(GPIOG,(uint16_t)0xffff);
}

void GPIO_Flip(void)
{
	
	if(GPIO_ReadOutputDataBit(GPIOF,GPIO_Pin_1) == 0){
		GPIO_SetBits(GPIOA,(uint16_t)0x071c);
		GPIO_SetBits(GPIOB,(uint16_t)0x0de3);
		GPIO_SetBits(GPIOC,(uint16_t)0x3fc1);
		GPIO_SetBits(GPIOD,(uint16_t)0x3800);
		GPIO_SetBits(GPIOE,(uint16_t)0x007f);
		GPIO_SetBits(GPIOF,(uint16_t)0xfffe);
		GPIO_SetBits(GPIOG,(uint16_t)0xffff);
		
	}else {
		GPIO_ResetBits(GPIOA,(uint16_t)0x071c);
		GPIO_ResetBits(GPIOB,(uint16_t)0x0de3);
		GPIO_ResetBits(GPIOC,(uint16_t)0x3fc1);
		GPIO_ResetBits(GPIOD,(uint16_t)0x3800);
		GPIO_ResetBits(GPIOE,(uint16_t)0x007f);
		GPIO_ResetBits(GPIOF,(uint16_t)0xfffe);
		GPIO_ResetBits(GPIOG,(uint16_t)0xffff);
		
	}
	
}

void GPIO_input(u8 flag)
{
	
	if(flag == 1)
	{
		GPIOA->MODER = 0x00000000;
		GPIOB->MODER = 0x00000000;
		GPIOC->MODER = 0x00000000;
		GPIOD->MODER = 0x00000000;
		GPIOE->MODER = 0x00000000;
		GPIOF->MODER = 0x00000000;
		GPIOG->MODER = 0x00000000;
		
	}else if(flag == 0)
	{
		GPIOA->MODER = 0x55555555;
		GPIOB->MODER = 0x55555555;
		GPIOC->MODER = 0x55555555;
		GPIOD->MODER = 0x55555555;
		GPIOE->MODER = 0x55555555;
		GPIOF->MODER = 0x55555555;
		GPIOG->MODER = 0x55555555;
	}
	
}


//void GPIO_ShowInput(void)
//{
//	
//	if(PCin(12) == 1)
//		OS_String_Show(700, 20, 24, 1, "1");
//	else 
//		OS_String_Show(700, 20, 24, 1, "0");
//	
//	if(PCin(11) == 1)
//		OS_String_Show(750, 20, 24, 1, "1");
//	else 
//		OS_String_Show(750, 20, 24, 1, "0");
//	
//	if(PCin(10) == 1)
//		OS_String_Show(700, 50, 24, 1, "1");
//	else 
//		OS_String_Show(700, 50, 24, 1, "0");
//	
//	if(PCin(0) == 1)
//		OS_String_Show(750, 50, 24, 1, "1");
//	else 
//		OS_String_Show(750, 50, 24, 1, "0");
//	
//	
//}

void GPIOTEST_main(void)
{
	u8 key;
	GPIOTEST_Init();
	while(1)
	{
		key = KEY_Scan(0);
		Key_Handle(key);
		
//		if(mode_flag == 1){
//			
//		}	
	}
}

void Key_Handle(u8 key)
{
	if(key)
	{
		switch(key)
		{
			case 1:{
					GPIO_Flip();
				
			}break;
			case 2:{
				v += 0.5;
				if(v > 3.3)
					v = 3.3;
				DAC_Out();
				Refresh();
			}break;
			case 3:{
					GPIO_Flip();	
				
			}break;
			case 4:{
				v -= 0.5;
				if(v < 0)
					v = 0;
				DAC_Out();
				Refresh();
				
			}break;
			default: break;
		}
	
	}
}


