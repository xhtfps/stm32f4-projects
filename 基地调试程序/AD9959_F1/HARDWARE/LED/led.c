
#include "led.h"

/**********************************************************
* 函数功能 ---> LED接口初始化
* 入口参数 ---> none
* 返回参数 ---> none 
* 功能说明 ---> none
**********************************************************/
void LED_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOA, ENABLE);	//开启GPIOC外设时钟	

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15|GPIO_Pin_14|GPIO_Pin_13|GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
  
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
//	GPIO_Init(GPIOA, &GPIO_InitStructure);

//	GPIO_SetBits(GPIOB, GPIO_Pin_15);	//输出高电平，熄灭灯
//	GPIO_SetBits(GPIOB, GPIO_Pin_2);	//输出高电平，熄灭灯
}

void ledOpen(uint8_t Number)
{
    if(Number & LED_0)
        GPIO_SetBits(GPIOB, GPIO_Pin_15);
    if(Number & LED_1)
        GPIO_SetBits(GPIOB, GPIO_Pin_14);
    if(Number & LED_2)
        GPIO_SetBits(GPIOB, GPIO_Pin_13);
    if(Number & LED_3)
        GPIO_SetBits(GPIOB, GPIO_Pin_12);
}

void ledClose(uint8_t Number)
{
	  
    if(Number & LED_0)
        GPIO_ResetBits(GPIOB, GPIO_Pin_15);
    if(Number & LED_1)
        GPIO_ResetBits(GPIOB, GPIO_Pin_14);
    if(Number & LED_2)
        GPIO_ResetBits(GPIOB, GPIO_Pin_13);
    if(Number & LED_3)
        GPIO_ResetBits(GPIOB, GPIO_Pin_12);
}

void time1Init(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure; 
	
	/* ??TIM2CLK ? 72MHZ */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
	
	TIM_TimeBaseStructure.TIM_Period = 65535;
	TIM_TimeBaseStructure.TIM_Prescaler = 329;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);
	 													
	NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_IRQn;	  
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 9;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);		
	
	TIM_ClearFlag(TIM1, 0x1FFF);
	TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);
	
	TIM_Cmd(TIM1, ENABLE);
}

uint8_t channel = 0x01;

void TIM1_UP_IRQHandler(void)
{
    if(TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET)
    {
        channel=channel<<1;
        if(channel == 0x10)//0001 0000
            channel = 0x01;
        
//        ledClose(LED_0 | LED_1 | LED_2 | LED_3);
				ledClose(0x0F);
        ledOpen(channel);
        
        TIM1->SR = (uint16_t)~TIM_IT_Update;
    }
		//delay_ms(500);
}