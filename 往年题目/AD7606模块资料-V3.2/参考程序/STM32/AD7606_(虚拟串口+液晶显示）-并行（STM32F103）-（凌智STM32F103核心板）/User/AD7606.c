
#include <stm32f10x.h>
#include "Delay.h"
#include "AD7606.h"
#include "lze_lcd.h"

extern u8 IO_TAG;	
//-----------------------------------------------------------------
// 初始化程序区
//-----------------------------------------------------------------
//-----------------------------------------------------------------
// void GPIO_AD7606_Configuration(void)
//-----------------------------------------------------------------
//
// 函数功能: AD7606引脚配置函数
// 入口参数: 无
// 返回参数: 无
// 全局变量: 无
// 调用模块: RCC_APB2PeriphClockCmd(); GPIO_Init();
// 注意事项: 用GPIO驱动方式和FSMC驱动方式下的引脚配置不一样
//			
//-----------------------------------------------------------------
void GPIO_AD7606_Configuration(void)
{ 
	GPIO_InitTypeDef GPIO_InitStructure;

	// 使能IO口时钟
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA |	RCC_APB2Periph_GPIOB |
													RCC_APB2Periph_GPIOC |	RCC_APB2Periph_GPIOD |	
													RCC_APB2Periph_GPIOE,
													 ENABLE);  
	GPIO_DeInit(GPIOA);//将GPIOx外设寄存器初始化为默认复位值
	GPIO_DeInit(GPIOB);
	GPIO_DeInit(GPIOC);
	GPIO_DeInit(GPIOD);
	GPIO_DeInit(GPIOE);
	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	// AD7606 
	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	// 控制线配置 
	//         CS_N       RD/SCLK      
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 ;							
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;						 
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	
	
	//               FRSTDATA     BUSY  
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_4 | GPIO_Pin_5 	;							
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode =GPIO_Mode_IN_FLOATING ;						 
	GPIO_Init(GPIOA, &GPIO_InitStructure);
					 
	//  rst convstB convstA STby OS12 OS11 OS10
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 	| GPIO_Pin_9 	|	
																GPIO_Pin_10 | GPIO_Pin_11 | 
																GPIO_Pin_13 | GPIO_Pin_14 | 
																GPIO_Pin_15 ;								
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;						 
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	// 数据线配置(8)
	// DB0 DB1 DB2 DB3 DB4 DB5 DB6 DB7 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 	| GPIO_Pin_1 |	
																GPIO_Pin_2  | GPIO_Pin_3 | 
																GPIO_Pin_4  | GPIO_Pin_5 | 
																GPIO_Pin_6  | GPIO_Pin_7 ;								
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;						 
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	//DB8 DB9 DB10 DB11 DB12 DB13 DB14 DB15 
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 	| GPIO_Pin_9 |	
																GPIO_Pin_10  | GPIO_Pin_11 | 
																GPIO_Pin_12  | GPIO_Pin_13 | 
																GPIO_Pin_14  | GPIO_Pin_15 ;								
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;						 
	GPIO_Init(GPIOC, &GPIO_InitStructure);
}

//-----------------------------------------------------------------
// void AD7606_Init(void)
//-----------------------------------------------------------------
//
// 函数功能: AD7606初始化函数
// 入口参数: 无
// 返回参数: 无
// 全局变量: 无
// 调用模块:    
// 注意事项: 无
//-----------------------------------------------------------------
void AD7606_Init(void)
{
	convstA_Set;
	convstB_Set;
	Delay_1ms(1);
	STby_Set;
	rd_Set;
	cs_Set;	
  OS10_Reset;
	OS11_Reset;
	OS12_Reset;
	AD7606_reset();  
	Delay_1ms(1);
	AD7606_startconvst();	
}

/*   * 名称：AD7606_startconvst()  * 功能：启动转换  */  
void AD7606_startconvst(void)
{  
	convstA_Reset;	
	convstB_Reset;	
	Delay_ns (100);
	convstA_Set;
	convstB_Set;
}
  
/*   * 名称：AD7606_reset()  * 功能：复位模块  */
void AD7606_reset(void) 
{ 
	rst_Reset;
	Delay_ns (10);
	rst_Set; 
	Delay_1us(1);
	rst_Reset; 
}  

/* 
* 名称：AD7606_read_data() 
* 功能：读取数据 
* 返回值：返回一个结构体指针，该指针为指向结构体数组的首地址  
*/ 
void AD7606_read_data(s16 * DB_data) 
{  
	u8 i; 
//	u16 DB_data_H;
//	u16 DB_data_L;
	
	cs_Reset; 
	Delay_ns(20);	
	for(i=0;i<8;i++)  
	{
		rd_Reset;
		Delay_ns(25);	
		rd_Set;
		Delay_ns(25);	
		DB_data[i] = GPIO_ReadInputData(GPIOC);
	}	
	cs_Set;		
} 

