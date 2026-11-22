//-----------------------------------------------------------------
// 程序描述:
//     AD7606并行驱动程序
// 作    者: 凌智电子
// 开始日期: 2020-11-11
// 完成日期: 2020-11-11
// 修改日期: 
// 当前版本: V1.0
// 历史版本:
//  - V1.0:  AD7606驱动程序
// 调试工具: 凌智STM32H750核心板、LZE_ST_LINK2
// 说    明: 
//    
//-----------------------------------------------------------------

//-----------------------------------------------------------------
// 头文件包含
//-----------------------------------------------------------------
#include "AD7606.h"
#include "delay.h"
#include "lcd.h"
//-----------------------------------------------------------------

//-----------------------------------------------------------------
// void GPIO_AD7606_Configuration(void)
//-----------------------------------------------------------------
// 
// 函数功能: AD7606引脚配置函数
// 入口参数: 无 
// 返 回 值: 无
// 注意事项: 无
//
//-----------------------------------------------------------------
void GPIO_AD7606_Configuration(void)
{ 
	GPIO_InitTypeDef  GPIO_InitStruct;
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();

	// OSI0 -> PC0
	// OSI1 -> PC1
	// OSI2 -> PC2
	// C0-A -> PC3
	// STBY -> PC4
	// REST -> PC5
	// C0-B -> PC6
	// CS_N -> PC7
	GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | 
												GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;		// 推挽输出
  GPIO_InitStruct.Pull = GPIO_PULLUP;						// 上拉
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;	// 高速
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);				// 初始化
	
	// RD/SCLK -> PA0
	GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;		// 推挽输出
  GPIO_InitStruct.Pull = GPIO_PULLUP;						// 上拉
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;	// 高速
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);				// 初始化
	
	// FR-D -> PA1
	// BUSY -> PA4
	GPIO_InitStruct.Pin = GPIO_PIN_1 | GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;				// 输入
  GPIO_InitStruct.Pull = GPIO_NOPULL;						// 无上下拉
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;	// 高速
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);				// 初始化
	
	// DB0～DB15 -> PB0～PB15
	GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | 
												GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7 |
												GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10| GPIO_PIN_11|
												GPIO_PIN_12| GPIO_PIN_13| GPIO_PIN_14| GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;				// 输入
  GPIO_InitStruct.Pull = GPIO_NOPULL;						// 无上下拉
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;	// 高速
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);				// 初始化
}

//-----------------------------------------------------------------
// void AD7606_Init(void)
//-----------------------------------------------------------------
// 
// 函数功能: AD7606初始化函数
// 入口参数: 无 
// 返 回 值: 无
// 注意事项: 无
//
//-----------------------------------------------------------------
void AD7606_Init(void)
{
	delay_ms(5);
	GPIO_AD7606_Configuration();
	delay_ms(5);
	CO_A_H;
	CO_B_H;
	delay_ms(1);
	STBY_H;
	RD_SCLK_H;
	CS_N_H;	
  OS10_L;
	OS11_L;
	OS12_L;
	AD7606_RESET();  
	delay_ms(1);
	AD7606_StartConvst();	
}

//-----------------------------------------------------------------
// void AD7606_StartConvst(void)
//-----------------------------------------------------------------
// 
// 函数功能: 启动转换
// 入口参数: 无 
// 返 回 值: 无
// 注意事项: 无
//
//-----------------------------------------------------------------
void AD7606_StartConvst(void)
{  
	CO_A_L;	
	CO_B_L;	
	AD7606_Delay(100);
	CO_A_H;
	CO_B_H;
}
  
//-----------------------------------------------------------------
// void AD7606_RESET(void) 
//-----------------------------------------------------------------
// 
// 函数功能: 复位
// 入口参数: 无 
// 返 回 值: 无
// 注意事项: 无
//
//-----------------------------------------------------------------
void AD7606_RESET(void) 
{ 
	REST_L;
	AD7606_Delay(10);
	REST_H; 
	delay_us(1);
	REST_L; 
}  

//-----------------------------------------------------------------
// void AD7606_ReadData(s16 * DB_data)
//-----------------------------------------------------------------
// 
// 函数功能: 读取数据 
// 入口参数: s16 * DB_data：结构体指针，该指针为指向结构体数组的首地址 
// 返 回 值: 无
// 注意事项: 无
//
//-----------------------------------------------------------------
void AD7606_ReadData(s16 * DB_data)
{  
	uint8_t i; 
	
	CS_N_L; 
	AD7606_Delay(20);	
	for(i=0;i<8;i++)  
	{
		RD_SCLK_L;
		AD7606_Delay(25);	
		RD_SCLK_H;
		AD7606_Delay(25);	
		DB_data[i] = GPIOB->IDR;
	}	
	CS_N_H;		
} 

//-----------------------------------------------------------------
// void AD7606_Delay(uint16_t t) 
//-----------------------------------------------------------------
// 
// 函数功能: 读取数据 
// 入口参数: uint16_t t：延时数 
// 返 回 值: 无
// 注意事项: 无
//
//-----------------------------------------------------------------
void AD7606_Delay(uint16_t t)
{
	do {
			;
	} while (--t); 
}

