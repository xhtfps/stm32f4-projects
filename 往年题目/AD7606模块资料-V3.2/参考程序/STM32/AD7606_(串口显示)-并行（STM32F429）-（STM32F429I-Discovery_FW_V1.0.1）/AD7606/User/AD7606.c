#include "stm32f4xx.h"
#include "AD7606.h"
#include "delay.h"
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
// 调用模块: __HAL_RCC_GPIOA_CLK_ENABLE(); HAL_GPIO_DeInit();
// 注意事项: 用GPIO驱动方式和FSMC驱动方式下的引脚配置不一样
//			
//-----------------------------------------------------------------
void GPIO_AD7606_Configuration(void)
{ 
	GPIO_InitTypeDef GPIO_InitStructure;

	// 使能IO口时钟
	    __HAL_RCC_GPIOA_CLK_ENABLE();
			__HAL_RCC_GPIOB_CLK_ENABLE();
		  __HAL_RCC_GPIOE_CLK_ENABLE();
			__HAL_RCC_GPIOF_CLK_ENABLE();
		
	HAL_GPIO_DeInit(GPIOA,GPIO_PIN_5 );//将GPIOx外设寄存器初始化为默认复位值
	HAL_GPIO_DeInit(GPIOB, GPIO_PIN_3 |GPIO_PIN_4 |GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9);
	HAL_GPIO_DeInit(GPIOE,GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 |GPIO_PIN_5);
	HAL_GPIO_DeInit(GPIOF,GPIO_PIN_All);
	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	// AD7606 
	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	// 控制线配置 
	//             CS_N       RD/SCLK      
			GPIO_InitStructure.Pin = GPIO_PIN_4 | GPIO_PIN_5;
			GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_LOW;
			GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
			GPIO_InitStructure.Pull = GPIO_NOPULL;
			HAL_GPIO_Init(GPIOE, &GPIO_InitStructure);

	//               FRSTDATA     BUSY  
			GPIO_InitStructure.Pin = GPIO_PIN_2 | GPIO_PIN_3;
			GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
			GPIO_InitStructure.Mode = GPIO_MODE_EVT_RISING_FALLING;
			GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
			GPIO_InitStructure.Pull = GPIO_NOPULL;
			HAL_GPIO_Init(GPIOE, &GPIO_InitStructure);

	//    rst convstB convstA STby OS12 OS11 OS10
      GPIO_InitStructure.Pin = GPIO_PIN_3 | GPIO_PIN_4 | 
			                         GPIO_PIN_5 | GPIO_PIN_6 |
															 GPIO_PIN_7 | GPIO_PIN_8 | 
			                         GPIO_PIN_9 ;
			GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
			GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
			GPIO_InitStructure.Pull = GPIO_NOPULL;
			HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);

    //数据线配置(16)
		//DB0 DB1 DB2 DB3 DB4 DB5 DB6 DB7 DB8 DB9 DB10 DB11 DB12 DB13 DB14 DB15 
		  GPIO_InitStructure.Pin = GPIO_PIN_0 | GPIO_PIN_1 | 
			                         GPIO_PIN_2 | GPIO_PIN_3 |
															 GPIO_PIN_4 | GPIO_PIN_5 | 
			                         GPIO_PIN_6 | GPIO_PIN_7 |
                               GPIO_PIN_8 | GPIO_PIN_9 | 
			                         GPIO_PIN_10 | GPIO_PIN_11 |
															 GPIO_PIN_12 | GPIO_PIN_13 |
			                         GPIO_PIN_14 | GPIO_PIN_15;
			GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
			GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
			GPIO_InitStructure.Pull = GPIO_NOPULL;
			HAL_GPIO_Init(GPIOF, &GPIO_InitStructure);		
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
	STby_Set;
	rd_Set;
	cs_Set;	
  OS10_Reset;
	OS11_Reset;
	OS12_Reset;
	AD7606_reset();  
	AD7606_startconvst();
	
}
/*   * 名称：AD7606_startconvst()  * 功能：启动转换  */  
void AD7606_startconvst(void)
{  
	convstA_Reset;	
	convstB_Reset;	
	delay_us(11);
	convstA_Set;
	convstB_Set;
}
  
/*   * 名称：AD7606_reset()  * 功能：复位模块  */
void AD7606_reset(void) 
{ 
	rst_Reset;
	delay_100ns();
	rst_Set; 
	delay_us(1);
	rst_Reset; 
}  
uint16_t GPIO_ReadInputData(GPIO_TypeDef* GPIOx)
{
  /* Check the parameters */
  assert_param(IS_GPIO_ALL_PERIPH(GPIOx));
  
  return ((uint16_t)GPIOx->IDR);
}
/* 
* 名称：AD7606_read_data() 
* 功能：读取数据 
* 返回值：返回一个结构体指针，该指针为指向结构体数组的首地址  
*/ 
void AD7606_read_data(int16_t * DB_data) 
{  
	uint8_t i,j; 	
	uint16_t DB_data_H;
	uint16_t DB_data_L;
	
	cs_Reset; 
	delay_us (3);
	for(i=0;i<8;i++)  
	{
		rd_Reset;
		delay_us (4);		
		rd_Set;
		delay_us (4);		
		DB_data[i] = GPIO_ReadInputData(GPIOF);
	}	
	      cs_Set;	
} 
