/**********************************************************
* @ File name -> usart.c
* @ Version   -> V1.0
* @ Date      -> 12-26-2013
* @ Brief     -> 系统串口设置相关的函数
**********************************************************/

#include "usart.h"
#include "Communication.h"

/**********************************************************
   加入以下代码，支持printf函数，而不需要选择use MicroLIB
**********************************************************/

#if 1

#pragma import(__use_no_semihosting)             
//标准库需要的支持函数                 
struct __FILE 
{ 
	int handle; 

}; 
/* FILE is typedef’ d in stdio.h. */ 
FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式    
_sys_exit(int x) 
{ 
	x = x; 
} 
//重定义fputc函数 
int fputc(int ch, FILE *f)
{
	while((USART3->SR & 0x40) == 0);	//循环发送，直到发送完毕   
    USART3->DR = (uint8_t)ch;	//发送数据      
	return ch;
}
#endif

/**********************   end   **************************/


/**********************************************************
* 函数功能 ---> 初始化IO 串口1
* 入口参数 ---> bound：波特率	
* 返回数值 ---> none
* 功能说明 ---> none
**********************************************************/
void USARTx_Init(uint32_t bound)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	
#ifdef EN_USART1_RX	//如果使能了接收中断
	
	USART1->CR1 |= 1 << 8;	//PE中断使能
	USART1->CR1 |= 1 << 5;	//接收缓冲区非空中断使能
	
	MY_NVIC_Init(3, 3, USART1_IRQn, NVIC_PriorityGroup_2);	//中断分组2，最低优先级
	
#endif
	 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);	//使能USART1，GPIOA时钟
		
	USART_DeInit(USART1);  //复位串口1
	
	//USART1_TX   PA.9
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; //PA.9
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//复用推挽输出
	GPIO_Init(GPIOA, &GPIO_InitStructure); //初始化PA9
   
	//USART1_RX	  PA.10
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
	GPIO_Init(GPIOA, &GPIO_InitStructure);  //初始化PA10
  
	//USART 初始化设置
	USART_InitStructure.USART_BaudRate = bound;	//设置波特率，一般设置为9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;	//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;	//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;	//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;	//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
	
	USART_Init(USART1, &USART_InitStructure); //初始化串口
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//开启中断
	USART_Cmd(USART1, ENABLE);                    //使能串口 
}

void USART3_Init(u32 bound)
{
  //GPIO端口设置
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB| RCC_APB2Periph_AFIO, ENABLE);	//使能USART3，GPIOB时钟
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
	//USART3_TX   GPIOB.9
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; 
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//复用推挽输出
  GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化GPIOB.9
   
  //USART3_RX	  GPIOB.10初始化
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
  GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化GPIOB.10  

   //USART 初始化设置

	USART_InitStructure.USART_BaudRate = bound;//串口波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
  USART_Init(USART3, &USART_InitStructure); //初始化串口3
	
	//Usart1 NVIC 配置
  NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0 ;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器
	
  USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);//开启串口接受中断
  USART_Cmd(USART3, ENABLE);                    //使能串口1 

}

//=========================================================
 
#ifdef EN_USART1_RX   //如果使能了接收

//=========================================================

//串口1中断服务程序
//注意,读取USARTx->SR能避免莫名其妙的错误 

uint8_t USART_RX_BUF[USART_REC_LEN];	//接收缓冲,最大USART_REC_LEN个字节

//接收状态
//bit15：接收完成标志
//bit14：接收到0x0d
//bit13~0：接收到的有效字节数目，最大512字节

uint16_t USART_RX_STA=0;	//接收状态标记	  

/**********************************************************
* 函数功能 ---> 串口1接收中断服务程序
* 入口参数 ---> bound：波特率	
* 返回数值 ---> none
* 功能说明 ---> none
**********************************************************/
void USART1_IRQHandler(void)
{
	uint8_t res;
	static u16 i=0;
	static u8 rx_flag =0;
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)  //接收到数据
	{
		USART_ClearFlag(USART1, USART_FLAG_RXNE);  
		USART_ClearITPendingBit(USART1, USART_IT_RXNE); 
		res = USART_ReceiveData(USART1);	//读取接收到的数据
		 if(i == 0 && res == 0xAA)//???????·
					rx_flag =1;
		 if(rx_flag == 1)
				 USART_RX_BUF[i++] = res; 
		 if(i == RX_LENGTH)
		 {
				i=0;
				rx_flag =0;

        rxDataprocess();//???????????í				 
		 }	 
	}	//end 接收到数据
}

//=========================================================

#endif	//end使能接收

void USART3_IRQHandler(void)                	
{
 	static u16 i=0;
	static u8 rx_flag =0;
	u8 Res;
  
	
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)  
	{     
		 USART_ClearFlag(USART3, USART_FLAG_RXNE);  
		 USART_ClearITPendingBit(USART3, USART_IT_RXNE);  
		 Res =USART_ReceiveData(USART3);	//读取接收到的数据
		//printf("%02x ",Res);
		 if(i == 0 && Res == 0xAA)//找到帧头
					rx_flag =1;
		 if(rx_flag == 1)
				 USART_RX_BUF[i++] = Res; 
		 if(i == RX_LENGTH)
		 {
				i=0;
				rx_flag =0;
				printf("return");
        rxDataprocess();//串口接收处理				 
		 }	
	}   

} 
//=========================================================

