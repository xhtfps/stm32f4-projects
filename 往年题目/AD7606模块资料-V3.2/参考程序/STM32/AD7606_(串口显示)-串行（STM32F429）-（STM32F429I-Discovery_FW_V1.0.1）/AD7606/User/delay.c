
#include "stm32f4xx.h"
#include "delay.h"

void delay_us(uint32_t nus)
{
 uint32_t temp;
 SysTick->LOAD = 18*nus;
 SysTick->VAL=0X00;//Çå¿Õ¼Ä´æÆ÷
 SysTick->CTRL=0X01;//
 do
 {
  temp=SysTick->CTRL;//
 }while((temp&0x01)&&(!(temp&(1<<16))));//
     SysTick->CTRL=0x00; //¹Ø±Õ¼Ä´æÆ÷
    SysTick->VAL =0X00; //Çå¿Õ¼Ä´æÆ÷
}
void delay_ms(uint16_t nms)
{
 uint32_t temp;
 SysTick->LOAD = 500*nms;
 SysTick->VAL=0X00;//Çå¿Õ¼Ä´æÆ÷
 SysTick->CTRL=0X01;//
 do
 {
  temp=SysTick->CTRL;//
 }while((temp&0x01)&&(!(temp&(1<<16))));//
    SysTick->CTRL=0x00; //
    SysTick->VAL =0X00; //Çå¿Õ¼Ä´æÆ÷
}
//void delay_84ns(uint16_t ns)
//{	
//	do{
//	}while(--ns);
//}
void delay_100ns(void)
{	
	
}
