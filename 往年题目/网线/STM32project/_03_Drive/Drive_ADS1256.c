#include "Drive_ADS1256.h"


//***************************
//		Pin assign	   	
//		STM32						ADS1256

//		GPIOF_Pin_0 		---> SCLK
//		GPIOF_Pin_2			<--- DIN
//		GPIOF_Pin_4  		---> DOUT
//		GPIOF_Pin_6 		<--- DRDY
//		GPIOF_Pin_8 		---> CS
//		GPIOF_Pin_10  	<--- PDWN

//***************************	

/*绔彛瀹氫箟*/ 
#define SCLK		 	 GPIO_Pin_2
#define DIN	 			 GPIO_Pin_6
#define DOUT			 GPIO_Pin_4
#define DRDY			 GPIO_Pin_8
#define CS				 GPIO_Pin_10
#define PDWN		   GPIO_Pin_12

#define ADS1256_CS_0()   		GPIO_ResetBits(GPIOF,CS)
#define ADS1256_CS_1()   		GPIO_SetBits(GPIOF,CS)

#define ADS1256_SCLK_0()  	GPIO_ResetBits(GPIOF,SCLK)
#define ADS1256_SCLK_1()  	GPIO_SetBits(GPIOF,SCLK)

#define ADS1256_DIN_0()   	GPIO_ResetBits(GPIOF,DOUT) //娉ㄦ剰杩欓噷锛屽崟鐗囨満鐨凞OUT杩炴帴ADS1256鐨凞IN
#define ADS1256_DIN_1()   	GPIO_SetBits(GPIOF,DOUT)

#define ADS1256_DOUT			 (GPIOF->IDR & DIN)          //
#define ADS1256_DRDY			 (GPIOF->IDR & DRDY)
#define ADS1256_PDWN		   (GPIOF->IDR & PDWN)

#define ADS1256_WAIT_READY_TIMEOUT_US 50000U

static uint8_t ADS1256_WaitReady(uint32_t timeout_us);



/*
*********************************************************************************************************
*	鍑?鏁?鍚? Init_ADS1256_GPIO
*	鍔熻兘璇存槑: 鍒濆鍖朅DS1256 GPIO
*	褰?   鍙? 鏃?
*	杩?鍥?鍊? 鏃?
*********************************************************************************************************
*/
void Init_ADS1256_GPIO(void)
{

    GPIO_InitTypeDef  GPIO_InitStructure;

		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);

		GPIO_InitStructure.GPIO_Pin = SCLK | DOUT | CS ;
		GPIO_InitStructure.GPIO_Speed=GPIO_Speed_100MHz;
		GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT;
		GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;
		GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_DOWN; 
		GPIO_Init(GPIOF,&GPIO_InitStructure);
	
	  ADS1256_CS_1();
	
	  GPIO_InitStructure.GPIO_Pin = DIN | DRDY | PDWN;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; 
		GPIO_Init(GPIOF,&GPIO_InitStructure);
}


/*
*********************************************************************************************************
*	鍑?鏁?鍚? SPI_WriteByte
*	鍔熻兘璇存槑: 鍚慡PI鎬荤嚎鍙戦€?涓猙it鏁版嵁銆?涓嶅甫CS鎺у埗銆?
*	褰?   鍙? _data : 鏁版嵁
*	杩?鍥?鍊? 鏃?
*********************************************************************************************************
*/
void SPI_WriteByte(u8 TxData)
{
	u8 i;
	
	/* 杩炵画鍙戦€佸涓瓧鑺傛椂锛岄渶瑕佸欢杩熶竴涓?*/
	delay_us(10);
	
	for(i = 0; i < 8; i++)
	{
		ADS1256_SCLK_1();
		
		if (TxData & 0x80)
			ADS1256_DIN_1();  
		else
			ADS1256_DIN_0();
		
		TxData <<= 1;
		
		delay_us(5);
		ADS1256_SCLK_0();	
		delay_us(5);
	}
	
} 

/*
*********************************************************************************************************
*	鍑?鏁?鍚? SPI_ReadByte
*	鍔熻兘璇存槑: 浠嶴PI鎬荤嚎鎺ユ敹8涓猙it鏁版嵁銆?涓嶅甫CS鎺у埗銆?
*	褰?   鍙? 鏃?
*	杩?鍥?鍊? 鏃?
*********************************************************************************************************
*/
u8 SPI_ReadByte(void)
{
	u8 i;
	u8 read = 0;
	
	for (i = 0; i < 8; i++)
	{
		ADS1256_SCLK_1();
		read = read<<1;
		delay_us(5);
		ADS1256_SCLK_0();
		
		if(ADS1256_DOUT)
				read ++;
		
		delay_us(5);
	}
	
	return read;
}


/*
*********************************************************************************************************
*	鍑?鏁?鍚? ADS1256WREG
*	鍔熻兘璇存槑: ADS1256 鍐欐暟鎹?
*	褰?   鍙? regaddr:瀵勫瓨鍣ㄥ湴鍧€
						databyte:寰呭啓鍏ユ暟鎹?
*	杩?鍥?鍊? 鏃?
*********************************************************************************************************
*/
void ADS1256WREG(u8 regaddr,u8 databyte)
{
	
	ADS1256_CS_0();
	if(ADS1256_WaitReady(ADS1256_WAIT_READY_TIMEOUT_US) == 0U)
	{
		ADS1256_CS_1();
		return;
	}
	
	//鍚戝瘎瀛樺櫒鍐欏叆鏁版嵁鍦板潃
	SPI_WriteByte(ADS1256_CMD_WREG | (regaddr & 0x0F));
	
	//鍐欏叆鏁版嵁鐨勪釜鏁皀-1
	SPI_WriteByte(0x00);
	
	delay_us(10);
	
	//鍚憆egaddr鍦板潃鎸囧悜鐨勫瘎瀛樺櫒鍐欏叆鏁版嵁databyte
	SPI_WriteByte(databyte);
	
	ADS1256_CS_1();
}

/*
*********************************************************************************************************
*	鍑?鏁?鍚? ADS1256RREG
*	鍔熻兘璇存槑: ADS1256 璇绘暟鎹?
*	褰?   鍙? regaddr:瀵勫瓨鍣ㄥ湴鍧€
*	杩?鍥?鍊? 鏃?
*********************************************************************************************************
*/
u8 ADS1256RREG(u8 regaddr)
{
	u8 databyte;
	
	ADS1256_CS_0();
	if(ADS1256_WaitReady(ADS1256_WAIT_READY_TIMEOUT_US) == 0U)
	{
		ADS1256_CS_1();
		return 0xFFU;
	}
	
	//鍚戝瘎瀛樺櫒鍐欏叆鏁版嵁鍦板潃
	SPI_WriteByte(ADS1256_CMD_RREG | (regaddr & 0x0F));
	
	//鍐欏叆鏁版嵁鐨勪釜鏁皀-1
	SPI_WriteByte(0x00);
	
	delay_us(10);
	
	//鍚憆egaddr鍦板潃鎸囧悜鐨勫瘎瀛樺櫒鍐欏叆鏁版嵁databyte
	databyte=SPI_ReadByte();
	
	ADS1256_CS_1();

  return databyte;
}


/*
*********************************************************************************************************
*	鍑?鏁?鍚? ADS1256_Init
*	鍔熻兘璇存槑: 鍒濆鍖朅DS1256
*	褰?   鍙? 鏃?
*	杩?鍥?鍊? 鏃?
*********************************************************************************************************
*/
void ADS1256_Init(void)
{
	Init_ADS1256_GPIO(); 																							  //ADS1256_GPIO鍒濆鍖?
	delay_ms(10);
	
	ADS1256_CS_0();
	delay_us(100);
	if(ADS1256_WaitReady(ADS1256_WAIT_READY_TIMEOUT_US) == 0U)
	{
		ADS1256_CS_1();
		return;
	}
	SPI_WriteByte(ADS1256_CMD_REST);																		//澶嶄綅
	delay_ms(10);
	
	if(ADS1256_WaitReady(ADS1256_WAIT_READY_TIMEOUT_US) == 0U)
	{
		ADS1256_CS_1();
		return;
	}
	SPI_WriteByte(ADS1256_CMD_SELFCAL);																	 //鑷姩鏍″噯
	
	if(ADS1256_WaitReady(ADS1256_WAIT_READY_TIMEOUT_US) == 0U)
	{
		ADS1256_CS_1();
		return;
	}
	SPI_WriteByte(ADS1256_CMD_SYNC);                 										//鍚屾
	SPI_WriteByte(ADS1256_CMD_WAKEUP);              										//鍚屾鍞ら啋
	
	if(ADS1256_WaitReady(ADS1256_WAIT_READY_TIMEOUT_US) == 0U)
	{
		ADS1256_CS_1();
		return;
	}
	ADS1256WREG(ADS1256_STATUS,0x04);               										// 楂樹綅鍦ㄥ墠銆佷笉浣跨敤缂撳啿
	ADS1256WREG(ADS1256_MUX,ADS1256_MUXP_AIN0 | ADS1256_MUXN_AINCOM);		//璁剧疆閫氶亾
	ADS1256WREG(ADS1256_ADCON,ADS1256_GAIN_1);      										// 鏀惧ぇ鍊嶆暟1
	ADS1256WREG(ADS1256_DRATE,ADS1256_DRATE_30000SPS); 									// 鏁版嵁1000sps,涓嶅悓閫熺巼娉ㄦ剰閲囨牱鏃堕棿闂撮殧
	ADS1256WREG(ADS1256_IO,0x00);  																			//涓嶄娇鐢ㄦ暟鎹甀O鍙?
	
	if(ADS1256_WaitReady(ADS1256_WAIT_READY_TIMEOUT_US) == 0U)
	{
		ADS1256_CS_1();
		return;
	}
	SPI_WriteByte(ADS1256_CMD_SELFCAL);																	 //鑷姩鏍″噯
	ADS1256_CS_1();

}             



/*
*********************************************************************************************************
*	鍑?鏁?鍚? ADS1256ReadData
*	鍔熻兘璇存槑: 璇诲彇24浣岮D鍊?
*	褰?   鍙? channel:閫氶亾閫夋嫨
*	杩?鍥?鍊? AD鍊?
*********************************************************************************************************
*/
u32 ADS1256ReadData(u8 channel)  
{
	u32 sum=0;

	if(ADS1256_WaitReady(ADS1256_WAIT_READY_TIMEOUT_US) == 0U)
	{
		return 0xFFFFFFFFU;
	}
	ADS1256WREG(ADS1256_MUX,channel);		//璁剧疆閫氶亾
	
  ADS1256_CS_0();
	delay_us(200);
	
	SPI_WriteByte(ADS1256_CMD_SYNC);		//鍚屾
	SPI_WriteByte(ADS1256_CMD_WAKEUP);	//鍚屾鍞ら啋               
	SPI_WriteByte(ADS1256_CMD_RDATA);   //璇诲彇鏁版嵁

	sum = (SPI_ReadByte() << 16);
	sum |= (SPI_ReadByte() << 8);
	sum |= SPI_ReadByte();

	ADS1256_CS_1();
	
	if(ADS1256_WaitReady(ADS1256_WAIT_READY_TIMEOUT_US) == 0U)
	{
		return 0xFFFFFFFFU;
	}
	
	return sum;
}

/*
*********************************************************************************************************
*	鍑?鏁?鍚? Get_Val
*	鍔熻兘璇存槑: 灏嗚鍙栫殑24浣岮D鍊艰浆鍖栨垚鐢靛帇鍊?
*	褰?   鍙? addata:AD鍊?
*	杩?鍥?鍊? 鐢靛帇鍊?
*********************************************************************************************************
*/
float Get_Val(u32 addata)
{
	 /*		PGA鐢盇DS1256鍒濆鍖栬缃紝VREF涓虹數璺殑鍩哄噯婧愶紝鏍规嵁瀹為檯鎯呭喌淇敼*/
	 u8 PGA=1;
	 float VREF=2.4918f;
	
	 float val;
	
	 /*  璐熺數鍘嬬殑杞寲  */
	 if(addata > 0x800000)
	 {
			addata=0xFFFFFF-addata;
		  val= -2*VREF/(PGA*8388607)*addata;
	 }
   else
			val=2*VREF/(PGA*8388607)*addata;

	 return val;
}



/*
*********************************************************************************************************
*	鍑?鏁?鍚? Middle_Filter
*	鍔熻兘璇存槑: 婊戝姩骞冲潎婊ゆ尝
*	褰?   鍙? channel:閫氶亾閫夋嫨
						times:鏁版嵁涓暟
*	杩?鍥?鍊? 婊ゆ尝鍚庣殑AD鍊?
*********************************************************************************************************
*/
u32 Moving_Average_Filter(u8 channel,u16 times) 
{

	 u16 i,counts=0;
	 u32 sum=0,data[100];
	 u32 value;
	 u32 max,min;


	 //璇诲彇times涓暟鎹?
	 while(counts < times)
	 {
			data[counts] = ADS1256ReadData(channel);
			if(data[counts] == 0xFFFFFFFFU)
			{
				return 0xFFFFFFFFU;
			}
			counts++;
	 }

	 max=min=data[0];
	 
	 for(i=0;i<times;i++)
	 {
		  if(data[i] > max)
				max=data[i];
			
			if(data[i] < min)
				min=data[i];
			
			sum+=data[i];
	 }

	 value=(sum - max - min)/(times - 2);
	 
	 return value;
}







static uint8_t ADS1256_WaitReady(uint32_t timeout_us)
{
	while(timeout_us-- > 0U)
	{
		if(ADS1256_DRDY == 0U)
		{
			return 1U;
		}
		delay_us(1);
	}

	return 0U;
}
