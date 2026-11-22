#include "Communication.h"
#include "AD9959.h"
#include "Delay.h"
#include "usart.h"
#include "task_manage.h"

/*通信数据结构体*/
communicationDataStruct communicationData = {{0},{0}};

DDSDataStruct dds[4]; 
//扫频选择的通道
uint8_t sweep_channel;


extern uint8_t USART_RX_BUF[USART_REC_LEN];
/**
* @brief  获取CRC校验码
* @param  none
* @retval CRC校验码
*/
static u16 crc_16(u8 *Datas, u16 Length) 
{
	u8 j;
	u16 temp = 0xFFFF, i;

	for(i=0; i<Length; ++i)
	{
		temp ^= Datas[i];

		for(j=0; j<8; ++j)
		{
			if(temp & 1)
			{
				temp >>= 1;
				temp ^= 0xA001;
			}
			else
			{
				temp >>= 1;
			}
		}
	}

	return temp;
}


///**
// * @brief  USART1发送数据
// * @param  *sendData : 待发送的数据
//           length    : 数据长度
// * @retval 无
// */
//static void usartSendData(u8 *sendData,u16 length)
//{
//	u16 i;
//	
//	for(i=0 ; i<length ; i++)
//	{
//		USART_SendData(USART3,sendData[i]);
//		
//		while(USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
//	}
//}


///**
//* @brief  发送波形数据
//* @param  none
//* @retval none
//*/
//void sendOSCData(u32 fre) 
//{
//    
//    
//}

/**
* @brief  接收数据处理
* @param  none
* @retval none
*/
void rxDataprocess(void)
{	//uint8_t USART_RX_BUF[USART_REC_LEN];
	  u8  output,ch;
    u16 crcValue,value;
    u16 command;
	  u32 fre,range,phase,start_fre,stop_fre,step,step_time;
    
		//检测帧头帧尾
		if(USART_RX_BUF[0]==0xAA && USART_RX_BUF[1]==0xA5 &&USART_RX_BUF[RX_LENGTH-1] ==0x55)
		{ }
		else
			return ;
		
 
		crcValue = crc_16(&USART_RX_BUF[0], RX_LENGTH - 3);
		
		value = (u16)USART_RX_BUF[RX_LENGTH-3] << 8 ;
		
		value |=USART_RX_BUF[RX_LENGTH-2];
		
		//CRC验证
		if(value == crcValue)
		{
				//得到指令
				command = (u16)((USART_RX_BUF[2]<<8)|USART_RX_BUF[3]);
				
			    ch 	=  USART_RX_BUF[4];				//通道
				if(ch >= 4)
					return;
				sweep_channel = USART_RX_BUF[4];
				output 	=  USART_RX_BUF[5];				//输出开关
			
//			  if(output == 0)
//				{	
//					  /*	关闭扫频		*/
//						TIM_Cmd(TIM4, DISABLE);

//					  //将角度频率相位全部清空			
//					  Write_frequence(ch,0);
//					  Write_Amplitude(ch,0);
//					  Write_Phase(ch,0);
//				}
				
				 if(command == NORMAL) //普通模式
				{				
					    fre = (u32) USART_RX_BUF[6]<<24;	//频率
						fre |=(u32) USART_RX_BUF[7]<<16;
						fre |=(u32) USART_RX_BUF[8]<<8;
						fre |=(u32) USART_RX_BUF[9];
						
						dds[ch].fre = fre;
					
						range = (u32) USART_RX_BUF[10]<<24;//幅度
						range |=(u32) USART_RX_BUF[11]<<16;
						range |=(u32) USART_RX_BUF[12]<<8;
						range |=(u32) USART_RX_BUF[13];
					
					  range = range/1000.0f;
					  if(output == 0)
						  range = 0;
					  dds[ch].range = range;
					
					    phase = (u32) USART_RX_BUF[14]<<24;//相位
						phase |=(u32) USART_RX_BUF[15]<<16;
						phase |=(u32) USART_RX_BUF[16]<<8;
						phase |=(u32) USART_RX_BUF[17];
						
					  phase = phase/100.0f;
						dds[ch].phase = phase;
						
						dds[ch].flag = USART_RX_BUF[34];
						
							  /*	关闭扫频		*/
								TIM_Cmd(TIM4, DISABLE);

								//设置频率角度和相位				


								/* F4 sweep frames update one physical DDS channel. */
								if(dds[ch].flag == DDS_UPDATE_SINGLE_FLAG)
								{
									Write_frequence_no_update(ch, dds[ch].fre);
									Write_Amplitude_no_update(ch, dds[ch].range);
									Write_Phase_no_update(ch, dds[ch].phase);
									IO_Update();
									dds[ch].flag = 0;
								}
								else if(dds[0].flag==DDS_UPDATE_BATCH_FLAG && dds[1].flag==DDS_UPDATE_BATCH_FLAG && dds[2].flag==DDS_UPDATE_BATCH_FLAG && dds[3].flag==DDS_UPDATE_BATCH_FLAG)
								{
									dds[0].flag = dds[1].flag = dds[2].flag = dds[3].flag = 0;
								Write_frequence_no_update(0,dds[0].fre);
								Write_frequence_no_update(1,dds[1].fre);
								Write_frequence_no_update(2,dds[2].fre);
								Write_frequence_no_update(3,dds[3].fre);
								
								Write_Amplitude_no_update(0,dds[0].range);
								Write_Amplitude_no_update(1,dds[1].range);
								Write_Amplitude_no_update(2,dds[2].range);
								Write_Amplitude_no_update(3,dds[3].range);

				
								Write_Phase_no_update(0,dds[0].phase	);
								Write_Phase_no_update(1,dds[1].phase);
								Write_Phase_no_update(2,dds[2].phase);
								Write_Phase_no_update(3,dds[3].phase);

								IO_Update();
							}
				}
				else if(command == SWEEP)//扫频模式
				{					
					  range = (u32) USART_RX_BUF[10]<<24;			//幅度
						range |=(u32) USART_RX_BUF[11]<<16;
						range |=(u32) USART_RX_BUF[12]<<8;
						range |=(u32) USART_RX_BUF[13];
						
						range = range/1000.0f;
					  //Extern_ad9958_data.channel[ch].range=range/1000.0f;
					
					  step = (u32) USART_RX_BUF[18]<<24;			//步进
						step |=(u32) USART_RX_BUF[19]<<16;
						step |=(u32) USART_RX_BUF[20]<<8;
						step |=(u32) USART_RX_BUF[21];
					
					  //Extern_ad9958_data.channel[ch].rising_delta=step;
					  //Extern_ad9958_data.channel[ch].falling_delta=step;
					
					  step_time = (u32) USART_RX_BUF[22]<<24;	//步进时间
						step_time |=(u32) USART_RX_BUF[23]<<16;
						step_time |=(u32) USART_RX_BUF[24]<<8;
						step_time |=(u32) USART_RX_BUF[25];
						
						//Extern_ad9958_data.channel[ch].rising_blanking_time=step_time;
						//Extern_ad9958_data.channel[ch].falling_blanking_time=step_time;
						
					
						start_fre = (u32) USART_RX_BUF[26]<<24;	//起始频率
						start_fre |=(u32) USART_RX_BUF[27]<<16;
						start_fre |=(u32) USART_RX_BUF[28]<<8;
						start_fre |=(u32) USART_RX_BUF[29];
						
						//Extern_ad9958_data.channel[ch].start=start_fre;
					
						stop_fre = (u32) USART_RX_BUF[30]<<24;	//截止频率
						stop_fre |=(u32) USART_RX_BUF[31]<<16;
						stop_fre |=(u32) USART_RX_BUF[32]<<8;
						stop_fre |=(u32) USART_RX_BUF[33];
						
						//Extern_ad9958_data.channel[ch].stop=stop_fre;
						
						Task3_SweepFre(start_fre,stop_fre,step_time);
						TIM_Cmd(TIM4, ENABLE); //开启扫频
				}
		}
		else
		{
				return;
		}
    
}



