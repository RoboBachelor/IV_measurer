#include "can.h"
#include "stm32f10x.h"
#include <stdio.h>
#include "stm32f10x.h"
#include "led.h"
/*
 * CAN初始化
 * tsjw:重新同步跳跃时间单元.范围:CAN_SJW_1tq~ CAN_SJW_4tq
 * tbs2:时间段2的时间单元.   范围:CAN_BS2_1tq~CAN_BS2_8tq;
 * tbs1:时间段1的时间单元.   范围:CAN_BS1_1tq ~CAN_BS1_16tq
 * brp :波特率分频器.范围:1~1024;  tq=(brp)*tpclk1
 * 波特率=Fpclk1/((tbs1+1+tbs2+1+1)*brp);
 * mode:CAN_Mode_Normal,普通模式;CAN_Mode_LoopBack,回环模式;
 * Fpclk1的时钟在初始化的时候设置为36M,如果设置CAN_Mode_Init(CAN_SJW_1tq,CAN_BS2_8tq,CAN_BS1_9tq,4,CAN_Mode_LoopBack);
 * 则波特率为:36M/((8+9+1)*4)=500Kbps
 * 返回值:0,初始化OK;
 * 其他,初始化失败;
 */

u8 CAN_Mode_Init(u8 mode)
{
	GPIO_InitTypeDef	GPIO_InitStructure;
	CAN_InitTypeDef		CAN_InitStructure;
	CAN_FilterInitTypeDef	CAN_FilterInitStructure;

	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA, ENABLE );   /* 使能PORTA时钟 */

	RCC_APB1PeriphClockCmd( RCC_APB1Periph_CAN1, ENABLE );                          /* 使能CAN1时钟 */


	GPIO_InitStructure.GPIO_Pin	= GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Speed	= GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode	= GPIO_Mode_AF_PP;                              /*复用推挽 */
	GPIO_Init( GPIOA, &GPIO_InitStructure );                                        /* 初始化IO */

	GPIO_InitStructure.GPIO_Pin	= GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Speed	= GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode	= GPIO_Mode_IPU;                                /*上拉输入 */
	GPIO_Init( GPIOA, &GPIO_InitStructure );                                        /* 初始化IO */


	/* CAN单元设置 */
	CAN_InitStructure.CAN_TTCM	= DISABLE;                                      /* 非时间触发通信模式 */
	CAN_InitStructure.CAN_ABOM	= ENABLE;                                       /* 软件自动离线管理 */
	CAN_InitStructure.CAN_AWUM	= DISABLE;                                      /* 睡眠模式通过软件唤醒(清除CAN->MCR的SLEEP位) */
	CAN_InitStructure.CAN_NART	= ENABLE;                                       /* 禁止报文自动传送 */
	CAN_InitStructure.CAN_RFLM	= DISABLE;                                      /* 报文不锁定,新的覆盖旧的 */
	CAN_InitStructure.CAN_TXFP	= ENABLE;                                       /* 优先级由报文标识符决定 */
	CAN_InitStructure.CAN_Mode	= mode;                                         /* 模式设置： mode:0,普通模式;1,回环模式; */
	/* 设置波特率 */
	CAN_InitStructure.CAN_SJW	= CAN_SJW_1tq;                                         /* 重新同步跳跃宽度(Tsjw)为tsjw+1个时间单位  CAN_SJW_1tq	 CAN_SJW_2tq CAN_SJW_3tq CAN_SJW_4tq */
	CAN_InitStructure.CAN_BS1	= CAN_BS1_3tq;                                         /* Tbs1=tbs1+1个时间单位CAN_BS1_1tq ~CAN_BS1_16tq */
	CAN_InitStructure.CAN_BS2	= CAN_BS2_2tq;                                         /* Tbs2=tbs2+1个时间单位CAN_BS2_1tq ~	CAN_BS2_8tq */
	CAN_InitStructure.CAN_Prescaler = 6;                                          /* 分频系数(Fdiv)为brp+1 */
	CAN_Init( CAN1, &CAN_InitStructure );                                           /* 初始化CAN1 */


	CAN_FilterInitStructure.CAN_FilterNumber		= 10;
	CAN_FilterInitStructure.CAN_FilterMode		= CAN_FilterMode_IdMask;
	CAN_FilterInitStructure.CAN_FilterScale		= CAN_FilterScale_32bit;
	CAN_FilterInitStructure.CAN_FilterIdHigh		= 0x8000;
	CAN_FilterInitStructure.CAN_FilterIdLow		= 0x0000;
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh		= 0x0000;
	CAN_FilterInitStructure.CAN_FilterMaskIdLow		= 0x0000;
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment	= 0;
	CAN_FilterInitStructure.CAN_FilterActivation		= ENABLE;
	CAN_FilterInit( &CAN_FilterInitStructure );
	CAN_ITConfig( CAN1, CAN_IT_FMP0, ENABLE );
	CAN_ITConfig( CAN1, CAN_IT_TME, ENABLE );
	

	/* NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); */

	NVIC_InitStructure.NVIC_IRQChannel			= USB_LP_CAN1_RX0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority	= 0;                    /* 主优先级为1 */
	NVIC_InitStructure.NVIC_IRQChannelSubPriority		= 1;                    /* 次优先级为0 */
	NVIC_InitStructure.NVIC_IRQChannelCmd			= ENABLE;
	NVIC_Init( &NVIC_InitStructure );

	return(0);
}

struct POS{
	int16_t AccX;
	int16_t AccY;
	int16_t AccZ;
	int16_t GyoX;
	int16_t GyoY;
	int16_t GyoZ;
	int16_t Pitch;
	int16_t Roll;
	int16_t Yaw;
}pos_mod;

struct RPOS{
	float AccX;
	float AccY;
	float AccZ;
	float GyoX;
	float GyoY;
	float GyoZ;
	float Pitch;
	float Roll;
	float Yaw;
}rpos_mod;

float Pitch,Roll,Yaw;
float AccX,AccY,AccZ;
float GyoX,GyoY,GyoZ;
void n_2_r(void)
{
	rpos_mod.AccX=((float)pos_mod.AccX)*0.001f;
	rpos_mod.AccY=((float)pos_mod.AccY)*0.001f;
	rpos_mod.AccZ=((float)pos_mod.AccZ)*0.001f;
	
	rpos_mod.GyoX=(float)pos_mod.GyoX*0.1f;
	rpos_mod.GyoY=(float)pos_mod.GyoY*0.1f;
	rpos_mod.GyoZ=(float)pos_mod.GyoZ*0.1f;
	
	rpos_mod.Pitch=(float)pos_mod.Pitch*0.01f;
	rpos_mod.Roll=(float)pos_mod.Roll*0.01f;
	rpos_mod.Yaw=(float)pos_mod.Yaw*0.1f;
	
	Pitch=rpos_mod.Pitch;
	Roll=rpos_mod.Roll;
	Yaw=rpos_mod.Yaw;
	
	AccX=rpos_mod.AccX;
	AccY=rpos_mod.AccY;
	AccZ=rpos_mod.AccZ;
	
  GyoX=rpos_mod.GyoX;
	GyoY=rpos_mod.GyoY;
	GyoZ=rpos_mod.GyoZ;
} 

/********************************************************************
 ********************************************************************
 * Name：        void CAN1_RX0_IRQHandler(void) //CAN RX
 *
 * Description： CAN1的接收中断函数
 *
 *********************************************************************/
CanRxMsg rx_message;
void USB_LP_CAN1_RX0_IRQHandler( void )
{
	if ( CAN_GetITStatus( CAN1, CAN_IT_FMP0 ) != RESET )
	{
		CAN_ITConfig(CAN1,CAN_IT_TME,DISABLE);
		CAN_ClearITPendingBit( CAN1, CAN_IT_FMP0 );
		CAN_Receive( CAN1, CAN_FIFO0, &rx_message );
		if(rx_message.StdId==0x514)
		{
			pos_mod.AccX=(int16_t)((rx_message.Data[1]<<8)+rx_message.Data[0]);
			pos_mod.AccY=(int16_t)((rx_message.Data[3]<<8)+rx_message.Data[2]);
			pos_mod.AccZ=(int16_t)((rx_message.Data[5]<<8)+rx_message.Data[4]);
			n_2_r();
		}
		else if(rx_message.StdId==0x515)
		{
			pos_mod.GyoX=(int16_t)((rx_message.Data[1]<<8)+rx_message.Data[0]);
			pos_mod.GyoY=(int16_t)((rx_message.Data[3]<<8)+rx_message.Data[2]);
			pos_mod.GyoZ=(int16_t)((rx_message.Data[5]<<8)+rx_message.Data[4]);		
		}
		else if(rx_message.StdId==0x516)
		{
			pos_mod.Pitch=(int16_t)((rx_message.Data[1]<<8)+rx_message.Data[0]);
			pos_mod.Roll=(int16_t)((rx_message.Data[3]<<8)+rx_message.Data[2]);
			pos_mod.Yaw=(int16_t)((rx_message.Data[5]<<8)+rx_message.Data[4]);				
		}
	
		CAN_ITConfig(CAN1,CAN_IT_TME,ENABLE); 
	}
}
/********************************************************************************
 * 发送指令
 *********************************************************************************/
void CAN1_send_data(void)
{
	u8		mbox;
	u16		i = 0;
	CanTxMsg	TxMessage;

	TxMessage.StdId = 0x532;
	TxMessage.IDE	= CAN_Id_Standard;
	TxMessage.RTR	= CAN_RTR_Data;
	TxMessage.DLC	= 0x08;
  

	TxMessage.Data[0]	= 0x01;
	TxMessage.Data[1]	= 0x02;
	TxMessage.Data[2]	= 0x03;
	TxMessage.Data[3]	= 0x04;
	TxMessage.Data[4]	= 0x05;
	TxMessage.Data[5]	= 0x06;
	TxMessage.Data[6]	= 0x07;
	TxMessage.Data[7]	= 0x08;

  
	mbox = CAN_Transmit( CAN1, &TxMessage );
	while ( (CAN_TransmitStatus( CAN1, mbox ) == CAN_TxStatus_Failed) && (i < 0XFFF) )
		i++;                                                        /* 等待发送完成 */
}

