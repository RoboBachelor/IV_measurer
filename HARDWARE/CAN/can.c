#include "can.h"
#include "stm32f10x.h"
#include <stdio.h>
#include "stm32f10x.h"
#include "led.h"
/*
 * CAN��ʼ��
 * tsjw:����ͬ����Ծʱ�䵥Ԫ.��Χ:CAN_SJW_1tq~ CAN_SJW_4tq
 * tbs2:ʱ���2��ʱ�䵥Ԫ.   ��Χ:CAN_BS2_1tq~CAN_BS2_8tq;
 * tbs1:ʱ���1��ʱ�䵥Ԫ.   ��Χ:CAN_BS1_1tq ~CAN_BS1_16tq
 * brp :�����ʷ�Ƶ��.��Χ:1~1024;  tq=(brp)*tpclk1
 * ������=Fpclk1/((tbs1+1+tbs2+1+1)*brp);
 * mode:CAN_Mode_Normal,��ͨģʽ;CAN_Mode_LoopBack,�ػ�ģʽ;
 * Fpclk1��ʱ���ڳ�ʼ����ʱ������Ϊ36M,�������CAN_Mode_Init(CAN_SJW_1tq,CAN_BS2_8tq,CAN_BS1_9tq,4,CAN_Mode_LoopBack);
 * ������Ϊ:36M/((8+9+1)*4)=500Kbps
 * ����ֵ:0,��ʼ��OK;
 * ����,��ʼ��ʧ��;
 */

u8 CAN_Mode_Init(u8 mode)
{
	GPIO_InitTypeDef	GPIO_InitStructure;
	CAN_InitTypeDef		CAN_InitStructure;
	CAN_FilterInitTypeDef	CAN_FilterInitStructure;

	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA, ENABLE );   /* ʹ��PORTAʱ�� */

	RCC_APB1PeriphClockCmd( RCC_APB1Periph_CAN1, ENABLE );                          /* ʹ��CAN1ʱ�� */


	GPIO_InitStructure.GPIO_Pin	= GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Speed	= GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode	= GPIO_Mode_AF_PP;                              /*�������� */
	GPIO_Init( GPIOA, &GPIO_InitStructure );                                        /* ��ʼ��IO */

	GPIO_InitStructure.GPIO_Pin	= GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Speed	= GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode	= GPIO_Mode_IPU;                                /*�������� */
	GPIO_Init( GPIOA, &GPIO_InitStructure );                                        /* ��ʼ��IO */


	/* CAN��Ԫ���� */
	CAN_InitStructure.CAN_TTCM	= DISABLE;                                      /* ��ʱ�䴥��ͨ��ģʽ */
	CAN_InitStructure.CAN_ABOM	= ENABLE;                                       /* ����Զ����߹��� */
	CAN_InitStructure.CAN_AWUM	= DISABLE;                                      /* ˯��ģʽͨ���������(���CAN->MCR��SLEEPλ) */
	CAN_InitStructure.CAN_NART	= ENABLE;                                       /* ��ֹ�����Զ����� */
	CAN_InitStructure.CAN_RFLM	= DISABLE;                                      /* ���Ĳ�����,�µĸ��Ǿɵ� */
	CAN_InitStructure.CAN_TXFP	= ENABLE;                                       /* ���ȼ��ɱ��ı�ʶ������ */
	CAN_InitStructure.CAN_Mode	= mode;                                         /* ģʽ���ã� mode:0,��ͨģʽ;1,�ػ�ģʽ; */
	/* ���ò����� */
	CAN_InitStructure.CAN_SJW	= CAN_SJW_1tq;                                         /* ����ͬ����Ծ���(Tsjw)Ϊtsjw+1��ʱ�䵥λ  CAN_SJW_1tq	 CAN_SJW_2tq CAN_SJW_3tq CAN_SJW_4tq */
	CAN_InitStructure.CAN_BS1	= CAN_BS1_3tq;                                         /* Tbs1=tbs1+1��ʱ�䵥λCAN_BS1_1tq ~CAN_BS1_16tq */
	CAN_InitStructure.CAN_BS2	= CAN_BS2_2tq;                                         /* Tbs2=tbs2+1��ʱ�䵥λCAN_BS2_1tq ~	CAN_BS2_8tq */
	CAN_InitStructure.CAN_Prescaler = 6;                                          /* ��Ƶϵ��(Fdiv)Ϊbrp+1 */
	CAN_Init( CAN1, &CAN_InitStructure );                                           /* ��ʼ��CAN1 */


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
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority	= 0;                    /* �����ȼ�Ϊ1 */
	NVIC_InitStructure.NVIC_IRQChannelSubPriority		= 1;                    /* �����ȼ�Ϊ0 */
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
 * Name��        void CAN1_RX0_IRQHandler(void) //CAN RX
 *
 * Description�� CAN1�Ľ����жϺ���
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
 * ����ָ��
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
		i++;                                                        /* �ȴ�������� */
}

