#ifndef __LED_H
#define __LED_H	 
#include "sys.h"
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEKս��STM32������
//LED��������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//�޸�����:2012/9/2
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2009-2019
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 
#define LED0 PCout(11)	// PC11 �� (�°�PCBȡ����)
#define LED1 PCout(12)	// PC12	��
#define SW1 PCout(5)	// PC5
#define SW2 PAout(15)	// PA15
#define EnableOut PCout(2) // PC2

void LED_Init(void);//��ʼ��
void SW_Init(void);//��ʼ��
	 				    
#endif
