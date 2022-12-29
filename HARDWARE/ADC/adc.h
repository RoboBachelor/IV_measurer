#ifndef __ADC_H
#define __ADC_H	
#include "sys.h"
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEKս��STM32������
//ADC ����	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//�޸�����:2012/9/7
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2009-2019
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 

#define ADC_DMA_BUF_LEN 100

void ADC1_Config(void);
void Copy_ADC_Buf(uint16_t *volt_dist, uint16_t* curr_dist, uint16_t dist_length, uint16_t s);


void Adc_Init(void);
u16  Get_Adc(u8 ch); 
u16 Get_Adc_Average(u8 ch,u8 times); 
float Get_Temprate(void);    //��ȡ�ڲ��¶ȴ������¶�ֵ 
#endif 
