 #include "adc.h"
 #include "delay.h"
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK miniSTM32开发板
//ADC 代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//修改日期:2012/9/7
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2009-2019
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 
	   
		   
//初始化ADC
//这里我们仅以规则通道为例
//我们默认将开启通道0~3																	   
void  Adc_Init(void)
{ 	
	ADC_InitTypeDef ADC_InitStructure; 
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA |RCC_APB2Periph_ADC1	, ENABLE );	  //使能ADC1通道时钟
 

	RCC_ADCCLKConfig(RCC_PCLK2_Div6);   //设置ADC分频因子6 72M/6=12,ADC最大时间不能超过14M

	//PA0 作为模拟通道输入引脚                         
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;		//模拟输入引脚
	GPIO_Init(GPIOA, &GPIO_InitStructure);	
	
	//PA1 作为模拟通道输入引脚                         
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;		//模拟输入引脚
	GPIO_Init(GPIOA, &GPIO_InitStructure);	
	
	//PA2 作为模拟通道输入引脚                         
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;		//模拟输入引脚
	GPIO_Init(GPIOA, &GPIO_InitStructure);	
	
	//PA4 作为模拟通道输入引脚                         
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;		//模拟输入引脚
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	ADC_DeInit(ADC1);  //复位ADC1,将外设 ADC1 的全部寄存器重设为缺省值

	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;	//ADC工作模式:ADC1和ADC2工作在独立模式
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;	//模数转换工作在单通道模式
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;	//模数转换工作在单次转换模式
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;	//转换由软件而不是外部触发启动
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;	//ADC数据右对齐
	ADC_InitStructure.ADC_NbrOfChannel = 1;	//顺序进行规则转换的ADC通道的数目
	ADC_Init(ADC1, &ADC_InitStructure);	//根据ADC_InitStruct中指定的参数初始化外设ADCx的寄存器   

  
	ADC_Cmd(ADC1, ENABLE);	//使能指定的ADC1
	
	ADC_ResetCalibration(ADC1);	//使能复位校准  
	 
	while(ADC_GetResetCalibrationStatus(ADC1));	//等待复位校准结束
	
	ADC_StartCalibration(ADC1);	 //开启AD校准
 
	while(ADC_GetCalibrationStatus(ADC1));	 //等待校准结束
 
//	ADC_SoftwareStartConvCmd(ADC1, ENABLE);		//使能指定的ADC1的软件转换启动功能

}



static void ADC1_Init(void)
{
    ADC_InitTypeDef ADC_InitStructure;
		GPIO_InitTypeDef GPIO_InitStructure;

		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_ADC1, ENABLE );	  //使能ADC1通道时钟

		RCC_ADCCLKConfig(RCC_PCLK2_Div6);   //设置ADC分频因子6 72M/6=12,ADC最大时间不能超过14M

	//PA0 作为模拟通道输入引脚                         
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;		//模拟输入引脚
	GPIO_Init(GPIOA, &GPIO_InitStructure);	
	
	//PA1 作为模拟通道输入引脚                         
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;		//模拟输入引脚
	GPIO_Init(GPIOA, &GPIO_InitStructure);	
	
	//PA2 作为模拟通道输入引脚                         
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;		//模拟输入引脚
	GPIO_Init(GPIOA, &GPIO_InitStructure);	
	
	//PA4 作为模拟通道输入引脚                         
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;		//模拟输入引脚
	GPIO_Init(GPIOA, &GPIO_InitStructure);


	ADC_InitStructure.ADC_Mode=ADC_Mode_Independent; //????
	ADC_InitStructure.ADC_ScanConvMode=ENABLE;	//????,???????
	ADC_InitStructure.ADC_ContinuousConvMode=ENABLE; //??????
	ADC_InitStructure.ADC_ExternalTrigConv=ADC_ExternalTrigConv_None;//b?????????
	ADC_InitStructure.ADC_DataAlign=ADC_DataAlign_Right;//???????
	ADC_InitStructure.ADC_NbrOfChannel=2;		//??????
	ADC_Init(ADC1,&ADC_InitStructure);
	
	ADC_RegularChannelConfig(ADC1,ADC_Channel_0,1,ADC_SampleTime_239Cycles5);
	ADC_RegularChannelConfig(ADC1,ADC_Channel_1,2,ADC_SampleTime_239Cycles5);
	
	ADC_DMACmd(ADC1,ENABLE);	//?DMA????
	
	ADC_Cmd(ADC1,ENABLE);
	
	ADC_ResetCalibration(ADC1);	//???????
	
	while(ADC_GetResetCalibrationStatus(ADC1)); //??????
	
	ADC_StartCalibration(ADC1);	//ADC??
 
	while(ADC_GetCalibrationStatus(ADC1));	//??????
	
	ADC_SoftwareStartConvCmd(ADC1,ENABLE);	//????ADC??
 
	
}

#define ADC1_DR_Address ((u32)0x40012400+0X4C)

uint16_t ADC_DMA_Buf[ADC_DMA_BUF_LEN];

static void ADC1_DMA1_Init(void)
{
    DMA_InitTypeDef DMA_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);

    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    /* DMA1 Channel1 Configuration ----------------------------------------------*/
    DMA_DeInit(DMA1_Channel1);
    DMA_InitStructure.DMA_PeripheralBaseAddr = ADC1_DR_Address;
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)ADC_DMA_Buf;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
    DMA_InitStructure.DMA_BufferSize = ADC_DMA_BUF_LEN;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel1, &DMA_InitStructure);

    DMA_ITConfig(DMA1_Channel1, DMA_IT_TC | DMA_IT_HT, ENABLE);

		/* Enable DMA1 channel1 */
		DMA_Cmd(DMA1_Channel1, ENABLE);

}

void ADC1_Config(){
	ADC1_DMA1_Init();
	Adc_Init();
	ADC_DMACmd(ADC1, DISABLE);	//?DMA????
	DMA_Cmd(DMA1_Channel1, DISABLE);
}


/* --- ADC DMA Buffer Handler --- */

static uint16_t dist_cnt, dist_len;
static uint16_t source_cur_index, stride;
static uint16_t *volt_addr, *curr_addr;
static volatile uint8_t copy_flag;
static uint32_t debug_adc1_dma_it_tc_cnt = 0;

void Copy_ADC_Buf(uint16_t *volt_dist, uint16_t* curr_dist, uint16_t dist_length, uint16_t s){
	
	/* Check */
	
		
	/* Reset DMA */
	
	ADC_SoftwareStartConvCmd(ADC1, DISABLE);
	ADC_DMACmd(ADC1, DISABLE);	//?DMA????
	DMA_Cmd(DMA1_Channel1, DISABLE);
	ADC_Cmd(ADC1,DISABLE);
	ADC1_Init();
  DMA1_Channel1->CNDTR = ADC_DMA_BUF_LEN;
	
	/* Reset counter */
	dist_cnt = 0;
	dist_len = dist_length;
	source_cur_index = 0;
	stride = s;
	
	volt_addr = volt_dist;
	curr_addr = curr_dist;
	
	copy_flag = 1;
	
	/* Start DMA */
	DMA_Cmd(DMA1_Channel1, ENABLE);
	ADC_DMACmd(ADC1, ENABLE);	//?DMA????
	ADC_Cmd(ADC1, ENABLE);
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);
	
	while(copy_flag);
	
	ADC_DMACmd(ADC1, DISABLE);	//?DMA????
	DMA_Cmd(DMA1_Channel1, DISABLE);
	ADC_Cmd(ADC1,DISABLE);
	Adc_Init();
}


static void Copy_ADC_Buf_ISR(uint8_t HT){
	while(copy_flag){
		
		if(source_cur_index < ADC_DMA_BUF_LEN){
			volt_addr[dist_cnt] = ADC_DMA_Buf[source_cur_index];
			curr_addr[dist_cnt++] = ADC_DMA_Buf[source_cur_index+1];
			source_cur_index += stride + stride;
		}
			
		if(dist_cnt >= dist_len){
			copy_flag = 0;
			break;
		}
		
		if(source_cur_index >= ADC_DMA_BUF_LEN){
			source_cur_index -= ADC_DMA_BUF_LEN;
			break;
		}
		if(source_cur_index >= (ADC_DMA_BUF_LEN >> 1) && HT){
			break;
		}
	}

}


void DMA1_Channel1_IRQHandler(void)
{
    if(DMA_GetFlagStatus(DMA1_FLAG_HT1))
    {
        DMA_ClearITPendingBit(DMA_IT_HT);
				Copy_ADC_Buf_ISR(1);
        //memcpy(WriteBuff, ADC_ConvertedValue, ADC_BUFF_LEN*sizeof(uint16_t)/2);
    }

    if(DMA_GetFlagStatus(DMA1_FLAG_TC1))
    {
        DMA_ClearITPendingBit(DMA1_FLAG_TC1);
				Copy_ADC_Buf_ISR(0);
				++debug_adc1_dma_it_tc_cnt;
			//memcpy(WriteBuff+ADC_BUFF_LEN/2, ADC_ConvertedValue+ADC_BUFF_LEN/2, ADC_BUFF_LEN*sizeof(uint16_t)/2);
    }
}

//获得ADC值
//ch:通道值 0~3
u16 Get_Adc(u8 ch)   
{
  	//设置指定ADC的规则组通道，一个序列，采样时间
	ADC_RegularChannelConfig(ADC1, ch, 1, ADC_SampleTime_239Cycles5 );	//ADC1,ADC通道,采样时间为239.5周期	  			    
  
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);		//使能指定的ADC1的软件转换启动功能	
	 
	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC ));//等待转换结束

	return ADC_GetConversionValue(ADC1);	//返回最近一次ADC1规则组的转换结果
}

u16 Get_Adc_Average(u8 ch,u8 times)
{
	u32 temp_val=0;
	u8 t;
	for(t=0;t<times;t++)
	{
		temp_val+=Get_Adc(ch);
	}
	return temp_val/times;
	
} 	 



























