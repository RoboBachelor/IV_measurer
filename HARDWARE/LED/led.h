#ifndef __LED_H
#define __LED_H	 
#include "sys.h"
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK战舰STM32开发板
//LED驱动代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//修改日期:2012/9/2
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2009-2019
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 
#define LED0 PCout(11)	// PC11 红 (新版PCB取消了)
#define LED1 PCout(12)	// PC12	绿
#define SW1 PCout(5)	// PC5
#define SW2 PAout(15)	// PA15
#define EnableOut PCout(2) // PC2

void LED_Init(void);//初始化
void SW_Init(void);//初始化
	 				    
#endif
