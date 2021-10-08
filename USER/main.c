#include "led.h"
#include "delay.h"
#include "sys.h"
#include "adc.h"
#include "dac.h"
#include "lcd.h"
#include "stdio.h"
#include "usart.h"
#include "key.h"
#include "usmart.h"
#include "malloc.h"
#include "MMC_SD.h"
#include "ff.h"
#include "exfuns.h"
#include "string.h"
#define max_storage		400
#define PLOT_ORIG_X		40
#define PLOT_ORIG_Y		300
#define PLOT_WIDTH		400.0f
#define PLOT_HEIGHT		240.0f
#define dis_scale		1.1f
#define sampling_resistor	0.008f
#define common_delay		1000

#define Cap_Val		0.0047f //4700uF
#define Vol_Step  0.5f //0.5V

u8 SD_successed = 0;
u32	sd_total, sd_free;

float	Vpv;
float	Ipv;
float	Voc;
float	Isc;
float	Pmpp;
float	Vmpp;
float	Impp;

float V_max_plot, I_max_plot, P_max_plot;

/* 设计期望可以满足 50V 5A */
float	vol_normal	= (3.3 / 4096.0) * 10.5;
float	cur_normal	= (3.3 / 4096.0) / 50.0 / 0.008;//(3.3 / 4096.0) / 50.0 / 0.008
float	cap_normal	= (3.3 / 4096.0) * 21.0;

//float	v_record[max_storage] = { 0.0 };        /* 存储记录的电压值 */
//float	i_record[max_storage] = { 0.0 };        /* 存储记录的电流值 */

u32 delay_cal=1;

u8 test_mode=1;//0 固定时间延迟 1 模式1
float Vol_Step_Times_Cap_Val=Vol_Step*Cap_Val;

#define RECORD_LEN 800
uint16_t voltage_rec[RECORD_LEN];
uint16_t current_rec[RECORD_LEN];


void plot_dot( float x, float y )
{
	u16	real_x	= 0;
	u16	real_y	= 0;
	if ( Voc > 0.0 && Isc > 0.0f )
	{
		real_x	= PLOT_ORIG_X + (u16) ( (x / (Voc * dis_scale) ) * PLOT_WIDTH);
		real_y	= PLOT_ORIG_Y - (u16) ( (y / (Isc * dis_scale) ) * PLOT_HEIGHT);
		LCD_Fast_DrawPoint( real_x, real_y, BLACK );
	}
}


void plot_xy_axis( void )
{
	POINT_COLOR = BLACK;
	LCD_DrawLine( PLOT_ORIG_X, PLOT_ORIG_Y, PLOT_ORIG_X, PLOT_ORIG_Y - PLOT_HEIGHT );
	LCD_DrawLine( PLOT_ORIG_X + PLOT_WIDTH, PLOT_ORIG_Y, PLOT_ORIG_X + PLOT_WIDTH, PLOT_ORIG_Y - PLOT_HEIGHT );
	LCD_DrawLine( PLOT_ORIG_X, PLOT_ORIG_Y, PLOT_ORIG_X + PLOT_WIDTH, PLOT_ORIG_Y );
}


void plot_lineIV( float v1, float i1, float v2, float i2 )
{
	if ( v1 < 0.0f || v2 <  0.0f || i1 < 0.0f || i2 < 0.f ){
		return;
	}
	if ( v1 > V_max_plot || v2 > V_max_plot || i1 > I_max_plot || i2 > I_max_plot){
		return;
	}
	
		
	uint16_t real_v1 = PLOT_ORIG_X + v1 / V_max_plot * PLOT_WIDTH;
	uint16_t real_i1 = PLOT_ORIG_Y - i1 / I_max_plot * PLOT_HEIGHT;
	uint16_t real_v2 = PLOT_ORIG_X + v2 / V_max_plot * PLOT_WIDTH;
	uint16_t real_i2 = PLOT_ORIG_Y - i2 / I_max_plot * PLOT_HEIGHT;
	LCD_DrawLine( real_v1, real_i1, real_v2, real_i2 );

}


void plot_linePV( float v1, float p1, float v2, float p2 )
{	
	if ( v1 < 0.0f || v2 <  0.0f || p1 < 0.0f || p2 < 0.f ){
		return;
	}
	if ( v1 > V_max_plot || v2 > V_max_plot || p1 > P_max_plot || p2 > P_max_plot){
		return;
	}
	
		
	uint16_t real_v1 = PLOT_ORIG_X + v1 / V_max_plot * PLOT_WIDTH;
	uint16_t real_p1 = PLOT_ORIG_Y - p1 / P_max_plot * PLOT_HEIGHT;
	uint16_t real_v2 = PLOT_ORIG_X + v2 / V_max_plot * PLOT_WIDTH;
	uint16_t real_p2 = PLOT_ORIG_Y - p2 / P_max_plot * PLOT_HEIGHT;
	LCD_DrawLine( real_v1, real_p1, real_v2, real_p2 );

}


void dis_initial_info()
{
	u8 lcd_id[12];                                          /* 存放LCD ID字符串 */
	sprintf( (char *) lcd_id, "LCD ID:%04X", lcddev.id );   /* 将LCD ID打印到lcd_id数组。 */
	LCD_Clear( WHITE );
	POINT_COLOR = RED;
	LCD_ShowString( 30, 40, 200, 24, 24, "STM32 CPU Runing ^_^" );
	LCD_ShowString( 30, 70, 200, 16, 16, "TFTLCD TEST" );
	LCD_ShowString( 30, 90, 200, 16, 16, "WULI @ HITI Tech Co." );
	LCD_ShowString( 30, 110, 200, 16, 16, lcd_id );         /* 显示LCD ID */
	if ( SD_successed == 1 )
	{
		LCD_ShowString( 30, 130, 200, 16, 16, "SD Card Online" );
		//LCD_ShowNum( 30, 150, sd_total >> 10, 5, 16 );  /* 显示SD卡总容量 MB */
		//LCD_ShowNum( 100, 150, sd_free >> 10, 5, 16 );  /* 显示SD卡剩余容量 MB */
	}else  {
		LCD_ShowString( 30, 130, 200, 16, 16, "SD Card Offline" );
	}
}

void start_measure( void )
{
	u16	i = 0;
	u8	lcd_info[30];                   /* 存放LCD ID字符串 */
	u8	lcd_str[50];                    /* 存放LCD ID字符串 */
	u16	adcx, adcx1, adcx2;

	LCD_Clear( WHITE );
	POINT_COLOR = RED;

	//adcx	= Get_Adc_Average( ADC_Channel_0, 1 );
	//Vpv	= (float) adcx * vol_normal;

	LCD_ShowString( 10, 30, 200, 16, 16, "Vol Normal, start init..." );
	LED0 = 0;

	LCD_ShowString( 10, 30, 200, 16, 16, "Init Finshed, start mear..." );
	LED1	= 0;
	SW1	= 0;
	SW2	= 0;
	ADC_Cmd(ADC1, DISABLE);
	delay_ms( 200 );
	
	EnableOut = 1;
	
	Copy_ADC_Buf(voltage_rec, current_rec, RECORD_LEN, 16);

	LCD_ShowString( 10, 30, 200, 16, 16, "Mear Finshed, start dis..." );
	EnableOut	= 0;
	LED0		= 1;
	LED1		= 1;
	SW1	= 1;
	SW2	= 1;
	
	
	float voltage_tmp, current_tmp, power_tmp;
	float max_current = 0, max_power = 0;
	
	POINT_COLOR = BLACK;
	plot_xy_axis();
	for ( i = 0; i < RECORD_LEN; i++)
	{
		
		current_tmp = current_rec[i] * cur_normal;
		voltage_tmp = voltage_rec[i] * cap_normal - current_tmp * sampling_resistor;
		power_tmp = voltage_tmp * current_tmp;
		
		if ( current_tmp > max_current )
			max_current = current_tmp;
		if ( voltage_tmp * current_tmp > max_power){
			max_power = voltage_tmp * current_tmp;
			Vmpp = voltage_tmp;
			Impp = current_tmp;
		}
		
	}
	
	
	Voc	= voltage_tmp;
	Isc	= max_current;
	Pmpp = max_power;
	
	V_max_plot = Voc;
	I_max_plot = Isc * dis_scale;
	P_max_plot = Pmpp * dis_scale;

	
	for ( i = 0; i < RECORD_LEN; i++ )
	{
		
		float current_tmp_new = current_rec[i] * cur_normal;
		float voltage_tmp_new = voltage_rec[i] * cap_normal - current_tmp * sampling_resistor;
		float power_tmp_new = current_tmp_new * voltage_tmp_new;
		
		POINT_COLOR = RED;
		if ( i > 1 )
		{
			plot_lineIV(voltage_tmp, current_tmp, voltage_tmp_new, current_tmp_new);
		}
		
		POINT_COLOR = BLUE;
		if ( i > 1 )
		{
			plot_linePV(voltage_tmp, power_tmp, voltage_tmp_new, power_tmp_new);
		}
		
		current_tmp = current_tmp_new;
		voltage_tmp = voltage_tmp_new;
		power_tmp = power_tmp_new;
		
	}
	
	
	POINT_COLOR = LIGHTBLUE;
	uint16_t screen_Vmpp_x = PLOT_ORIG_X + Vmpp / V_max_plot * PLOT_WIDTH;
	uint16_t screen_Impp_y = PLOT_ORIG_Y - Impp / I_max_plot * PLOT_HEIGHT;
	uint16_t screen_Pmax_y = PLOT_ORIG_Y - Pmpp / P_max_plot * PLOT_HEIGHT;
	uint16_t screen_Imax_y = PLOT_ORIG_Y - Isc / I_max_plot * PLOT_HEIGHT;
	
	LCD_DrawLine( screen_Vmpp_x, PLOT_ORIG_Y, screen_Vmpp_x, PLOT_ORIG_Y - PLOT_HEIGHT );
	LCD_DrawLine( PLOT_ORIG_X, screen_Impp_y, screen_Vmpp_x, screen_Impp_y );
	LCD_DrawLine( screen_Vmpp_x, screen_Pmax_y, PLOT_ORIG_X + PLOT_WIDTH, screen_Pmax_y );

	sprintf( (char *) lcd_str, "Voc:%2.2fV Isc:%2.2fA Pmax:%3.2fW | Vmpp:%2.2fV Impp:%2.2fA", Voc, Isc, Pmpp, Vmpp, Impp ); /* 将Voc打印到lcd_id数组。 */

	POINT_COLOR = RED;
	LCD_ShowString( 10, 30, 460, 16, 16, lcd_str );
	
	POINT_COLOR = BLACK;
	sprintf((char *)lcd_str, "%.2fA", Isc);
	LCD_ShowString(0, screen_Imax_y - 8, 50, 16, 16, lcd_str);
	
	sprintf((char *)lcd_str, "%.2fA", Impp);
	LCD_ShowString(0, screen_Impp_y - 8, 50, 16, 16, lcd_str);

	sprintf((char *)lcd_str, "%.2fW", Pmpp);
	LCD_ShowString(PLOT_ORIG_X + PLOT_WIDTH, screen_Pmax_y - 8, 40, 16, 16, lcd_str);

	sprintf((char *)lcd_str, "%.2fV", Vmpp);
	LCD_ShowString(screen_Vmpp_x - 20, PLOT_ORIG_Y + 2, 48, 16, 16, lcd_str);
	
	sprintf((char *)lcd_str, "%.2fV", V_max_plot);
	LCD_ShowString(PLOT_ORIG_X + PLOT_WIDTH - 20, PLOT_ORIG_Y + 2, 48, 16, 16, lcd_str);
}



float v_real, v_cap;

void monitor_task()
{
	u8	lcd_str[30]; /* 存放LCD ID字符串 */
	u16	adcx;

	adcx	= Get_Adc_Average( ADC_Channel_0, 1 );
	v_real	= (float) adcx * vol_normal;

	adcx	= Get_Adc_Average( ADC_Channel_2, 1 );
	v_cap	= (float) adcx * cap_normal;

	sprintf( (char *) lcd_str, "Vol:%2.2fV Cap:%2.2fV   ", v_real, v_cap ); /* 将LCD ID打印到lcd_id数组。 */

	POINT_COLOR = RED;
	LCD_ShowString( 10, 10, 200, 24, 16, lcd_str );
}


/*******变量定义*****/
FIL	fil;
FRESULT res;
UINT	bww;
char	buf[100];

void save_SD( void )
{
	u8	num	= 0;
	u16	i	= 0;
	u8	res	= 0;
	u8	buf_str[50];                            /* 存放字符串 */
	u8	s_buf[50];                              /* 存放字符串 */
	u32	total, free;
	u8	t	= 0;
	u16	count	= 0;

	char str3[] = { "0" };

	exfuns_init();                                  /* 为fatfs相关变量申请内存 */
	mem_init();                                     /* 初始化内存池 */

	while ( SD_Initialize() )                       /* 检测SD卡 */
	{
		SD_successed = 0;
	}

	exfuns_init();                                  /* 为fatfs相关变量申请内存 */
	f_mount( fs[0], "0:", 1 );                      /* 挂载SD卡 */
	f_mount( fs[1], "1:", 1 );                      /* 挂载FLASH. */
	while ( exf_getfree( "0", &total, &free ) )     /* 得到SD卡的总容量和剩余容量 */
	{
		SD_successed = 0;
	}

	do
	{
		count += 1;
		sprintf( (char *) str3, "0:/%d.txt\0", count );                         /* 产生"123" */
		res = f_open( &fil, str3, FA_READ );
	}
	while ( res == 0 );

	sprintf( (char *) str3, "0:/%d.txt\0", count );                                 /* 产生"123" */
/********************start*************************/
	for ( i = 0; i < max_storage; i++ )
	{
		//sprintf( (char *) buf_str, "V:%f I:%f\r\n", v_record[i], i_record[i] ); /* 将所需要保存的内容打印到数组 */

		res = f_open( &fil, str3, FA_OPEN_ALWAYS | FA_WRITE );

		f_lseek( &fil, f_size( &fil ) );

		f_write( &fil, buf_str, strlen( buf_str ), &bww );

		f_close( &fil );
	}
	//Adc_Init(); /* ADC初始化 */
/********************start*************************/
}

uint32_t cnt = 0;

int main( void )
{
	u8 t = 0;
	delay_init();                                           /* 延时函数初始化 */
	uart_init( 9600 );                                      /* 串口初始化为9600 */
	LED_Init();                                             /* 初始化与LED连接的硬件接口 */
	
	KEY_Init();                                             /* 初始化与按键连接的硬件接口 */
	//Adc_Init();                                             /* ADC初始化 */
	ADC1_Config();
	
	LCD_Init();
	LCD_Display_Dir( 1 );
	delay_ms( 10 );
	dis_initial_info();
  SW_Init();

	NVIC_PriorityGroupConfig( NVIC_PriorityGroup_2 );       /* 设置中断优先级分组2 */
	//exfuns_init();                                          /* 为fatfs相关变量申请内存 */
	usmart_dev.init( 72 );
	mem_init();                                             /* 初始化内存池 */
 
	while ( SD_Initialize() )                               /* 检测SD卡 */
	{
		SD_successed = 0;
		/* LCD_ShowString(10,10,200,16,16,"SD Card Error!"); */
	}
	SD_successed = 1;
	
	exfuns_init();                                          /* 为fatfs相关变量申请内存 */
	f_mount( fs[0], "0:", 1 );                              /* 挂载SD卡 */
  /* f_mount(fs[1],"1:",1);                                       //挂载FLASH. */
	//while ( exf_getfree( "0", &sd_total, &sd_free ) )       /* 得到SD卡的总容量和剩余容量 */
	//{
	//	SD_successed = 0;
		/* LCD_ShowString(10,10,200,16,16,"Fatfs Error!"); */
	//}
	dis_initial_info();
 
	
	while ( 1 )
	{
		//printf( "Hello World\r\n" );
		t = KEY_Scan( 0 ); /* 得到键值 */
		switch ( t )
		{
		case KEY0_PRES:
		{
			start_measure();
			//dis_curve_IV();
			//dis_curve_PV();
			continue;
		}
		case KEY1_PRES:
		{
			LCD_Clear( WHITE );
			continue;
		}
		case WKUP_PRES:
		{
			LCD_ShowString( 10, 50, 200, 24, 24, "SD Runing ^_^" );
			save_SD();
			LCD_ShowString( 10, 50, 200, 24, 24, "SD END ^_^" );
		}
		}
		EnableOut	= 0;
		SW1		= 1;
		delay_ms( 10 );
		SW2 = 1;
		delay_ms( 10 );

		LED0	= 1;
		LED1	= 0;
		delay_ms( 50 );
		LED0	= 0;
		LED1	= 1;
		monitor_task();
	}
}


