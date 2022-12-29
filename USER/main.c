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
#include "rtc.h"
#include "can.h"
#include "timer.h"
#include <math.h>

/*
改进 
1.增加实时时间现实
2.不插SD卡进入设置模式，可以通过按键设置时间，设置采样间隔，设置采样速度
3.大电容采样 小电容采样 充电模式（输入开路电压小于20V）
*/

struct RPOS{
	float Start_AccX;
	float Start_AccY;
	float Start_AccZ;
	float Start_GyoX;
	float Start_GyoY;
	float Start_GyoZ;
	float Start_Pitch;
	float Start_Roll;
	float Start_Yaw;
	float End_AccX;
	float End_AccY;
	float End_AccZ;
	float End_GyoX;
	float End_GyoY;
	float End_GyoZ;
	float End_Pitch;
	float End_Roll;
	float End_Yaw;
}pos_data;

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
float	Voc_collected;// //测量到的Voc 用于确保全量程
float	Voc_tmp;// //开始测量时的光伏板子电压
float V_max_plot, I_max_plot, P_max_plot;
float temp_inner_stm32;//单片机内部温度传感器的数值
/* 设计期望可以满足 70V 8A */
//分压比例（1000+49.9）/49.9约等于21 最大测69.3V 新版本换成了 10k和1M
float	vol_normal	= (3.3 / 4096.0) * 101.0 / 2.0;
//电流采样 INA240A2 50倍放大 0.008R 最大测8.25A
float	cur_normal	= (3.3 / 4096.0) / 50.0 / 0.008;//(3.3 / 4096.0) / 50.0 / 0.008
float	cap_normal	= (3.3 / 4096.0) * 21.0  ;
float diode_fd=1.0; //二极管正向压降
u32 delay_cal=1;

u8 test_mode=1;//0 固定时间延迟 1 模式1
float Vol_Step_Times_Cap_Val=Vol_Step*Cap_Val;

u8 TIME3_100MS_COUNT=0;// 循环模式的测量时间计数

/*** Config measurtimg time here. ***/
/*** Total measuring time = 42us * RECORD_LEN (max: 800) * RECORD_STRIDE (range: 1 - 50X) ***/
#define RECORD_LEN 800 //RECORD_LEN (max: 800)
#define RECORD_LEN_MAX 800 //RECORD_LEN (max: 800)
u8 RECORD_STRIDE=32; //RECORD_STRIDE (range: 1 - 50X)
u16 DELAY_STRIDE=60; //默认60s记录一次

uint16_t voltage_rec[RECORD_LEN_MAX];
uint16_t current_rec[RECORD_LEN_MAX];

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
	sprintf( (char *) lcd_id, "SD State:%x",SD_successed); /* 将LCD ID打印到lcd_id数组。 */
  LCD_ShowString( 30, 150, 200, 16, 16, lcd_id);  
	POINT_COLOR = DARKBLUE;
  LCD_ShowString( 30, 170, 300, 16, 16, "Left: Test Once"); 
	LCD_ShowString( 30, 190, 300, 16, 16, "Mid: Loop Test & Save to SD"); 
	LCD_ShowString( 30, 210, 300, 16, 16, "Right: Save to SD");
}

float v_real, v_cap;
extern float Pitch,Roll,Yaw;
extern float AccX,AccY,AccZ;
extern float GyoX,GyoY,GyoZ;
void monitor_task()
{
	u8	lcd_str[50]; /* 存放LCD ID字符串 */
	u16	adcx;

	adcx	= Get_Adc_Average( ADC_Channel_0, 1 );
	v_real	= (float) adcx * vol_normal;

	adcx	= Get_Adc_Average( ADC_Channel_2, 1 );
	v_cap	= (float) adcx * cap_normal;
	
  temp_inner_stm32=Get_Temprate();

	sprintf( (char *) lcd_str, "Vol:%2.2fV Cap:%2.2fV Temp:%2.2fC", v_real, v_cap, temp_inner_stm32); /* 将LCD ID打印到lcd_id数组。 */
	POINT_COLOR = RED;
	LCD_ShowString( 10, 5, 400, 24, 16, lcd_str );
	
	sprintf( (char *) lcd_str, "P:%3.1f R:%3.1f Y:%3.1f AXYZ:%1.3f %1.3f %1.3f", Pitch, Roll, Yaw, AccX, AccY, AccZ); /* 将LCD ID打印到lcd_id数组。 */
	POINT_COLOR = BLACK;
	LCD_ShowString( 10, 25, 500, 24, 16, lcd_str );
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
  
	LED0 = 0;
	
	SW1	= 0;//=0 关断MOS 即不放电
	SW2	= 0;//=0 关断MOS 即不放电
	
	LCD_ShowString( 10, 25, 200, 16, 16, "Vol Normal, start init..." ); 
	LCD_ShowString( 10, 25, 200, 16, 16, "Init Finshed, start mear..." );
	
	ADC_Cmd(ADC1, DISABLE);
	//delay_ms( 200 );
	
	Voc_tmp=v_real;//记录开始测量时的板子电压，即开路电压
	
	//记录姿态数据（测量开始时）
	pos_data.Start_AccX=AccX;pos_data.Start_AccY=AccY;pos_data.Start_AccZ=AccZ;
	pos_data.Start_GyoX=GyoX;pos_data.Start_GyoY=GyoY;pos_data.Start_GyoZ=GyoZ;
	pos_data.Start_Pitch=Pitch;pos_data.Start_Roll=Roll;pos_data.Start_Yaw=Yaw;
	
	EnableOut = 1;
	
	Copy_ADC_Buf(voltage_rec, current_rec, RECORD_LEN, RECORD_STRIDE);////RECORD_STRIDE (range: 1 - 50X) 完成测量
  
	//记录姿态数据（测量结束时）
	pos_data.End_AccX=AccX;pos_data.End_AccY=AccY;pos_data.End_AccZ=AccZ;
	pos_data.End_GyoX=GyoX;pos_data.End_GyoY=GyoY;pos_data.End_GyoZ=GyoZ;
	pos_data.End_Pitch=Pitch;pos_data.End_Roll=Roll;pos_data.End_Yaw=Yaw;
	
	LCD_ShowString( 10, 25, 200, 16, 16, "Mear Finshed, start dis..." );
	EnableOut	= 0;
	
	monitor_task();
	/*给电容放电*/
	if(v_cap>10.0f)//电容电压大于10V，先用100R的电阻放电
	{
		SW1	= 1;
		SW2 = 0; 
	}
	else //电容电压小于10V，用所有电阻放电
	{
		SW1	= 1; 
		SW2 = 1; 
	}
	
	float voltage_tmp, current_tmp, power_tmp;
	float max_current = 0, max_power = 0;
	
	POINT_COLOR = BLACK;
	plot_xy_axis();
	
	for ( i = 0; i < RECORD_LEN; i++)//计算最大功率点的位置
	{
		current_tmp = current_rec[i] * cur_normal;
		voltage_tmp = voltage_rec[i] * cap_normal + current_tmp * sampling_resistor;
		power_tmp = voltage_tmp * current_tmp;
		
		if ( current_tmp > max_current )
			max_current = current_tmp;
		if ( voltage_tmp * current_tmp > max_power){
			max_power = voltage_tmp * current_tmp;
			Vmpp = voltage_tmp;
			Impp = current_tmp;
		}
	}
	
	Voc_collected = voltage_tmp;
	if(Voc_collected < 0.95* Voc_tmp)//没有测量full range 增加测量时间
	{
//		RECORD_STRIDE=RECORD_STRIDE+5;//增加测量时间
//		if(RECORD_STRIDE>49)
//		{
//			RECORD_STRIDE=49;//最大49
//		}
	}

	Voc	= voltage_tmp;
	Isc	= max_current;
	Pmpp = max_power;
	
	V_max_plot = Voc;
	I_max_plot = Isc * dis_scale;
	P_max_plot = Pmpp * dis_scale;
	
	for ( i = 0; i < RECORD_LEN; i+=10 )//循环打点现实，应该不用循环每一个 for(i=10;控制条件;i+=10)
	{
		float current_tmp_new = current_rec[i] * cur_normal;
		float voltage_tmp_new = voltage_rec[i] * cap_normal - current_tmp_new * sampling_resistor;
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

	sprintf( (char *) lcd_str, "OC:%2.2fV SC:%2.2fA Pm:%3.2fW Vm:%2.2fV Im:%2.2fA T:%dms", Voc, Isc, Pmpp, Vmpp, Impp , TIME3_100MS_COUNT*100); /* 将Voc打印到lcd_id数组。 */

	POINT_COLOR = RED;
	LCD_ShowString( 10, 45, 460, 16, 16, lcd_str );
	
	POINT_COLOR = BLACK;
	sprintf((char *)lcd_str, "%.2fA", Isc);
	LCD_ShowString(0, screen_Imax_y - 8, 50, 16, 16, lcd_str);//打标 短路电流
	
	sprintf((char *)lcd_str, "%.2fA", Impp);
	LCD_ShowString(0, screen_Impp_y - 8, 50, 16, 16, lcd_str);//打标 最大功率点电流

	sprintf((char *)lcd_str, "%.2fW", Pmpp);//打标 最大功率点功率
	LCD_ShowString(PLOT_ORIG_X + PLOT_WIDTH, screen_Pmax_y - 8, 40, 16, 16, lcd_str);

	sprintf((char *)lcd_str, "%.2fV", Vmpp);//打标 最大功率点电压
	LCD_ShowString(screen_Vmpp_x - 20, PLOT_ORIG_Y + 2, 48, 16, 16, lcd_str);
	
	sprintf((char *)lcd_str, "%.2fV", V_max_plot);
	LCD_ShowString(PLOT_ORIG_X + PLOT_WIDTH - 20, PLOT_ORIG_Y + 2, 48, 16, 16, lcd_str);
	
	monitor_task();
	/*给电容放电*/
	if(v_cap>10.0f)//电容电压大于10V，先用100R的电阻放电
	{
		SW1		= 1;
		SW2 = 0; 
	}
	else //电容电压小于10V，用所有电阻放电
	{
		SW1		= 1; 
		SW2 = 1; 
	}
	
	LED0 = 1;
}


/*******变量定义*****/
FIL	fil;
FRESULT res;
UINT	bww;
char fname_str[50];

void save_SD( void )
{
	u8	num	= 0;
	u16	i	= 0;
	u8	res	= 0;
	char buf_str[50];                            /* 存放字符串 */
	u8	s_buf[50];                              /* 存放字符串 */
	u32	total, free;
	u8	t	= 0;
	u16	count	= 0;

	LED1 = 0; 
	
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
  
	sprintf(fname_str, "0:/%04d%02d%02d_%02d%02d%02d.csv", calendar.w_year, calendar.w_month, calendar.w_date, calendar.hour, calendar.min, calendar.sec);
	
	res = f_open( &fil, fname_str, FA_OPEN_ALWAYS | FA_WRITE );
	f_lseek( &fil, f_size( &fil ) );
	
	//电气测量数据
	sprintf( (char *) buf_str, "Voc, %f\n",Voc); //开路电压1
	f_write( &fil, buf_str, strlen( buf_str ), &bww );
	sprintf( (char *) buf_str, "Isc, %f\n",Isc);//短路电流2
	f_write( &fil, buf_str, strlen( buf_str ), &bww );
	sprintf( (char *) buf_str, "Vmpp, %f\n",Vmpp);//最大功率点电压3
	f_write( &fil, buf_str, strlen( buf_str ), &bww );
	sprintf( (char *) buf_str, "Impp, %f\n",Impp);//最大功率点电流4
	f_write( &fil, buf_str, strlen( buf_str ), &bww );
	sprintf( (char *) buf_str, "Pmpp, %f\n",Pmpp);//最大功率点功率5
	f_write( &fil, buf_str, strlen( buf_str ), &bww );
	sprintf( (char *) buf_str, "Time Cost, %d\n",TIME3_100MS_COUNT*100); //测量时间6
	f_write( &fil, buf_str, strlen( buf_str ), &bww );//TIME3_100MS_COUNT 测量时间
	sprintf( (char *) buf_str, "Time Cost, %d\n",TIME3_100MS_COUNT);//测量时间7
	f_write( &fil, buf_str, strlen( buf_str ), &bww );
	sprintf( (char *) buf_str, "Time Cost, %d\n",TIME3_100MS_COUNT);//测量时间8
	f_write( &fil, buf_str, strlen( buf_str ), &bww );
	sprintf( (char *) buf_str, "Time Cost, %d\n",TIME3_100MS_COUNT);//测量时间9
	f_write( &fil, buf_str, strlen( buf_str ), &bww );
	sprintf( (char *) buf_str, "Time Cost, %d\n",TIME3_100MS_COUNT);//测量时间10
	f_write( &fil, buf_str, strlen( buf_str ), &bww );
	sprintf( (char *) buf_str, "Time Cost, %d\n",TIME3_100MS_COUNT);//测量时间11
	f_write( &fil, buf_str, strlen( buf_str ), &bww );
	sprintf( (char *) buf_str, "Time Cost, %d\n",TIME3_100MS_COUNT);//测量时间12
	f_write( &fil, buf_str, strlen( buf_str ), &bww );
	sprintf( (char *) buf_str, "Time Cost, %d\n",TIME3_100MS_COUNT);//测量时间13
	f_write( &fil, buf_str, strlen( buf_str ), &bww );
	sprintf( (char *) buf_str, "Time Cost, %d\n",TIME3_100MS_COUNT);//测量时间14
	f_write( &fil, buf_str, strlen( buf_str ), &bww );
	
	//开始的姿态数据
	sprintf( (char *) buf_str, "Start_AccX, %f\n",pos_data.Start_AccX);
	f_write( &fil, buf_str, strlen( buf_str ), &bww );
	
	sprintf( (char *) buf_str, "Start_AccY, %f\n",pos_data.Start_AccY);
	f_write( &fil, buf_str, strlen( buf_str ), &bww );
	
	sprintf( (char *) buf_str, "Start_AccZ, %f\n",pos_data.Start_AccZ);
	f_write( &fil, buf_str, strlen( buf_str ), &bww );
	
	sprintf( (char *) buf_str, "Start_GyoX, %f\n",pos_data.Start_GyoX);
	f_write( &fil, buf_str, strlen( buf_str ), &bww );
	
	sprintf( (char *) buf_str, "Start_GyoY, %f\n",pos_data.Start_GyoY);
	f_write( &fil, buf_str, strlen( buf_str ), &bww );
	
	sprintf( (char *) buf_str, "Start_GyoZ, %f\n",pos_data.Start_GyoZ);
	f_write( &fil, buf_str, strlen( buf_str ), &bww );
	
	sprintf( (char *) buf_str, "Start_Pitch, %f\n",pos_data.Start_Pitch);
	f_write( &fil, buf_str, strlen( buf_str ), &bww );
	
	sprintf( (char *) buf_str, "Start_Roll, %f\n",pos_data.Start_Roll);
	f_write( &fil, buf_str, strlen( buf_str ), &bww );
	
	sprintf( (char *) buf_str, "Start_Yaw, %f\n",pos_data.Start_Yaw);
	f_write( &fil, buf_str, strlen( buf_str ), &bww );
	
	//结束时的姿态数据
	sprintf( (char *) buf_str, "End_AccX, %f\n",pos_data.End_AccX);
	f_write( &fil, buf_str, strlen( buf_str ), &bww );
	
	sprintf( (char *) buf_str, "End_AccY, %f\n",pos_data.End_AccY);
	f_write( &fil, buf_str, strlen( buf_str ), &bww );
	
	sprintf( (char *) buf_str, "End_AccZ, %f\n",pos_data.End_AccZ);
	f_write( &fil, buf_str, strlen( buf_str ), &bww );
	
	sprintf( (char *) buf_str, "End_GyoX, %f\n",pos_data.End_GyoX);
	f_write( &fil, buf_str, strlen( buf_str ), &bww );
	
	sprintf( (char *) buf_str, "End_GyoY, %f\n",pos_data.End_GyoY);
	f_write( &fil, buf_str, strlen( buf_str ), &bww );
	
	sprintf( (char *) buf_str, "End_GyoZ, %f\n",pos_data.End_GyoZ);
	f_write( &fil, buf_str, strlen( buf_str ), &bww );
	
	sprintf( (char *) buf_str, "End_Pitch, %f\n",pos_data.End_Pitch);
	f_write( &fil, buf_str, strlen( buf_str ), &bww );
	
	sprintf( (char *) buf_str, "End_Roll, %f\n",pos_data.End_Roll);
	f_write( &fil, buf_str, strlen( buf_str ), &bww );
	
	sprintf( (char *) buf_str, "End_Yaw, %f\n",pos_data.End_Yaw);
	f_write( &fil, buf_str, strlen( buf_str ), &bww );
	
	//IV 数据
	sprintf( (char *) buf_str, "Voltage, Curent\n");
	f_write( &fil, buf_str, strlen( buf_str ), &bww );
	
	for ( i = 0; i < RECORD_LEN; i++ )
	{
		float current_tmp = current_rec[i] * cur_normal;
		float voltage_tmp = voltage_rec[i] * cap_normal - current_tmp * sampling_resistor;
		
		sprintf( (char *) buf_str, "%f, %f\n", voltage_tmp, current_tmp ); /* 将所需要保存的内容打印到数组 */
		f_write( &fil, buf_str, strlen( buf_str ), &bww );
	}
	f_close( &fil );
	
	LED1 = 1;
}

uint32_t cnt = 0;
u8 Delay_Constant=1;  //两次ADC测量的延时 * 42us
u16 Record_Delay=1;  //两次记录的延时  
u8 Mode_Constant=1; //中间键选择设置模式
u8 Mode_Set=1;		//设置模式
u8 Dis_Flag=0;  //如果间隔时间小于10s则不显示
u32 Record_Len=800; //默认记录长度为800
int main( void )
{
	char buf_str[50];                            /* 存放字符串 */
	u8 t = 0;
	delay_init();                                           /* 延时函数初始化 */
	uart_init( 9600 );                                      /* 串口初始化为9600 */
	LED_Init();                                             /* 初始化与LED连接的硬件接口 */
	
	KEY_Init();                                             /* 初始化与按键连接的硬件接口 */ 
	ADC1_Config();                                          /* ADC初始化 */
	
	LCD_Init();
	LCD_Display_Dir( 1 );
	delay_ms( 10 );
	dis_initial_info();
  SW_Init();
	RTC_Init();
	
	NVIC_PriorityGroupConfig( NVIC_PriorityGroup_2 );       /* 设置中断优先级分组2 */
	usmart_dev.init( 72 );
	mem_init();                                             /* 初始化内存池 */
  
	LCD_Clear( WHITE );
	
	Delay_Constant=RECORD_STRIDE;//读取默认设置
	Record_Delay=DELAY_STRIDE; //读取默认设置
	
	TIM3_Int_Init(999,7199);//10Khz的计数频率，计数到1000为100ms 
	TIM_Cmd(TIM3, DISABLE);  //不使能TIMx外设	
	while (Mode_Set==1)                               /* 检测SD卡 */
	{
		POINT_COLOR = BLACK;
		sprintf( (char *) buf_str, "Now:%04d%02d%02d_%02d%02d%02d",calendar.w_year, calendar.w_month, calendar.w_date, calendar.hour, calendar.min, calendar.sec); /* 将LCD ID打印到lcd_id数组。 */
    LCD_ShowString( 30, 40, 450, 24, 24, buf_str); 
		LCD_ShowString( 30, 70, 450, 24, 24, "L:+(UP) M: Choose R:-(Down)" ); 
		
		sprintf( (char *) buf_str, "Mode Set:%d",Mode_Constant); /* 将LCD ID打印到lcd_id数组。 */
    LCD_ShowString( 30, 130, 200, 24, 24, buf_str); 
		
		POINT_COLOR = RED;
		if(Mode_Constant==1)
		{
			sprintf( (char *) buf_str, "Any Key to Check SD"); /* 将LCD ID打印到lcd_id数组。 */
			LCD_ShowString( 180, 130, 250, 24, 24, buf_str);
		}
		else if(Mode_Constant==2)//2 设置两次测量间的延时
		{
			sprintf( (char *) buf_str, "Record_Delay (s)"); /* 将LCD ID打印到lcd_id数组。 */
			LCD_ShowString( 180, 130, 250, 24, 24, buf_str);
		}
		else if(Mode_Constant==3)
		{
			sprintf( (char *) buf_str, "Delay_Constant*42us"); /* 将LCD ID打印到lcd_id数组。 */
			LCD_ShowString( 180, 130, 250, 24, 24, buf_str);
		}
		else if(Mode_Constant==4)
		{
			sprintf( (char *) buf_str, "Set Time"); /* 将LCD ID打印到lcd_id数组。 */
			LCD_ShowString( 180, 130, 250, 24, 24, buf_str);
		}
		else if(Mode_Constant==5)
		{
			sprintf( (char *) buf_str, "Record Data Len"); /* 将LCD ID打印到lcd_id数组。 */
			LCD_ShowString( 180, 130, 250, 24, 24, buf_str);
		}
		else if(Mode_Constant==6)
		{
			sprintf( (char *) buf_str, "Any Key to Continue"); /* 将LCD ID打印到lcd_id数组。 */
			LCD_ShowString( 180, 130, 250, 24, 24, buf_str);
		}
		
		POINT_COLOR = BLACK;
		sprintf( (char *) buf_str, "Delay_Constant:%d",Delay_Constant); /* 将LCD ID打印到lcd_id数组。 */
    LCD_ShowString( 30, 160, 200, 24, 24, buf_str); 
		
		SD_successed = SD_Initialize();
		POINT_COLOR = BLACK;
		sprintf( (char *) buf_str, "SD State:%x",SD_successed); /* 将LCD ID打印到lcd_id数组。 */
    LCD_ShowString( 30, 190, 200, 24, 24, buf_str); 
		
		POINT_COLOR = BLACK;
		sprintf( (char *) buf_str, "Record_Delay:%ds",Record_Delay); /* 将LCD ID打印到lcd_id数组。 */
    LCD_ShowString( 30, 220, 220, 24, 24, buf_str);

    POINT_COLOR = BLACK;
		sprintf( (char *) buf_str, "Data Len:%d",Record_Len); /* 将LCD ID打印到lcd_id数组。 */
    LCD_ShowString( 30, 250, 220, 24, 24, buf_str);		
		
		t = KEY_Scan( 1 ); /* 得到键值 */
		switch ( t )
		{
		case KEY0_PRES://左
		{
			LCD_Clear(WHITE);			
			POINT_COLOR = BLUE;
			LCD_ShowString( 30, 100, 200, 24, 24, "Left ^_^" ); 
			if(Mode_Constant==1)
			{	
				SD_successed = SD_Initialize();
				POINT_COLOR = BLACK;
				sprintf( (char *) buf_str, "SD State:%x",SD_successed); /* 将LCD ID打印到lcd_id数组。 */
				LCD_ShowString( 30, 190, 200, 24, 24, buf_str); 
			}
			else if(Mode_Constant==2)
			{
				if(Record_Delay<360)
				{
					Record_Delay+=1;
				} 
			}
			else if(Mode_Constant==3)
			{
				if(Delay_Constant<50)
				{
					Delay_Constant+=1;
				} 
			}
			else if(Mode_Constant==4)
			{
				RTC_Set(2020,12,7,12,0,0);
			}
			else if(Mode_Constant==5)
			{
				if(Record_Len<800)
				{
					Record_Len+=100;
				} 
			}
			else if(Mode_Constant==6)
			{
				Mode_Set=0;
			}
			//RTC_Set(2020,12,7,12,0,0); 
			continue;
		}
		case KEY1_PRES://中
		{ 
			LCD_Clear(WHITE);	
			POINT_COLOR = BLUE;
			LCD_ShowString( 30, 100, 200, 24, 24, "Middle ^_^" );  
			Mode_Constant+=1;
			if(Mode_Constant>6) //更改设置变量
				Mode_Constant=1;
			continue;
		}
		case WKUP_PRES://右
		{ 
			LCD_Clear(WHITE);	
			POINT_COLOR = BLUE;
			LCD_ShowString( 30, 100, 200, 24, 24, "Right ^_^" ); 
			if(Mode_Constant==1)
			{
				SD_successed = SD_Initialize();
				POINT_COLOR = BLACK;
				sprintf( (char *) buf_str, "SD State:%x",SD_successed); /* 将LCD ID打印到lcd_id数组。 */
				LCD_ShowString( 30, 190, 200, 24, 24, buf_str); 
			}
			else if(Mode_Constant==2)//设置两次测量间隔延时
			{
				if(Record_Delay>0)
				{
					Record_Delay-=1;
				}
			}
			else if(Mode_Constant==3)
			{
				if(Delay_Constant>0)
				{
					Delay_Constant-=1;
				} 
			}
			else if(Mode_Constant==4)
			{
				RTC_Set(2020,12,7,12,0,0);
			}
			else if(Mode_Constant==5)
			{
				if(Record_Len>100)
				{
					Record_Len-=100;
				} 
			}
			else if(Mode_Constant==6)
			{
				Mode_Set=0;
			}
			continue;
		}
		} 
		/* LCD_ShowString(10,10,200,16,16,"SD Card Error!"); */
	}
	RECORD_STRIDE=Delay_Constant;//更改默认设置
	DELAY_STRIDE=Record_Delay; //更改默认设置
	while(SD_Initialize())//检测不到 SD 卡
	{ 
		POINT_COLOR = BROWN;
		LCD_ShowString(30,220,200,24,24,"SD Card Error!"); delay_ms(500);
		LCD_ShowString(30,220,200,24,24,"Please Check! "); delay_ms(500);
	} 
	SD_successed = 1;
	exfuns_init();                                          /* 为fatfs相关变量申请内存 */
	f_mount( fs[0], "0:", 1 );                              /* 挂载SD卡 */ 
	dis_initial_info();
	
	CAN_Mode_Init(0);
	while ( 1 )
	{
		t = KEY_Scan( 0 ); /* 得到键值 */
		switch ( t )
		{
		case KEY0_PRES:
		{
			TIME3_100MS_COUNT=0;
			TIM_Cmd(TIM3, ENABLE);  //使能TIMx外设 记录时间
			start_measure(); 
			TIM_Cmd(TIM3, DISABLE);  //不使能TIMx外设 停止记录时间 
			continue;
		}
		case KEY1_PRES:
		{
			while(1){
				TIME3_100MS_COUNT=0;
				TIM_Cmd(TIM3, ENABLE);  //使能TIMx外设 记录时间
				start_measure();
				save_SD(); 
				TIM_Cmd(TIM3, DISABLE);  //不使能TIMx外设 停止记录时间
				float waiting_time_buf=DELAY_STRIDE-(TIME3_100MS_COUNT*100.0)/1000.0;
				uint8_t waiting_time = 0;//测量时间约花费2s
				if(waiting_time_buf<=1)
				{
					waiting_time=0;
				}
				else
				{
					waiting_time=ceil(waiting_time_buf);
				}
				while(waiting_time) {
					char lcd_str[30];
					sprintf(lcd_str, "Waiting : %02d", waiting_time);
					LCD_ShowString( 300, 5, 200, 16, 16, lcd_str);
				  monitor_task();
					/*给电容放电*/
					if(v_cap>10.0f)//电容电压大于10V，先用100R的电阻放电
					{
						SW1		= 1;
						SW2 = 0;
					}
					else //电容电压小于10V，用所有电阻放电
					{
						SW1		= 1;
						SW2 = 1;
					}
					delay_ms(1000);
					-- waiting_time;
				}
			}
			continue;
		}
		case WKUP_PRES:
		{
			LCD_ShowString( 10, 50, 200, 24, 24, "SD Runing ^_^" );
			save_SD();
			LCD_ShowString( 10, 50, 200, 24, 24, "SD END ^_^" );
			LCD_ShowString( 10, 150, 350, 24, 24, fname_str);
		}
		} 
		EnableOut	= 0;//断开光伏板和电容连接
		LED0	= 1;
		LED1	= 0;
		delay_ms( 50 );
		LED0	= 0;
		LED1	= 1;
		monitor_task();
		/*给电容放电*/
		if(v_cap>10.0f)//电容电压大于10V，先用100R的电阻放电
		{
			SW1 = 1;
			SW2 = 0;
			delay_ms( 20 );
    }
		else //电容电压小于10V，用所有电阻放电
		{
			SW1 = 1;
			delay_ms( 10 );
			SW2 = 1;
			delay_ms( 10 );
		}
	}
}

void TIM3_IRQHandler(void)   //TIM3中断
{
	if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET) //检查指定的TIM中断发生与否:TIM 中断源 
	{
		TIME3_100MS_COUNT+=1;
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update);  //清除TIMx的中断待处理位:TIM 中断源 
	}
}

