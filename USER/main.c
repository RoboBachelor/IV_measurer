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
�Ľ� 
1.����ʵʱʱ����ʵ
2.����SD����������ģʽ������ͨ����������ʱ�䣬���ò�����������ò����ٶ�
3.����ݲ��� С���ݲ��� ���ģʽ�����뿪·��ѹС��20V��
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
float	Voc_collected;// //��������Voc ����ȷ��ȫ����
float	Voc_tmp;// //��ʼ����ʱ�Ĺ�����ӵ�ѹ
float V_max_plot, I_max_plot, P_max_plot;
float temp_inner_stm32;//��Ƭ���ڲ��¶ȴ���������ֵ
/* ��������������� 70V 8A */
//��ѹ������1000+49.9��/49.9Լ����21 ����69.3V �°汾������ 10k��1M
float	vol_normal	= (3.3 / 4096.0) * 101.0 / 2.0;
//�������� INA240A2 50���Ŵ� 0.008R ����8.25A
float	cur_normal	= (3.3 / 4096.0) / 50.0 / 0.008;//(3.3 / 4096.0) / 50.0 / 0.008
float	cap_normal	= (3.3 / 4096.0) * 21.0  ;
float diode_fd=1.0; //����������ѹ��
u32 delay_cal=1;

u8 test_mode=1;//0 �̶�ʱ���ӳ� 1 ģʽ1
float Vol_Step_Times_Cap_Val=Vol_Step*Cap_Val;

u8 TIME3_100MS_COUNT=0;// ѭ��ģʽ�Ĳ���ʱ�����

/*** Config measurtimg time here. ***/
/*** Total measuring time = 42us * RECORD_LEN (max: 800) * RECORD_STRIDE (range: 1 - 50X) ***/
#define RECORD_LEN 800 //RECORD_LEN (max: 800)
#define RECORD_LEN_MAX 800 //RECORD_LEN (max: 800)
u8 RECORD_STRIDE=32; //RECORD_STRIDE (range: 1 - 50X)
u16 DELAY_STRIDE=60; //Ĭ��60s��¼һ��

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
	u8 lcd_id[12];                                          /* ���LCD ID�ַ��� */
	sprintf( (char *) lcd_id, "LCD ID:%04X", lcddev.id );   /* ��LCD ID��ӡ��lcd_id���顣 */
	LCD_Clear( WHITE );
	POINT_COLOR = RED;
	LCD_ShowString( 30, 40, 200, 24, 24, "STM32 CPU Runing ^_^" );
	LCD_ShowString( 30, 70, 200, 16, 16, "TFTLCD TEST" );
	LCD_ShowString( 30, 90, 200, 16, 16, "WULI @ HITI Tech Co." );
	LCD_ShowString( 30, 110, 200, 16, 16, lcd_id );         /* ��ʾLCD ID */
	if ( SD_successed == 1 )
	{
		LCD_ShowString( 30, 130, 200, 16, 16, "SD Card Online" );
		//LCD_ShowNum( 30, 150, sd_total >> 10, 5, 16 );  /* ��ʾSD�������� MB */
		//LCD_ShowNum( 100, 150, sd_free >> 10, 5, 16 );  /* ��ʾSD��ʣ������ MB */
	}else  {
		LCD_ShowString( 30, 130, 200, 16, 16, "SD Card Offline" );
	}
	sprintf( (char *) lcd_id, "SD State:%x",SD_successed); /* ��LCD ID��ӡ��lcd_id���顣 */
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
	u8	lcd_str[50]; /* ���LCD ID�ַ��� */
	u16	adcx;

	adcx	= Get_Adc_Average( ADC_Channel_0, 1 );
	v_real	= (float) adcx * vol_normal;

	adcx	= Get_Adc_Average( ADC_Channel_2, 1 );
	v_cap	= (float) adcx * cap_normal;
	
  temp_inner_stm32=Get_Temprate();

	sprintf( (char *) lcd_str, "Vol:%2.2fV Cap:%2.2fV Temp:%2.2fC", v_real, v_cap, temp_inner_stm32); /* ��LCD ID��ӡ��lcd_id���顣 */
	POINT_COLOR = RED;
	LCD_ShowString( 10, 5, 400, 24, 16, lcd_str );
	
	sprintf( (char *) lcd_str, "P:%3.1f R:%3.1f Y:%3.1f AXYZ:%1.3f %1.3f %1.3f", Pitch, Roll, Yaw, AccX, AccY, AccZ); /* ��LCD ID��ӡ��lcd_id���顣 */
	POINT_COLOR = BLACK;
	LCD_ShowString( 10, 25, 500, 24, 16, lcd_str );
}

void start_measure( void )
{
	u16	i = 0;
	u8	lcd_info[30];                   /* ���LCD ID�ַ��� */
	u8	lcd_str[50];                    /* ���LCD ID�ַ��� */
	u16	adcx, adcx1, adcx2;

	LCD_Clear( WHITE );
	POINT_COLOR = RED;

	//adcx	= Get_Adc_Average( ADC_Channel_0, 1 );
	//Vpv	= (float) adcx * vol_normal;
  
	LED0 = 0;
	
	SW1	= 0;//=0 �ض�MOS �����ŵ�
	SW2	= 0;//=0 �ض�MOS �����ŵ�
	
	LCD_ShowString( 10, 25, 200, 16, 16, "Vol Normal, start init..." ); 
	LCD_ShowString( 10, 25, 200, 16, 16, "Init Finshed, start mear..." );
	
	ADC_Cmd(ADC1, DISABLE);
	//delay_ms( 200 );
	
	Voc_tmp=v_real;//��¼��ʼ����ʱ�İ��ӵ�ѹ������·��ѹ
	
	//��¼��̬���ݣ�������ʼʱ��
	pos_data.Start_AccX=AccX;pos_data.Start_AccY=AccY;pos_data.Start_AccZ=AccZ;
	pos_data.Start_GyoX=GyoX;pos_data.Start_GyoY=GyoY;pos_data.Start_GyoZ=GyoZ;
	pos_data.Start_Pitch=Pitch;pos_data.Start_Roll=Roll;pos_data.Start_Yaw=Yaw;
	
	EnableOut = 1;
	
	Copy_ADC_Buf(voltage_rec, current_rec, RECORD_LEN, RECORD_STRIDE);////RECORD_STRIDE (range: 1 - 50X) ��ɲ���
  
	//��¼��̬���ݣ���������ʱ��
	pos_data.End_AccX=AccX;pos_data.End_AccY=AccY;pos_data.End_AccZ=AccZ;
	pos_data.End_GyoX=GyoX;pos_data.End_GyoY=GyoY;pos_data.End_GyoZ=GyoZ;
	pos_data.End_Pitch=Pitch;pos_data.End_Roll=Roll;pos_data.End_Yaw=Yaw;
	
	LCD_ShowString( 10, 25, 200, 16, 16, "Mear Finshed, start dis..." );
	EnableOut	= 0;
	
	monitor_task();
	/*�����ݷŵ�*/
	if(v_cap>10.0f)//���ݵ�ѹ����10V������100R�ĵ���ŵ�
	{
		SW1	= 1;
		SW2 = 0; 
	}
	else //���ݵ�ѹС��10V�������е���ŵ�
	{
		SW1	= 1; 
		SW2 = 1; 
	}
	
	float voltage_tmp, current_tmp, power_tmp;
	float max_current = 0, max_power = 0;
	
	POINT_COLOR = BLACK;
	plot_xy_axis();
	
	for ( i = 0; i < RECORD_LEN; i++)//��������ʵ��λ��
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
	if(Voc_collected < 0.95* Voc_tmp)//û�в���full range ���Ӳ���ʱ��
	{
//		RECORD_STRIDE=RECORD_STRIDE+5;//���Ӳ���ʱ��
//		if(RECORD_STRIDE>49)
//		{
//			RECORD_STRIDE=49;//���49
//		}
	}

	Voc	= voltage_tmp;
	Isc	= max_current;
	Pmpp = max_power;
	
	V_max_plot = Voc;
	I_max_plot = Isc * dis_scale;
	P_max_plot = Pmpp * dis_scale;
	
	for ( i = 0; i < RECORD_LEN; i+=10 )//ѭ�������ʵ��Ӧ�ò���ѭ��ÿһ�� for(i=10;��������;i+=10)
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

	sprintf( (char *) lcd_str, "OC:%2.2fV SC:%2.2fA Pm:%3.2fW Vm:%2.2fV Im:%2.2fA T:%dms", Voc, Isc, Pmpp, Vmpp, Impp , TIME3_100MS_COUNT*100); /* ��Voc��ӡ��lcd_id���顣 */

	POINT_COLOR = RED;
	LCD_ShowString( 10, 45, 460, 16, 16, lcd_str );
	
	POINT_COLOR = BLACK;
	sprintf((char *)lcd_str, "%.2fA", Isc);
	LCD_ShowString(0, screen_Imax_y - 8, 50, 16, 16, lcd_str);//��� ��·����
	
	sprintf((char *)lcd_str, "%.2fA", Impp);
	LCD_ShowString(0, screen_Impp_y - 8, 50, 16, 16, lcd_str);//��� ����ʵ����

	sprintf((char *)lcd_str, "%.2fW", Pmpp);//��� ����ʵ㹦��
	LCD_ShowString(PLOT_ORIG_X + PLOT_WIDTH, screen_Pmax_y - 8, 40, 16, 16, lcd_str);

	sprintf((char *)lcd_str, "%.2fV", Vmpp);//��� ����ʵ��ѹ
	LCD_ShowString(screen_Vmpp_x - 20, PLOT_ORIG_Y + 2, 48, 16, 16, lcd_str);
	
	sprintf((char *)lcd_str, "%.2fV", V_max_plot);
	LCD_ShowString(PLOT_ORIG_X + PLOT_WIDTH - 20, PLOT_ORIG_Y + 2, 48, 16, 16, lcd_str);
	
	monitor_task();
	/*�����ݷŵ�*/
	if(v_cap>10.0f)//���ݵ�ѹ����10V������100R�ĵ���ŵ�
	{
		SW1		= 1;
		SW2 = 0; 
	}
	else //���ݵ�ѹС��10V�������е���ŵ�
	{
		SW1		= 1; 
		SW2 = 1; 
	}
	
	LED0 = 1;
}


/*******��������*****/
FIL	fil;
FRESULT res;
UINT	bww;
char fname_str[50];

void save_SD( void )
{
	u8	num	= 0;
	u16	i	= 0;
	u8	res	= 0;
	char buf_str[50];                            /* ����ַ��� */
	u8	s_buf[50];                              /* ����ַ��� */
	u32	total, free;
	u8	t	= 0;
	u16	count	= 0;

	LED1 = 0; 
	
	exfuns_init();                                  /* Ϊfatfs��ر��������ڴ� */
	mem_init();                                     /* ��ʼ���ڴ�� */

	while ( SD_Initialize() )                       /* ���SD�� */
	{
		SD_successed = 0;
	}

	exfuns_init();                                  /* Ϊfatfs��ر��������ڴ� */
	f_mount( fs[0], "0:", 1 );                      /* ����SD�� */
	f_mount( fs[1], "1:", 1 );                      /* ����FLASH. */
	while ( exf_getfree( "0", &total, &free ) )     /* �õ�SD������������ʣ������ */
	{
		SD_successed = 0;
	}
  
	sprintf(fname_str, "0:/%04d%02d%02d_%02d%02d%02d.csv", calendar.w_year, calendar.w_month, calendar.w_date, calendar.hour, calendar.min, calendar.sec);
	
	res = f_open( &fil, fname_str, FA_OPEN_ALWAYS | FA_WRITE );
	f_lseek( &fil, f_size( &fil ) );
	
	//������������
	sprintf( (char *) buf_str, "Voc, %f\n",Voc); //��·��ѹ1
	f_write( &fil, buf_str, strlen( buf_str ), &bww );
	sprintf( (char *) buf_str, "Isc, %f\n",Isc);//��·����2
	f_write( &fil, buf_str, strlen( buf_str ), &bww );
	sprintf( (char *) buf_str, "Vmpp, %f\n",Vmpp);//����ʵ��ѹ3
	f_write( &fil, buf_str, strlen( buf_str ), &bww );
	sprintf( (char *) buf_str, "Impp, %f\n",Impp);//����ʵ����4
	f_write( &fil, buf_str, strlen( buf_str ), &bww );
	sprintf( (char *) buf_str, "Pmpp, %f\n",Pmpp);//����ʵ㹦��5
	f_write( &fil, buf_str, strlen( buf_str ), &bww );
	sprintf( (char *) buf_str, "Time Cost, %d\n",TIME3_100MS_COUNT*100); //����ʱ��6
	f_write( &fil, buf_str, strlen( buf_str ), &bww );//TIME3_100MS_COUNT ����ʱ��
	sprintf( (char *) buf_str, "Time Cost, %d\n",TIME3_100MS_COUNT);//����ʱ��7
	f_write( &fil, buf_str, strlen( buf_str ), &bww );
	sprintf( (char *) buf_str, "Time Cost, %d\n",TIME3_100MS_COUNT);//����ʱ��8
	f_write( &fil, buf_str, strlen( buf_str ), &bww );
	sprintf( (char *) buf_str, "Time Cost, %d\n",TIME3_100MS_COUNT);//����ʱ��9
	f_write( &fil, buf_str, strlen( buf_str ), &bww );
	sprintf( (char *) buf_str, "Time Cost, %d\n",TIME3_100MS_COUNT);//����ʱ��10
	f_write( &fil, buf_str, strlen( buf_str ), &bww );
	sprintf( (char *) buf_str, "Time Cost, %d\n",TIME3_100MS_COUNT);//����ʱ��11
	f_write( &fil, buf_str, strlen( buf_str ), &bww );
	sprintf( (char *) buf_str, "Time Cost, %d\n",TIME3_100MS_COUNT);//����ʱ��12
	f_write( &fil, buf_str, strlen( buf_str ), &bww );
	sprintf( (char *) buf_str, "Time Cost, %d\n",TIME3_100MS_COUNT);//����ʱ��13
	f_write( &fil, buf_str, strlen( buf_str ), &bww );
	sprintf( (char *) buf_str, "Time Cost, %d\n",TIME3_100MS_COUNT);//����ʱ��14
	f_write( &fil, buf_str, strlen( buf_str ), &bww );
	
	//��ʼ����̬����
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
	
	//����ʱ����̬����
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
	
	//IV ����
	sprintf( (char *) buf_str, "Voltage, Curent\n");
	f_write( &fil, buf_str, strlen( buf_str ), &bww );
	
	for ( i = 0; i < RECORD_LEN; i++ )
	{
		float current_tmp = current_rec[i] * cur_normal;
		float voltage_tmp = voltage_rec[i] * cap_normal - current_tmp * sampling_resistor;
		
		sprintf( (char *) buf_str, "%f, %f\n", voltage_tmp, current_tmp ); /* ������Ҫ��������ݴ�ӡ������ */
		f_write( &fil, buf_str, strlen( buf_str ), &bww );
	}
	f_close( &fil );
	
	LED1 = 1;
}

uint32_t cnt = 0;
u8 Delay_Constant=1;  //����ADC��������ʱ * 42us
u16 Record_Delay=1;  //���μ�¼����ʱ  
u8 Mode_Constant=1; //�м��ѡ������ģʽ
u8 Mode_Set=1;		//����ģʽ
u8 Dis_Flag=0;  //������ʱ��С��10s����ʾ
u32 Record_Len=800; //Ĭ�ϼ�¼����Ϊ800
int main( void )
{
	char buf_str[50];                            /* ����ַ��� */
	u8 t = 0;
	delay_init();                                           /* ��ʱ������ʼ�� */
	uart_init( 9600 );                                      /* ���ڳ�ʼ��Ϊ9600 */
	LED_Init();                                             /* ��ʼ����LED���ӵ�Ӳ���ӿ� */
	
	KEY_Init();                                             /* ��ʼ���밴�����ӵ�Ӳ���ӿ� */ 
	ADC1_Config();                                          /* ADC��ʼ�� */
	
	LCD_Init();
	LCD_Display_Dir( 1 );
	delay_ms( 10 );
	dis_initial_info();
  SW_Init();
	RTC_Init();
	
	NVIC_PriorityGroupConfig( NVIC_PriorityGroup_2 );       /* �����ж����ȼ�����2 */
	usmart_dev.init( 72 );
	mem_init();                                             /* ��ʼ���ڴ�� */
  
	LCD_Clear( WHITE );
	
	Delay_Constant=RECORD_STRIDE;//��ȡĬ������
	Record_Delay=DELAY_STRIDE; //��ȡĬ������
	
	TIM3_Int_Init(999,7199);//10Khz�ļ���Ƶ�ʣ�������1000Ϊ100ms 
	TIM_Cmd(TIM3, DISABLE);  //��ʹ��TIMx����	
	while (Mode_Set==1)                               /* ���SD�� */
	{
		POINT_COLOR = BLACK;
		sprintf( (char *) buf_str, "Now:%04d%02d%02d_%02d%02d%02d",calendar.w_year, calendar.w_month, calendar.w_date, calendar.hour, calendar.min, calendar.sec); /* ��LCD ID��ӡ��lcd_id���顣 */
    LCD_ShowString( 30, 40, 450, 24, 24, buf_str); 
		LCD_ShowString( 30, 70, 450, 24, 24, "L:+(UP) M: Choose R:-(Down)" ); 
		
		sprintf( (char *) buf_str, "Mode Set:%d",Mode_Constant); /* ��LCD ID��ӡ��lcd_id���顣 */
    LCD_ShowString( 30, 130, 200, 24, 24, buf_str); 
		
		POINT_COLOR = RED;
		if(Mode_Constant==1)
		{
			sprintf( (char *) buf_str, "Any Key to Check SD"); /* ��LCD ID��ӡ��lcd_id���顣 */
			LCD_ShowString( 180, 130, 250, 24, 24, buf_str);
		}
		else if(Mode_Constant==2)//2 �������β��������ʱ
		{
			sprintf( (char *) buf_str, "Record_Delay (s)"); /* ��LCD ID��ӡ��lcd_id���顣 */
			LCD_ShowString( 180, 130, 250, 24, 24, buf_str);
		}
		else if(Mode_Constant==3)
		{
			sprintf( (char *) buf_str, "Delay_Constant*42us"); /* ��LCD ID��ӡ��lcd_id���顣 */
			LCD_ShowString( 180, 130, 250, 24, 24, buf_str);
		}
		else if(Mode_Constant==4)
		{
			sprintf( (char *) buf_str, "Set Time"); /* ��LCD ID��ӡ��lcd_id���顣 */
			LCD_ShowString( 180, 130, 250, 24, 24, buf_str);
		}
		else if(Mode_Constant==5)
		{
			sprintf( (char *) buf_str, "Record Data Len"); /* ��LCD ID��ӡ��lcd_id���顣 */
			LCD_ShowString( 180, 130, 250, 24, 24, buf_str);
		}
		else if(Mode_Constant==6)
		{
			sprintf( (char *) buf_str, "Any Key to Continue"); /* ��LCD ID��ӡ��lcd_id���顣 */
			LCD_ShowString( 180, 130, 250, 24, 24, buf_str);
		}
		
		POINT_COLOR = BLACK;
		sprintf( (char *) buf_str, "Delay_Constant:%d",Delay_Constant); /* ��LCD ID��ӡ��lcd_id���顣 */
    LCD_ShowString( 30, 160, 200, 24, 24, buf_str); 
		
		SD_successed = SD_Initialize();
		POINT_COLOR = BLACK;
		sprintf( (char *) buf_str, "SD State:%x",SD_successed); /* ��LCD ID��ӡ��lcd_id���顣 */
    LCD_ShowString( 30, 190, 200, 24, 24, buf_str); 
		
		POINT_COLOR = BLACK;
		sprintf( (char *) buf_str, "Record_Delay:%ds",Record_Delay); /* ��LCD ID��ӡ��lcd_id���顣 */
    LCD_ShowString( 30, 220, 220, 24, 24, buf_str);

    POINT_COLOR = BLACK;
		sprintf( (char *) buf_str, "Data Len:%d",Record_Len); /* ��LCD ID��ӡ��lcd_id���顣 */
    LCD_ShowString( 30, 250, 220, 24, 24, buf_str);		
		
		t = KEY_Scan( 1 ); /* �õ���ֵ */
		switch ( t )
		{
		case KEY0_PRES://��
		{
			LCD_Clear(WHITE);			
			POINT_COLOR = BLUE;
			LCD_ShowString( 30, 100, 200, 24, 24, "Left ^_^" ); 
			if(Mode_Constant==1)
			{	
				SD_successed = SD_Initialize();
				POINT_COLOR = BLACK;
				sprintf( (char *) buf_str, "SD State:%x",SD_successed); /* ��LCD ID��ӡ��lcd_id���顣 */
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
		case KEY1_PRES://��
		{ 
			LCD_Clear(WHITE);	
			POINT_COLOR = BLUE;
			LCD_ShowString( 30, 100, 200, 24, 24, "Middle ^_^" );  
			Mode_Constant+=1;
			if(Mode_Constant>6) //�������ñ���
				Mode_Constant=1;
			continue;
		}
		case WKUP_PRES://��
		{ 
			LCD_Clear(WHITE);	
			POINT_COLOR = BLUE;
			LCD_ShowString( 30, 100, 200, 24, 24, "Right ^_^" ); 
			if(Mode_Constant==1)
			{
				SD_successed = SD_Initialize();
				POINT_COLOR = BLACK;
				sprintf( (char *) buf_str, "SD State:%x",SD_successed); /* ��LCD ID��ӡ��lcd_id���顣 */
				LCD_ShowString( 30, 190, 200, 24, 24, buf_str); 
			}
			else if(Mode_Constant==2)//�������β��������ʱ
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
	RECORD_STRIDE=Delay_Constant;//����Ĭ������
	DELAY_STRIDE=Record_Delay; //����Ĭ������
	while(SD_Initialize())//��ⲻ�� SD ��
	{ 
		POINT_COLOR = BROWN;
		LCD_ShowString(30,220,200,24,24,"SD Card Error!"); delay_ms(500);
		LCD_ShowString(30,220,200,24,24,"Please Check! "); delay_ms(500);
	} 
	SD_successed = 1;
	exfuns_init();                                          /* Ϊfatfs��ر��������ڴ� */
	f_mount( fs[0], "0:", 1 );                              /* ����SD�� */ 
	dis_initial_info();
	
	CAN_Mode_Init(0);
	while ( 1 )
	{
		t = KEY_Scan( 0 ); /* �õ���ֵ */
		switch ( t )
		{
		case KEY0_PRES:
		{
			TIME3_100MS_COUNT=0;
			TIM_Cmd(TIM3, ENABLE);  //ʹ��TIMx���� ��¼ʱ��
			start_measure(); 
			TIM_Cmd(TIM3, DISABLE);  //��ʹ��TIMx���� ֹͣ��¼ʱ�� 
			continue;
		}
		case KEY1_PRES:
		{
			while(1){
				TIME3_100MS_COUNT=0;
				TIM_Cmd(TIM3, ENABLE);  //ʹ��TIMx���� ��¼ʱ��
				start_measure();
				save_SD(); 
				TIM_Cmd(TIM3, DISABLE);  //��ʹ��TIMx���� ֹͣ��¼ʱ��
				float waiting_time_buf=DELAY_STRIDE-(TIME3_100MS_COUNT*100.0)/1000.0;
				uint8_t waiting_time = 0;//����ʱ��Լ����2s
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
					/*�����ݷŵ�*/
					if(v_cap>10.0f)//���ݵ�ѹ����10V������100R�ĵ���ŵ�
					{
						SW1		= 1;
						SW2 = 0;
					}
					else //���ݵ�ѹС��10V�������е���ŵ�
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
		EnableOut	= 0;//�Ͽ������͵�������
		LED0	= 1;
		LED1	= 0;
		delay_ms( 50 );
		LED0	= 0;
		LED1	= 1;
		monitor_task();
		/*�����ݷŵ�*/
		if(v_cap>10.0f)//���ݵ�ѹ����10V������100R�ĵ���ŵ�
		{
			SW1 = 1;
			SW2 = 0;
			delay_ms( 20 );
    }
		else //���ݵ�ѹС��10V�������е���ŵ�
		{
			SW1 = 1;
			delay_ms( 10 );
			SW2 = 1;
			delay_ms( 10 );
		}
	}
}

void TIM3_IRQHandler(void)   //TIM3�ж�
{
	if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET) //���ָ����TIM�жϷ������:TIM �ж�Դ 
	{
		TIME3_100MS_COUNT+=1;
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update);  //���TIMx���жϴ�����λ:TIM �ж�Դ 
	}
}

