#include "stm32f10x.h"
#include "delay.h"
#include "sys.h"
#include "usart.h"
#include "MY_time_Conf.h"
#include "lcd.h"
#include "adc.h"
#include "string.h"
#include "LDC1000.h"
/**************************
ģ��������Ϣ��
ѡ��оƬ�� stm32f103vc
������Ϣ��stm32f10x_hd.s
�̼��⣺stm32�̼���v3.5
***************************/

#define led_PB12   PBout(12)	   //λ�󶨣�λ����PB12����ΪLED����������ٶ����
//#define led_PB1   PBout(1)	   //λ�󶨣�λ����PB1����ΪLED����������ٶ����
//#define PA9   PAout(10)
//#define CLK1   PDout(0)
//#define CWW1   PDout(1)
//#define CLK2   PDout(2)
//#define CWW2   PDout(3)


//uint8_t i = 0;    //������

float a[3],w[3],angle[3],T;
extern unsigned char Re_buf[11],temp_buf[11],counter;
extern unsigned char sign;

u8 cishu=0,cishu1=0;






/*************************************************
����: int main(void)
����: main������
����: ��
����: ��
**************************************************/

int main(void)
{
		
		char xianshi[7];
		
		//u8 i;

		RCC_Configuration();      //��ʼ��ʱ��
		delay_init(72);            //��ʼ����ʱ����
		ADC_Configuration();
		Timer2_Init(1000,7200);
	  TIM3_PWM_Init(7200,1);
	 	TIM_SetCompare1(TIM3,3600);//�ֱ��Ӧ ���1��
		TIM_SetCompare2(TIM3,3600);//���1��
		TIM_SetCompare3(TIM3,3600);//���2��
		TIM_SetCompare4(TIM3,3600);//���2��
		TIM1_Configuration(2000,7200);
		TIM4_Configuration();
		TIM8_Configuration();
		TIM5_PWM_Init(9,1);
		LDC_SPI_Init();
		LDC1000_init();
		USART1_Config();   //��ʼ�� ����USART1
		USART2_Config();   //��ʼ�� ����USART2 �����ڽ���MPU6050���ݣ�
		LCD_Init();				//lcd��ʾ��ʼ��
		POINT_COLOR=RED;	//��������Ϊ��ɫ
		delay_ms(200);
    LCD_ShowString(30,40,210,24,24, "anjian start ^_^");//��ʾ����  
   while (1)
   {
      
						cishu=TIM4->CNT;
						cishu1=TIM8->CNT;
						sprintf(xianshi,"%06.2f",angle[2]);//z�� ��ʾ���
						LCD_ShowString(30,80,210,24,24,xianshi);
		 
						
					
         
         
      
      //delay_ms(50);
   }
}




