#include "MY_time_Conf.h"

/************************************************************************************************
	* ��������TIM1_Configuration
	* �������ã�TIM1�ⲿ�������ģʽ��ʼ��
	* ��������
	* ��汾  ��ST3.5.0
*************************************************************************************************/
void TIM1_Configuration(void)
{
		TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure; 
		GPIO_InitTypeDef GPIO_InitStructure;
		//����GPIO
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA , ENABLE);
		//����AFIO
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
		//����TIM1
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; //
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; //50Mʱ���ٶ�
		GPIO_Init(GPIOA, &GPIO_InitStructure);
	 TIM_DeInit(TIM1);
	 TIM_TimeBaseStructure.TIM_Period = 0xFFFF; 
	 TIM_TimeBaseStructure.TIM_Prescaler = 0x00; 
	 TIM_TimeBaseStructure.TIM_ClockDivision = 0x0;  
	 TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  
	 TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure); // 
	 TIM_ETRClockMode1Config(TIM1, TIM_ExtTRGPSC_OFF, TIM_ExtTRGPolarity_NonInverted, 0x00); 
	 TIM_SetCounter(TIM1, 0); 
	 TIM_Cmd(TIM1, ENABLE); 
}
void Timer2_Init(u16 arr,u16 psc)  //��ʱ��2��ʼ������
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);	//ʹ��TIM2ʱ��
	//��ʼ����ʱ��2 TIM2	 
	TIM_TimeBaseStructure.TIM_Period = arr-1; //�趨�������Զ���װֵ 
	TIM_TimeBaseStructure.TIM_Prescaler =psc-1; 	//Ԥ��Ƶ��   
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure); //����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ

	//�жϷ����ʼ��
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;  //TIM2�ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;  //��ռ���ȼ�2��
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  //�����ȼ�0��
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQͨ����ʹ��
	NVIC_Init(&NVIC_InitStructure);  //����NVIC_InitStruct��ָ���Ĳ�����ʼ������NVIC�Ĵ��� 
	
	TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);//��������ж� ,����CC1IE�����ж�	
	
   	TIM_Cmd(TIM2,ENABLE ); 	//ʹ�ܶ�ʱ��2
}


void TIM3_PWM_Init(u16 arr,u16 psc)
{  
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);	//ʹ�ܶ�ʱ��4ʱ��
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC  | RCC_APB2Periph_AFIO, ENABLE);  //ʹ��GPIO�����AFIO���ù���ģ��ʱ��.��������
	GPIO_PinRemapConfig(GPIO_FullRemap_TIM3 , ENABLE);
   //���ø�����Ϊ�����������,���TIM3 CH2��PWM���岨��	GPIOB.8
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9; //TIM_CH2
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //�����������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);//��ʼ��GPIO
   //��ʼ��TIM3
	TIM_TimeBaseStructure.TIM_Period = arr-1; //��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ
	TIM_TimeBaseStructure.TIM_Prescaler =psc-1; //����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ 
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); //����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ
	
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; //ѡ��ʱ��ģʽ:TIM�����ȵ���ģʽ2
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //�Ƚ����ʹ��
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low; //�������:TIM����Ƚϼ��Ը�
	TIM_OC1Init(TIM3, &TIM_OCInitStructure);  //����Tָ���Ĳ�����ʼ������TIM4 OC1
	
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; //ѡ��ʱ��ģʽ:TIM�����ȵ���ģʽ2
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //�Ƚ����ʹ��
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low; //�������:TIM����Ƚϼ��Ը�
	TIM_OC2Init(TIM3, &TIM_OCInitStructure);  //����Tָ���Ĳ�����ʼ������TIM4 OC2
	
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; //ѡ��ʱ��ģʽ:TIM�����ȵ���ģʽ2
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //�Ƚ����ʹ��
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low; //�������:TIM����Ƚϼ��Ը�
	TIM_OC3Init(TIM3, &TIM_OCInitStructure);  //����Tָ���Ĳ�����ʼ������TIM4 OC3
	
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; //ѡ��ʱ��ģʽ:TIM�����ȵ���ģʽ2
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //�Ƚ����ʹ��
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low; //�������:TIM����Ƚϼ��Ը�
	TIM_OC4Init(TIM3, &TIM_OCInitStructure);  //����Tָ���Ĳ�����ʼ������TIM4 OC4

	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);  //ʹ��TIM3��CCR3�ϵ�Ԥװ�ؼĴ���
	TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);
	TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);
	TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);
	
 
	TIM_Cmd(TIM3, ENABLE);  //ʹ��TIM3
 	TIM_SetCompare1(TIM3,0);
 	TIM_SetCompare2(TIM3,0);
 	TIM_SetCompare3(TIM3,0);
 	TIM_SetCompare4(TIM3,0);//��ֹ�������ת��


}
/************************************************************************************************
	* ��������TIM4_Configuration
	* �������ã�TIM4�ⲿ�������ģʽ��ʼ��
	* ��������
	* ��汾  ��ST3.5.0
*************************************************************************************************/
void TIM4_Configuration(void)
{
		TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure; 
		GPIO_InitTypeDef GPIO_InitStructure;
		
		//����GPIO
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE , ENABLE);
		//����AFIO
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
		//����TIM1
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
		
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; //
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; //50Mʱ���ٶ�
		GPIO_Init(GPIOE, &GPIO_InitStructure);
	
		TIM_DeInit(TIM4);
		TIM_TimeBaseStructure.TIM_Period = 0xFFFF; 
		TIM_TimeBaseStructure.TIM_Prescaler = 0x00; 
		TIM_TimeBaseStructure.TIM_ClockDivision = 0x0;  
		TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  
		TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure); // 
	
		TIM_ITRxExternalClockConfig(TIM4,TIM_TS_ETRF); //�����ⲿ���������򲻻����
	
		TIM_ETRClockMode1Config(TIM4, TIM_ExtTRGPSC_OFF, TIM_ExtTRGPolarity_NonInverted, 0x00); 
		TIM_SetCounter(TIM4, 0); 
		TIM_Cmd(TIM4, ENABLE); 
}
void TIM5_PWM_Init(u16 arr,u16 psc)//LDC1000 ʱ��
{  
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);	//ʹ�ܶ�ʱ��4ʱ��
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA  | RCC_APB2Periph_AFIO, ENABLE);  //ʹ��GPIO�����AFIO���ù���ģ��ʱ��.��������
	
   //���ø�����Ϊ�����������,���TIM3 CH2��PWM���岨��	GPIOB.8
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //�����������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��GPIO
   //��ʼ��TIM3
	TIM_TimeBaseStructure.TIM_Period = arr-1; //��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ
	TIM_TimeBaseStructure.TIM_Prescaler =psc-1; //����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ 
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
	TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure); //����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ
	
	
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; //ѡ��ʱ��ģʽ:TIM�����ȵ���ģʽ2
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //�Ƚ����ʹ��
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low; //�������:TIM����Ƚϼ��Ը�
	TIM_OC2Init(TIM5, &TIM_OCInitStructure);  //����Tָ���Ĳ�����ʼ������TIM4 OC2

	TIM_OC2PreloadConfig(TIM5, TIM_OCPreload_Enable);

	TIM_Cmd(TIM5, ENABLE);  //ʹ��TIM3

 	TIM_SetCompare2(TIM5,5);



}
void Timer7_Init(u16 arr,u16 psc)  //��ʱ��7��ʼ������
{
	TIM_TimeBaseInitTypeDef   TIM_TimeBaseStructure;  //�������ö�ʱ���Ľṹ�����
	
	//��ʱ��7�⺯����ʼ�����֣�
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE); //ʹ��TIM7ʱ��
	TIM_DeInit(TIM7);	            //������TIMx�Ĵ�������Ϊȱʡֵ 
	TIM_InternalClockConfig(TIM7);  //��ʱ��ʱ��Դ��ѡ���ڲ�ʱ��
	/*��������*/
	TIM_TimeBaseStructure.TIM_Period = arr - 1;                     //�趨�������Զ���װֵ  
	TIM_TimeBaseStructure.TIM_Prescaler = psc - 1;    	            //Ԥ��Ƶ,��ֵ+1Ϊ��Ƶ�ĳ���
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;  	                //����ʱ�ӷָ�  ������Ƶ
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; 	//���ϼ���
	TIM_TimeBaseInit(TIM7, &TIM_TimeBaseStructure);					//��ʼ��

	TIM_Cmd(TIM7, ENABLE);						 //ʹ�ܶ�ʱ��7����ʼ��ʱ
	TIM_PrescalerConfig(TIM7, psc - 1, TIM_PSCReloadMode_Immediate);//�ö�ʱ��Ԥ��Ƶֵ����װ��
	TIM_ClearFlag(TIM7,TIM_FLAG_Update);     //����жϱ�־
	TIM_ITConfig(TIM7,TIM_IT_Update,ENABLE); //ʹ�ܶ�ʱ���ж�
}


