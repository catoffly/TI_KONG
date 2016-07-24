#include "sys.h"

u16 DATA_ADC=0;
void ADC_Configuration(void)
{
	ADC_InitTypeDef ADC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,ENABLE);
	RCC_ADCCLKConfig(RCC_PCLK2_Div6);//12M ADCʱ�� ��72M����Ƶ��
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_1;//ADC1 �˿ڳ�ʼ��
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AIN;
	GPIO_Init(GPIOC,&GPIO_InitStructure);
	
	ADC_InitStructure.ADC_Mode=ADC_Mode_Independent;//ADC��ʼ��
	ADC_InitStructure.ADC_ScanConvMode= ENABLE;//��ͨ��ģʽ
	ADC_InitStructure.ADC_ContinuousConvMode=DISABLE;//����ģʽ                                                                                                  ;
	ADC_InitStructure.ADC_ExternalTrigConv=ADC_ExternalTrigConv_None;//�������ģʽ
	ADC_InitStructure.ADC_DataAlign=ADC_DataAlign_Right;//�����Ҷ���
	ADC_InitStructure.ADC_NbrOfChannel=1;//ͨ����Ŀ
	ADC_Init(ADC1,&ADC_InitStructure);
	
	ADC_RegularChannelConfig(ADC1,ADC_Channel_11,1,ADC_SampleTime_239Cycles5);
	//ADC_RegularChannelConfig(ADC1,ADC_Channel_12,2,ADC_SampleTime_1Cycles5);
	//ADC_RegularChannelConfig(ADC1,ADC_Channel_13,3,ADC_SampleTime_1Cycles5);
	ADC_DMACmd(ADC1, ENABLE);//dma����
	ADC_Cmd(ADC1,ENABLE);//adcʹ��
	
	ADC_ResetCalibration(ADC1);//��λ У׼
	while(ADC_GetResetCalibrationStatus(ADC1));
	ADC_StartCalibration(ADC1);
	while(ADC_GetCalibrationStatus(ADC1));
	
	ADC_SoftwareStartConvCmd(ADC1,ENABLE);//ADC �������
    
}


