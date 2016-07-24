#include "sys.h"

u16 DATA_ADC=0;
void ADC_Configuration(void)
{
	ADC_InitTypeDef ADC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,ENABLE);
	RCC_ADCCLKConfig(RCC_PCLK2_Div6);//12M ADC时钟 在72M的主频下
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_1;//ADC1 端口初始化
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AIN;
	GPIO_Init(GPIOC,&GPIO_InitStructure);
	
	ADC_InitStructure.ADC_Mode=ADC_Mode_Independent;//ADC初始化
	ADC_InitStructure.ADC_ScanConvMode= ENABLE;//多通道模式
	ADC_InitStructure.ADC_ContinuousConvMode=DISABLE;//单次模式                                                                                                  ;
	ADC_InitStructure.ADC_ExternalTrigConv=ADC_ExternalTrigConv_None;//软件触发模式
	ADC_InitStructure.ADC_DataAlign=ADC_DataAlign_Right;//数据右对齐
	ADC_InitStructure.ADC_NbrOfChannel=1;//通道数目
	ADC_Init(ADC1,&ADC_InitStructure);
	
	ADC_RegularChannelConfig(ADC1,ADC_Channel_11,1,ADC_SampleTime_239Cycles5);
	//ADC_RegularChannelConfig(ADC1,ADC_Channel_12,2,ADC_SampleTime_1Cycles5);
	//ADC_RegularChannelConfig(ADC1,ADC_Channel_13,3,ADC_SampleTime_1Cycles5);
	ADC_DMACmd(ADC1, ENABLE);//dma开启
	ADC_Cmd(ADC1,ENABLE);//adc使能
	
	ADC_ResetCalibration(ADC1);//复位 校准
	while(ADC_GetResetCalibrationStatus(ADC1));
	ADC_StartCalibration(ADC1);
	while(ADC_GetCalibrationStatus(ADC1));
	
	ADC_SoftwareStartConvCmd(ADC1,ENABLE);//ADC 软件启动
    
}


