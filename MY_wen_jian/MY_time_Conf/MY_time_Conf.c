#include "MY_time_Conf.h"

/************************************************************************************************
	* 函数名：TIM1_Configuration
	* 函数作用：TIM1外部脉冲计数模式初始化
	* 参数：空
	* 库版本  ：ST3.5.0
*************************************************************************************************/
void TIM1_Configuration(void)
{
		TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure; 
		GPIO_InitTypeDef GPIO_InitStructure;
		//启动GPIO
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA , ENABLE);
		//启动AFIO
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
		//启动TIM1
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; //
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; //50M时钟速度
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
void Timer2_Init(u16 arr,u16 psc)  //定时器2初始化函数
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);	//使能TIM2时钟
	//初始化定时器2 TIM2	 
	TIM_TimeBaseStructure.TIM_Period = arr-1; //设定计数器自动重装值 
	TIM_TimeBaseStructure.TIM_Prescaler =psc-1; 	//预分频器   
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure); //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位

	//中断分组初始化
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;  //TIM2中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;  //先占优先级2级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  //从优先级0级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道被使能
	NVIC_Init(&NVIC_InitStructure);  //根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器 
	
	TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);//允许更新中断 ,允许CC1IE捕获中断	
	
   	TIM_Cmd(TIM2,ENABLE ); 	//使能定时器2
}


void TIM3_PWM_Init(u16 arr,u16 psc)
{  
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);	//使能定时器4时钟
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC  | RCC_APB2Periph_AFIO, ENABLE);  //使能GPIO外设和AFIO复用功能模块时钟.。。。。
	GPIO_PinRemapConfig(GPIO_FullRemap_TIM3 , ENABLE);
   //设置该引脚为复用输出功能,输出TIM3 CH2的PWM脉冲波形	GPIOB.8
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9; //TIM_CH2
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //复用推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);//初始化GPIO
   //初始化TIM3
	TIM_TimeBaseStructure.TIM_Period = arr-1; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值
	TIM_TimeBaseStructure.TIM_Prescaler =psc-1; //设置用来作为TIMx时钟频率除数的预分频值 
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位
	
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; //选择定时器模式:TIM脉冲宽度调制模式2
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low; //输出极性:TIM输出比较极性高
	TIM_OC1Init(TIM3, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM4 OC1
	
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; //选择定时器模式:TIM脉冲宽度调制模式2
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low; //输出极性:TIM输出比较极性高
	TIM_OC2Init(TIM3, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM4 OC2
	
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; //选择定时器模式:TIM脉冲宽度调制模式2
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low; //输出极性:TIM输出比较极性高
	TIM_OC3Init(TIM3, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM4 OC3
	
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; //选择定时器模式:TIM脉冲宽度调制模式2
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low; //输出极性:TIM输出比较极性高
	TIM_OC4Init(TIM3, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM4 OC4

	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);  //使能TIM3在CCR3上的预装载寄存器
	TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);
	TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);
	TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);
	
 
	TIM_Cmd(TIM3, ENABLE);  //使能TIM3
 	TIM_SetCompare1(TIM3,0);
 	TIM_SetCompare2(TIM3,0);
 	TIM_SetCompare3(TIM3,0);
 	TIM_SetCompare4(TIM3,0);//防止开机电机转动


}
/************************************************************************************************
	* 函数名：TIM4_Configuration
	* 函数作用：TIM4外部脉冲计数模式初始化
	* 参数：空
	* 库版本  ：ST3.5.0
*************************************************************************************************/
void TIM4_Configuration(void)
{
		TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure; 
		GPIO_InitTypeDef GPIO_InitStructure;
		
		//启动GPIO
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE , ENABLE);
		//启动AFIO
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
		//启动TIM1
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
		
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; //
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; //50M时钟速度
		GPIO_Init(GPIOE, &GPIO_InitStructure);
	
		TIM_DeInit(TIM4);
		TIM_TimeBaseStructure.TIM_Period = 0xFFFF; 
		TIM_TimeBaseStructure.TIM_Prescaler = 0x00; 
		TIM_TimeBaseStructure.TIM_ClockDivision = 0x0;  
		TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  
		TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure); // 
	
		TIM_ITRxExternalClockConfig(TIM4,TIM_TS_ETRF); //配置外部触发，否则不会记数
	
		TIM_ETRClockMode1Config(TIM4, TIM_ExtTRGPSC_OFF, TIM_ExtTRGPolarity_NonInverted, 0x00); 
		TIM_SetCounter(TIM4, 0); 
		TIM_Cmd(TIM4, ENABLE); 
}
void TIM5_PWM_Init(u16 arr,u16 psc)//LDC1000 时钟
{  
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);	//使能定时器4时钟
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA  | RCC_APB2Periph_AFIO, ENABLE);  //使能GPIO外设和AFIO复用功能模块时钟.。。。。
	
   //设置该引脚为复用输出功能,输出TIM3 CH2的PWM脉冲波形	GPIOB.8
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //复用推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIO
   //初始化TIM3
	TIM_TimeBaseStructure.TIM_Period = arr-1; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值
	TIM_TimeBaseStructure.TIM_Prescaler =psc-1; //设置用来作为TIMx时钟频率除数的预分频值 
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure); //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位
	
	
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; //选择定时器模式:TIM脉冲宽度调制模式2
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low; //输出极性:TIM输出比较极性高
	TIM_OC2Init(TIM5, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM4 OC2

	TIM_OC2PreloadConfig(TIM5, TIM_OCPreload_Enable);

	TIM_Cmd(TIM5, ENABLE);  //使能TIM3

 	TIM_SetCompare2(TIM5,5);



}
void Timer7_Init(u16 arr,u16 psc)  //定时器7初始化函数
{
	TIM_TimeBaseInitTypeDef   TIM_TimeBaseStructure;  //定义配置定时器的结构体变量
	
	//定时器7库函数初始化部分：
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE); //使能TIM7时钟
	TIM_DeInit(TIM7);	            //将外设TIMx寄存器重设为缺省值 
	TIM_InternalClockConfig(TIM7);  //定时器时钟源，选择内部时钟
	/*基础设置*/
	TIM_TimeBaseStructure.TIM_Period = arr - 1;                     //设定计数器自动重装值  
	TIM_TimeBaseStructure.TIM_Prescaler = psc - 1;    	            //预分频,此值+1为分频的除数
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;  	                //设置时钟分割  采样分频
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; 	//向上计数
	TIM_TimeBaseInit(TIM7, &TIM_TimeBaseStructure);					//初始化

	TIM_Cmd(TIM7, ENABLE);						 //使能定时器7，开始定时
	TIM_PrescalerConfig(TIM7, psc - 1, TIM_PSCReloadMode_Immediate);//让定时器预分频值立即装入
	TIM_ClearFlag(TIM7,TIM_FLAG_Update);     //清除中断标志
	TIM_ITConfig(TIM7,TIM_IT_Update,ENABLE); //使能定时器中断
}


