/**
  ******************************************************************************
  * @file    GPIO/JTAG_Remap/stm32f10x_it.c 
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and peripherals
  *          interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h"
#include "sys.h"
#include "string.h"
//#include "uart_1.h"
//#include "dma.h"
//#include "RS485.h"
//#include "74hc595.h"

//u8 t = 0;	 //定义临时变量 测试用
//u8 he = 0;	//校验和
//
//extern u8 f;    //测试用,串口1准备发送标志
//
//#define led_PB13   PBout(13)	   //位绑定，位定义PB13口作为LED输出。这样速度最快
//#define feng      PBout(14)	   //位绑定，位定义PB14口作为LED输出(蜂鸣器)。这样速度最快
//
//#define CLK1   PDout(0)
//#define CWW1   PDout(1)
//#define CLK2   PDout(2)
//#define CWW2   PDout(3)
//
//u8 buff[5] = {0,0,0,0,0}; //测试
//u8 p = 0;  //测试
//u8 LR_uart_buf = 0;
//u8 UD_uart_buf = 0;
//u8 fang_xiang = 0;
//u8 Led_T = 0; //LED发光时间变量
////u8 Time2_OK = 0;  //定时器中断完成标志

unsigned char Re_buf[11],temp_buf[11],counter=0;
unsigned char sign,t,he;
u32 time_z;






/** @addtogroup STM32F10x_StdPeriph_Examples
  * @{
  */

/** @addtogroup GPIO_JTAG_Remap
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSV_Handler exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */

void TIM2_IRQHandler(void)			   //定时器2全局中断服务函数
{

	char xianshi1[5];
	
	ADC_SoftwareStartConvCmd(ADC1,ENABLE);//ADC 软件启动
	while(!ADC_GetFlagStatus(ADC1,ADC_FLAG_EOC));
	DATA_ADC=ADC_GetConversionValue(ADC1);
//	sprintf(xianshi1,"%04d",DATA_ADC);//z轴 显示输出
//	LCD_ShowString(30,110,210,24,24,xianshi1);
	pid();
	dianji1_con();
	dianji2_con();
	LDC_RP();
	sprintf(xianshi1,"%04d",Rp1);//z轴 显示输出
		LCD_ShowString(30,110,210,24,24,xianshi1);
	time_z++;
	if(time_z%10==0)
	{
		cishu=TIM4->CNT;
		cishu1=TIM8->CNT;
		TIM4->CNT=0;
		TIM8->CNT=0;

	}
	if(time_z%50==0)
	{
		sprintf(xianshi1,"%04d",Rp1);//z轴 显示输出
		LCD_ShowString(30,110,210,24,24,xianshi1);
		sprintf(xianshi1,"%04d",cishu);//z轴 显示输出
		LCD_ShowString(30,140,210,24,24,xianshi1);
		sprintf(xianshi1,"%04d",cishu1);//z轴 显示输出
		LCD_ShowString(30,160,210,24,24,xianshi1);
	
	}
	TIM_ClearITPendingBit(TIM2, TIM_IT_Update); //清除中断标志位
	
}

void TIM3_IRQHandler(void)			   //定时器3全局中断服务函数
{
//	if(TIM3->SR & 0X0001)//溢出中断
//	{	
//	    //添加中断处理程序
////		t = !t;	   //取反标志位
////		if(t == 1)
////			
////			GPIO_ResetBits(GPIOB , GPIO_Pin_1);		  //一个位操作PB1口输出低电平点亮LED灯
////		else
////	 		GPIO_SetBits(GPIOB, GPIO_Pin_1);	      // 一个位操作 PB1口输出高电平关闭LED灯
//		
//		CLK2 = ~CLK2;
//	}				   
//	TIM3->SR &= ~(1<<0);//清除中断标志位
//	TIM_Cmd(TIM3, DISABLE);						 //关定时器3，在main函数内打开
}

void TIM4_IRQHandler(void)			   //定时器4全局中断服务函数
{
//	if(TIM4->SR & 0X0001)//溢出中断
//	{	
//	    //添加中断处理程序
//		t = !t;	   //取反标志位
//		if(t == 1)
//			
//			GPIO_ResetBits(GPIOB , GPIO_Pin_1);		  //一个位操作PB1口输出低电平点亮LED灯
//		else
//	 		GPIO_SetBits(GPIOB, GPIO_Pin_1);	      // 一个位操作 PB1口输出高电平关闭LED灯			    				   				     	    	
//	}				   
//	TIM4->SR &= ~(1<<0);//清除中断标志位
}



void USART1_IRQHandler(void)		   //串口1全局中断服务函数
{
//	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)  //接收中断有效,若接收数据寄存器满
//  	{
//}
	USART_ClearITPendingBit(USART1,USART_IT_RXNE);

}  	 

void USART2_IRQHandler(void)		   //串口2全局中断服务函数
{

   
		USART_ClearITPendingBit(USART2,USART_IT_RXNE);
}



void USART3_IRQHandler(void)		   //串口3全局中断服务函数
{
//	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)  //接收中断有效,若接收数据寄存器满
//  	{
//  	}
	USART_ClearITPendingBit(USART3,USART_IT_RXNE);
}










/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */

/**
  * @}
  */

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
