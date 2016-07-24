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

static unsigned char Temp[11];





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
void SysTick_Handler(void)
{
}

void WWDG_IRQHandler(void)		 //窗口定时器中断服务函数
{
}

void PVD_IRQHandler(void)		 //联到EXTI的电源电压检测（PVD）中断服务函数
{
}

void TAMPER_IRQHandler(void)	 //侵入检测中断服务函数
{
}

void RTC_IRQHandler(void)		 //实时时钟（RTC）全局中断服务函数
{

}

void FLASH_IRQHandler(void)		 //闪存全局中断服务函数
{
}

void RCC_IRQHandler(void)		 //复位和时钟控制（RCC）中断服务函数
{
}

void EXTI0_IRQHandler(void)		 //EXTI线0中断服务函数
{

}

void EXTI1_IRQHandler(void)		 //EXTI线1中断服务函数
{

}

void EXTI2_IRQHandler(void)		 //EXTI线2中断服务函数
{
}

void EXTI3_IRQHandler(void)		 //EXTI线3中断服务函数
{
}

void EXTI4_IRQHandler(void)		 //EXTI线4中断服务函数
{
 
}

void DMA1_Channel1_IRQHandler(void)	   //DMA1通道1全局中断服务函数
{
}

void DMA1_Channel2_IRQHandler(void)	   //DMA1通道2全局中断服务函数
{
}

void DMA1_Channel3_IRQHandler(void)	   //DMA1通道3全局中断服务函数
{
}

void DMA1_Channel4_IRQHandler(void)	   //DMA1通道4全局中断服务函数
{

}

void DMA1_Channel5_IRQHandler(void)	   //DMA1通道5全局中断服务函数
{

}

void DMA1_Channel6_IRQHandler(void)	   //DMA1通道6全局中断服务函数
{
}

void DMA1_Channel7_IRQHandler(void)	   //DMA1通道7全局中断服务函数
{
}

void ADC1_2_IRQHandler(void)		   //ADC全局中断服务函数
{

}

void USB_HP_CAN_TX_IRQHandler(void)	   //USB高优先级或CAN发送中断服务函数
{
}

void USB_LP_CAN_RX0_IRQHandler(void)   //USB低优先级或CAN接收0中断服务函数
{
}

void CAN_RX1_IRQHandler(void)		   //CAN接收1中断服务函数
{
}

void CAN_SCE_IRQHandler(void)		   //CAN SCE中断服务函数
{
}

void EXTI9_5_IRQHandler(void)		   //外部中断9线【9:5】中断服务函数
{

}

void TIM1_BRK_IRQHandler(void)		   //定时器1断开中断服务函数
{
}

void TIM1_UP_IRQHandler(void)		   //定时器1更新中断服务函数
{
}

void TIM1_TRG_COM_IRQHandler(void)	   //定时器1触发和通信中断服务函数
{
}

void TIM1_CC_IRQHandler(void)		   //定时器1捕获比较中断服务函数
{
}

void TIM2_IRQHandler(void)			   //定时器2全局中断服务函数
{

	char xianshi1[5];
	ADC_SoftwareStartConvCmd(ADC1,ENABLE);//ADC 软件启动
	while(!ADC_GetFlagStatus(ADC1,ADC_FLAG_EOC));
	DATA_ADC=ADC_GetConversionValue(ADC1);
	sprintf(xianshi1,"%04d",DATA_ADC);//z轴 显示输出
	LCD_ShowString(30,110,210,24,24,xianshi1);
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

void I2C1_EV_IRQHandler(void)		   //I2C1事件中断服务函数
{
}

void I2C1_ER_IRQHandler(void)		   //I2C1错误中断服务函数
{
}

void I2C2_EV_IRQHandler(void)		   //I2C2事件中断服务函数
{
}

void I2C2_ER_IRQHandler(void)		   //I2C2错误中断服务函数
{
}

void SPI1_IRQHandler(void)			   //SPI1全局中断服务函数
{
}

void SPI2_IRQHandler(void)			   //SPI2全局中断服务函数
{
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

   if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)  //接收中断有效,若接收数据寄存器满
     {
				Temp[counter] = USART_ReceiveData(USART2);   //接收数据
				//网购给的程序
				//if(counter == 0 && Re_buf[0] != 0x55) return;      //第 0 号数据不是帧头，跳过
				if(counter == 0 && Temp[0] != 0x55) return;      //第 0 号数据不是帧头，跳过
				counter++; 
 				if(counter==11) //接收到 11 个数据
				{ 
					 memcpy(Re_buf,Temp,11);
					 counter=0; //重新赋值，准备下一帧数据的接收
					 sign=1;
				}    
		}
		USART_ClearITPendingBit(USART2,USART_IT_RXNE);
}



void USART3_IRQHandler(void)		   //串口3全局中断服务函数
{
//	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)  //接收中断有效,若接收数据寄存器满
//  	{
//  	}
	USART_ClearITPendingBit(USART3,USART_IT_RXNE);
}

void EXTI15_10_IRQHandler(void)		   //外部中断10【15:10】中断服务函数
{

}

void RTCAlarm_IRQHandler(void)		   //联到EXTI的RTC闹钟中断服务函数
{
}

void USBWakeUp_IRQHandler(void)		   //联到EXTI的从USB待机唤醒中断服务函数
{
}

void TIM8_BRK_IRQHandler(void)		   //定时器8断开中断服务函数
{
}

void TIM8_UP_IRQHandler(void)		   //定时器8更新中断服务函数
{
}

void TIM8_TRG_COM_IRQHandler(void)	   //定时器8触发和通信中断服务函数
{
}

void TIM8_CC_IRQHandler(void)		   //定时器8捕获比较中断服务函数
{
}

void ADC3_IRQHandler(void)			   //ADC3全局中断服务函数
{
}

void FSMC_IRQHandler(void)			   //FSMC全局中断服务函数
{
}

void SDIO_IRQHandler(void)			   //SDIO全局中断服务函数
{
}

void TIM5_IRQHandler(void)			   //定时器5全局中断服务函数
{
//	if(TIM5->SR & 0X0001)//溢出中断
//	{	
		    				   				     	    	
//	}				   
//	TIM5->SR &= ~(1<<0);//清除中断标志位
}

void SPI3_IRQHandler(void)			   //SPI3全局中断服务函数
{
}





void TIM6_IRQHandler(void)			   //定时器6全局中断服务函数
{
//	if(TIM6->SR & 0X0001)//溢出中断
//	{	
	    				   				     	    	
//	}				   
//	TIM6->SR &= ~(1<<0);//清除中断标志位
}

void TIM7_IRQHandler(void)			   //定时器7全局中断服务函数
{
//	if(TIM7->SR & 0X0001)//溢出中断
//	{	

//	}				   
//	TIM7->SR &= ~(1<<0);//清除中断标志位
}

void DMA2_Channel1_IRQHandler(void)	   //DMA2通道1全局中断服务函数
{
}

void DMA2_Channel2_IRQHandler(void)	   //DMA2通道2全局中断服务函数
{
}

void DMA2_Channel3_IRQHandler(void)	   //DMA2通道3全局中断服务函数
{
}

void DMA2_Channel4_5_IRQHandler(void)  //DMA2通道4和DMA2通道5全局中断服务函数
{
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
