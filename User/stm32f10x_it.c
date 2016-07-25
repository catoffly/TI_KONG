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

//u8 t = 0;	 //������ʱ���� ������
//u8 he = 0;	//У���
//
//extern u8 f;    //������,����1׼�����ͱ�־
//
//#define led_PB13   PBout(13)	   //λ�󶨣�λ����PB13����ΪLED����������ٶ����
//#define feng      PBout(14)	   //λ�󶨣�λ����PB14����ΪLED���(������)�������ٶ����
//
//#define CLK1   PDout(0)
//#define CWW1   PDout(1)
//#define CLK2   PDout(2)
//#define CWW2   PDout(3)
//
//u8 buff[5] = {0,0,0,0,0}; //����
//u8 p = 0;  //����
//u8 LR_uart_buf = 0;
//u8 UD_uart_buf = 0;
//u8 fang_xiang = 0;
//u8 Led_T = 0; //LED����ʱ�����
////u8 Time2_OK = 0;  //��ʱ���ж���ɱ�־

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

void TIM2_IRQHandler(void)			   //��ʱ��2ȫ���жϷ�����
{

	char xianshi1[5];
	
	ADC_SoftwareStartConvCmd(ADC1,ENABLE);//ADC �������
	while(!ADC_GetFlagStatus(ADC1,ADC_FLAG_EOC));
	DATA_ADC=ADC_GetConversionValue(ADC1);
//	sprintf(xianshi1,"%04d",DATA_ADC);//z�� ��ʾ���
//	LCD_ShowString(30,110,210,24,24,xianshi1);
	pid();
	dianji1_con();
	dianji2_con();
	LDC_RP();
	sprintf(xianshi1,"%04d",Rp1);//z�� ��ʾ���
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
		sprintf(xianshi1,"%04d",Rp1);//z�� ��ʾ���
		LCD_ShowString(30,110,210,24,24,xianshi1);
		sprintf(xianshi1,"%04d",cishu);//z�� ��ʾ���
		LCD_ShowString(30,140,210,24,24,xianshi1);
		sprintf(xianshi1,"%04d",cishu1);//z�� ��ʾ���
		LCD_ShowString(30,160,210,24,24,xianshi1);
	
	}
	TIM_ClearITPendingBit(TIM2, TIM_IT_Update); //����жϱ�־λ
	
}

void TIM3_IRQHandler(void)			   //��ʱ��3ȫ���жϷ�����
{
//	if(TIM3->SR & 0X0001)//����ж�
//	{	
//	    //����жϴ������
////		t = !t;	   //ȡ����־λ
////		if(t == 1)
////			
////			GPIO_ResetBits(GPIOB , GPIO_Pin_1);		  //һ��λ����PB1������͵�ƽ����LED��
////		else
////	 		GPIO_SetBits(GPIOB, GPIO_Pin_1);	      // һ��λ���� PB1������ߵ�ƽ�ر�LED��
//		
//		CLK2 = ~CLK2;
//	}				   
//	TIM3->SR &= ~(1<<0);//����жϱ�־λ
//	TIM_Cmd(TIM3, DISABLE);						 //�ض�ʱ��3����main�����ڴ�
}

void TIM4_IRQHandler(void)			   //��ʱ��4ȫ���жϷ�����
{
//	if(TIM4->SR & 0X0001)//����ж�
//	{	
//	    //����жϴ������
//		t = !t;	   //ȡ����־λ
//		if(t == 1)
//			
//			GPIO_ResetBits(GPIOB , GPIO_Pin_1);		  //һ��λ����PB1������͵�ƽ����LED��
//		else
//	 		GPIO_SetBits(GPIOB, GPIO_Pin_1);	      // һ��λ���� PB1������ߵ�ƽ�ر�LED��			    				   				     	    	
//	}				   
//	TIM4->SR &= ~(1<<0);//����жϱ�־λ
}



void USART1_IRQHandler(void)		   //����1ȫ���жϷ�����
{
//	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)  //�����ж���Ч,���������ݼĴ�����
//  	{
//}
	USART_ClearITPendingBit(USART1,USART_IT_RXNE);

}  	 

void USART2_IRQHandler(void)		   //����2ȫ���жϷ�����
{

   
		USART_ClearITPendingBit(USART2,USART_IT_RXNE);
}



void USART3_IRQHandler(void)		   //����3ȫ���жϷ�����
{
//	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)  //�����ж���Ч,���������ݼĴ�����
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
