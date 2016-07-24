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

void WWDG_IRQHandler(void)		 //���ڶ�ʱ���жϷ�����
{
}

void PVD_IRQHandler(void)		 //����EXTI�ĵ�Դ��ѹ��⣨PVD���жϷ�����
{
}

void TAMPER_IRQHandler(void)	 //�������жϷ�����
{
}

void RTC_IRQHandler(void)		 //ʵʱʱ�ӣ�RTC��ȫ���жϷ�����
{

}

void FLASH_IRQHandler(void)		 //����ȫ���жϷ�����
{
}

void RCC_IRQHandler(void)		 //��λ��ʱ�ӿ��ƣ�RCC���жϷ�����
{
}

void EXTI0_IRQHandler(void)		 //EXTI��0�жϷ�����
{

}

void EXTI1_IRQHandler(void)		 //EXTI��1�жϷ�����
{

}

void EXTI2_IRQHandler(void)		 //EXTI��2�жϷ�����
{
}

void EXTI3_IRQHandler(void)		 //EXTI��3�жϷ�����
{
}

void EXTI4_IRQHandler(void)		 //EXTI��4�жϷ�����
{
 
}

void DMA1_Channel1_IRQHandler(void)	   //DMA1ͨ��1ȫ���жϷ�����
{
}

void DMA1_Channel2_IRQHandler(void)	   //DMA1ͨ��2ȫ���жϷ�����
{
}

void DMA1_Channel3_IRQHandler(void)	   //DMA1ͨ��3ȫ���жϷ�����
{
}

void DMA1_Channel4_IRQHandler(void)	   //DMA1ͨ��4ȫ���жϷ�����
{

}

void DMA1_Channel5_IRQHandler(void)	   //DMA1ͨ��5ȫ���жϷ�����
{

}

void DMA1_Channel6_IRQHandler(void)	   //DMA1ͨ��6ȫ���жϷ�����
{
}

void DMA1_Channel7_IRQHandler(void)	   //DMA1ͨ��7ȫ���жϷ�����
{
}

void ADC1_2_IRQHandler(void)		   //ADCȫ���жϷ�����
{

}

void USB_HP_CAN_TX_IRQHandler(void)	   //USB�����ȼ���CAN�����жϷ�����
{
}

void USB_LP_CAN_RX0_IRQHandler(void)   //USB�����ȼ���CAN����0�жϷ�����
{
}

void CAN_RX1_IRQHandler(void)		   //CAN����1�жϷ�����
{
}

void CAN_SCE_IRQHandler(void)		   //CAN SCE�жϷ�����
{
}

void EXTI9_5_IRQHandler(void)		   //�ⲿ�ж�9�ߡ�9:5���жϷ�����
{

}

void TIM1_BRK_IRQHandler(void)		   //��ʱ��1�Ͽ��жϷ�����
{
}

void TIM1_UP_IRQHandler(void)		   //��ʱ��1�����жϷ�����
{
}

void TIM1_TRG_COM_IRQHandler(void)	   //��ʱ��1������ͨ���жϷ�����
{
}

void TIM1_CC_IRQHandler(void)		   //��ʱ��1����Ƚ��жϷ�����
{
}

void TIM2_IRQHandler(void)			   //��ʱ��2ȫ���жϷ�����
{

	char xianshi1[5];
	ADC_SoftwareStartConvCmd(ADC1,ENABLE);//ADC �������
	while(!ADC_GetFlagStatus(ADC1,ADC_FLAG_EOC));
	DATA_ADC=ADC_GetConversionValue(ADC1);
	sprintf(xianshi1,"%04d",DATA_ADC);//z�� ��ʾ���
	LCD_ShowString(30,110,210,24,24,xianshi1);
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

void I2C1_EV_IRQHandler(void)		   //I2C1�¼��жϷ�����
{
}

void I2C1_ER_IRQHandler(void)		   //I2C1�����жϷ�����
{
}

void I2C2_EV_IRQHandler(void)		   //I2C2�¼��жϷ�����
{
}

void I2C2_ER_IRQHandler(void)		   //I2C2�����жϷ�����
{
}

void SPI1_IRQHandler(void)			   //SPI1ȫ���жϷ�����
{
}

void SPI2_IRQHandler(void)			   //SPI2ȫ���жϷ�����
{
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

   if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)  //�����ж���Ч,���������ݼĴ�����
     {
				Temp[counter] = USART_ReceiveData(USART2);   //��������
				//�������ĳ���
				//if(counter == 0 && Re_buf[0] != 0x55) return;      //�� 0 �����ݲ���֡ͷ������
				if(counter == 0 && Temp[0] != 0x55) return;      //�� 0 �����ݲ���֡ͷ������
				counter++; 
 				if(counter==11) //���յ� 11 ������
				{ 
					 memcpy(Re_buf,Temp,11);
					 counter=0; //���¸�ֵ��׼����һ֡���ݵĽ���
					 sign=1;
				}    
		}
		USART_ClearITPendingBit(USART2,USART_IT_RXNE);
}



void USART3_IRQHandler(void)		   //����3ȫ���жϷ�����
{
//	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)  //�����ж���Ч,���������ݼĴ�����
//  	{
//  	}
	USART_ClearITPendingBit(USART3,USART_IT_RXNE);
}

void EXTI15_10_IRQHandler(void)		   //�ⲿ�ж�10��15:10���жϷ�����
{

}

void RTCAlarm_IRQHandler(void)		   //����EXTI��RTC�����жϷ�����
{
}

void USBWakeUp_IRQHandler(void)		   //����EXTI�Ĵ�USB���������жϷ�����
{
}

void TIM8_BRK_IRQHandler(void)		   //��ʱ��8�Ͽ��жϷ�����
{
}

void TIM8_UP_IRQHandler(void)		   //��ʱ��8�����жϷ�����
{
}

void TIM8_TRG_COM_IRQHandler(void)	   //��ʱ��8������ͨ���жϷ�����
{
}

void TIM8_CC_IRQHandler(void)		   //��ʱ��8����Ƚ��жϷ�����
{
}

void ADC3_IRQHandler(void)			   //ADC3ȫ���жϷ�����
{
}

void FSMC_IRQHandler(void)			   //FSMCȫ���жϷ�����
{
}

void SDIO_IRQHandler(void)			   //SDIOȫ���жϷ�����
{
}

void TIM5_IRQHandler(void)			   //��ʱ��5ȫ���жϷ�����
{
//	if(TIM5->SR & 0X0001)//����ж�
//	{	
		    				   				     	    	
//	}				   
//	TIM5->SR &= ~(1<<0);//����жϱ�־λ
}

void SPI3_IRQHandler(void)			   //SPI3ȫ���жϷ�����
{
}





void TIM6_IRQHandler(void)			   //��ʱ��6ȫ���жϷ�����
{
//	if(TIM6->SR & 0X0001)//����ж�
//	{	
	    				   				     	    	
//	}				   
//	TIM6->SR &= ~(1<<0);//����жϱ�־λ
}

void TIM7_IRQHandler(void)			   //��ʱ��7ȫ���жϷ�����
{
//	if(TIM7->SR & 0X0001)//����ж�
//	{	

//	}				   
//	TIM7->SR &= ~(1<<0);//����жϱ�־λ
}

void DMA2_Channel1_IRQHandler(void)	   //DMA2ͨ��1ȫ���жϷ�����
{
}

void DMA2_Channel2_IRQHandler(void)	   //DMA2ͨ��2ȫ���жϷ�����
{
}

void DMA2_Channel3_IRQHandler(void)	   //DMA2ͨ��3ȫ���жϷ�����
{
}

void DMA2_Channel4_5_IRQHandler(void)  //DMA2ͨ��4��DMA2ͨ��5ȫ���жϷ�����
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
