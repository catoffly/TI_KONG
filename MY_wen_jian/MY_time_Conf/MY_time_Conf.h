#ifndef __MY_TIME_CONF_H
#define __MY_TIME_CONF_H

#include "stm32f10x.h"

void TIM1_Configuration(void);
void Timer2_Init(u16 arr,u16 psc);  //��ʱ��2��ʼ������
void TIM3_PWM_Init(u16 arr,u16 psc);
void TIM4_Configuration(void);
void TIM5_PWM_Init(u16 arr,u16 psc);
void Timer7_Init(u16 arr,u16 psc);  //��ʱ��7��ʼ������


#endif

