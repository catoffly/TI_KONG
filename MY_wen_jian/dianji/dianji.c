#include "dianji.h"

void dianji1_con(void)//���1����
{
	if(dianji1.SHUCHU>0)
	{
		TIM_SetCompare1(TIM3,dianji1.SHUCHU);//�ֱ��Ӧ ���1��
		TIM_SetCompare2(TIM3,0);//���1��
	}
	else
	{
		dianji1.SHUCHU=abs(dianji1.SHUCHU);
		TIM_SetCompare1(TIM3,0);//�ֱ��Ӧ ���1��
		TIM_SetCompare2(TIM3,dianji1.SHUCHU);//���1��
	}
	
	
	
}



void dianji2_con(void)//���2����
{
	if(dianji2.SHUCHU>0)
	{
		TIM_SetCompare3(TIM3,dianji2.SHUCHU);//�ֱ��Ӧ ���1��
		TIM_SetCompare4(TIM3,0);//���1��
	}
	else
	{
		dianji2.SHUCHU=abs(dianji2.SHUCHU);
		TIM_SetCompare3(TIM3,0);//�ֱ��Ӧ ���1��
		TIM_SetCompare4(TIM3,dianji2.SHUCHU);//���1��
	}
	
	
	
}
 