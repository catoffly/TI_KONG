#include "dianji.h"

void dianji1_con(void)//电机1控制
{
	if(dianji1.SHUCHU>0)
	{
		TIM_SetCompare1(TIM3,dianji1.SHUCHU);//分别对应 电机1正
		TIM_SetCompare2(TIM3,0);//电机1反
	}
	else
	{
		dianji1.SHUCHU=abs(dianji1.SHUCHU);
		TIM_SetCompare1(TIM3,0);//分别对应 电机1正
		TIM_SetCompare2(TIM3,dianji1.SHUCHU);//电机1反
	}
	
	
	
}



void dianji2_con(void)//电机2控制
{
	if(dianji2.SHUCHU>0)
	{
		TIM_SetCompare3(TIM3,dianji2.SHUCHU);//分别对应 电机1正
		TIM_SetCompare4(TIM3,0);//电机1反
	}
	else
	{
		dianji2.SHUCHU=abs(dianji2.SHUCHU);
		TIM_SetCompare3(TIM3,0);//分别对应 电机1正
		TIM_SetCompare4(TIM3,dianji2.SHUCHU);//电机1反
	}
	
	
	
}
 