#include "pid.h"

struct PID dianji1 = {0 , 0 , 0 , 0 , 0 , 0 , 0 , 0};//P I D QIWANG EK EK1 UK
struct PID dianji2 = {0 , 0 , 0 , 0 , 0 , 0 , 0 , 0};

void pid(void)
{
	
	dianji1.EK=dianji1.QIWANG-cishu;//����ƫ��
	dianji2.EK=dianji2.QIWANG-cishu1;
	
	dianji1.UK=dianji1.KP*dianji1.EK+dianji1.KD*(dianji1.EK-dianji1.EK1);//pids ����
	dianji2.UK=dianji2.KP*dianji2.EK+dianji2.KD*(dianji2.EK-dianji2.EK1);
	
	dianji1.EK1=dianji1.EK;//��¼��ǰƫ��
	dianji2.EK1=dianji2.EK;
	
	if(dianji1.UK>7200)//�����޷�
	{
		dianji1.UK=7200;
	}
	if(dianji2.UK>7200)
	{
		dianji2.UK=7200;
	}
	
	if(dianji1.UK<-7200)//�����޷�
	{
		dianji1.UK=-7200;
	}
	if(dianji2.UK<-7200)
	{
		dianji2.UK=-7200;
	}
	
	dianji1.SHUCHU=(int)dianji1.UK;//float ǿתInt
	dianji2.SHUCHU=(int)dianji2.UK;
	
	
	
}

