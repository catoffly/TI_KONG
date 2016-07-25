#ifndef __PID_H
#define __PID_H
#include "sys.h"
struct PID
{
	float KP;
	float KI;
	float KD;
	float QIWANG;
	float EK;
	float EK1;
	float UK;
	int   SHUCHU;
		
};
extern struct PID dianji1;
extern struct PID dianji2;
void pid(void);
#endif
