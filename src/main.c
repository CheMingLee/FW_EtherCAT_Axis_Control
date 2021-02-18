/*
 * main.c
 *
 *  Created on: 2021/2/3
 *      Author: cmleex
 */

#include "setting.h"

MOTION_PARAMS g_Motion_Params[2];

int main()
{
	SetupInterruptSystem();

	while(1)
	{
		GetAppCmd();
	}

	return 0;
}
