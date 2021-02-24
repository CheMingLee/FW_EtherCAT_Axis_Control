#include "setting.h"

MOTION_PARAMS g_Motion_Params[TEST_SERVO_CNT];
POSITION_PARAMS g_Position_Params[TEST_SERVO_CNT];
bool g_bInterruptFlag;
bool g_bStopFlag[TEST_SERVO_CNT];

void InitParameters()
{
	int i;
	for (i = 0; i < TEST_SERVO_CNT; i++)
	{
		g_Motion_Params[i].m_dJogSpeed = 0.0;
		g_Motion_Params[i].m_dJagAcc = 0.0;
		g_Motion_Params[i].m_dMotionSpeed = 0.0;
		g_Motion_Params[i].m_dMotionAcc = 0.0;
		g_Motion_Params[i].m_dComeHomeSpeed = 0.0;
		g_Motion_Params[i].m_dLeftHomeSpeed = 0.0;
		g_Motion_Params[i].m_dHomeAcc = 0.0;

		g_Position_Params[i].m_uMode = MODE_IDLE;
		g_Position_Params[i].m_dTarPos = 0.0;
		g_Position_Params[i].m_dCmdPos = 0.0;
		g_Position_Params[i].m_dCurPos = 0.0;
		g_Position_Params[i].m_uInput = 0;

		g_bStopFlag[i] = false;
	}

	g_bInterruptFlag = false;
}

int main()
{
	InitParameters();
	
	SetupInterruptSystem();

	while(1)
	{
		GetAppCmd();
	}

	return 0;
}
