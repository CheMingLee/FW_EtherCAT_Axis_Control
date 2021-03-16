#include "setting.h"

uint8_t g_u8TxBuf[PKG_MAX_SIZE];
uint8_t g_u8RxBuf[PKG_MAX_SIZE];
uint8_t g_u8CmdIdx;
MOTION_PARAMS g_Motion_Params[TEST_SERVO_CNT];
POSITION_PARAMS g_Position_Params[TEST_SERVO_CNT];
bool g_bStopFlag[TEST_SERVO_CNT];
bool g_bHomingFlag[TEST_SERVO_CNT];
int g_iCnt[TEST_SERVO_CNT];
double g_dDistance[TEST_SERVO_CNT];
double g_dStartPos[TEST_SERVO_CNT];
double g_dDirection[TEST_SERVO_CNT];
double g_dVel[TEST_SERVO_CNT];
double g_dTime[TEST_SERVO_CNT];
double g_dVm[TEST_SERVO_CNT];
double g_dS[TEST_SERVO_CNT];
double g_dS1[TEST_SERVO_CNT];
double g_dS2[TEST_SERVO_CNT];
double g_dS3[TEST_SERVO_CNT];
double g_dT1[TEST_SERVO_CNT];
double g_dT2[TEST_SERVO_CNT];
double g_dT3[TEST_SERVO_CNT];
double g_dTtotal[TEST_SERVO_CNT];
double g_dt;
bool g_bInterruptFlag;
int g_iServoCnt;
uint16_t g_u16PDOSize;
uint16_t g_u16PDOSizeRet;
uint8_t g_RxData[TEST_SPI_DATA_SIZE];
uint8_t g_TxData[TEST_SPI_DATA_SIZE];
XScuGic g_Intc;
RXPDO_ST_DEF_T *g_pRxPDOData;
TXPDO_ST_DEF_T *g_pTxPDOData;

FILE_CMD g_CmdBuf[100];
bool g_bRunFileFlag;
CMD_FILE_PARAMS g_cmd_file_params;
bool g_bBeginPosFlag[2];
int g_iFileCmdIndex;

double g_dXY_V, g_dXY_Dis, g_dXY_S, g_dXY_Vm, g_dXY_S1, g_dXY_S2, g_dXY_S3, g_dXY_T1, g_dXY_T2, g_dXY_T3, g_dXY_Ttotal, g_dXY_Vs, g_dXY_Ve, g_dXY_Time;

u32 g_u32LEDout;
u16 g_u16JF8out;

void InitParameters()
{
	int i, j;

	for (i = 0; i < PKG_MAX_SIZE; i++)
	{
		g_u8TxBuf[i] = 0;
		g_u8RxBuf[i] = 0;
	}

	g_u8CmdIdx = 0;

	for (i = 0; i < TEST_SERVO_CNT; i++)
	{
		g_Motion_Params[i].m_dJogSpeed = 0.0;
		g_Motion_Params[i].m_dJogAcc = 0.0;
		g_Motion_Params[i].m_dMotionSpeed = 0.0;
		g_Motion_Params[i].m_dMotionAcc = 0.0;
		g_Motion_Params[i].m_dComeHomeSpeed = 0.0;
		g_Motion_Params[i].m_dLeftHomeSpeed = 0.0;
		g_Motion_Params[i].m_dHomeAcc = 0.0;

		g_Position_Params[i].m_uMode = MODE_IDLE;
		g_Position_Params[i].m_dTarPos = 0.0;
		g_Position_Params[i].m_dCmdPos = 0.0;
		g_Position_Params[i].m_iCurPos = 0.0;
		g_Position_Params[i].m_uInput = 0;

		g_bStopFlag[i] = false;
		g_bHomingFlag[i] = false;
		g_iCnt[i] = 5;

		g_dDistance[i] = 0.0;
		g_dStartPos[i] = 0.0;
		g_dDirection[i] = 1.0;
		g_dVel[i] = 0.0;
		g_dTime[i] = 0.0;
		g_dVm[i] = 0.0;
		g_dS[i] = 0.0;
		g_dS1[i] = 0.0;
		g_dS2[i] = 0.0;
		g_dS3[i] = 0.0;
		g_dT1[i] = 0.0;
		g_dT2[i] = 0.0;
		g_dT3[i] = 0.0;
		g_dTtotal[i] = 0.0;
	}

	g_dt = TEST_CYCTIME * pow(10, -9);
	g_bInterruptFlag = false;
	g_iServoCnt = 2;

	g_u16PDOSize = RXPDO_SIZE_BYTES * TEST_SERVO_CNT;
	g_u16PDOSizeRet = TXPDO_SIZE_BYTES * TEST_SERVO_CNT;

	for (i = 0; i < TEST_SPI_DATA_SIZE; i++)
	{
		g_RxData[i] = 0;
		g_TxData[i] = 0;
	}

	g_pRxPDOData = (RXPDO_ST_DEF_T *)g_RxData;
	g_pTxPDOData = (TXPDO_ST_DEF_T *)g_TxData;

	for (i = 0; i < 100; i++)
	{
		g_CmdBuf[i].m_iID = 0;
		for (j = 0; j < 5; j++)
		{
			g_CmdBuf[i].m_dParams[j] = 0.0;
		}
	}
	
	g_cmd_file_params.m_dBegPos[0] = 0.0;
	g_cmd_file_params.m_dBegPos[1] = 0.0;
	g_cmd_file_params.m_dEndPos[0] = 0.0;
	g_cmd_file_params.m_dEndPos[1] = 0.0;
	g_cmd_file_params.m_dRatio[0] = 0.0;
	g_cmd_file_params.m_dRatio[1] = 0.0;
	g_cmd_file_params.m_dSpeed = 0.0;
	g_cmd_file_params.m_dFSpeed = 0.0;
	g_cmd_file_params.m_dAcc = 0.0;
	g_cmd_file_params.m_dFAcc = 0.0;

	g_bRunFileFlag = false;
	g_bBeginPosFlag[0] = false;
	g_bBeginPosFlag[1] = false;
	g_iFileCmdIndex = 0;

	g_u32LEDout = 0;
	g_u16JF8out = 0;
}

int main()
{
	InitParameters();
	
	SetupInterruptSystem();

	while(1)
	{
		g_u32LEDout ^= 0x01;
		Xil_Out32(IO_ADDR_LEDOUT, g_u32LEDout);
		GetAppCmd();
	}

	return 0;
}
