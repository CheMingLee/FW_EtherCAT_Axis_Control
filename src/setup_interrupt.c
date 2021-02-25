#include "setting.h"

XScuGic Intc; //GIC
int i = 0;
int nret = 0;

uint16_t u16PDOSize = 6 * TEST_SERVO_CNT;
uint16_t u16PDOSizeRet = 10 * TEST_SERVO_CNT;

uint8_t RxData[TEST_SPI_DATA_SIZE]={0};
uint8_t TxData[TEST_SPI_DATA_SIZE]={0};

RXPDO_ST_DEF_T *pRxPDOData = (RXPDO_ST_DEF_T *)RxData;
TXPDO_ST_DEF_T *pTxPDOData = (TXPDO_ST_DEF_T *)TxData;

memset(RxData,0,sizeof(RxData));
memset(TxData,0,sizeof(TxData));

int g_Vel[TEST_SERVO_CNT] = {0};
double g_dt = TEST_CYCTIME * pow(10, -9);
double g_dDistance[TEST_SERVO_CNT] = {0};

void ECM_intr_Handler(void *CallBackRef)
{
	if(g_bInterruptFlag)
	{
		nret = ECM_EcatPdoFifoDataGet(TxData, u16PDOSizeRet);
		if (nret > 0)
		{
			for (i = 0; i < TEST_SERVO_CNT; i++)
			{
				g_Position_Params[i].m_iCurPos = pTxPDOData[i].n32AcuPos;
				g_Position_Params[i].m_uInput = pTxPDOData[i].u32DigInputs;
				if (g_Position_Params[i].m_uInput & DIGINPUT_LIMIT_LEFT || g_Position_Params[i].m_uInput & DIGINPUT_LIMIT_RIGHT)
				{
					g_Position_Params[i].m_uMode = MODE_IDLE;
				}
			}
		}
		else
		{
			g_Position_Params[i].m_uMode = MODE_IDLE;
		}
		
		for (i = 0; i < TEST_SERVO_CNT; i++)
		{
			switch (g_Position_Params[i].m_uMode)
			{
				case MODE_IDLE:
				{
					g_Position_Params[i].m_iCmdPos = g_Position_Params[i].m_iCurPos;
					g_Vel[i] = 0;

					break;
				}
				case MODE_JOG:
				{
					if (!g_bStopFlag)
					{
						if (abs(g_Vel[i]) < abs(g_Motion_Params[i].m_dJogSpeed))
						{
							g_Position_Params[i].m_iCmdPos = g_Position_Params[i].m_iCurPos + (int)((g_Vel[i] * g_dt + 0.5 * g_Motion_Params[i].m_dJogAcc * pow(g_dt, 2)) * g_Motion_Params[i].m_dRatio);
							g_Vel[i] += g_Motion_Params[i].m_dJogAcc * g_dt;
							if (abs(Vel[i]) > abs(g_Motion_Params[i].m_dJogSpeed))
							{
								g_Vel[i] = g_Motion_Params[i].m_dJogSpeed;
							}
						}
						else
						{
							g_Position_Params[i].m_iCmdPos = g_Position_Params[i].m_iCurPos + (int)((g_Vel[i] * g_dt) * g_Motion_Params[i].m_dRatio);
						}
					}
					else
					{
						g_Position_Params[i].m_iCmdPos = g_Position_Params[i].m_iCurPos;
						g_Position_Params[i].m_uMode = MODE_JOGEND;
					}

					break;
				}
				case MODE_MOTION:
				{
					double Vm; // 加速段與減速段交會處之速度
					double S1; // 加速段位移
					double S2; // 均速段位移
					double S3; // 減速段位移
					double T1; // 加速時間
					double T2; // 均速時間
					double T3; // 減速時間

					if (!g_bStopFlag)
					{
						/* code */
					}
					else
					{

					}
					
					break;
				}
				case MODE_HOME:
				{
					if (!g_bStopFlag)
					{
						/* code */
					}
					else
					{

					}
					
					break;
				}
				case MODE_JOGEND:
				{
					g_Position_Params[i].m_iCmdPos = g_Position_Params[i].m_iCurPos + (int)((g_Vel[i] * g_dt - 0.5 * g_Motion_Params[i].m_dJogAcc * pow(g_dt, 2)) * g_Motion_Params[i].m_dRatio);
					g_Vel[i] -= g_Motion_Params[i].m_dJogAcc * g_dt;
					if (g_Vel[i] < 0)
					{
						g_Position_Params[i].m_iCmdPos = g_Position_Params[i].m_iCurPos;
						g_Vel[i] = 0;
						g_Position_Params[i].m_uMode = MODE_IDLE;
					}

					break;
				}
				default:
				{
					break;
				}
			}

			pRxPDOData[i].n32TarPos = g_Position_Params[i].m_iCmdPos;
		}
	}
	
	ECM_EcatPdoFifoDataSend(RxData, u16PDOSize);

	Xil_Out32(ECM_INTR_RESET, 1);
}

void SetupInterruptSystem()
{
	XScuGic *GicInstancePtr = &Intc;
    XScuGic_Config *IntcConfig; //GIC config
	Xil_ExceptionInit();

	//initialise the GIC
	IntcConfig = XScuGic_LookupConfig(INTC_DEVICE_ID);

	XScuGic_CfgInitialize(GicInstancePtr, IntcConfig, IntcConfig->CpuBaseAddress);

	//connect to the hardware
	Xil_ExceptionRegisterHandler(XIL_EXCEPTION_ID_INT, (Xil_ExceptionHandler)XScuGic_InterruptHandler, GicInstancePtr);

	XScuGic_Connect(GicInstancePtr, ECM_INTR_ID, (Xil_ExceptionHandler)ECM_intr_Handler, (void *)&Intc);

	XScuGic_Enable(GicInstancePtr, ECM_INTR_ID);

	// Enable interrupts in the Processor.
	Xil_ExceptionEnableMask(XIL_EXCEPTION_IRQ);
}
