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

double g_dVel[TEST_SERVO_CNT] = {0};
double g_dt = TEST_CYCTIME * pow(10, -9);

void GetCmdPos_Acc(double dAcc)
{
	g_Position_Params[i].m_dCmdPos = g_Position_Params[i].m_dCurPos + (g_dVel[i] * g_dt + 0.5 * dAcc * pow(g_dt, 2)) * g_Motion_Params[i].m_dRatio;
	g_dVel[i] += dAcc * g_dt;
}

void GetCmdPos_ConstVel()
{
	g_Position_Params[i].m_dCmdPos = g_Position_Params[i].m_dCurPos + (g_dVel[i] * g_dt) * g_Motion_Params[i].m_dRatio;
}

void GetCmdPos_Dec(double dAcc)
{
	g_Position_Params[i].m_dCmdPos = g_Position_Params[i].m_dCurPos + (g_dVel[i] * g_dt - 0.5 * dAcc * pow(g_dt, 2)) * g_Motion_Params[i].m_dRatio;
	g_dVel[i] -= dAcc * g_dt;
}

void ECM_intr_Handler(void *CallBackRef)
{
	if(g_bInterruptFlag)
	{
		nret = ECM_EcatPdoFifoDataGet(TxData, u16PDOSizeRet);
		if (nret > 0)
		{
			for (i = 0; i < TEST_SERVO_CNT; i++)
			{
				g_Position_Params[i].m_dCurPos = (double)pTxPDOData[i].n32AcuPos;
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
					g_Position_Params[i].m_dCmdPos = g_Position_Params[i].m_dCurPos;
					g_dVel[i] = 0;
					g_bStopFlag[i] = false;

					break;
				}
				case MODE_JOG:
				{
					if (!g_bStopFlag[i])
					{
						if (abs(g_dVel[i]) < abs(g_Motion_Params[i].m_dJogSpeed))
						{
							GetCmdPos_Acc(g_Motion_Params[i].m_dJogAcc);
							if (abs(g_dVel[i]) > abs(g_Motion_Params[i].m_dJogSpeed))
							{
								g_dVel[i] = g_Motion_Params[i].m_dJogSpeed;
							}
						}
						else
						{
							GetCmdPos_ConstVel();
						}
					}
					else
					{
						GetCmdPos_Dec(g_Motion_Params[i].m_dJogAcc);
						if (g_Motion_Params[i].m_dJogSpeed > 0)
						{
							if (g_dVel[i] < 0)
							{
								g_Position_Params[i].m_dCmdPos = g_Position_Params[i].m_dCurPos;
								g_dVel[i] = 0;
							}
						}
						else
						{
							if (g_dVel[i] > 0)
							{
								g_Position_Params[i].m_dCmdPos = g_Position_Params[i].m_dCurPos;
								g_dVel[i] = 0;
							}
						}
					}

					if (g_Position_Params[i].m_dCmdPos == g_Position_Params[i].m_dCurPos)
					{
						g_Position_Params[i].m_uMode = MODE_IDLE;
					}

					break;
				}
				case MODE_MOTION:
				{
					double Vm; // 加速段與減速段交會處之速度
					double S1; // 加速段位移
					double S2; // 均速段位移
					double S3; // 減速段位移

					if (!g_bStopFlag[i])
					{	
						if (g_dDistance[i] > 0)
						{
							if (g_Position_Params[i].m_dCurPos >= g_Position_Params[i].m_dTarPos)
							{
								g_Position_Params[i].m_dCmdPos = g_Position_Params[i].m_dTarPos;
								break;
							}
						}
						else if (g_dDistance[i] < 0)
						{
							if (g_Position_Params[i].m_dCurPos >= g_Position_Params[i].m_dTarPos)
							{
								g_Position_Params[i].m_dCmdPos = g_Position_Params[i].m_dTarPos;
								break;
							}
						}
						else
						{
							g_Position_Params[i].m_dCmdPos = g_Position_Params[i].m_dCurPos;
							break;
						}
						
						S1 = pow(g_Motion_Params[i].m_dMotionSpeed, 2) / (2.0 * abs(g_Motion_Params[i].m_dMotionAcc));
						S3 = S1;
						S2 = abs(g_dDistance[i]) - S1 - S3;
						if (S2 <= 0.0) // 三角形
						{
							Vm = sqrt((2.0 * abs(g_Motion_Params[i].m_dMotionAcc) * abs(g_dDistance[i])) / 2.0);
							if (abs(g_dVel[i]) < Vm)
							{
								GetCmdPos_Acc(g_Motion_Params[i].m_dMotionAcc);
								if (abs(g_dVel[i]) > Vm)
								{
									if (g_Motion_Params[i].m_dMotionSpeed > 0)
									{
										g_dVel[i] = Vm;
									}
									else
									{
										g_dVel[i] = -Vm;
									}
								}
							}
							else
							{
								GetCmdPos_Dec(g_Motion_Params[i].m_dMotionAcc);
							}
						}
						else // 梯形
						{
							if ((abs(g_dDistance[i]) - g_Position_Params[i].m_dCurPos) > (S2 + S3))
							{
								GetCmdPos_Acc(g_Motion_Params[i].m_dMotionAcc);
								if (abs(g_dVel[i]) > abs(g_Motion_Params[i].m_dMotionSpeed))
								{
									g_dVel[i] = g_Motion_Params[i].m_dMotionSpeed;
								}
							}
							else if (g_dVel[i] == g_Motion_Params[i].m_dMotionSpeed && (abs(g_dDistance[i]) - g_Position_Params[i].m_dCurPos) > S3))
							{
								GetCmdPos_ConstVel();
							}
							else
							{
								GetCmdPos_Dec(g_Motion_Params[i].m_dMotionAcc);
							}
						}

						if (g_Motion_Params[i].m_dMotionSpeed > 0)
						{
							if (g_dVel[i] <= 0)
							{
								g_Position_Params[i].m_dCmdPos = g_Position_Params[i].m_dTarPos;
							}
						}
						else
						{
							if (g_dVel[i] >= 0)
							{
								g_Position_Params[i].m_dCmdPos = g_Position_Params[i].m_dTarPos;
							}
						}
					}
					else // Stop
					{
						GetCmdPos_Dec(g_Motion_Params[i].m_dMotionAcc);
						if (g_Motion_Params[i].m_dMotionSpeed > 0)
						{
							if (g_dVel[i] < 0)
							{
								g_Position_Params[i].m_dCmdPos = g_Position_Params[i].m_dCurPos;
							}
						}
						else
						{
							if (g_dVel[i] > 0)
							{
								g_Position_Params[i].m_dCmdPos = g_Position_Params[i].m_dCurPos;
							}
						}
					}

					if (g_Position_Params[i].m_dCmdPos == g_Position_Params[i].m_dCurPos)
					{
						g_Position_Params[i].m_uMode = MODE_IDLE;
					}

					break;
				}
				case MODE_HOME:
				{
					if (!g_bStopFlag[i])
					{
						// RxPDO u16CtlWord Bit4 -> 1
						// check TxPDO u16StaWord Bit10, Bit12 -> 1
						// RxPDO u16CtlWord Bit4 -> 0
					}
					else
					{
						// RxPDO u16CtlWord Bit4 -> 0, Bit8 -> 1
					}
					
					break;
				}
				case MODE_JOGEND:
				{
					GetCmdPos_Dec(g_Motion_Params[i].m_dJogAcc);
					if (g_Motion_Params[i].m_dJogSpeed > 0)
					{
						if (g_dVel[i] < 0)
						{
							g_Position_Params[i].m_dCmdPos = g_Position_Params[i].m_dCurPos;
							g_dVel[i] = 0;
						}
					}
					else
					{
						if (g_dVel[i] > 0)
						{
							g_Position_Params[i].m_dCmdPos = g_Position_Params[i].m_dCurPos;
							g_dVel[i] = 0;
						}
					}

					if (g_Position_Params[i].m_dCmdPos == g_Position_Params[i].m_dCurPos)
					{
						g_Position_Params[i].m_uMode = MODE_IDLE;
					}

					break;
				}
				default:
				{
					break;
				}
			}

			pRxPDOData[i].n32TarPos = (int)g_Position_Params[i].m_dCmdPos;
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
