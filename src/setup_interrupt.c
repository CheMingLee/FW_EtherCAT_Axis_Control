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

double g_dVel[TEST_SERVO_CNT] = {0};
double g_dt = TEST_CYCTIME * pow(10, -9);

u32 u32LEDout = 0;

int GetCmdPos_Acc(double dSpeed, double dAcc)
{
	if (abs(g_dVel[i]) >= abs(dSpeed))
	{
		return -1;
	}
	else if (abs(g_dVel[i] + dAcc * g_dt) > abs(dSpeed))
	{
		double dS, dt;
		dS = (pow(dSpeed, 2) - pow(g_dVel[i], 2)) / (2 * dAcc);
		dt = g_dt - ((dSpeed - g_dVel[i]) / dAcc);
		g_Position_Params[i].m_dCmdPos += (dS + dSpeed * dt) * g_Motion_Params[i].m_dAxisUnit;
		g_dVel[i] = dSpeed;

		return 0;
	}

	g_Position_Params[i].m_dCmdPos += (g_dVel[i] * g_dt + 0.5 * dAcc * pow(g_dt, 2)) * g_Motion_Params[i].m_dAxisUnit;
	g_dVel[i] += dAcc * g_dt;

	return 1;
}

void GetCmdPos_ConstVel(double dSpeed)
{
	g_Position_Params[i].m_dCmdPos += (dSpeed * g_dt) * g_Motion_Params[i].m_dAxisUnit;
}

int GetCmdPos_Dec(double dSpeed, double dAcc)
{
	if ((dSpeed > 0 && g_dVel[i] - dAcc * g_dt < 0) || (dSpeed < 0 && g_dVel[i] - dAcc * g_dt > 0))
	{
		g_Position_Params[i].m_dCmdPos += (pow(g_dVel[i], 2) / (2 * dAcc)) * g_Motion_Params[i].m_dAxisUnit;
		g_dVel[i] = 0;

		return 0;
	}
	
	g_Position_Params[i].m_dCmdPos += (g_dVel[i] * g_dt - 0.5 * dAcc * pow(g_dt, 2)) * g_Motion_Params[i].m_dAxisUnit;
	g_dVel[i] -= dAcc * g_dt;

	return 1;
}

void ECM_intr_Handler(void *CallBackRef)
{
	if((u32LEDout & 0x0c) == 0)
		u32LEDout = (u32LEDout & 0x03) | 0x04;
	else
		u32LEDout = (u32LEDout & 0x03) | ((u32LEDout & 0x0c) << 1);

	if(g_bInterruptFlag)
	{
		u32LEDout |= 2;
		nret = ECM_EcatPdoFifoDataGet(TxData, u16PDOSizeRet);
		if (nret > 0)
		{
			for (i = 0; i < g_iServoCnt; i++)
			{
				g_Position_Params[i].m_iCurPos = pTxPDOData[i].n32AcuPos;
				g_Position_Params[i].m_uInput = pTxPDOData[i].u32DigInputs;
			}
		}
		else
		{
			g_Position_Params[i].m_uMode = MODE_IDLE;
		}
		
		for (i = 0; i < g_iServoCnt; i++)
		{
			switch (g_Position_Params[i].m_uMode)
			{
				case MODE_IDLE:
				{
					g_dVel[i] = 0;
					g_bStopFlag[i] = false;

					break;
				}
				case MODE_JOG:
				{
					if (g_Position_Params[i].m_uInput & DIGINPUT_LIMIT_LEFT && g_Motion_Params[i].m_dJogSpeed < 0)
					{
						g_Position_Params[i].m_uMode = MODE_IDLE;
						break;
					}
					else if (g_Position_Params[i].m_uInput & DIGINPUT_LIMIT_RIGHT && g_Motion_Params[i].m_dJogSpeed > 0)
					{
						g_Position_Params[i].m_uMode = MODE_IDLE;
						break;
					}
					
					int iRet;

					if (!g_bStopFlag[i])
					{
						iRet = GetCmdPos_Acc(g_Motion_Params[i].m_dJogSpeed, g_Motion_Params[i].m_dJogAcc);
						if (iRet < 0)
						{
							GetCmdPos_ConstVel(g_Motion_Params[i].m_dJogSpeed);
						}
					}
					else
					{
						GetCmdPos_Dec(g_Motion_Params[i].m_dJogSpeed, g_Motion_Params[i].m_dJogAcc);
						g_Position_Params[i].m_uMode = MODE_JOGEND;
					}

					break;
				}
				case MODE_MOTION:
				{
					if (g_Position_Params[i].m_uInput & DIGINPUT_LIMIT_LEFT && g_dDistance[i] < 0)
					{
						g_Position_Params[i].m_uMode = MODE_IDLE;
						break;
					}
					else if (g_Position_Params[i].m_uInput & DIGINPUT_LIMIT_RIGHT && g_dDistance[i] > 0)
					{
						g_Position_Params[i].m_uMode = MODE_IDLE;
						break;
					}

					int iRet;

					if (!g_bStopFlag[i])
					{
						double Vm, S1, S2, S3;
						
						S1 = pow(g_Motion_Params[i].m_dMotionSpeed, 2) / (2.0 * g_Motion_Params[i].m_dMotionAcc);
						S3 = S1;
						S2 = (g_dDistance[i] / g_Motion_Params[i].m_dAxisUnit) - S1 - S3;
						if (abs(g_dDistance[i] / g_Motion_Params[i].m_dAxisUnit) - abs(S1) - abs(S3) <= 0.0)
						{
							Vm = sqrt(abs(g_Motion_Params[i].m_dMotionAcc) * abs(g_dDistance[i] / g_Motion_Params[i].m_dAxisUnit));
							if (g_dDistance[i] < 0)
							{
								Vm = -Vm;
							}
							S1 = (g_dDistance[i] / g_Motion_Params[i].m_dAxisUnit) / 2.0;
							if (abs(g_Position_Params[i].m_dCmdPos - g_dStartPos[i]) / g_Motion_Params[i].m_dAxisUnit < abs(S1))
							{
								if (abs(g_dVel[i] + g_Motion_Params[i].m_dMotionAcc * g_dt) > Vm)
								{
									double dS, dt;
									g_Position_Params[i].m_dCmdPos = g_dStartPos[i] + S1 * g_Motion_Params[i].m_dAxisUnit;
									dt = g_dt - ((Vm - g_dVel[i]) / g_Motion_Params[i].m_dMotionAcc);
									g_dVel[i] = Vm - g_Motion_Params[i].m_dMotionAcc * dt;
									dS = (pow(Vm, 2) - pow(g_dVel[i], 2)) / (2 * g_Motion_Params[i].m_dMotionAcc);
									g_Position_Params[i].m_dCmdPos += dS * g_Motion_Params[i].m_dAxisUnit;
									break;
								}
								else
								{
									g_Position_Params[i].m_dCmdPos += (g_dVel[i] * g_dt + 0.5 * g_Motion_Params[i].m_dMotionAcc * pow(g_dt, 2)) * g_Motion_Params[i].m_dAxisUnit;
									g_dVel[i] += g_Motion_Params[i].m_dMotionAcc * g_dt;
									break;
								}
							}
							else
							{
								iRet = GetCmdPos_Dec(Vm, g_Motion_Params[i].m_dMotionAcc);
								if (iRet <= 0)
								{
									g_Position_Params[i].m_dCmdPos = g_Position_Params[i].m_dTarPos;
									g_Position_Params[i].m_uMode = MODE_IDLE;
									break;
								}
								break;
							}
						}
						else
						{
							if (abs(g_Position_Params[i].m_dCmdPos - g_dStartPos[i]) / g_Motion_Params[i].m_dAxisUnit < abs(S1))
							{
								GetCmdPos_Acc(g_Motion_Params[i].m_dMotionSpeed, g_Motion_Params[i].m_dMotionAcc);
								break;
							}
							else if (abs(g_Position_Params[i].m_dCmdPos - g_dStartPos[i]) / g_Motion_Params[i].m_dAxisUnit > abs(S1) && abs(g_Position_Params[i].m_dCmdPos - g_dStartPos[i]) / g_Motion_Params[i].m_dAxisUnit < abs(S1 + S2))
							{
								GetCmdPos_ConstVel(g_Motion_Params[i].m_dMotionSpeed);
								if (abs(g_Position_Params[i].m_dCmdPos - g_dStartPos[i]) / g_Motion_Params[i].m_dAxisUnit > abs(S1 + S2))
								{
									double dt, dS;
									dt = (abs(g_Position_Params[i].m_dCmdPos - g_dStartPos[i]) / g_Motion_Params[i].m_dAxisUnit - abs(S1) - abs(S2)) / abs(g_Motion_Params[i].m_dMotionSpeed);
									g_dVel[i] = g_Motion_Params[i].m_dMotionSpeed - g_Motion_Params[i].m_dMotionAcc * dt;
									dS = (pow(g_Motion_Params[i].m_dMotionSpeed, 2) - pow(g_dVel[i], 2)) / (2 * g_Motion_Params[i].m_dMotionAcc);
									g_Position_Params[i].m_dCmdPos = g_dStartPos[i] + (S1 + S2 + dS) * g_Motion_Params[i].m_dAxisUnit;
									break;
								}
								break;
							}
							else
							{
								iRet = GetCmdPos_Dec(g_Motion_Params[i].m_dMotionSpeed, g_Motion_Params[i].m_dMotionAcc);
								if (iRet <= 0)
								{
									g_Position_Params[i].m_uMode = MODE_IDLE;
									break;
								}
								break;
							}
						}
					}
					else // Stop
					{
						iRet = GetCmdPos_Dec(g_Motion_Params[i].m_dMotionSpeed, g_Motion_Params[i].m_dMotionAcc);
						if (iRet <= 0)
						{
							g_Position_Params[i].m_uMode = MODE_IDLE;
						}
					}

					break;
				}
				case MODE_HOME:
				{
					if (!g_bStopFlag[i])
					{
						// RxPDO u16CtlWord Bit4 -> 1
						pRxPDOData[i].u16CtlWord = 0x001f;
						// check TxPDO u16StaWord Bit10, Bit12 -> 1
						if (pTxPDOData[i].u16StaWord & 0x3400 == 0x1400)
						{
							// RxPDO u16CtlWord Bit4 -> 0
							pRxPDOData[i].u16CtlWord = 0x000f;
							g_Position_Params[i].m_uMode = MODE_IDLE;
						}
					}
					else
					{
						// RxPDO u16CtlWord Bit4 -> 0, Bit8 -> 1
						pRxPDOData[i].u16CtlWord = 0x010f;
						g_Position_Params[i].m_uMode = MODE_IDLE;
					}

					g_Position_Params[i].m_dCmdPos = (double)g_Position_Params[i].m_iCurPos;
					
					break;
				}
				case MODE_JOGEND:
				{
					if (g_Position_Params[i].m_uInput & DIGINPUT_LIMIT_LEFT || g_Position_Params[i].m_uInput & DIGINPUT_LIMIT_RIGHT)
					{
						g_Position_Params[i].m_uMode = MODE_IDLE;
						break;
					}
					
					int iRet;

					iRet = GetCmdPos_Dec(g_Motion_Params[i].m_dJogSpeed, g_Motion_Params[i].m_dJogAcc);
					if (iRet <= 0)
					{
						g_Position_Params[i].m_uMode = MODE_IDLE;
					}

					break;
				}
				default:
					break;
			}

			pRxPDOData[i].n32TarPos = (int)g_Position_Params[i].m_dCmdPos;
		}
		ECM_EcatPdoFifoDataSend(RxData, u16PDOSize);
	}
	
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
