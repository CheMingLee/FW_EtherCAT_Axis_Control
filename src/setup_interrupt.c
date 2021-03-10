#include "setting.h"

int GetCmdPos_Acc_Jog(double dSpeed, double dAcc, int iAxis)
{
	if (fabs(g_dVel[iAxis]) >= fabs(dSpeed))
	{
		return -1;
	}
	else if (fabs(g_dVel[iAxis] + dAcc * g_dt) > fabs(dSpeed))
	{
		double dS, dt;
		dS = (pow(dSpeed, 2) - pow(g_dVel[iAxis], 2)) / (2 * dAcc);
		dt = g_dt - ((dSpeed - g_dVel[iAxis]) / dAcc);
		g_Position_Params[iAxis].m_dCmdPos += dS + dSpeed * dt;
		g_dVel[iAxis] = dSpeed;

		return 0;
	}

	g_Position_Params[iAxis].m_dCmdPos += g_dVel[iAxis] * g_dt + 0.5 * dAcc * pow(g_dt, 2);
	g_dVel[iAxis] += dAcc *g_dt;

	return 1;
}

void GetCmdPos_ConstVel_Jog(double dSpeed, int iAxis)
{
	g_Position_Params[iAxis].m_dCmdPos += dSpeed * g_dt;
	g_dVel[iAxis] = dSpeed;
}

int GetCmdPos_Dec_Jog(double dSpeed, double dAcc, int iAxis)
{
	if ((dSpeed > 0 && (g_dVel[iAxis] - dAcc * g_dt) < 0) || (dSpeed < 0 && (g_dVel[iAxis] - dAcc * g_dt) > 0))
	{
		g_Position_Params[iAxis].m_dCmdPos += pow(g_dVel[iAxis], 2) / (2 * dAcc);
		g_dVel[iAxis] = 0;

		return 0;
	}
	
	g_Position_Params[iAxis].m_dCmdPos += g_dVel[iAxis] * g_dt - 0.5 * dAcc * pow(g_dt, 2);
	g_dVel[iAxis] -= dAcc * g_dt;

	return 1;
}

void ECM_intr_Handler(void *CallBackRef)
{
	int i;
	int nret = 0;

	u32 uBusyFlag;
	uBusyFlag = Xil_In32(ECM_ADDR_BUSY);
	if (uBusyFlag)
	{
		return;
	}
	
	if(g_bInterruptFlag)
	{
		g_u16JF8out ^= 0xffff;

		nret = ECM_EcatPdoFifoDataGet(g_TxData, g_u16PDOSizeRet);
		if (nret > 0)
		{
			for (i = 0; i < g_iServoCnt; i++)
			{
				g_Position_Params[i].m_iCurPos = g_pTxPDOData[i].n32AcuPos;
				g_Position_Params[i].m_uInput = g_pTxPDOData[i].u32DigInputs;
			}
		}
		
		for (i = 0; i < g_iServoCnt; i++)
		{
			switch (g_Position_Params[i].m_uMode)
			{
				case MODE_IDLE:
				{
					g_dVel[i] = 0.0;
					g_dTime[i] = 0.0;
					g_dS[i] = 0.0;
					g_bStopFlag[i] = false;
					g_bHomingFlag[i] = false;
					g_iCnt[i] = 5;
					g_pRxPDOData[i].u16CtlWord = 0x000f;
					break;
				}
				case MODE_JOG:
				{
					if (g_Position_Params[i].m_uInput & DIGINPUT_LIMIT_LEFT && g_Motion_Params[i].m_dJogSpeed < 0.0)
					{
						g_Position_Params[i].m_uMode = MODE_IDLE;
						break;
					}
					else if (g_Position_Params[i].m_uInput & DIGINPUT_LIMIT_RIGHT && g_Motion_Params[i].m_dJogSpeed > 0.0)
					{
						g_Position_Params[i].m_uMode = MODE_IDLE;
						break;
					}

					g_pRxPDOData[i].u16CtlWord = 0x000f;

					if (!g_bStopFlag[i])
					{
						nret = GetCmdPos_Acc_Jog(g_Motion_Params[i].m_dJogSpeed, g_Motion_Params[i].m_dJogAcc, i);
						if (nret <= 0)
						{
							GetCmdPos_ConstVel_Jog(g_Motion_Params[i].m_dJogSpeed, i);
						}
					}
					else
					{
						GetCmdPos_Dec_Jog(g_Motion_Params[i].m_dJogSpeed, g_Motion_Params[i].m_dJogAcc, i);
						g_Position_Params[i].m_uMode = MODE_JOGEND;
					}
					break;
				}
				case MODE_MOTION:
				{
					if ((g_Position_Params[i].m_uInput & DIGINPUT_LIMIT_LEFT) || (g_Position_Params[i].m_uInput & DIGINPUT_LIMIT_RIGHT))
					{
						g_Position_Params[i].m_uMode = MODE_IDLE;
						break;
					}

					g_pRxPDOData[i].u16CtlWord = 0x000f;

					if (!g_bStopFlag[i])
					{
						g_dTime[i] += g_dt;
						if (g_dTime[i] <= g_dT1[i])
						{
							g_dVel[i] = g_Motion_Params[i].m_dMotionAcc * g_dTime[i];
							g_dS[i] = 0.5 * g_Motion_Params[i].m_dMotionAcc * pow(g_dTime[i], 2);
							g_Position_Params[i].m_dCmdPos = g_dStartPos[i] + g_dS[i] * g_dDirection[i];
						}
						else if (g_dTime[i] > g_dT1[i] && g_dTime[i] <= g_dT1[i] + g_dT2[i])
						{
							g_dVel[i] = g_Motion_Params[i].m_dMotionSpeed;
							g_dS[i] = g_dS1[i] + g_Motion_Params[i].m_dMotionSpeed * (g_dTime[i] - g_dT1[i]);
							g_Position_Params[i].m_dCmdPos = g_dStartPos[i] + g_dS[i] * g_dDirection[i];
						}
						else if (g_dTime[i] > g_dT1[i] + g_dT2[i] && g_dTime[i] <= g_dTtotal[i])
						{
							g_dVel[i] = g_dVm[i] - g_Motion_Params[i].m_dMotionAcc * (g_dTime[i] - g_dT1[i] - g_dT2[i]);
							g_dS[i] = g_dS1[i] + g_dS2[i] + g_dVm[i] * (g_dTime[i] - g_dT1[i] - g_dT2[i]) - 0.5 * g_Motion_Params[i].m_dMotionAcc * pow((g_dTime[i] - g_dT1[i] - g_dT2[i]), 2);
							g_Position_Params[i].m_dCmdPos = g_dStartPos[i] + g_dS[i] * g_dDirection[i];
						}
						else
						{
							g_dVel[i] = 0.0;
							g_Position_Params[i].m_dCmdPos = g_Position_Params[i].m_dTarPos;
							g_Position_Params[i].m_uMode = MODE_IDLE;
						}
					}
					else // Stop
					{	
						double dStopSpeed, dStopAcc;
						
						dStopSpeed = g_Motion_Params[i].m_dMotionSpeed;
						dStopAcc = g_Motion_Params[i].m_dMotionAcc;
						if (g_dDirection[i] < 0)
						{
							dStopSpeed = -fabs(dStopSpeed);
							dStopAcc = -fabs(dStopAcc);
							g_dVel[i] = -fabs(g_dVel[i]);
						}
						
						nret = GetCmdPos_Dec_Jog(dStopSpeed, dStopAcc, i);
						if (nret <= 0)
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
						g_u32LEDout |= 2;

						// RxPDO u16CtlWord Bit4 -> 1
						g_pRxPDOData[i].u16CtlWord = 0x001f;
						if (g_bHomingFlag[i])
						{
							// TxPDO u16StaWord Bit12 -> 1
							if (g_pTxPDOData[i].u16StaWord & 0x1000)
							{
								g_u32LEDout &= 0xfffffffd;

								// RxPDO u16CtlWord Bit4 -> 0
								g_pRxPDOData[i].u16CtlWord = 0x000f;
								g_Position_Params[i].m_uMode = MODE_IDLE;
								g_bHomingFlag[i] = false;
								g_iCnt[i] = 5;
								g_Position_Params[i].m_dCmdPos = (double)g_Position_Params[i].m_iCurPos;
								break;
							}
						}
						else
						{
							if (g_iCnt[i]-- < 0)
							{
								g_bHomingFlag[i] = true;
							}
						}
					}
					else
					{
						// RxPDO u16CtlWord Bit4 -> 0, Bit8 -> 1
						g_pRxPDOData[i].u16CtlWord = 0x010f;
						g_Position_Params[i].m_uMode = MODE_IDLE;
						g_bHomingFlag[i] = false;
						g_iCnt[i] = 5;
					}
					g_Position_Params[i].m_dCmdPos = (double)g_Position_Params[i].m_iCurPos;
					break;
				}
				case MODE_JOGEND:
				{
					g_pRxPDOData[i].u16CtlWord = 0x000f;

					int iRet = GetCmdPos_Dec_Jog(g_Motion_Params[i].m_dJogSpeed, g_Motion_Params[i].m_dJogAcc, i);
					if (iRet <= 0)
					{
						g_Position_Params[i].m_uMode = MODE_IDLE;
					}
					break;
				}
				default:
					break;
			}

			g_pRxPDOData[i].n32TarPos = (int)g_Position_Params[i].m_dCmdPos;
		}

		ECM_EcatPdoFifoDataSend(g_RxData, g_u16PDOSize);
	}
	
	Xil_Out32(ECM_INTR_RESET, 1);
}

void SetupInterruptSystem()
{
	XScuGic *GicInstancePtr = &g_Intc;
    XScuGic_Config *IntcConfig; //GIC config
	Xil_ExceptionInit();

	//initialise the GIC
	IntcConfig = XScuGic_LookupConfig(INTC_DEVICE_ID);

	XScuGic_CfgInitialize(GicInstancePtr, IntcConfig, IntcConfig->CpuBaseAddress);

	//connect to the hardware
	Xil_ExceptionRegisterHandler(XIL_EXCEPTION_ID_INT, (Xil_ExceptionHandler)XScuGic_InterruptHandler, GicInstancePtr);

	XScuGic_Connect(GicInstancePtr, ECM_INTR_ID, (Xil_ExceptionHandler)ECM_intr_Handler, (void *)&g_Intc);

	XScuGic_Enable(GicInstancePtr, ECM_INTR_ID);

	// Enable interrupts in the Processor.
	Xil_ExceptionEnableMask(XIL_EXCEPTION_IRQ);
}
