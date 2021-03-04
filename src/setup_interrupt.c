#include "setting.h"

int GetCmdPos_Acc(double dSpeed, double dAcc, int iAxis)
{
	if (fabs(g_dVel[iAxis]) >= fabs(dSpeed))
	{
		return -1;
	}
	else if (fabs(g_dVel[iAxis] + dAcc * g_dt) > fabs(dSpeed))
	{
		double dS, dt;
		dS = pow(dSpeed, 2) / (2 * dAcc);
		dt = g_dt - ((dSpeed - g_dVel[iAxis]) / dAcc);
		g_Position_Params[iAxis].m_dCmdPos = g_dStartPos[iAxis] + dS + dSpeed * dt;
		g_dVel[iAxis] = dSpeed;
		g_dTime[iAxis] = 0;
		g_dStartPos[iAxis] = g_Position_Params[iAxis].m_dCmdPos;
		return 0;
	}

	g_Position_Params[iAxis].m_dCmdPos = g_dStartPos[iAxis] + 0.5 * dAcc * pow(g_dTime[iAxis], 2);
	g_dVel[iAxis] = dAcc * g_dTime[iAxis];
	g_dTime[iAxis] += g_dt;

	return 1;
}

void GetCmdPos_ConstVel(double dSpeed, int iAxis)
{
	g_Position_Params[iAxis].m_dCmdPos = g_dStartPos[iAxis] + dSpeed * g_dTime[iAxis];
	g_dTime[iAxis] += g_dt;
	g_dVel[iAxis] = dSpeed;
}

int GetCmdPos_Dec(double dSpeed, double dAcc, int iAxis)
{
	if ((dSpeed > 0 && (g_dVel[iAxis] - dAcc * g_dt) < 0) || (dSpeed < 0 && (g_dVel[iAxis] - dAcc) * g_dt > 0))
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
	
	if(g_bInterruptFlag)
	{
		g_u16JF8out ^= 0xffff;

		nret = ECM_EcatPdoFifoDataGet(g_TxData, g_u16PDOSizeRet);
		if (nret > 0)
		{
			g_u32LEDout |= 2;

			for (i = 0; i < g_iServoCnt; i++)
			{
				g_Position_Params[i].m_iCurPos = g_pTxPDOData[i].n32AcuPos;
				g_Position_Params[i].m_uInput = g_pTxPDOData[i].u32DigInputs;
			}
		}
		else
		{
			g_u32LEDout &= 0xfffffffd;

			for (i = 0; i < g_iServoCnt; i++)
			{
				g_Position_Params[i].m_uMode = MODE_IDLE;
			}
		}
		
		for (i = 0; i < g_iServoCnt; i++)
		{
			switch (g_Position_Params[i].m_uMode)
			{
				case MODE_IDLE:
				{
					g_dVel[i] = 0;
					g_dTime[i] = 0;
					g_bStopFlag[i] = false;

					break;
				}
				case MODE_JOG:
				{
					// if (g_Position_Params[i].m_uInput & DIGINPUT_LIMIT_LEFT && g_Motion_Params[i].m_dJogSpeed < 0)
					// {
					// 	g_Position_Params[i].m_uMode = MODE_IDLE;
					// 	break;
					// }
					// else if (g_Position_Params[i].m_uInput & DIGINPUT_LIMIT_RIGHT && g_Motion_Params[i].m_dJogSpeed > 0)
					// {
					// 	g_Position_Params[i].m_uMode = MODE_IDLE;
					// 	break;
					// }
					
					int iRet;

					if (!g_bStopFlag[i])
					{
						iRet = GetCmdPos_Acc(g_Motion_Params[i].m_dJogSpeed, g_Motion_Params[i].m_dJogAcc, i);
						if (iRet < 0)
						{
							GetCmdPos_ConstVel(g_Motion_Params[i].m_dJogSpeed, i);
						}
					}
					else
					{
						GetCmdPos_Dec(g_Motion_Params[i].m_dJogSpeed, g_Motion_Params[i].m_dJogAcc, i);
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
								iRet = GetCmdPos_Dec(Vm, g_Motion_Params[i].m_dMotionAcc, i);
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
								GetCmdPos_Acc(g_Motion_Params[i].m_dMotionSpeed, g_Motion_Params[i].m_dMotionAcc, i);
								break;
							}
							else if (abs(g_Position_Params[i].m_dCmdPos - g_dStartPos[i]) / g_Motion_Params[i].m_dAxisUnit > abs(S1) && abs(g_Position_Params[i].m_dCmdPos - g_dStartPos[i]) / g_Motion_Params[i].m_dAxisUnit < abs(S1 + S2))
							{
								GetCmdPos_ConstVel(g_Motion_Params[i].m_dMotionSpeed, i);
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
								iRet = GetCmdPos_Dec(g_Motion_Params[i].m_dMotionSpeed, g_Motion_Params[i].m_dMotionAcc, i);
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
						iRet = GetCmdPos_Dec(g_Motion_Params[i].m_dMotionSpeed, g_Motion_Params[i].m_dMotionAcc, i);
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
						g_pRxPDOData[i].u16CtlWord = 0x001f;
						g_Position_Params[i].m_dCmdPos = (double)g_Position_Params[i].m_iCurPos;
						// check TxPDO u16StaWord Bit12 -> 1
						if (g_pTxPDOData[i].u16StaWord & 0x1000)
						{
							// RxPDO u16CtlWord Bit4 -> 0
							g_pRxPDOData[i].u16CtlWord = 0x000f;
							g_Position_Params[i].m_uMode = MODE_IDLE;
						}
					}
					else
					{
						// RxPDO u16CtlWord Bit4 -> 0, Bit8 -> 1
						g_pRxPDOData[i].u16CtlWord = 0x010f;
						g_Position_Params[i].m_uMode = MODE_IDLE;
					}
					
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

					iRet = GetCmdPos_Dec(g_Motion_Params[i].m_dJogSpeed, g_Motion_Params[i].m_dJogAcc, i);
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
