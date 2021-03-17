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

void CheckVend(double dXY_Dis, double dAcc)
{
	double dVs = g_dXY_Ve;
	double dS = pow(dVs, 2) / (2.0 * dAcc);

	if (dS > dXY_Dis)
	{
		g_dXY_Ve = sqrt(2.0 * dAcc * dXY_Dis);
	}
}

int CheckTheta(int iIndex)
{
	double dCosTheta;
	double dX2, dX1, dX0;
	double dY2, dY1, dY0;
	double dXY_DisNext;
	
	if (g_CmdBuf[iIndex % 100 + 1].m_iID == LINEXY || g_CmdBuf[iIndex % 100 + 1].m_iID == FLINEXY)
	{
		dX0 = g_cmd_file_params.m_dBegPos[0];
		dX1 = g_cmd_file_params.m_dEndPos[0];
		dX2 = g_CmdBuf[iIndex % 100 + 1].m_dParams[0];
		dY0 = g_cmd_file_params.m_dBegPos[1];
		dY1 = g_cmd_file_params.m_dEndPos[1];
		dY2 = g_CmdBuf[iIndex % 100 + 1].m_dParams[1];
		dXY_DisNext = sqrt(pow(dX2 - dX1, 2) + pow(dY2 - dY1, 2));
		if (dXY_DisNext != 0)
		{
			dCosTheta = ((dX1 - dX0) * (dX2 - dX1) + (dY1 - dY0) * (dY2 - dY1)) / (dXY_DisNext * g_dXY_Dis);
			if (dCosTheta > cos(g_dThetaMax * PI / 180.0))
			{
				if (g_CmdBuf[iIndex % 100].m_iID == LINEXY)
				{
					g_dXY_Ve = g_cmd_file_params.m_dSpeed;
					if (g_CmdBuf[iIndex % 100 + 1].m_iID == FLINEXY)
					{
						CheckVend(dXY_DisNext, g_cmd_file_params.m_dFAcc);
					}
					else
					{
						CheckVend(dXY_DisNext, g_cmd_file_params.m_dAcc);
					}
				}
				else
				{
					if (g_CmdBuf[iIndex % 100 + 1].m_iID == LINEXY)
					{
						g_dXY_Ve = g_cmd_file_params.m_dSpeed;
						CheckVend(dXY_DisNext, g_cmd_file_params.m_dAcc);
					}
					else
					{
						g_dXY_Ve = g_cmd_file_params.m_dFSpeed;
						CheckVend(dXY_DisNext, g_cmd_file_params.m_dFAcc);
					}
				}
				return 1;
			}
		}
		else
		{
			iIndex++;
			return CheckTheta(iIndex);
		}
	}

	g_dXY_Ve = 0.0;
	return 0;
}

int CalDistance()
{
	g_dXY_Dis = sqrt(pow(g_cmd_file_params.m_dEndPos[0] - g_cmd_file_params.m_dBegPos[0], 2) + pow(g_cmd_file_params.m_dEndPos[1] - g_cmd_file_params.m_dBegPos[1], 2));
	if (g_dXY_Dis <= 0.0)
	{
		return 0;
	}
	return 1;
}

int CalRatio()
{
	if (g_dXY_Dis != 0.0)
	{
		g_cmd_file_params.m_dRatio[0] = (g_cmd_file_params.m_dEndPos[0] - g_cmd_file_params.m_dBegPos[0]) / g_dXY_Dis;
		g_cmd_file_params.m_dRatio[1] = (g_cmd_file_params.m_dEndPos[1] - g_cmd_file_params.m_dBegPos[1]) / g_dXY_Dis;
		return 1;
	}
	else
	{
		return 0;
	}
}

void GetPathParams(double dSpeed, double dAcc)
{
	g_dXY_S1 = (pow(dSpeed, 2) - pow(g_dXY_Vs, 2)) / (2.0 * dAcc);
	g_dXY_S3 = (pow(dSpeed, 2) - pow(g_dXY_Ve, 2)) / (2.0 * dAcc);
	g_dXY_S2 = g_dXY_Dis - g_dXY_S1 - g_dXY_S3;
	if (g_dXY_S2 <= 0.0)
	{
		g_dXY_S1 = (2.0 * dAcc * g_dXY_Dis - pow(g_dXY_Vs, 2)) / (4.0 * dAcc);
		if (g_dXY_S1 < 0.0)
		{
			g_dXY_Ve = sqrt(pow(g_dXY_Vs, 2) - 2.0 * dAcc * g_dXY_Dis);
			g_dXY_Vm = g_dXY_Vs;
			g_dXY_S1 = 0.0;
			g_dXY_S2 = 0.0;
			g_dXY_S3 = g_dXY_Dis;
			g_dXY_T1 = 0.0;
			g_dXY_T2 = 0.0;
			g_dXY_T3 = 2.0 * g_dXY_S3 / (g_dXY_Ve + g_dXY_Vm);
			g_dXY_Ttotal = g_dXY_T1 + g_dXY_T2 + g_dXY_T3;
		}
		else
		{
			g_dXY_Vm = sqrt((2.0 * dAcc * g_dXY_Dis + pow(g_dXY_Vs, 2)) / 2.0);
			g_dXY_S1 = (pow(g_dXY_Vm, 2) - pow(g_dXY_Vs, 2)) / (2.0 * dAcc);
			g_dXY_S2 = 0.0;
			g_dXY_S3 = g_dXY_Dis - g_dXY_S1 - g_dXY_S2;
			g_dXY_T1 = 2.0 * g_dXY_S1 / (g_dXY_Vs + g_dXY_Vm);
			g_dXY_T2 = 0.0;
			g_dXY_T3 = 2.0 * g_dXY_S3 / (g_dXY_Ve + g_dXY_Vm);
			g_dXY_Ttotal = g_dXY_T1 + g_dXY_T2 + g_dXY_T3;
		}
	}
	else
	{
		g_dXY_Vm = dSpeed;
		g_dXY_T1 = 2.0 * g_dXY_S1 / (g_dXY_Vs + g_dXY_Vm);
		g_dXY_T2 = g_dXY_S2 / g_dXY_Vm;
		g_dXY_T3 = 2.0 * g_dXY_S3 / (g_dXY_Ve + g_dXY_Vm);
		g_dXY_Ttotal = g_dXY_T1 + g_dXY_T2 + g_dXY_T3;
	}
}

int GetPathCmdPos(double dSpeed, double dAcc)
{
	g_dXY_Time += g_dt;
	if (g_dXY_Time <= g_dXY_T1)
	{
		g_dXY_V = dAcc * g_dXY_Time;
		g_dXY_S = 0.5 * dAcc * pow(g_dXY_Time, 2);
	}
	else if (g_dXY_Time > g_dXY_T1 && g_dXY_Time <= g_dXY_T1 + g_dXY_T2)
	{
		g_dXY_V = dSpeed;
		g_dXY_S = g_dXY_S1 + dSpeed * (g_dXY_Time - g_dXY_T1);
	}
	else if (g_dXY_Time > g_dXY_T1 + g_dXY_T2 && g_dXY_Time <= g_dXY_Ttotal)
	{
		g_dXY_V = g_dXY_Vm - dAcc * (g_dXY_Time - g_dXY_T1 - g_dXY_T2);
		g_dXY_S = g_dXY_S1 + g_dXY_S2 + g_dXY_Vm * (g_dXY_Time - g_dXY_T1 - g_dXY_T2) - 0.5 * dAcc * pow((g_dXY_Time - g_dXY_T1 - g_dXY_T2), 2);
	}
	else
	{
		g_dXY_V = 0.0;
		g_Position_Params[0].m_dCmdPos = g_cmd_file_params.m_dEndPos[0];
		g_Position_Params[1].m_dCmdPos = g_cmd_file_params.m_dEndPos[1];
		return 0;
	}

	g_Position_Params[0].m_dCmdPos = g_cmd_file_params.m_dBegPos[0] + g_dXY_S * g_cmd_file_params.m_dRatio[0];
	g_Position_Params[1].m_dCmdPos = g_cmd_file_params.m_dBegPos[1] + g_dXY_S * g_cmd_file_params.m_dRatio[1];

	return 1;
}

void SetNextCmd()
{
	g_cmd_file_params.m_dBegPos[0] = g_CmdBuf[g_iFileCmdIndex % 100].m_dParams[0];
	g_cmd_file_params.m_dBegPos[1] = g_CmdBuf[g_iFileCmdIndex % 100].m_dParams[1];
	g_dXY_Time = 0.0;
	g_dXY_V = 0.0;
	g_dXY_Vs = g_dXY_Ve;
	g_iFileCmdIndex++;
}

void ECM_intr_Handler(void *CallBackRef)
{
	int i; // Axis
	int nret = 0;

	u32 uBusyFlag;
	uBusyFlag = Xil_In32(ECM_ADDR_BUSY);
	if (uBusyFlag)
	{
		return;
	}
	
	if(g_bInterruptFlag)
	{
		g_u16JF8out = 1;
		Xil_Out16(IO_ADDR_OUTPUT, g_u16JF8out);

		nret = ECM_EcatPdoFifoDataGet(g_TxData, g_u16PDOSizeRet);
		if (nret > 0)
		{
			for (i = 0; i < g_iServoCnt; i++)
			{
				g_Position_Params[i].m_iCurPos = g_pTxPDOData[i].n32AcuPos;
				g_Position_Params[i].m_uInput = g_pTxPDOData[i].u32DigInputs;
			}
		}
		
		if (g_bRunFileFlag)
		{
			g_bBeginPosFlag[0] = false;
			g_bBeginPosFlag[1] = false;

			if (g_bStopFlag[0] || g_bStopFlag[1])
			{
				// Stop
				g_bRunFileFlag = false;
				g_iFileCmdIndex = 0;
				g_Position_Params[0].m_uMode = MODE_IDLE;
				g_Position_Params[1].m_uMode = MODE_IDLE;
			}
			else
			{
				FILE_CMD CurCmd;
				CurCmd = g_CmdBuf[g_iFileCmdIndex % 100];

				switch (CurCmd.m_iID)
				{
					case BEGIN:
					{
						g_dXY_Vs = 0.0;
						g_dXY_Ve = 0.0;
						SetNextCmd();
						g_Position_Params[0].m_dCmdPos = g_cmd_file_params.m_dBegPos[0];
						g_Position_Params[1].m_dCmdPos = g_cmd_file_params.m_dBegPos[1];
						break;
					}
					case SPEED:
					{
						g_cmd_file_params.m_dSpeed = CurCmd.m_dParams[0];
						g_cmd_file_params.m_dFSpeed = CurCmd.m_dParams[1];
						g_iFileCmdIndex++;
						break;
					}	
					case ACC:
					{
						g_cmd_file_params.m_dAcc = CurCmd.m_dParams[0];
						g_cmd_file_params.m_dFAcc = CurCmd.m_dParams[1];
						g_iFileCmdIndex++;
						break;
					}
					case LINEXY:
					{
						g_cmd_file_params.m_dEndPos[0] = CurCmd.m_dParams[0];
						g_cmd_file_params.m_dEndPos[1] = CurCmd.m_dParams[1];
						nret = CalDistance();
						if (nret <= 0)
						{
							SetNextCmd();
							break;
						}
						else
						{
							CalRatio();
							CheckTheta(g_iFileCmdIndex);
							GetPathParams(g_cmd_file_params.m_dSpeed, g_cmd_file_params.m_dAcc);
						}
						nret = GetPathCmdPos(g_cmd_file_params.m_dSpeed, g_cmd_file_params.m_dAcc);
						if (nret <= 0)
						{
							SetNextCmd();
						}
						break;
					}
					case FLINEXY:
					{
						g_u32LEDout |= 0x02;

						g_cmd_file_params.m_dEndPos[0] = CurCmd.m_dParams[0];
						g_cmd_file_params.m_dEndPos[1] = CurCmd.m_dParams[1];
						nret = CalDistance();
						if (nret <= 0)
						{
							SetNextCmd();
							break;
						}
						else
						{
							CalRatio();
							CheckTheta(g_iFileCmdIndex);
							GetPathParams(g_cmd_file_params.m_dFSpeed, g_cmd_file_params.m_dFAcc);
						}
						nret = GetPathCmdPos(g_cmd_file_params.m_dFSpeed, g_cmd_file_params.m_dFAcc);
						if (nret <= 0)
						{
							SetNextCmd();
						}
						break;
					}
					case END:
					{
						g_u32LEDout &= 0xfffffffd;

						g_bRunFileFlag = false;
						g_Position_Params[0].m_uMode = MODE_IDLE;
						g_Position_Params[1].m_uMode = MODE_IDLE;
					}
					default:
						break;
				}
			}

			g_pRxPDOData[0].n32TarPos = (int)g_Position_Params[0].m_dCmdPos;
			g_pRxPDOData[1].n32TarPos = (int)g_Position_Params[1].m_dCmdPos;
		}
		else
		{
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
						g_bRunFileFlag = false;
						g_iFileCmdIndex = 0;

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
								g_bBeginPosFlag[i] = true;
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
							// RxPDO u16CtlWord Bit4 -> 1
							g_pRxPDOData[i].u16CtlWord = 0x001f;
							if (g_bHomingFlag[i])
							{
								// TxPDO u16StaWord Bit12 -> 1
								if (g_pTxPDOData[i].u16StaWord & 0x1000)
								{
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

						nret = GetCmdPos_Dec_Jog(g_Motion_Params[i].m_dJogSpeed, g_Motion_Params[i].m_dJogAcc, i);
						if (nret <= 0)
						{
							g_Position_Params[i].m_uMode = MODE_IDLE;
						}
						break;
					}
					case MODE_RUNFILE:
					{
						break;
					}
					default:
						break;
				}

				g_pRxPDOData[i].n32TarPos = (int)g_Position_Params[i].m_dCmdPos;
			}
		}

		ECM_EcatPdoFifoDataSend(g_RxData, g_u16PDOSize);

		g_u16JF8out = 0;
		Xil_Out16(IO_ADDR_OUTPUT, g_u16JF8out);
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
