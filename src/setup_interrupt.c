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

void GetCmdPos_Motion(int iAxis)
{
	g_dTime[iAxis] += g_dt;
	if (g_dTime[iAxis] <= g_dT1[iAxis])
	{
		g_dVel[iAxis] = g_Motion_Params[iAxis].m_dMotionAcc * g_dTime[iAxis];
		g_dS[iAxis] = 0.5 * g_Motion_Params[iAxis].m_dMotionAcc * pow(g_dTime[iAxis], 2);
		g_Position_Params[iAxis].m_dCmdPos = g_dStartPos[iAxis] + g_dS[iAxis] * g_dDirection[iAxis];
	}
	else if (g_dTime[iAxis] > g_dT1[iAxis] && g_dTime[iAxis] <= g_dT1[iAxis] + g_dT2[iAxis])
	{
		g_dVel[iAxis] = g_Motion_Params[iAxis].m_dMotionSpeed;
		g_dS[iAxis] = g_dS1[iAxis] + g_Motion_Params[iAxis].m_dMotionSpeed * (g_dTime[iAxis] - g_dT1[iAxis]);
		g_Position_Params[iAxis].m_dCmdPos = g_dStartPos[iAxis] + g_dS[iAxis] * g_dDirection[iAxis];
	}
	else if (g_dTime[iAxis] > g_dT1[iAxis] + g_dT2[iAxis] && g_dTime[iAxis] <= g_dTtotal[iAxis])
	{
		g_dVel[iAxis] = g_dVm[iAxis] - g_Motion_Params[iAxis].m_dMotionAcc * (g_dTime[iAxis] - g_dT1[iAxis] - g_dT2[iAxis]);
		g_dS[iAxis] = g_dS1[iAxis] + g_dS2[iAxis] + g_dVm[iAxis] * (g_dTime[iAxis] - g_dT1[iAxis] - g_dT2[iAxis]) - 0.5 * g_Motion_Params[iAxis].m_dMotionAcc * pow((g_dTime[iAxis] - g_dT1[iAxis] - g_dT2[iAxis]), 2);
		g_Position_Params[iAxis].m_dCmdPos = g_dStartPos[iAxis] + g_dS[iAxis] * g_dDirection[iAxis];
	}
	else
	{
		g_dVel[iAxis] = 0.0;
		g_Position_Params[iAxis].m_dCmdPos = g_Position_Params[iAxis].m_dTarPos;
		g_Position_Params[iAxis].m_uMode = MODE_IDLE;
		g_bBeginPosFlag[iAxis] = true;
	}
}

void GetBegParams()
{
	g_Position_Params[0].m_dCmdPos = g_cmd_file_params.m_dBegPos[0];
	g_Position_Params[1].m_dCmdPos = g_cmd_file_params.m_dBegPos[1];
}

void GetSpeedParams()
{
	g_cmd_file_params.m_dSpeed = g_CmdBuf[g_iFileCmdIndex % 100].m_dParams[0];
	g_cmd_file_params.m_dFSpeed = g_CmdBuf[g_iFileCmdIndex % 100].m_dParams[1];
}

void GetAccParams()
{
	g_cmd_file_params.m_dAcc = g_CmdBuf[g_iFileCmdIndex % 100].m_dParams[0];
	g_cmd_file_params.m_dFAcc = g_CmdBuf[g_iFileCmdIndex % 100].m_dParams[1];
}

void GetLineParams()
{
	g_cmd_file_params.m_dEndPos[0] = g_CmdBuf[g_iFileCmdIndex % 100].m_dParams[0];
	g_cmd_file_params.m_dEndPos[1] = g_CmdBuf[g_iFileCmdIndex % 100].m_dParams[1];
}

void GetArcParams()
{
	g_cmd_file_params.m_dEndPos[0] = g_CmdBuf[g_iFileCmdIndex % 100].m_dParams[0];
	g_cmd_file_params.m_dEndPos[1] = g_CmdBuf[g_iFileCmdIndex % 100].m_dParams[1];
	g_cmd_file_params.m_dCenPos[0] = g_CmdBuf[g_iFileCmdIndex % 100].m_dParams[2];
	g_cmd_file_params.m_dCenPos[1] = g_CmdBuf[g_iFileCmdIndex % 100].m_dParams[3];
	g_cmd_file_params.m_dArcDir = g_CmdBuf[g_iFileCmdIndex % 100].m_dParams[4];
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
		g_dXY_V = g_dXY_Vs + dAcc * g_dXY_Time;
		g_dXY_S = g_dXY_Vs * g_dXY_Time + 0.5 * dAcc * pow(g_dXY_Time, 2);
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
		g_dXY_V = g_dXY_Ve;
		g_Position_Params[0].m_dCmdPos = g_cmd_file_params.m_dEndPos[0];
		g_Position_Params[1].m_dCmdPos = g_cmd_file_params.m_dEndPos[1];
		return 0;
	}

	g_Position_Params[0].m_dCmdPos = g_cmd_file_params.m_dBegPos[0] + g_dXY_S * g_cmd_file_params.m_dRatio[0];
	g_Position_Params[1].m_dCmdPos = g_cmd_file_params.m_dBegPos[1] + g_dXY_S * g_cmd_file_params.m_dRatio[1];

	return 1;
}

int CalRadius()
{
	double dX1, dX0, dXc, dY1, dY0, dYc;
	double dR1, dR0;

	dX1 = g_cmd_file_params.m_dEndPos[0];
	dX0 = g_cmd_file_params.m_dBegPos[0];
	dXc = g_cmd_file_params.m_dCenPos[0];
	dY1 = g_cmd_file_params.m_dEndPos[1];
	dY0 = g_cmd_file_params.m_dBegPos[1];
	dYc = g_cmd_file_params.m_dCenPos[1];
	dR1 = sqrt(pow(dX1 - dXc, 2) + pow(dY1 - dYc, 2));
	dR0 = sqrt(pow(dX0 - dXc, 2) + pow(dY0 - dYc, 2));
	if (dR1 == dR0)
	{
		g_cmd_file_params.m_dRadius = dR0;
		return 1;
	}
	return 0;
}

double ChangeThetaRange(double CosTheta, double SinTheta, double Theta)
{
	if (SinTheta < 0) // 3 and 4
	{
		if (CosTheta > 0) // 4
		{
			Theta = 2.0 * PI - Theta;
		}
		else if (CosTheta < 0) // 3
		{
			Theta = Theta + (PI - Theta);
		}
		else // y Axis
		{
			Theta = 1.5 * PI;
		}
	}
	return Theta;
}

int CalArcThetaDis()
{
	if (g_cmd_file_params.m_dRadius <= 0.0)
	{
		return 0;
	}

	double dX1, dX0, dXc, dY1, dY0, dYc, dR;
	double dCosThetaS, dSinThetaS, dThetaS, dCosThetaE, dSinThetaE, dThetaE;

	dX1 = g_cmd_file_params.m_dEndPos[0];
	dX0 = g_cmd_file_params.m_dBegPos[0];
	dXc = g_cmd_file_params.m_dCenPos[0];
	dY1 = g_cmd_file_params.m_dEndPos[1];
	dY0 = g_cmd_file_params.m_dBegPos[1];
	dYc = g_cmd_file_params.m_dCenPos[1];
	dR = g_cmd_file_params.m_dRadius;
	dCosThetaS = (dX0 - dXc) / dR;
	dSinThetaS = (dY0 - dYc) / dR;
	dCosThetaE = (dX1 - dXc) / dR;
	dSinThetaE = (dY1 - dYc) / dR;

	dThetaS = acos((dX0 - dXc) / dR); // range is [0, PI]
	g_dXY_Theta_s = ChangeThetaRange(dCosThetaS, dSinThetaS, dThetaS); // range is [0, 2*PI]
	
	dThetaE = acos((dX1 - dXc) / dR); // range is [0, PI]
	g_dXY_Theta_e = ChangeThetaRange(dCosThetaE, dSinThetaE, dThetaE); // range is [0, 2*PI]

	if (g_cmd_file_params.m_dArcDir >= 0.0) // Counterclockwise
	{
		if (g_dXY_Theta_s == g_dXY_Theta_e) // Counterclockwise Circle
		{
			g_dXY_ThetaDis = 2.0 * PI;
		}
		else
		{
			g_dXY_ThetaDis = g_dXY_Theta_e - g_dXY_Theta_s;
			if (g_dXY_ThetaDis < 0.0)
			{
				g_dXY_ThetaDis = g_dXY_ThetaDis + 2.0 * PI;
			}

		}
	}
	else // Clockwise
	{
		if (g_dXY_Theta_s == g_dXY_Theta_e) // Clockwise Circle
		{
			g_dXY_ThetaDis = 2.0 * PI;
		}
		else
		{
			g_dXY_ThetaDis = g_dXY_Theta_s - g_dXY_Theta_e;
			if (g_dXY_ThetaDis < 0.0)
			{
				g_dXY_ThetaDis = g_dXY_ThetaDis + 2.0 * PI;
			}
		}
	}

	return 1;
}

void GetArcPathParams(double dSpeed, double dAcc)
{
	double dW = dSpeed / g_cmd_file_params.m_dRadius;
	double dAlpha = dAcc / g_cmd_file_params.m_dRadius;
	
	g_dXY_Ws = g_dXY_Vs / g_cmd_file_params.m_dRadius;
	g_dXY_We = g_dXY_Ve / g_cmd_file_params.m_dRadius;
	
	g_dXY_S1 = (pow(dW, 2) - pow(g_dXY_Ws, 2)) / (2.0 * dAlpha);
	g_dXY_S3 = (pow(dW, 2) - pow(g_dXY_We, 2)) / (2.0 * dAlpha);
	g_dXY_S2 = g_dXY_ThetaDis - g_dXY_S1 - g_dXY_S3;
	if (g_dXY_S2 <= 0.0)
	{
		g_dXY_S1 = (2.0 * dAlpha * g_dXY_ThetaDis - pow(g_dXY_Ws, 2)) / (4.0 * dAlpha);
		if (g_dXY_S1 < 0.0)
		{
			g_dXY_We = sqrt(pow(g_dXY_Ws, 2) - 2.0 * dAlpha * g_dXY_ThetaDis);
			g_dXY_Ve = g_dXY_We * g_cmd_file_params.m_dRadius;
			g_dXY_Wm = g_dXY_Ws;
			g_dXY_Vm = g_dXY_Wm * g_cmd_file_params.m_dRadius;
			g_dXY_S1 = 0.0;
			g_dXY_S2 = 0.0;
			g_dXY_S3 = g_dXY_ThetaDis;
			g_dXY_T1 = 0.0;
			g_dXY_T2 = 0.0;
			g_dXY_T3 = 2.0 * g_dXY_S3 / (g_dXY_We + g_dXY_Wm);
			g_dXY_Ttotal = g_dXY_T1 + g_dXY_T2 + g_dXY_T3;
		}
		else
		{
			g_dXY_Wm = sqrt((2.0 * dAlpha * g_dXY_ThetaDis + pow(g_dXY_Ws, 2)) / 2.0);
			g_dXY_Vm = g_dXY_Wm * g_cmd_file_params.m_dRadius;
			g_dXY_S1 = (pow(g_dXY_Wm, 2) - pow(g_dXY_Ws, 2)) / (2.0 * dAlpha);
			g_dXY_S2 = 0.0;
			g_dXY_S3 = g_dXY_ThetaDis - g_dXY_S1 - g_dXY_S2;
			g_dXY_T1 = 2.0 * g_dXY_S1 / (g_dXY_Ws + g_dXY_Wm);
			g_dXY_T2 = 0.0;
			g_dXY_T3 = 2.0 * g_dXY_S3 / (g_dXY_We + g_dXY_Wm);
			g_dXY_Ttotal = g_dXY_T1 + g_dXY_T2 + g_dXY_T3;
		}
	}
	else
	{
		g_dXY_Wm = dW;
		g_dXY_Vm = dSpeed;
		g_dXY_T1 = 2.0 * g_dXY_S1 / (g_dXY_Ws + g_dXY_Wm);
		g_dXY_T2 = g_dXY_S2 / g_dXY_Wm;
		g_dXY_T3 = 2.0 * g_dXY_S3 / (g_dXY_We + g_dXY_Wm);
		g_dXY_Ttotal = g_dXY_T1 + g_dXY_T2 + g_dXY_T3;
	}
}

int GetArcPathCmdPos(double dSpeed, double dAcc)
{
	double dW = dSpeed / g_cmd_file_params.m_dRadius;
	double dAlpha = dAcc / g_cmd_file_params.m_dRadius;
	
	g_dXY_Time += g_dt;
	if (g_dXY_Time <= g_dXY_T1)
	{
		g_u32LEDout |= 0x02;

		g_dXY_V = g_dXY_Vs + dAcc * g_dXY_Time;
		g_dXY_S = g_dXY_Ws * g_dXY_Time + 0.5 * dAlpha * pow(g_dXY_Time, 2);
	}
	else if (g_dXY_Time > g_dXY_T1 && g_dXY_Time <= g_dXY_T1 + g_dXY_T2)
	{
		g_dXY_V = dSpeed;
		g_dXY_S = g_dXY_S1 + dW * (g_dXY_Time - g_dXY_T1);
	}
	else if (g_dXY_Time > g_dXY_T1 + g_dXY_T2 && g_dXY_Time <= g_dXY_Ttotal)
	{
		g_dXY_V = g_dXY_Vm - dAcc * (g_dXY_Time - g_dXY_T1 - g_dXY_T2);
		g_dXY_S = g_dXY_S1 + g_dXY_S2 + g_dXY_Wm * (g_dXY_Time - g_dXY_T1 - g_dXY_T2) - 0.5 * dAlpha * pow((g_dXY_Time - g_dXY_T1 - g_dXY_T2), 2);
	}
	else
	{
		g_dXY_V = g_dXY_Ve;
		g_Position_Params[0].m_dCmdPos = g_cmd_file_params.m_dEndPos[0];
		g_Position_Params[1].m_dCmdPos = g_cmd_file_params.m_dEndPos[1];
		return 0;
	}

	if (g_cmd_file_params.m_dArcDir >= 0.0) // Counterclockwise
	{
		g_Position_Params[0].m_dCmdPos = g_cmd_file_params.m_dCenPos[0] + g_cmd_file_params.m_dRadius * cos(g_dXY_Theta_s + g_dXY_S);
		g_Position_Params[1].m_dCmdPos = g_cmd_file_params.m_dCenPos[1] + g_cmd_file_params.m_dRadius * sin(g_dXY_Theta_s + g_dXY_S);
	}
	else // Clockwise
	{
		g_Position_Params[0].m_dCmdPos = g_cmd_file_params.m_dCenPos[0] + g_cmd_file_params.m_dRadius * cos(g_dXY_Theta_s - g_dXY_S);
		g_Position_Params[1].m_dCmdPos = g_cmd_file_params.m_dCenPos[1] + g_cmd_file_params.m_dRadius * sin(g_dXY_Theta_s - g_dXY_S);
	}

	return 1;
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
	// else if (g_CmdBuf[iIndex % 100 + 1].m_iID == ARCXY || g_CmdBuf[iIndex % 100 + 1].m_iID == FARCXY)
	// {

	// }

	g_dXY_Ve = 0.0;
	return 0;
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

void InitParams(int iAxis)
{
	g_dVel[iAxis] = 0.0;
	g_dTime[iAxis] = 0.0;
	g_dS[iAxis] = 0.0;
	g_bStopFlag[iAxis] = false;
	g_bHomingFlag[iAxis] = false;
	g_iCnt[iAxis] = 5;
	g_bRunFileFlag = false;
	g_iFileCmdIndex = 0;
	g_dXY_V = 0.0;
	g_dXY_Dis = 0.0;
	g_dXY_S = 0.0;
	g_dXY_Vm = 0.0;
	g_dXY_S1 = 0.0;
	g_dXY_S2 = 0.0;
	g_dXY_S3 = 0.0;
	g_dXY_T1 = 0.0;
	g_dXY_T2 = 0.0;
	g_dXY_T3 = 0.0;
	g_dXY_Ttotal = 0.0;
	g_dXY_Vs = 0.0;
	g_dXY_Ve = 0.0;
	g_dXY_Time = 0.0;
	g_dXY_Theta_s = 0.0;
	g_dXY_Theta_e = 0.0;
	g_dXY_ThetaDis = 0.0;
	g_dXY_Ws = 0.0;
	g_dXY_We = 0.0;
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
						GetBegParams();
						break;
					}
					case SPEED:
					{
						GetSpeedParams();
						g_iFileCmdIndex++;
						break;
					}	
					case ACC:
					{
						GetAccParams();
						g_iFileCmdIndex++;
						break;
					}
					case LINEXY:
					{
						GetLineParams();
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
						GetLineParams();
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
					case ARCXY:
					{						
						GetArcParams();
						nret = CalRadius();
						if (nret <= 0)
						{
							SetNextCmd();
							break;
						}

						nret = CalArcThetaDis();
						if (nret <= 0)
						{
							SetNextCmd();
							break;
						}
						
						// CheckTheta(g_iFileCmdIndex);
						g_dXY_Ve = 0.0;
						GetArcPathParams(g_cmd_file_params.m_dSpeed, g_cmd_file_params.m_dAcc);
						nret = GetArcPathCmdPos(g_cmd_file_params.m_dSpeed, g_cmd_file_params.m_dAcc);
						if (nret <= 0)
						{
							SetNextCmd();
						}
						break;
					}
					case FARCXY:
					{
						GetArcParams();
						nret = CalRadius();
						if (nret <= 0)
						{
							SetNextCmd();
							break;
						}

						nret = CalArcThetaDis();
						if (nret <= 0)
						{
							SetNextCmd();
							break;
						}
						
						// CheckTheta(g_iFileCmdIndex);
						g_dXY_Ve = 0.0;
						GetArcPathParams(g_cmd_file_params.m_dFSpeed, g_cmd_file_params.m_dFAcc);
						nret = GetArcPathCmdPos(g_cmd_file_params.m_dFSpeed, g_cmd_file_params.m_dFAcc);
						if (nret <= 0)
						{
							SetNextCmd();
						}
						break;
					}
					case END:
					{
						// g_u32LEDout &= 0xfffffffd;

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
						InitParams(i);

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
							GetCmdPos_Motion(i);
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
