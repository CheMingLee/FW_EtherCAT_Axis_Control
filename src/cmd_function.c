#include "setting.h"

/*************************************************************************/

bool CheckFlag()
{
	bool bFlag;
	u32 uFlag;

	uFlag = Xil_In32(IO_ADDR_BRAM_IN_FLAG);
	bFlag = false;

	if (uFlag & 0x01)
	{
		bFlag = true;
	}
	
	return bFlag;
}

void SetFlagInZero()
{
	u32 uFlag;
	u32 mask;
	int i;
	i = 0;
	mask = 1 << i;
	uFlag = Xil_In32(IO_ADDR_BRAM_IN_FLAG);
	uFlag &= ~mask;
	Xil_Out32(IO_ADDR_BRAM_IN_FLAG, uFlag);
}

void SetFlagOutOne()
{
	u32 uFlag;
	u32 mask;
	int i;
	i = 1;
	mask = 1 << i;
	uFlag = Xil_In32(IO_ADDR_BRAM_IN_FLAG);
	uFlag |= mask;
	Xil_Out32(IO_ADDR_BRAM_IN_FLAG, uFlag);
}

void CmdGetToApp(u16 usAck, u16 usSize)
{
	Xil_Out32(IO_ADDR_BRAM_OUT_SIZE, usSize+4);
	Xil_Out16(IO_ADDR_BRAM_OUT_ACK, usAck);
	Xil_Out16(IO_ADDR_BRAM_OUT_ACK_SIZE, usSize);
}

/*************************************************************************/

void GetAppCmd()
{
	if (CheckFlag())
	{
		u16 usCmd;
		u16 usSize;
		char pData[256];

		usCmd = Xil_In16(IO_ADDR_BRAM_IN_CMD);
		usSize = Xil_In16(IO_ADDR_BRAM_IN_CMD_SIZE);
		memcpy(pData, (void *)IO_ADDR_BRAM_IN_DATA, usSize);

		switch (usCmd)
		{
			case CMD_SET_DATASIZE:
			{
				u32 uSizeBuf;

				memcpy(&uSizeBuf, pData, 4);
				Xil_Out32(ECM_ADDR_DATA_BYTE, uSizeBuf);
				break;
			}
			case CMD_SET_TXDATA:
			{
				int iOffset = 0;

				do
				{
					Xil_Out8(ECM_ADDR_DATA_OUT + iOffset, *((u8 *)(pData + iOffset)));
					usSize -= 1;
					iOffset += 1;
				} while (usSize > 0);
				break;
			}
			case CMD_SET_SEND:
			{
				Xil_Out32(ECM_ADDR_SEND, 1);
				break;
			}
			case CMD_GET_BUSY:
			{
				u32 uBusyBuf;
				
				uBusyBuf = Xil_In32(ECM_ADDR_BUSY);
				CmdGetToApp(usCmd, 4);
				Xil_Out32(IO_ADDR_BRAM_OUT_DATA, uBusyBuf);
				SetFlagOutOne();
				break;
			}
			case CMD_GET_RXDATA:
			{
				u8 u8Data;
				u32 uSizeBuf;
				int iOffset = 0;
				
				memcpy(&uSizeBuf, pData, usSize);
				usSize = (u16)uSizeBuf;
				CmdGetToApp(usCmd, usSize);
				
				do
				{
					u8Data = Xil_In8(ECM_ADDR_DATA_IN + iOffset);
					Xil_Out8(IO_ADDR_BRAM_OUT_DATA + iOffset, u8Data);
					usSize -= 1;
					iOffset += 1;
				} while (usSize > 0);
				SetFlagOutOne();
				break;
			}
			case CMD_SET_PARAMS:
			{
				int iAxis;

				memcpy(&iAxis, pData, 4);
				if (g_Position_Params[iAxis].m_uMode == MODE_IDLE)
				{
					memcpy(&g_Motion_Params[iAxis], pData + 4, sizeof(MOTION_PARAMS));
				}
				break;
			}
			case CMD_SET_JOG:
			{
				int iAxis, iDirection;

				memcpy(&iAxis, pData, 4);
				memcpy(&iDirection, pData + 4, 4);
				if (g_Position_Params[iAxis].m_uMode == MODE_IDLE)
				{
					if (iDirection < 0)
					{
						g_Motion_Params[iAxis].m_dJogSpeed = -abs(g_Motion_Params[iAxis].m_dJogSpeed);
						g_Motion_Params[iAxis].m_dJogAcc = -abs(g_Motion_Params[iAxis].m_dJogAcc);
					}

					g_Position_Params[iAxis].m_uMode = MODE_JOG;
				}
				break;
			}
			case CMD_SET_MOTION:
			{
				int iAxis;
				double dTarPos;

				memcpy(&iAxis, pData, 4);
				memcpy(&dTarPos, pData + 4, 8);
				if (g_Position_Params[iAxis].m_uMode == MODE_IDLE)
				{
					g_Position_Params[iAxis].m_uMode = MODE_MOTION;
					g_Position_Params[iAxis].m_dTarPos = dTarPos * g_Motion_Params[iAxis].m_dAxisUnit;
					g_dStartPos[iAxis] = g_Position_Params[iAxis].m_dCmdPos;
					g_dDistance[iAxis] = g_Position_Params[iAxis].m_dTarPos - g_dStartPos[iAxis];
					if (g_dDistance[iAxis] == 0)
					{
						g_Position_Params[iAxis].m_uMode = MODE_IDLE;
					}
					else if (g_dDistance[iAxis] < 0)
					{
						g_Motion_Params[i].m_dMotionSpeed = -abs(g_Motion_Params[i].m_dMotionSpeed);
						g_Motion_Params[i].m_dMotionAcc = -abs(g_Motion_Params[i].m_dMotionAcc);
					}
				}
				break;
			}
			case CMD_SET_HOME:
			{
				int iAxis;

				memcpy(&iAxis, pData, 4);
				if (g_Position_Params[iAxis].m_uMode == MODE_IDLE)
				{
					g_Position_Params[iAxis].m_uMode = MODE_HOME;
					g_bInterruptFlag = false;
				}
				break;
			}
			case CMD_SET_STOP:
			{
				int iAxis;

				memcpy(&iAxis, pData, 4);
				g_bStopFlag[iAxis] = true;
				break;
			}
			case CMD_SET_JOGEND:
			{
				int iAxis;

				memcpy(&iAxis, pData, 4);
				if (g_Position_Params[iAxis].m_uMode == MODE_JOG)
				{
					g_Position_Params[iAxis].m_uMode = MODE_JOGEND;
				}
				break;
			}
			case CMD_SET_INTR:
			{
				g_bInterruptFlag = true;
				break;
			}
			case CMD_SET_CURPOS:
			{
				int iAxis, iCurPos;

				memcpy(&iAxis, pData, 4);
				memcpy(&iCurPos, pData + 4, 4);
				g_Position_Params[iAxis].m_dCmdPos = (double)iCurPos;
				g_Position_Params[iAxis].m_dCurPos = (double)iCurPos;
				break;
			}
			case CMD_SET_SERVOCNT:
			{
				int8_t iSlaveCnt;

				memcpy(&iSlaveCnt, pData, 1);
				g_iServoCnt = (int)iSlaveCnt;
			}
			case CMD_SET_INTR_DISABLE:
			{
				g_bInterruptFlag = false;
				break;
			}
			default:
			{
				break;
			}
		}
		SetFlagInZero();
	}
}

/*************************************************************************/
