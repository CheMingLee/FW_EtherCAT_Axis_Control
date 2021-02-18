#include "setting.h"

extern MOTION_PARAMS g_Motion_Params[2];

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
		u16 uCmd;
		u16 usSize;
		char pData[256];

		uCmd = Xil_In16(IO_ADDR_BRAM_IN_CMD);
		usSize = Xil_In16(IO_ADDR_BRAM_IN_CMD_SIZE);
		memcpy(pData, (void *)IO_ADDR_BRAM_IN_DATA, usSize);

		switch (uCmd)
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
				// memcpy((void *)ECM_ADDR_DATA_OUT, pData, usSize);
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
				CmdGetToApp(uCmd, 4);
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
				CmdGetToApp(uCmd, usSize);
				
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
			default:
			{
				break;
			}
		}
		SetFlagInZero();
	}
}

/*************************************************************************/
