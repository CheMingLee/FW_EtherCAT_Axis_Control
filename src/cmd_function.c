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
				memcpy((void *)ECM_ADDR_DATA_BYTE, pData, usSize);
				break;
			}
			case CMD_SET_TXDATA:
			{
				memcpy((void *)ECM_ADDR_DATA_OUT, pData, usSize);
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
				memcpy(pData, &uBusyBuf, sizeof(uBusyBuf));
				CmdGetToApp(uCmd, pData, usSize);
				SetFlagOutOne();
				break;
			}
			case CMD_GET_RXDATA:
			{
				memcpy(pData, (void *)ECM_ADDR_DATA_IN, usSize);
				CmdGetToApp(uCmd, pData, usSize);
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

void CmdGetToApp(u16 usAck, char *pData, u16 usSize)
{
	Xil_Out32(IO_ADDR_BRAM_OUT_SIZE, usSize+4);
	Xil_Out16(IO_ADDR_BRAM_OUT_ACK, usAck);
	Xil_Out16(IO_ADDR_BRAM_OUT_ACK_SIZE, usSize);
	memcpy((void *)IO_ADDR_BRAM_OUT_DATA, pData, usSize);
}

/*************************************************************************/
