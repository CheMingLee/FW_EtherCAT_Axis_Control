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
		uCmd = Xil_In16(IO_ADDR_BRAM_IN_CMD);

		switch (uCmd)
		{
			case 1:
			{
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
