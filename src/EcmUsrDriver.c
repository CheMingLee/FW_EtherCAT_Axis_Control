/******************************************************************************
 *	File	:	EcmUsrDriver.c
 *	Version :	0.1
 *	Date	:	2020/04/24
 *	Author	:	XFORCE
 *
 *	ECM-XF basic driver example - Source file
 *
 *	Demonstrate how to implement API type user driver
 *
 * @copyright (C) 2020 NEXTW TECHNOLOGY CO., LTD.. All rights reserved.
 *
 ******************************************************************************/

#include "EcmUsrDriver.h"
#include "sleep.h"

extern uint8_t g_u8TxBuf[PKG_MAX_SIZE];
extern uint8_t g_u8RxBuf[PKG_MAX_SIZE];

SPI_CMD_PACKAGE_T *pCmd=(SPI_CMD_PACKAGE_T *)g_u8TxBuf;
SPI_RET_PACKAGE_T *pRet=(SPI_RET_PACKAGE_T *)g_u8RxBuf;

extern uint8_t g_u8CmdIdx;

void PCC6SpiDataGet(uint8_t *pRxBuf, uint32_t u32TotalPackSize)
{
	if (u32TotalPackSize >= PKG_MIN_SIZE && u32TotalPackSize <= PKG_MAX_SIZE)
	{
		int iOffset;
		int iTotalPackSize;
		u32 uBusyFlag;
		u32 u32Data;
		
		do
		{
			uBusyFlag = Xil_In32(ECM_ADDR_BUSY);
		} while (uBusyFlag);

		iOffset = 0;
		iTotalPackSize = (int)u32TotalPackSize;
		do
		{
			u32Data = Xil_In32(ECM_ADDR_DATA_IN + iOffset);
			memcpy(pRxBuf + iOffset, &u32Data, 4);
			iTotalPackSize -= 4;
			iOffset += 4;
		} while (iTotalPackSize > 0);
	}
}

int SpiDataGet(uint8_t *RetIdx, uint8_t *RetCmd)
{
	PCC6SpiDataGet(g_u8RxBuf, sizeof(SPI_RET_PACKAGE_T));
	
	if(pRet->Crc == ECM_CRC_MAGIC_NUM){
		if(RetIdx)
			*RetIdx = pRet->Head.u8Idx;
		if(RetCmd)
			*RetCmd = pRet->Head.u8Cmd;
		return 1;
	}
	// printf("CRC Error(%d)\n",pCmd->Head.u8Idx);
	return 0;
}

int ECM_EcatPdoFifoDataGet(uint8_t *pTxData, uint16_t u16DataSize)
{
	if(SpiDataGet(0,0) == 0)
	{
		//CRC error
		return -1;
	}

	if(pRet->Head.u8Return & ECM_FIFO_RD)
	{
		if(pRet->Head.u16Size == u16DataSize)
		{
			memcpy(pTxData, pRet->Data, pRet->Head.u16Size);
			return 1;
		}
		else
		{
			return -2;
		}
	}
	
	// printf("TxPDO FIFO empty\n");
	return 0;
}

void PCC6SpiDataSend(uint8_t *pTxBuf, uint32_t u32TotalPackSize)
{
	if (u32TotalPackSize >= PKG_MIN_SIZE && u32TotalPackSize <= PKG_MAX_SIZE)
	{
		int iOffset;
		int iTotalPackSize;
		
		Xil_Out32(ECM_ADDR_DATA_BYTE, u32TotalPackSize);
		
		iOffset = 0;
		iTotalPackSize = (int)u32TotalPackSize;
		do
		{
			Xil_Out32(ECM_ADDR_DATA_OUT + iOffset, *((u32 *)(pTxBuf + iOffset)));
			iTotalPackSize -= 4;
			iOffset += 4;
		} while (iTotalPackSize > 0);

		Xil_Out32(ECM_ADDR_SEND, 1);
	}
}

void SpiDataSend()
{
	pCmd->Head.u32StartWord = ECM_START_WORD;
	pCmd->Crc = ECM_CRC_MAGIC_NUM;
	pCmd->StopWord = ECM_STOP_WORD;
	PCC6SpiDataSend(g_u8TxBuf, sizeof(SPI_CMD_PACKAGE_T));
}

void ECM_EcatPdoFifoDataSend(uint8_t *pRxData, uint16_t u16DataSize)
{
	pCmd->Head.u8Cmd = ECM_CMD_ECAT_PDO_DATA_FIFO_OP;
	pCmd->Head.u16Size = u16DataSize;
	pCmd->Head.u8Param = 1;
	pCmd->Head.u8Data[0] = (ECM_FIFO_WR | ECM_FIFO_RD);
	pCmd->Head.u8Idx = g_u8CmdIdx++;
	pCmd->Head.u32CompIntClr = 0x80000000;
	memcpy(pCmd->Data, pRxData, pCmd->Head.u16Size);
	SpiDataSend();
}

void PCC6SpiDataExchange(uint8_t *pTxBuf, uint8_t *pRxBuf, uint32_t u32TotalPackSize)
{
	if (u32TotalPackSize >= PKG_MIN_SIZE && u32TotalPackSize <= PKG_MAX_SIZE)
	{
		int iOffset;
		int iTotalPackSize;
		u32 uBusyFlag;
		u8 u8Data;
		
		Xil_Out32(ECM_ADDR_DATA_BYTE, u32TotalPackSize);
		
		iOffset = 0;
		iTotalPackSize = (int)u32TotalPackSize;
		do
		{
			Xil_Out8(ECM_ADDR_DATA_OUT + iOffset, *(pTxBuf + iOffset));
			iTotalPackSize -= 1;
			iOffset += 1;
		} while (iTotalPackSize > 0);

		Xil_Out32(ECM_ADDR_SEND, 1);

		do
		{
			uBusyFlag = Xil_In32(ECM_ADDR_BUSY);
		} while (uBusyFlag);

		iOffset = 0;
		iTotalPackSize = (int)u32TotalPackSize;
		do
		{
			u8Data = Xil_In8(ECM_ADDR_DATA_IN + iOffset);
			memcpy(pRxBuf + iOffset, &u8Data, 1);
			iTotalPackSize -= 1;
			iOffset += 1;
		} while (iTotalPackSize > 0);
	}
}

int SpiDataExchange(uint8_t *RetIdx, uint8_t *RetCmd)
{
	pCmd->Head.u32StartWord = ECM_START_WORD;
	pCmd->Crc = ECM_CRC_MAGIC_NUM;
	pCmd->StopWord = ECM_STOP_WORD;
	PCC6SpiDataExchange(g_u8TxBuf, g_u8RxBuf, sizeof(SPI_CMD_PACKAGE_T));
	
	if(pRet->Crc == ECM_CRC_MAGIC_NUM){
		if(RetIdx)
			*RetIdx = pRet->Head.u8Idx;
		if(RetCmd)
			*RetCmd = pRet->Head.u8Cmd;
		return 1;
	}
	// printf("CRC Error(%d)\n",pCmd->Head.u8Idx);
	return 0;
}
int ECM_GetFirmwareVersion(uint8_t *pVersion)
{
	int i=0;
	uint8_t IdxCheck;
	pCmd->Head.u8Cmd = ECM_CMD_FW_VERSION_GET;
	pCmd->Head.u16Size = 0;
	pCmd->Head.u8Idx = g_u8CmdIdx++;
	pCmd->Head.u8Ctrl = 0;
	for(i=0;i<100;i++){
		if(SpiDataExchange(&IdxCheck, 0)){
			if(pCmd->Head.u8Idx == IdxCheck){
				*pVersion = pRet->Head.u8Return;
				return 1;
			}
		}
	}
	return 0;
}
int ECM_InfoUpdate(uint8_t *pEcmStatus, uint8_t *pRxPDOFifoCnt, uint8_t *CrcErrCnt, uint8_t *WkcErrCnt)
{
	pCmd->Head.u8Cmd = ECM_CMD_INFO_UPDATE_OP;
	pCmd->Head.u16Size = 0;
	pCmd->Head.u8Idx = g_u8CmdIdx++;
	pCmd->Head.u8Ctrl = 0;
	if(SpiDataExchange(0,0)){
		if(pEcmStatus)
			*pEcmStatus = pRet->Head.u8Status;
		if(pRxPDOFifoCnt)
			*pRxPDOFifoCnt  = pRet->Head.u8RxFifoCnt;
		if(CrcErrCnt)
			*CrcErrCnt = pRet->Head.u8CrcErrCnt;
		if(WkcErrCnt)
			*WkcErrCnt = pRet->Head.u8WkcErrCnt;
		return 1;
	}
	return 0;
}
int ECM_IsAsyncBusy()
{
	uint8_t u8RetStatus=0;
	if(ECM_InfoUpdate(&u8RetStatus,0,0,0)){
		if(u8RetStatus & ECM_STA_ASYNC_OP_BUSY_MASK){
			return 1;
		}
	}else{
		return 1;
	}
	return 0;
}
int ECM_GetRetStatus(uint8_t *pStatus)
{
	*pStatus = pRet->Head.u8Status;
	return 1;
}
int ECM_GetRetErrStatus(uint8_t *pErrStatus)
{
	*pErrStatus = pRet->Head.u8ErrorStatus;
	return 1;
}
int ECM_WaitAsyncDone(int nMS)
{
	int i=0;
	for(i=0;i<nMS;i++){
		if(ECM_IsAsyncBusy()){
			usleep(1000); //1ms
		}else{
			return 1;
		}
	}
	// printf("Wait done timeout\n");
	return 0;
}
int ECM_EcatInit(uint8_t DCActCode, uint32_t CycTime, int32_t CycShift)
{
	int i=0;
	uint8_t IdxCheck;
	// uint8_t EcmStatus;
	EC_DCSYNC_H *pDcSyncCmd = (EC_DCSYNC_H *)pCmd->Data;
	pDcSyncCmd->Slave = ECM_INDEX;
	pDcSyncCmd->Mode = 0;
	pDcSyncCmd->Act = DCActCode;
	pDcSyncCmd->CyclTime0 = CycTime;
	pDcSyncCmd->CyclShift = CycShift;
	pCmd->Head.u8Cmd = ECM_CMD_ECAT_INIT_OP;
	pCmd->Head.u16Size = sizeof(EC_DCSYNC_H);
	pCmd->Head.u8Idx = g_u8CmdIdx++;
	for(i=0;i<100;i++){
		if(SpiDataExchange(&IdxCheck, 0)){
			if(pCmd->Head.u8Idx == IdxCheck){
				break;
			}
		}
	}
	if(i>=100){
		// printf("Timeout\n");
		return 0;
	}
	return ECM_WaitAsyncDone(1000);
}
int ECM_EcatReconfig()
{
	int i=0;
	uint8_t IdxCheck;
	// uint8_t EcmStatus;
	pCmd->Head.u8Cmd = ECM_CMD_ECAT_RECONFIG_OP;
	pCmd->Head.u16Size = 0;
	pCmd->Head.u8Idx = g_u8CmdIdx++;
	for(i=0;i<100;i++){
		if(SpiDataExchange(&IdxCheck, 0)){
			if(pCmd->Head.u8Idx == IdxCheck){
				break;
			}
		}
	}
	if(i>=100){
		// printf("Timeout\n");
		return 0;
	}
	return ECM_WaitAsyncDone(1000);
}

int8_t ECM_EcatSlvCntGet()
{
	int i=0;
	uint8_t IdxCheck;
	pCmd->Head.u8Cmd = ECM_CMD_ECAT_SLV_CNT_GET;
	pCmd->Head.u16Size = 0;
	pCmd->Head.u8Idx = g_u8CmdIdx++;
	for(i=0;i<100;i++){
		if(SpiDataExchange(&IdxCheck, 0)){
			if(pCmd->Head.u8Idx == IdxCheck){
				return pRet->Head.u8Return;
			}
		}
	}
	return 0;
}

int ECM_EcatStateSet(uint8_t u8Slave, uint8_t u8State)
{
	if(!ECM_WaitAsyncDone(2000))
		return 0;
	pCmd->Head.u8Cmd = ECM_CMD_ECAT_STATE_SET;
	pCmd->Head.u16Size = 0;
	pCmd->Head.u8Idx = g_u8CmdIdx++;
	pCmd->Head.u8Param = u8Slave;
	pCmd->Head.u8Data[0] = u8State;
	return SpiDataExchange(0,0);
}
int ECM_EcatStateGet(uint8_t u8Slave, uint8_t *pu8State)
{
	int i=0;
	uint8_t IdxCheck=0;
	if(!ECM_WaitAsyncDone(2000))
			return 0;
	pCmd->Head.u8Cmd = ECM_CMD_ECAT_STATE_GET;
	pCmd->Head.u16Size = 0;
	pCmd->Head.u8Idx = g_u8CmdIdx++;
	pCmd->Head.u8Param = u8Slave;
	for(i=0;i<100;i++){
		if(SpiDataExchange(&IdxCheck, 0)){
			if(pCmd->Head.u8Idx == IdxCheck){
				*pu8State = pRet->Head.u8Return;
				return 1;
			}
		}
	}
	return 0;
}
int ECM_EcatStateCheck(uint8_t u8Slave, uint8_t u8State)
{
	int i=0;
	uint8_t IdxCheck=0;
	if(!ECM_WaitAsyncDone(2000))
			return 0;
	pCmd->Head.u8Cmd = ECM_CMD_ECAT_STATE_CHECK;
	pCmd->Head.u16Size = 0;
	pCmd->Head.u8Idx = g_u8CmdIdx++;
	pCmd->Head.u8Param = u8Slave;
	pCmd->Head.u8Data[0] = u8State;
	for(i=0;i<100;i++){
		if(SpiDataExchange(&IdxCheck, 0)){
			if(pCmd->Head.u8Idx == IdxCheck){
				return 1;
			}
		}
	}
	return 0;
}
int ECM_StateCheck(uint8_t u8Slave, uint8_t u8ExpectState, int TimeOutMS)
{
	uint8_t u8State;
	int i=0;
	if(ECM_EcatStateSet(u8Slave, u8ExpectState)){
		for(i=0;i<TimeOutMS;i++){
			usleep(1000);
			if(ECM_EcatStateCheck(u8Slave, u8ExpectState)){
                int ret = ECM_EcatStateGet(u8Slave, &u8State);
				if(ret > 0 && u8State == u8ExpectState){
					return 1;
				}
			}
		}
	}
	return 0;
}
int ECM_EcatPdoConfigSet( uint8_t Slave, PDO_CONFIG_HEAD *pConfigData)
{
	if(!ECM_WaitAsyncDone(1000))
			return 0;
	pCmd->Head.u8Cmd = ECM_CMD_ECAT_PDO_CONFIG_SET;
	pCmd->Head.u16Size = sizeof(PDO_CONFIG_HEAD);
	pCmd->Head.u8Idx = g_u8CmdIdx++;
	pConfigData->Slave = Slave;
	memcpy(pCmd->Data, pConfigData, sizeof(PDO_CONFIG_HEAD));
	if(SpiDataExchange(0,0)){
		if(ECM_WaitAsyncDone(1000))
			return 1;
	}
	return 0;
}
int ECM_EcatPdoConfigReq(uint8_t Slave, uint16_t SmaIdx)
{
    PDO_CONFIG_HEAD *pTxCmd;
	if(!ECM_WaitAsyncDone(3000))
			return 0;
	pTxCmd = (PDO_CONFIG_HEAD *)pCmd->Data;
	pCmd->Head.u8Cmd = ECM_CMD_ECAT_PDO_CONFIG_REQ;
	pCmd->Head.u16Size = sizeof(PDO_CONFIG_HEAD);
	pCmd->Head.u8Idx = g_u8CmdIdx++;
	pTxCmd->Slave = Slave;
	pTxCmd->SmaIdx = SmaIdx;
	return SpiDataExchange(0,0);
}
int ECM_EcatPdoConfigGet(PDO_CONFIG_HEAD *pBuf)
{
	int i;
	uint8_t IdxCheck;
	if(!ECM_WaitAsyncDone(3000))
			return 0;
	pCmd->Head.u8Cmd = ECM_CMD_ECAT_PDO_CONFIG_GET;
	pCmd->Head.u16Size = sizeof(PDO_CONFIG_HEAD);
	pCmd->Head.u8Idx = g_u8CmdIdx++;
	for(i=0;i<100;i++){
		if(SpiDataExchange(&IdxCheck, 0)){
			if(pCmd->Head.u8Idx == IdxCheck){
				memcpy(pBuf, pRet->Data, sizeof(PDO_CONFIG_HEAD));
				return 1;
			}
		}
	}
	return 0;
}

int ECM_EcatSdoReq(uint8_t OP, \
		uint8_t Slave, \
		uint16_t Index, \
		uint8_t SubIndex, \
		uint16_t size, \
		int Timeout, \
		uint8_t *Data)
{

	SDO_CMD_HEAD *pSdoCmd = (SDO_CMD_HEAD *)pCmd->Data;
	if(!ECM_WaitAsyncDone(1000))
			return 0;
	pCmd->Head.u8Cmd = ECM_CMD_ECAT_SDO_REQ;
	pCmd->Head.u8Idx = g_u8CmdIdx++;
	pCmd->Head.u8Ctrl = 0;
	pSdoCmd->OP = OP;
	pSdoCmd->Slave = Slave;
	pSdoCmd->Index = Index;
	pSdoCmd->SubIndex = SubIndex;
	pSdoCmd->size = size;
	pSdoCmd->Timeout = Timeout;
	if(OP == ECM_SDO_OP_WR){
		pCmd->Head.u16Size = 12+size;
		memcpy(pSdoCmd->Data,Data,size);
	}else{
		pCmd->Head.u16Size = 12;
	}
	return SpiDataExchange(0,0);
}
int16_t ECM_EcatSdoGet(uint8_t *pBuf)
{
	int i;
	uint8_t IdxCheck;
	if(!ECM_WaitAsyncDone(1000))
			return 0;
	pCmd->Head.u8Cmd = ECM_CMD_ECAT_SDO_GET;
	pCmd->Head.u16Size = 0;
	pCmd->Head.u8Idx = g_u8CmdIdx++;
	pCmd->Head.u8Ctrl = 0;
	for(i=0;i<100;i++){
		if(SpiDataExchange(&IdxCheck, 0)){
			if(pCmd->Head.u8Idx == IdxCheck){
				memcpy(pBuf, pRet->Data, pRet->Head.u16Size);
				return pRet->Head.u16Size;
			}
		}
	}
	return 0;
}
int ECM_Drv402SM_Enable(uint8_t SlvIdx)
{
	pCmd->Head.u8Cmd = ECM_CMD_402_CTL_SET;
	pCmd->Head.u16Size = 0;
	pCmd->Head.u8Param = SlvIdx;
	pCmd->Head.u8Data[0] = (CIA402_FSM_CTL_ENABLE_MASK | CIA402_FSM_CTL_FAULT_RST_MASK);
	pCmd->Head.u8Idx = g_u8CmdIdx++;
	return SpiDataExchange(0,0);
}
int ECM_Drv402SM_StateSet(uint8_t SlvIdx, uint8_t State)
{
	pCmd->Head.u8Cmd = ECM_CMD_402_STATE_SET;
	pCmd->Head.u16Size = 0;
	pCmd->Head.u8Param = SlvIdx;
	pCmd->Head.u8Data[0] = State;
	pCmd->Head.u8Idx = g_u8CmdIdx++;
	return SpiDataExchange(0,0);
}
int ECM_Drv402SM_StateGet(uint8_t SlvIdx, uint8_t *pState)
{
	int i;
	uint8_t IdxCheck;
	pCmd->Head.u8Cmd = ECM_CMD_402_STATE_GET;
	pCmd->Head.u16Size = 0;
	pCmd->Head.u8Param = SlvIdx;
	pCmd->Head.u8Idx = g_u8CmdIdx++;
	for(i=0;i<100;i++){
		if(SpiDataExchange(&IdxCheck, 0)){
			if(pCmd->Head.u8Idx == IdxCheck){
				*pState = pRet->Head.u8Return;
				return 1;
			}
		}
	}
	return 0;
}
int ECM_Drv402SM_StateCheck(uint8_t SlvIdx, uint8_t ExceptState, int TimeOutMS)
{
	int i;
	uint8_t State;
	i=ECM_Drv402SM_StateSet(SlvIdx, ExceptState);
	if(i==0){
		return 0;
	}
	for(i=0;i<TimeOutMS;i++){
		usleep(1000);
		if(ECM_Drv402SM_StateGet(SlvIdx, &State)){
			if((State & CIA402_SW_STATE_MASK) == ExceptState){
				return 1;
			}
		}
	}
	// printf("(%d) 0x%X 0x%X\n",SlvIdx,State,ExceptState);
	return 0;
}
uint16_t ECM_FifoRxPdoSizeGet()
{
	int i;
	uint8_t IdxCheck;
	pCmd->Head.u8Cmd = ECM_CMD_FIFO_PACK_SIZE_GET;
	pCmd->Head.u16Size = 0;
	pCmd->Head.u8Idx = g_u8CmdIdx++;
	pCmd->Head.u8Param = 0;	//	0:	RX
							//	1:	TX
	for(i=0;i<10;i++){
		if(SpiDataExchange(&IdxCheck, 0)){
			if(pCmd->Head.u8Idx == IdxCheck){
				return *((uint16_t *)pRet->Data);
			}
		}
	}
	return 0;
}
uint16_t ECM_FifoTxPdoSizeGet()
{
	int i;
	uint8_t IdxCheck;
	pCmd->Head.u8Cmd = ECM_CMD_FIFO_PACK_SIZE_GET;
	pCmd->Head.u16Size = 0;
	pCmd->Head.u8Idx = g_u8CmdIdx++;
	pCmd->Head.u8Param = 1;	//	0:	RX
							//	1:	TX
	for(i=0;i<10;i++){
		if(SpiDataExchange(&IdxCheck, 0)){
			if(pCmd->Head.u8Idx == IdxCheck){
				return *((uint16_t *)pRet->Data);
			}
		}
	}
	return 0;
}
uint8_t ECM_EcatPdoDataExchange(uint8_t u8OP, uint8_t *pRxData, uint8_t *pTxData, uint16_t *pu16DataSize)
{
	pCmd->Head.u8Cmd = ECM_CMD_ECAT_PDO_DATA_OP;
	pCmd->Head.u16Size = *pu16DataSize;
	pCmd->Head.u8Idx = g_u8CmdIdx++;
	if(u8OP){
		pCmd->Head.u8Data[0] = u8OP;
	}else{
		pCmd->Head.u8Data[0] = 0;	//Read and Write
	}
	if(u8OP & ECM_PDO_WR_OP){
		memcpy(pCmd->Data, pRxData, pCmd->Head.u16Size);
	}
	if(SpiDataExchange(0,0) == 0){
		return 0;
	}
	if(pRet->Head.u8Cmd == ECM_CMD_ECAT_PDO_DATA_OP){
		if(pRet->Head.u8Return & ECM_PDO_RD_OP){
			memcpy(pTxData, pRet->Data, pRet->Head.u16Size);
			*pu16DataSize = pRet->Head.u16Size;
		}
		return pRet->Head.u8Return;
	}
	return ECM_PDO_WR_OP;
}
int ECM_EcatPdoFifoIsFull(uint8_t u8FifoThreshold)
{
	// Notice : FIFO count update has two times delay
	if(pRet->Head.u8RxFifoCnt >= u8FifoThreshold-2){
		return 1;// FIFO count threshold reached
	}else{
		return 0;
	}
}
int ECM_EcatPdoFifoDataExchange(uint8_t u8FifoThreshold, uint8_t *pRxData, uint8_t *pTxData, uint16_t u16DataSize, uint8_t *pu8RxPdoFifoCnt, uint8_t *CrcErrCnt, uint8_t *WkcErrCnt)
{
	// Notice : FIFO count update has two times delay
	if(pRet->Head.u8RxFifoCnt >= u8FifoThreshold-2){
		return -2;// FIFO count threshold reached
	}
	pCmd->Head.u8Cmd = ECM_CMD_ECAT_PDO_DATA_FIFO_OP;
	pCmd->Head.u16Size = u16DataSize;
	pCmd->Head.u8Param = 1;
	pCmd->Head.u8Data[0] = (ECM_FIFO_WR | ECM_FIFO_RD);
	pCmd->Head.u8Idx = g_u8CmdIdx++;
	memcpy(pCmd->Data, pRxData, pCmd->Head.u16Size);
	if(SpiDataExchange(0,0) == 0){
		// printf("CRC error\n");
		return -1;//CRC error
	}
	if(pu8RxPdoFifoCnt)
		*pu8RxPdoFifoCnt=pRet->Head.u8RxFifoCnt;
	if(CrcErrCnt)
		*CrcErrCnt = pRet->Head.u8CrcErrCnt;
	if(WkcErrCnt)
		*WkcErrCnt = pRet->Head.u8WkcErrCnt;
	if(pRet->Head.u8Cmd == ECM_CMD_ECAT_PDO_DATA_FIFO_OP){
		if(pRet->Head.u8Return & ECM_FIFO_RD){
			if(pRet->Head.u16Size){
				memcpy(pTxData, pRet->Data, pRet->Head.u16Size);
			}else{
				// printf("zero size\n");
				return -4;
			}
			return pRet->Head.u16Size;
		}else{
			// printf("TxPDO FIFO empty\n");
			return 0;
		}
	}else{
		return -3;
	}
	return -6;
}
int ECM_EcatEepromReq(
		uint16_t OP, \
		uint16_t slave, \
		uint16_t eeproma, \
		uint16_t data, \
		uint32_t timeout)
{
	// uint8_t u8RetIdx=0;
	ECM_EEPROM_REQ_T *pEepromReq = (ECM_EEPROM_REQ_T *)pCmd->Data;
	if(!ECM_WaitAsyncDone(1000))
			return 0;
	pCmd->Head.u8Cmd = ECM_EEPROM_REQ;
	pCmd->Head.u8Idx = g_u8CmdIdx++;
	pEepromReq->OP = OP;
	pEepromReq->slave = slave;
	pEepromReq->eeproma = eeproma;
	pEepromReq->data = data;
	pEepromReq->timeout = timeout;
	pCmd->Head.u16Size = sizeof(ECM_EEPROM_REQ_T);
	return SpiDataExchange(0,0);
}
int ECM_EcatEepromGet(uint64_t *pu64Data)
{
	int i;
	uint8_t IdxCheck;
	if(!ECM_WaitAsyncDone(1000))
			return 0;
	pCmd->Head.u8Cmd = ECM_EEPROM_GET;
	pCmd->Head.u16Size = 0;
	pCmd->Head.u8Idx = g_u8CmdIdx++;
	for(i=0;i<100;i++){
		if(SpiDataExchange(&IdxCheck, 0)){
			if(pCmd->Head.u8Idx == IdxCheck){
				memcpy(pu64Data,pRet->Data,pRet->Head.u16Size);
				return 1;
			}
		}
	}
	return 0;
}
int ECM_ShowPDOConfig(int Slave, int SmaIdx)
{
	int i=0,j=0;
	PDO_CONFIG_HEAD PdoConfigBuf;
	int nret = ECM_EcatPdoConfigReq(Slave, SmaIdx);
	if(nret <= 0){
		return 0;
	}
	nret = ECM_EcatPdoConfigGet(&PdoConfigBuf);
	if(nret <= 0){
		return 0;
	}
	// printf("(%d) 0x%X : \n",Slave, SmaIdx);
	for(i=0;i<PdoConfigBuf.PDOCnt;i++){
		// printf("PDO%d - MapIdx(0x%X)\n", i, PdoConfigBuf.MapIdx[i]);
		for(j=0; j<PdoConfigBuf.ObjsCnt[i]; j++){
			// printf("\t0x%X\n", PdoConfigBuf.Table[i][j]);
		}
	}
	return 1;
}

int ECM_EcatDatagramReq(
        ecm_datagram_commad_t cmd,
		uint16_t position,
		uint16_t offset,
        uint32_t logicalAddress,
		uint16_t length,
		int Timeout,
		uint8_t *Data)
{
    EC_RAW_API_PARAM_H *pDatagramCmd = (EC_RAW_API_PARAM_H *)pCmd->Data;
	if(!ECM_WaitAsyncDone(1000))
		return 0;
	pCmd->Head.u8Cmd = ECM_CMD_RAW_ECAT_FUNC_REQ;
	pCmd->Head.u8Idx = g_u8CmdIdx++;
    pCmd->Head.u16Size = sizeof(EC_RAW_API_PARAM_H) - 256 + length;
	pDatagramCmd->OP = cmd;
	pDatagramCmd->ADO = offset;
	pDatagramCmd->ADP = position;
	pDatagramCmd->LogAdr = logicalAddress;
	pDatagramCmd->length = length;
	pDatagramCmd->timeout = Timeout;
    memcpy(pDatagramCmd->data,Data,length);

	if(SpiDataExchange(0,0)){
		if(ECM_WaitAsyncDone(1000))
			return 1;
	}
	return 0;
}

int ECM_EcatDatagramGet(uint8_t *pBuf)
{
	int i;
	uint8_t IdxCheck;
	if(!ECM_WaitAsyncDone(1000))
		return 0;
	pCmd->Head.u8Cmd = ECM_CMD_RAW_ECAT_FUNC_GET;
	pCmd->Head.u16Size = 0;
	pCmd->Head.u8Idx = g_u8CmdIdx++;
	for(i=0;i<100;i++){
		if(SpiDataExchange(&IdxCheck, 0)){
			if(pCmd->Head.u8Idx == IdxCheck){
				memcpy(pBuf, pRet->Data, pRet->Head.u16Size);
				return pRet->Head.u16Size;
			}
		}
	}
	return ECM_WaitAsyncDone(1000);
}

int ECM_SlaveInfoGet(uint8_t slave, uint8_t info, uint8_t *pBuf)
{
	int i;
	uint8_t IdxCheck;
	if(!ECM_WaitAsyncDone(1000))
		return 0;
	pCmd->Head.u8Cmd = ECM_CMD_ECAT_SLV_INFO_GET;
    pCmd->Head.u8Param = slave;
	pCmd->Head.u16Size = 0;
    pCmd->Head.u8Data[0] = info;
	pCmd->Head.u8Idx = g_u8CmdIdx++;
	for(i=0;i<100;i++){
		if(SpiDataExchange(&IdxCheck, 0)){
			if(pCmd->Head.u8Idx == IdxCheck){
				memcpy(pBuf, pRet->Data, pRet->Head.u16Size);
				return pRet->Head.u16Size;
			}
		}
	}
	return ECM_WaitAsyncDone(1000);
}
//	(C) COPYRIGHT 2020 NEXTW Technology Corp.
