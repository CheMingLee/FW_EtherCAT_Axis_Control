#include "stdlib.h"
#include "stdbool.h"
#include "Xscugic.h"
#include "Xil_exception.h"
#include "EcmUsrDriver.h"
#include "math.h"

// interrupt
#define ECM_INTR_ID		    	    XPAR_FABRIC_EC01M_0_PLINT_OUT_INTR
#define INTC_DEVICE_ID			    XPAR_SCUGIC_SINGLE_DEVICE_ID

// BRAM define
#define IO_ADDR_BRAM_IN_FLAG		(XPAR_BRAM_0_BASEADDR + 0)
#define IO_ADDR_BRAM_IN_SIZE		(XPAR_BRAM_0_BASEADDR + 4)
#define IO_ADDR_BRAM_IN_CMD         (XPAR_BRAM_0_BASEADDR + 8)
#define IO_ADDR_BRAM_IN_CMD_SIZE    (XPAR_BRAM_0_BASEADDR + 10)
#define IO_ADDR_BRAM_IN_DATA        (XPAR_BRAM_0_BASEADDR + 12)
#define IO_ADDR_BRAM_OUT_NULL       (XPAR_BRAM_0_BASEADDR + 256)
#define IO_ADDR_BRAM_OUT_SIZE       (XPAR_BRAM_0_BASEADDR + 260)
#define IO_ADDR_BRAM_OUT_ACK        (XPAR_BRAM_0_BASEADDR + 264)
#define IO_ADDR_BRAM_OUT_ACK_SIZE   (XPAR_BRAM_0_BASEADDR + 266)
#define IO_ADDR_BRAM_OUT_DATA       (XPAR_BRAM_0_BASEADDR + 268)
#define IO_ADDR_LEDOUT				(XPAR_IO_CONTROL_0_S00_AXI_BASEADDR + 68)
#define IO_ADDR_OUTPUT				(XPAR_IO_CONTROL_0_S00_AXI_BASEADDR + 0)

// CMD define
#define CMD_SET_DATASIZE 0
#define CMD_SET_TXDATA 1
#define CMD_SET_SEND 2
#define CMD_GET_BUSY 3
#define CMD_GET_RXDATA 4
#define CMD_SET_PARAMS 5
#define CMD_SET_JOG 6
#define CMD_SET_MOTION 7
#define CMD_SET_HOME 8
#define CMD_SET_STOP 9
#define CMD_SET_JOGEND 10
#define CMD_SET_INTR 11
#define CMD_SET_CURPOS 12
#define CMD_SET_SERVOCNT 13
#define CMD_GET_CURPOS 14
#define CMD_GET_SERVOMODE 15
#define CMD_GET_DIGINPUT 16
#define CMD_SET_INTR_DISABLE 17
#define CMD_GET_CMDPOS 18
#define CMD_SET_RUNFILE 19
#define CMD_GET_RUNFILE_BEGINPOS_FLAG 20
#define CMD_SET_RUNFILE_CMDCNT 21
#define CMD_SET_RUNFILE_CMD 22

// mode define
#define MODE_IDLE 0
#define MODE_JOG 1
#define MODE_MOTION 2
#define MODE_HOME 3
#define MODE_JOGEND 4
#define MODE_RUNFILE 5

// digital input define
#define DIGINPUT_LIMIT_LEFT 0x01
#define DIGINPUT_LIMIT_RIGHT 0x02
#define DIGINPUT_HMOE 0x04

// Cmd file define
#define BEGIN 0
#define SPEED 1
#define ACC 2
#define LINEXY 3
#define FLINEXY 4
#define ARCXY 5
#define FARCXY 6
#define END 7

// File cmd structure
typedef struct file_cmd{
	int m_iID;
	double m_dParams[5];
}FILE_CMD;

// Run File params structure
typedef struct cmd_file_params{
	double m_dSpeed;
	double m_dFSpeed;
	double m_dAcc;
	double m_dFAcc;
} CMD_FILE_PARAMS;

// Params structure
typedef struct motion_params{
	double m_dJogSpeed;
	double m_dMotionSpeed;
	double m_dComeHomeSpeed;
	double m_dLeftHomeSpeed;
	double m_dJogAcc;
	double m_dMotionAcc;
	double m_dHomeAcc;
	double m_dAxisUnit;
} MOTION_PARAMS;

typedef struct position_params{
	u32 m_uMode;
	double m_dTarPos; // pulse
	double m_dCmdPos; // pulse
	int m_iCurPos; // pulse
	u32 m_uInput;
} POSITION_PARAMS;

extern uint8_t g_u8TxBuf[PKG_MAX_SIZE];
extern uint8_t g_u8RxBuf[PKG_MAX_SIZE];
extern uint8_t g_u8CmdIdx;
extern MOTION_PARAMS g_Motion_Params[TEST_SERVO_CNT];
extern POSITION_PARAMS g_Position_Params[TEST_SERVO_CNT];
extern bool g_bStopFlag[TEST_SERVO_CNT];
extern bool g_bHomingFlag[TEST_SERVO_CNT];
extern int g_iCnt[TEST_SERVO_CNT];
extern double g_dDistance[TEST_SERVO_CNT];
extern double g_dStartPos[TEST_SERVO_CNT];
extern double g_dDirection[TEST_SERVO_CNT];
extern double g_dVel[TEST_SERVO_CNT];
extern double g_dTime[TEST_SERVO_CNT];
extern double g_dVm[TEST_SERVO_CNT];
extern double g_dS[TEST_SERVO_CNT];
extern double g_dS1[TEST_SERVO_CNT];
extern double g_dS2[TEST_SERVO_CNT];
extern double g_dS3[TEST_SERVO_CNT];
extern double g_dT1[TEST_SERVO_CNT];
extern double g_dT2[TEST_SERVO_CNT];
extern double g_dT3[TEST_SERVO_CNT];
extern double g_dTtotal[TEST_SERVO_CNT];
extern double g_dt;
extern bool g_bInterruptFlag;
extern int g_iServoCnt;
extern uint16_t g_u16PDOSize;
extern uint16_t g_u16PDOSizeRet;
extern uint8_t g_RxData[TEST_SPI_DATA_SIZE];
extern uint8_t g_TxData[TEST_SPI_DATA_SIZE];
extern XScuGic g_Intc;
extern RXPDO_ST_DEF_T *g_pRxPDOData;
extern TXPDO_ST_DEF_T *g_pTxPDOData;

extern FILE_CMD g_CmdBuf[10];
extern int g_iCmdBufCnt;
extern bool g_bRunFileFlag;
extern int g_iFileCmdCnt;
extern CMD_FILE_PARAMS g_cmd_file_params;
extern bool g_bBeginPosFlag[2];

extern u32 g_u32LEDout;
extern u16 g_u16JF8out;
