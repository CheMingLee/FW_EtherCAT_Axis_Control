#include "stdlib.h"
#include "stdbool.h"
#include "Xscugic.h"
#include "Xil_exception.h"
#include "EcmUsrDriver.h"

// interrupt
#define ECM_INTR_ID		    	    XPAR_FABRIC_EC01M_0_PLINT_OUT_INTR
#define INTC_DEVICE_ID			    XPAR_SCUGIC_SINGLE_DEVICE_ID

// // Write
// #define ECM_ADDR_SEND				(XPAR_EC01M_0_S00_AXI_BASEADDR)
// #define ECM_ADDR_DATA_BYTE			(XPAR_EC01M_0_S00_AXI_BASEADDR + 4)
// #define ECM_INTR_RESET				(XPAR_EC01M_0_S00_AXI_BASEADDR + 8)
// // Read
// #define ECM_ADDR_BUSY				(XPAR_EC01M_0_S00_AXI_BASEADDR)
// #define ECM_ADDR_IC_BUSY			(XPAR_EC01M_0_S00_AXI_BASEADDR + 4)
// // DATA
// #define ECM_ADDR_DATA_OUT			(XPAR_BRAM_1_BASEADDR)
// #define ECM_ADDR_DATA_IN			(XPAR_BRAM_1_BASEADDR + 4096)

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

// mode define
#define MODE_IDLE 0
#define MODE_JOG 1
#define MODE_MOTION 2
#define MODE_HOME 3
#define MODE_JOGEND 4

// digital input define
#define DIGINPUT_LIMIT_LEFT 0x01
#define DIGINPUT_HMOE 0x02
#define DIGINPUT_LIMIT_RIGHT 0x04

// ECM SPI structure
// ECM_PACK_BEGIN
// typedef struct ECM_PACK spi_cmd_package_t{
// 	SPI_CMD_HEADER	Head;
// 	uint8_t			Data[PKG_DATA_DEFAULT_SIZE];
// 	uint32_t		Crc;
// 	uint32_t		StopWord;
// } SPI_CMD_PACKAGE_T;
// ECM_PACK_END
// ECM_PACK_BEGIN
// typedef struct ECM_PACK spi_ret_package_t{
// 	SPI_RET_HEADER	Head;
// 	uint8_t			Data[PKG_DATA_DEFAULT_SIZE];
// 	uint32_t		Crc;
// 	uint32_t		StopWord;
// } SPI_RET_PACKAGE_T;
// ECM_PACK_END

// Params structure
typedef struct motion_params{
	double m_dJogSpeed;
	double m_dMotionSpeed;
	double m_dComeHomeSpeed;
	double m_dLeftHomeSpeed;
	double m_dJogAcc;
	double m_dMotionAcc;
	double m_dHomeAcc;
	double m_dRatio; // Axis unit
} MOTION_PARAMS;

typedef struct position_params{
	u32 m_uMode;
	double m_dTarPos; // pulse
	int m_dCmdPos; // pulse
	int m_dCurPos; // pulse
	u32 m_uInput;
} POSITION_PARAMS;

extern MOTION_PARAMS g_Motion_Params[TEST_SERVO_CNT];
extern POSITION_PARAMS g_Position_Params[TEST_SERVO_CNT];
extern bool g_bInterruptFlag;
extern bool g_bStopFlag[TEST_SERVO_CNT];
extern double g_dDistance[TEST_SERVO_CNT]; // pulse
