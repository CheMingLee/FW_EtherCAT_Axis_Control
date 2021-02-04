#include "stdlib.h"
#include "stdbool.h"
#include "string.h"
#include "xil_io.h"
#include "Xscugic.h"
#include "Xil_exception.h"
#include "xparameters.h"
#include "EcmDriver.h"

// interrupt
#define ECM_INTR_ID		    	    XPAR_FABRIC_EC01M_0_PLINT_OUT_INTR
#define INTC_DEVICE_ID			    XPAR_SCUGIC_SINGLE_DEVICE_ID

// Write
#define ECM_ADDR_SEND				(XPAR_EC01M_0_S00_AXI_BASEADDR)
#define ECM_ADDR_DATA_BYTE			(XPAR_EC01M_0_S00_AXI_BASEADDR + 4)
#define ECM_INTR_RESET				(XPAR_EC01M_0_S00_AXI_BASEADDR + 8)
// Read
#define ECM_ADDR_BUSY				(XPAR_EC01M_0_S00_AXI_BASEADDR)
#define ECM_ADDR_IC_BUSY			(XPAR_EC01M_0_S00_AXI_BASEADDR + 4)
// DATA
#define ECM_ADDR_DATA_OUT			(XPAR_BRAM_1_BASEADDR)
#define ECM_ADDR_DATA_IN			(XPAR_BRAM_1_BASEADDR + 4096)

// BRAM define
#define IO_ADDR_BRAM_IN_FLAG		(XPAR_BRAM_0_BASEADDR + 0)
#define IO_ADDR_BRAM_IN_CMD         (XPAR_BRAM_0_BASEADDR + 8)
#define IO_ADDR_BRAM_IN_DATA        (XPAR_BRAM_0_BASEADDR + 12)
#define IO_ADDR_BRAM_OUT_SIZE       (XPAR_BRAM_0_BASEADDR + 260)
#define IO_ADDR_BRAM_OUT_ACK        (XPAR_BRAM_0_BASEADDR + 264)
#define IO_ADDR_BRAM_OUT_ACK_SIZE   (XPAR_BRAM_0_BASEADDR + 266)
#define IO_ADDR_BRAM_OUT_DATA       (XPAR_BRAM_0_BASEADDR + 268)

// CMD define


// ECM SPI structure
ECM_PACK_BEGIN
typedef struct ECM_PACK spi_cmd_package_t{
	SPI_CMD_HEADER	Head;
	uint8_t			Data[PKG_DATA_DEFAULT_SIZE];
	uint32_t		Crc;
	uint32_t		StopWord;
} SPI_CMD_PACKAGE_T;
ECM_PACK_END
ECM_PACK_BEGIN
typedef struct ECM_PACK spi_ret_package_t{
	SPI_RET_HEADER	Head;
	uint8_t			Data[PKG_DATA_DEFAULT_SIZE];
	uint32_t		Crc;
	uint32_t		StopWord;
} SPI_RET_PACKAGE_T;
ECM_PACK_END
