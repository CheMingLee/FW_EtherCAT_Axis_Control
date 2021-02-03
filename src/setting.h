#include "stdlib.h"
#include "stdbool.h"
#include "string.h"
#include "xil_io.h"
#include "Xscugic.h"
#include "Xil_exception.h"
#include "xparameters.h"

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
