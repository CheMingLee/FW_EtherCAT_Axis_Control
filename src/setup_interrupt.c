#include "setting.h"

XScuGic Intc; //GIC

void ECM_intr_Handler(void *CallBackRef)
{
	if(g_bInterruptFlag)
	{
		int i;
		for (i = 0; i < TEST_SERVO_CNT; i++)
		{
			switch (g_Position_Params[i].m_uMode)
			{
				case MODE_IDLE:
				{
					g_Position_Params[i].m_iCmdPos = g_Position_Params[i].m_iCurPos;
					break;
				}
				case MODE_JOG:
				{
					
					

					break;
				}
				case MODE_MOTION:
				{
					break;
				}
				case MODE_HOME:
				{
					break;
				}
				case MODE_JOGEND:
				{
					break;
				}
				default:
				{
					break;
				}
			}
		}
	}
	
	Xil_Out32(ECM_INTR_RESET, 1);
}

void SetupInterruptSystem()
{
	XScuGic *GicInstancePtr = &Intc;
    XScuGic_Config *IntcConfig; //GIC config
	Xil_ExceptionInit();

	//initialise the GIC
	IntcConfig = XScuGic_LookupConfig(INTC_DEVICE_ID);

	XScuGic_CfgInitialize(GicInstancePtr, IntcConfig, IntcConfig->CpuBaseAddress);

	//connect to the hardware
	Xil_ExceptionRegisterHandler(XIL_EXCEPTION_ID_INT, (Xil_ExceptionHandler)XScuGic_InterruptHandler, GicInstancePtr);

	XScuGic_Connect(GicInstancePtr, ECM_INTR_ID, (Xil_ExceptionHandler)ECM_intr_Handler, (void *)&Intc);

	XScuGic_Enable(GicInstancePtr, ECM_INTR_ID);

	// Enable interrupts in the Processor.
	Xil_ExceptionEnableMask(XIL_EXCEPTION_IRQ);
}
