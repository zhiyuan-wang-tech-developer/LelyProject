/*******************************************************************************
  System Initialization File

  File Name:
    system_init.c

  Summary:
    This file contains source code necessary to initialize the system.

  Description:
    This file contains source code necessary to initialize the system.  It
    implements the "SYS_Initialize" function, defines the configuration bits,
    and allocates any necessary global system resources, such as the
    sysObj structure that contains the object handles to all the MPLAB Harmony
    module objects in the system.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2015 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED AS IS WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 *******************************************************************************/
// DOM-IGNORE-END

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include "system_config.h"
#include "system_definitions.h"


// ****************************************************************************
// ****************************************************************************
// Section: Configuration Bits
// ****************************************************************************
// ****************************************************************************
// <editor-fold defaultstate="collapsed" desc="Configuration Bits">

/*** DEVCFG0 ***/

#pragma config DEBUG =      OFF
#pragma config JTAGEN =     OFF
#pragma config ICESEL =     ICS_PGx2
#pragma config TRCEN =      OFF
#pragma config BOOTISA =    MIPS32
#pragma config FSLEEP =     OFF
#pragma config DBGPER =     PG_ALL
#pragma config SMCLR =      MCLR_NORM
#pragma config SOSCGAIN =   GAIN_1X
#pragma config SOSCBOOST =  OFF
#pragma config POSCGAIN =   GAIN_LEVEL_0
#pragma config POSCBOOST =  OFF
#pragma config EJTAGBEN =   NORMAL
#pragma config CP =         OFF

/*** DEVCFG1 ***/

#pragma config FNOSC =      SPLL
#pragma config DMTINTV =    WIN_127_128
#pragma config FSOSCEN =    OFF
#pragma config IESO =       OFF
#pragma config POSCMOD =    EC
#pragma config OSCIOFNC =   OFF
#pragma config FCKSM =      CSECME
#pragma config WDTPS =      PS1048576
#pragma config WDTSPGM =    STOP
#pragma config FWDTEN =     OFF
#pragma config WINDIS =     NORMAL
#pragma config FWDTWINSZ =  WINSZ_25
#pragma config DMTCNT =     DMT31
#pragma config FDMTEN =     OFF
/*** DEVCFG2 ***/

#pragma config FPLLIDIV =   DIV_1
#pragma config FPLLRNG =    RANGE_8_16_MHZ
#pragma config FPLLICLK =   PLL_POSC
#pragma config FPLLMULT =   MUL_40
#pragma config FPLLODIV =   DIV_4
#pragma config VBATBOREN =  OFF
#pragma config DSBOREN =    OFF
#pragma config DSWDTPS =    DSPS32
#pragma config DSWDTOSC =   LPRC
#pragma config DSWDTEN =    OFF
#pragma config FDSEN =      OFF
#pragma config BORSEL =     HIGH
#pragma config UPLLEN =     OFF
/*** DEVCFG3 ***/

#pragma config USERID =     0xffff
#pragma config FUSBIDIO2 =   OFF
#pragma config FVBUSIO2 =  OFF
#pragma config PGL1WAY =    OFF
#pragma config PMDL1WAY =   OFF
#pragma config IOL1WAY =    OFF
#pragma config FUSBIDIO1 =   OFF
#pragma config FVBUSIO1 =  OFF
#pragma config PWMLOCK =  OFF

/*** BF1SEQ0 ***/

#pragma config TSEQ =       0x0000
#pragma config CSEQ =       0xffff  
// </editor-fold>

// *****************************************************************************
// *****************************************************************************
// Section: Driver Initialization Data
// *****************************************************************************
// *****************************************************************************
// <editor-fold defaultstate="collapsed" desc="DRV_Timer Initialization Data">
// </editor-fold>
// <editor-fold defaultstate="collapsed" desc="DRV_USART Initialization Data">
// </editor-fold>

// *****************************************************************************
// *****************************************************************************
// Section: System Data
// *****************************************************************************
// *****************************************************************************

/* Structure to hold the object handles for the modules in the system. */
SYSTEM_OBJECTS sysObj;

// *****************************************************************************
// *****************************************************************************
// Section: Module Initialization Data
// *****************************************************************************
// *****************************************************************************
// <editor-fold defaultstate="collapsed" desc="SYS_CONSOLE Initialization Data">
// </editor-fold>
// <editor-fold defaultstate="collapsed" desc="SYS_DEBUG Initialization Data">
/*** System Debug Initialization Data ***/

SYS_DEBUG_INIT debugInit =
{
    .moduleInit = {0},
    .errorLevel = SYS_ERROR_FATAL
};
// </editor-fold>

// *****************************************************************************
// *****************************************************************************
// Section: Library/Stack Initialization Data
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
// *****************************************************************************
// Section: System Initialization
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void SYS_Initialize ( void *data )

  Summary:
    Initializes the board, services, drivers, application and other modules.

  Remarks:
    See prototype in system/common/sys_module.h.
 */

void SYS_Initialize ( void* data )
{
    /* Core Processor Initialization */
    SYS_CLK_Initialize( NULL );
    SYS_DEVCON_Initialize(SYS_DEVCON_INDEX_0, (SYS_MODULE_INIT*)NULL);
    SYS_DEVCON_PerformanceConfig(SYS_CLK_SystemFrequencyGet());
    SYS_PORTS_Initialize();

    /* Initialize Drivers */
    
    /* Initialize ADC */
//    DRV_ADC_Initialize();

    /* Initialize CAN Driver 0 */
//    DRV_CAN0_Initialize();

    /*Initialize MCPWM */
//    DRV_MCPWM_Initialize();
    /*Initialize TMR0 */
//    DRV_TMR0_Initialize();
    /*Initialize TMR1 */
    DRV_TMR1_Initialize();

//    sysObj.drvTmr0 = DRV_TMR_Initialize(DRV_TMR_INDEX_0, (SYS_MODULE_INIT *)&drvTmr0InitData);

    SYS_INT_VectorPrioritySet(INT_VECTOR_T1, INT_PRIORITY_LEVEL4);
    SYS_INT_VectorSubprioritySet(INT_VECTOR_T1, INT_SUBPRIORITY_LEVEL0);
 
 
    sysObj.drvUsart0 = DRV_USART_Initialize(DRV_USART_INDEX_0, (SYS_MODULE_INIT *)NULL);
    SYS_INT_VectorPrioritySet(INT_VECTOR_UART2_TX, INT_DISABLE_INTERRUPT);
    SYS_INT_VectorSubprioritySet(INT_VECTOR_UART2_TX, INT_SUBPRIORITY_LEVEL0);
    SYS_INT_VectorPrioritySet(INT_VECTOR_UART2_RX, INT_PRIORITY_LEVEL3);
    SYS_INT_VectorSubprioritySet(INT_VECTOR_UART2_RX, INT_SUBPRIORITY_LEVEL0);
//    SYS_INT_VectorPrioritySet(INT_VECTOR_CAN1, INT_PRIORITY_LEVEL4);
//    SYS_INT_VectorSubprioritySet(INT_VECTOR_CAN1, INT_SUBPRIORITY_LEVEL0);

    /* Initialize System Services */
    sysObj.sysConsole0 = SYS_CONSOLE_Initialize(SYS_CONSOLE_INDEX_0, NULL);


    /*** Debug Service Initialization Code ***/
    sysObj.sysDebug = SYS_DEBUG_Initialize(SYS_DEBUG_INDEX_0, (SYS_MODULE_INIT*)&debugInit);

    /*** Interrupt Service Initialization Code ***/
    SYS_INT_Initialize();

    /* Initialize Middleware */

    /* Enable Global Interrupts */
    SYS_INT_Enable();

    /* Initialize the Application */
//    ANALOG_VOLTAGE_MONITOR_Initialize();
//    ERROR_HANDLER_Initialize();
//    LED_CONTROLLER_Initialize();
//    PWM_CONTROLLER_Initialize();
//    CAN_CONTROLLER_Initialize();
    UART_DEBUGGER_Initialize();
    
    /* Prompt the successful system initialization */
//    SYS_PRINT("System is initialized!\n");
}


/*******************************************************************************
 End of File
*/

