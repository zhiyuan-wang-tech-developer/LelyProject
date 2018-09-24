/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    uart_debugger.c

  Summary:
    This file contains the source code for the MPLAB Harmony application.

  Description:
    This file contains the source code for the MPLAB Harmony application.  It 
    implements the logic of the application's state machine and it may call 
    API routines of other MPLAB Harmony modules in the system, such as drivers,
    system services, and middleware.  However, it does not call any of the
    system interfaces (such as the "Initialize" and "Tasks" functions) of any of
    the modules in the system or make any assumptions about when those functions
    are called.  That is the responsibility of the configuration-specific system
    files.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
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

#include "uart_debugger.h"
      #include "global_event.h"

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    This structure should be initialized by the APP_Initialize function.
    
    Application strings and buffers are be defined outside this structure.
*/

UART_DEBUGGER_DATA uart_debuggerData;

//extern uint8_t uart2_buffer[128];

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary callback functions.
*/

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************


/* TODO:  Add any necessary local functions.
*/


// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void UART_DEBUGGER_Initialize ( void )

  Remarks:
    See prototype in uart_debugger.h.
 */

void UART_DEBUGGER_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    uart_debuggerData.state = UART_DEBUGGER_STATE_INIT;

    
    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
    
}


/******************************************************************************
  Function:
    void UART_DEBUGGER_Tasks ( void )

  Remarks:
    See prototype in uart_debugger.h.
 */

void UART_DEBUGGER_Tasks ( void )
{

    /* Check the application's current state. */
    switch ( uart_debuggerData.state )
    {
        /* Application's initial state. */
        case UART_DEBUGGER_STATE_INIT:
        {
            bool appInitialized = true;
       
        
            if (appInitialized)
            {            
                if(!CmdFifoInitialize(FIFO_SIZE))
                {
                    // return if FIFO initialization failed.
                    return;
                }
                printf("UART 2 is initialized!\n");
                printf("[test lely]");
                uart_debuggerData.state = UART_DEBUGGER_STATE_RUN;
            }
            break;
        }

        case UART_DEBUGGER_STATE_RUN:
        {
//            test_uart_processCommands(uart2_buffer, true);
            break;
        }

        /* TODO: implement your application state machine.*/
        

        /* The default state should never be executed. */
        default:
        {
            /* TODO: Handle error in application's state machine. */
            printf("UART Debugger Error!");
            break;
        }
    }
}

/*
 * @Desciption 
 *      Initialize the parsed command FIFO buffer
 * @Parameters 
 *      bufferSize: specify the FIFO buffer size
 * @Return 
 *      true: succeed to allocate memory for FIFO buffer
 *      false: fail to allocate memory for FIFO buffer
 */
bool CmdFifoInitialize(uint8_t bufferSize)
{
    uart_debuggerData.parsedCmdMsgBuffer.outIndex = 0;
    uart_debuggerData.parsedCmdMsgBuffer.inIndex = 0;
    uart_debuggerData.parsedCmdMsgBuffer.countUsedCells = 0;
    uart_debuggerData.parsedCmdMsgBuffer.size = bufferSize;
    // memory allocation
    // every item in the fifo buffer is a pointer to a parsed cmd string
    uart_debuggerData.parsedCmdMsgBuffer.pFIFO = (char **)calloc(bufferSize, sizeof(char *));
    if(uart_debuggerData.parsedCmdMsgBuffer.pFIFO == NULL)
    {   
        // memory allocation failed
        return false;
    }
    else
    {
        // memory allocation succeeded
        return true;
    }
}

/*
 * @Desciption 
 *      Check if the command FIFO buffer is empty
 * @Parameters 
 *      None
 * @Return 
 *      true: command FIFO buffer is empty
 *      false: command FIFO buffer is not empty
 */
bool isCmdFifoEmpty( void )
{
    if(uart_debuggerData.parsedCmdMsgBuffer.countUsedCells == 0)
    {
        return true;
    }
    else
    {
        return false;
    }
}

/*
 * @Desciption 
 *      Check if the command FIFO buffer is full
 * @Parameters 
 *      None
 * @Return 
 *      true: command FIFO buffer is full
 *      false: command FIFO buffer is not full
 */
bool isCmdFifoFull( void )
{
    if(uart_debuggerData.parsedCmdMsgBuffer.countUsedCells == uart_debuggerData.parsedCmdMsgBuffer.size)
    {
        return true;
    }
    else
    {
        return false;
    }
}

/*
 * @Desciption 
 *      Push a command string into the FIFO buffer
 * @Parameters 
 *      pStringIn: pointer to an input command string
 * @Return 
 *      true: pushing a string into the FIFO buffer is successful
 *      false: pushing a string into the FIFO buffer is failed
 */
bool CmdFifoPush(char *pStringIn)
{
    if(pStringIn == NULL)
    {
        return false;
    }
    if(isCmdFifoFull())
    {
        return false;
    }
    // get the input string length
    size_t strLength = strlen(pStringIn);
    // allocate memory for storing string
    uart_debuggerData.parsedCmdMsgBuffer.pFIFO[uart_debuggerData.parsedCmdMsgBuffer.inIndex] = (char *) calloc(strLength, sizeof(char));
    if(uart_debuggerData.parsedCmdMsgBuffer.pFIFO[uart_debuggerData.parsedCmdMsgBuffer.inIndex] == NULL)
    {
        // memory allocation request failed
        return false;
    }
    // copy the string to the allocated memory
    strcpy(uart_debuggerData.parsedCmdMsgBuffer.pFIFO[uart_debuggerData.parsedCmdMsgBuffer.inIndex], pStringIn);
    uart_debuggerData.parsedCmdMsgBuffer.countUsedCells++; // one cell has been occupied in FIFO buffer.
    uart_debuggerData.parsedCmdMsgBuffer.inIndex = (uart_debuggerData.parsedCmdMsgBuffer.inIndex + 1)%uart_debuggerData.parsedCmdMsgBuffer.size; // round FIFO buffer
    return true;
}

/*
 * @Desciption 
 *      Pop a command string from the FIFO buffer
 * @Parameters 
 *      pStringOut: pointer to an output command string
 *      strLength: indicate the output command string length
 * @Return 
 *      true: popping a string from the FIFO buffer is successful
 *      false: popping a string from the FIFO buffer is failed
 */
bool CmdFifoPop(char *pStringOut, size_t strLength)
{
    if(pStringOut == NULL)
    {
        return false;
    }
    if(isCmdFifoEmpty())
    {
        return false;
    }
    // Get the popped command string length
    strLength = strlen(uart_debuggerData.parsedCmdMsgBuffer.pFIFO[uart_debuggerData.parsedCmdMsgBuffer.outIndex]);
    // Copy the popped command string
    strcpy(pStringOut, uart_debuggerData.parsedCmdMsgBuffer.pFIFO[uart_debuggerData.parsedCmdMsgBuffer.outIndex]);
    // free the allocated memory
    free((void *)uart_debuggerData.parsedCmdMsgBuffer.pFIFO[uart_debuggerData.parsedCmdMsgBuffer.outIndex]);
    // set the popped FIFO item as NULL because the memory block it points to has been released.
    uart_debuggerData.parsedCmdMsgBuffer.pFIFO[uart_debuggerData.parsedCmdMsgBuffer.outIndex] = NULL;
    uart_debuggerData.parsedCmdMsgBuffer.countUsedCells--; // one cell has been freed in FIFO buffer.
    uart_debuggerData.parsedCmdMsgBuffer.outIndex = (uart_debuggerData.parsedCmdMsgBuffer.outIndex + 1)%uart_debuggerData.parsedCmdMsgBuffer.size; // round FIFO buffer
    return true;
}        

void extractCmdFromRxFifo( void )
{
    
}
/*******************************************************************************
 End of File
 */
