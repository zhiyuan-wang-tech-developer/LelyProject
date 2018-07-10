/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    error_handler.c

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

#include "error_handler.h"

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

ERROR_HANDLER_DATA error_handlerData;

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
/*
 * Voltage error handling functions
 */
void handleUndervoltageWarning( void )
{
    if( error_handlerData.voltageError.status_flags.UNDERVOLTAGE_WARNING_V380V )
    {
        SYS_PRINT("\nWarning: V380V undervoltage\r\n");
        error_handlerData.voltageError.status_flags.UNDERVOLTAGE_WARNING_V380V = 0;
    }
    if( error_handlerData.voltageError.status_flags.UNDERVOLTAGE_WARNING_V325V )
    {
        SYS_PRINT("\nWarning: V325V undervoltage\r\n");
        error_handlerData.voltageError.status_flags.UNDERVOLTAGE_WARNING_V325V = 0;
    }
    if( error_handlerData.voltageError.status_flags.UNDERVOLTAGE_WARNING_V18V )
    {
        SYS_PRINT("\nWarning: V18V undervoltage\r\n");
        error_handlerData.voltageError.status_flags.UNDERVOLTAGE_WARNING_V18V = 0;
    }    
    if( error_handlerData.voltageError.status_flags.UNDERVOLTAGE_WARNING_V12V )
    {
        SYS_PRINT("\nWarning: V12V undervoltage\r\n");
        error_handlerData.voltageError.status_flags.UNDERVOLTAGE_WARNING_V12V = 0;
    }
    if( error_handlerData.voltageError.status_flags.UNDERVOLTAGE_WARNING_V5V )
    {
        SYS_PRINT("\nWarning: V5V undervoltage\r\n");
        error_handlerData.voltageError.status_flags.UNDERVOLTAGE_WARNING_V5V = 0;
    }
    if( error_handlerData.voltageError.status_flags.UNDERVOLTAGE_WARNING_V3V3_0 )
    {
        SYS_PRINT("\nWarning: V3V3_0 undervoltage\r\n");
        error_handlerData.voltageError.status_flags.UNDERVOLTAGE_WARNING_V3V3_0 = 0;
    }
    if( error_handlerData.voltageError.status_flags.UNDERVOLTAGE_WARNING_V3V3_1 )
    {
        SYS_PRINT("\nWarning: V3V3_1 undervoltage\r\n");
        error_handlerData.voltageError.status_flags.UNDERVOLTAGE_WARNING_V3V3_1 = 0;
    }
    if( error_handlerData.voltageError.status_flags.UNDERVOLTAGE_WARNING_V3V3_2 )
    {
        SYS_PRINT("\nWarning: V3V3_2 undervoltage\r\n");
        error_handlerData.voltageError.status_flags.UNDERVOLTAGE_WARNING_V3V3_2 = 0;
    }
    if( error_handlerData.voltageError.status_flags.UNDERVOLTAGE_WARNING_V3V3AN1 )
    {
        SYS_PRINT("\nWarning: V3V3AN1 undervoltage\r\n");
        error_handlerData.voltageError.status_flags.UNDERVOLTAGE_WARNING_V3V3AN1 = 0;
    }
    if( error_handlerData.voltageError.status_flags.UNDERVOLTAGE_WARNING_V3V3AN2 )
    {
        SYS_PRINT("\nWarning: V3V3AN2 undervoltage\r\n");
        error_handlerData.voltageError.status_flags.UNDERVOLTAGE_WARNING_V3V3AN2 = 0;
    }
    if( error_handlerData.voltageError.status_flags.UNDERVOLTAGE_WARNING_V1V8_1 )
    {
        SYS_PRINT("\nWarning: V1V8_1 undervoltage\r\n");
        error_handlerData.voltageError.status_flags.UNDERVOLTAGE_WARNING_V1V8_1 = 0;
    }
    if( error_handlerData.voltageError.status_flags.UNDERVOLTAGE_WARNING_V1V8_2 )
    {
        SYS_PRINT("\nWarning: V1V8_2 undervoltage\r\n");
        error_handlerData.voltageError.status_flags.UNDERVOLTAGE_WARNING_V1V8_2 = 0;
    }
}

void handleUndervoltageFault( void )
{
    if( error_handlerData.voltageError.status_flags.UNDERVOLTAGE_FAULT_V380V )
    {
        SYS_PRINT("\nFault: V380V undervoltage\r\n");
        error_handlerData.voltageError.status_flags.UNDERVOLTAGE_FAULT_V380V = 0;
    }
    if( error_handlerData.voltageError.status_flags.UNDERVOLTAGE_FAULT_V325V )
    {
        SYS_PRINT("\nFault: V325V undervoltage\r\n");
        error_handlerData.voltageError.status_flags.UNDERVOLTAGE_FAULT_V325V = 0;
    }
    if( error_handlerData.voltageError.status_flags.UNDERVOLTAGE_FAULT_V18V )
    {
        SYS_PRINT("\nFault: V18V undervoltage\r\n");
        error_handlerData.voltageError.status_flags.UNDERVOLTAGE_FAULT_V18V = 0;
    }    
    if( error_handlerData.voltageError.status_flags.UNDERVOLTAGE_FAULT_V12V )
    {
        SYS_PRINT("\nFault: V12V undervoltage\r\n");
        error_handlerData.voltageError.status_flags.UNDERVOLTAGE_FAULT_V12V = 0;
    }
    if( error_handlerData.voltageError.status_flags.UNDERVOLTAGE_FAULT_V5V )
    {
        SYS_PRINT("\nFault: V5V undervoltage\r\n");
        error_handlerData.voltageError.status_flags.UNDERVOLTAGE_FAULT_V5V = 0;
    }
    if( error_handlerData.voltageError.status_flags.UNDERVOLTAGE_FAULT_V3V3_0 )
    {
        SYS_PRINT("\nFault: V3V3_0 undervoltage\r\n");
        error_handlerData.voltageError.status_flags.UNDERVOLTAGE_FAULT_V3V3_0 = 0;
    }
    if( error_handlerData.voltageError.status_flags.UNDERVOLTAGE_FAULT_V3V3_1 )
    {
        SYS_PRINT("\nFault: V3V3_1 undervoltage\r\n");
        error_handlerData.voltageError.status_flags.UNDERVOLTAGE_FAULT_V3V3_1 = 0;
    }
    if( error_handlerData.voltageError.status_flags.UNDERVOLTAGE_FAULT_V3V3_2 )
    {
        SYS_PRINT("\nFault: V3V3_2 undervoltage\r\n");
        error_handlerData.voltageError.status_flags.UNDERVOLTAGE_FAULT_V3V3_2 = 0;
    }
    if( error_handlerData.voltageError.status_flags.UNDERVOLTAGE_FAULT_V3V3AN1 )
    {
        SYS_PRINT("\nFault: V3V3AN1 undervoltage\r\n");
        error_handlerData.voltageError.status_flags.UNDERVOLTAGE_FAULT_V3V3AN1 = 0;
    }
    if( error_handlerData.voltageError.status_flags.UNDERVOLTAGE_FAULT_V3V3AN2 )
    {
        SYS_PRINT("\nFault: V3V3AN2 undervoltage\r\n");
        error_handlerData.voltageError.status_flags.UNDERVOLTAGE_FAULT_V3V3AN2 = 0;
    }
    if( error_handlerData.voltageError.status_flags.UNDERVOLTAGE_FAULT_V1V8_1 )
    {
        SYS_PRINT("\nFault: V1V8_1 undervoltage\r\n");
        error_handlerData.voltageError.status_flags.UNDERVOLTAGE_FAULT_V1V8_1 = 0;
    }
    if( error_handlerData.voltageError.status_flags.UNDERVOLTAGE_FAULT_V1V8_2 )
    {
        SYS_PRINT("\nFault: V1V8_2 undervoltage\r\n");
        error_handlerData.voltageError.status_flags.UNDERVOLTAGE_FAULT_V1V8_2 = 0;
    }
}

void handleOvervoltageWarning( void )
{
    if( error_handlerData.voltageError.status_flags.OVERVOLTAGE_WARNING_V380V )
    {
        SYS_PRINT("\nWarning: V380V overvoltage\r\n");
        error_handlerData.voltageError.status_flags.OVERVOLTAGE_WARNING_V380V = 0;
    }
    if( error_handlerData.voltageError.status_flags.OVERVOLTAGE_WARNING_V325V )
    {
        SYS_PRINT("\nWarning: V325V overvoltage\r\n");
        error_handlerData.voltageError.status_flags.OVERVOLTAGE_WARNING_V325V = 0;
    }
    if( error_handlerData.voltageError.status_flags.OVERVOLTAGE_WARNING_V18V )
    {
        SYS_PRINT("\nWarning: V18V overvoltage\r\n");
        error_handlerData.voltageError.status_flags.OVERVOLTAGE_WARNING_V18V = 0;
    }    
    if( error_handlerData.voltageError.status_flags.OVERVOLTAGE_WARNING_V12V )
    {
        SYS_PRINT("\nWarning: V12V overvoltage\r\n");
        error_handlerData.voltageError.status_flags.OVERVOLTAGE_WARNING_V12V = 0;
    }
    if( error_handlerData.voltageError.status_flags.OVERVOLTAGE_WARNING_V5V )
    {
        SYS_PRINT("\nWarning: V5V overvoltage\r\n");
        error_handlerData.voltageError.status_flags.OVERVOLTAGE_WARNING_V5V = 0;
    }
    if( error_handlerData.voltageError.status_flags.OVERVOLTAGE_WARNING_V3V3_0 )
    {
        SYS_PRINT("\nWarning: V3V3_0 overvoltage\r\n");
        error_handlerData.voltageError.status_flags.OVERVOLTAGE_WARNING_V3V3_0 = 0;
    }
    if( error_handlerData.voltageError.status_flags.OVERVOLTAGE_WARNING_V3V3_1 )
    {
        SYS_PRINT("\nWarning: V3V3_1 overvoltage\r\n");
        error_handlerData.voltageError.status_flags.OVERVOLTAGE_WARNING_V3V3_1 = 0;
    }
    if( error_handlerData.voltageError.status_flags.OVERVOLTAGE_WARNING_V3V3_2 )
    {
        SYS_PRINT("\nWarning: V3V3_2 overvoltage\r\n");
        error_handlerData.voltageError.status_flags.OVERVOLTAGE_WARNING_V3V3_2 = 0;
    }
    if( error_handlerData.voltageError.status_flags.OVERVOLTAGE_WARNING_V3V3AN1 )
    {
        SYS_PRINT("\nWarning: V3V3AN1 overvoltage\r\n");
        error_handlerData.voltageError.status_flags.OVERVOLTAGE_WARNING_V3V3AN1 = 0;
    }
    if( error_handlerData.voltageError.status_flags.OVERVOLTAGE_WARNING_V3V3AN2 )
    {
        SYS_PRINT("\nWarning: V3V3AN2 overvoltage\r\n");
        error_handlerData.voltageError.status_flags.OVERVOLTAGE_WARNING_V3V3AN2 = 0;
    }
    if( error_handlerData.voltageError.status_flags.OVERVOLTAGE_WARNING_V1V8_1 )
    {
        SYS_PRINT("\nWarning: V1V8_1 overvoltage\r\n");
        error_handlerData.voltageError.status_flags.OVERVOLTAGE_WARNING_V1V8_1 = 0;
    }
    if( error_handlerData.voltageError.status_flags.OVERVOLTAGE_WARNING_V1V8_2 )
    {
        SYS_PRINT("\nWarning: V1V8_2 overvoltage\r\n");
        error_handlerData.voltageError.status_flags.OVERVOLTAGE_WARNING_V1V8_2 = 0;
    }
}

void handleOvervoltageFault( void )
{
    if( error_handlerData.voltageError.status_flags.OVERVOLTAGE_FAULT_V380V )
    {
        SYS_PRINT("\nFault: V380V overvoltage\r\n");
        error_handlerData.voltageError.status_flags.OVERVOLTAGE_FAULT_V380V = 0;
    }
    if( error_handlerData.voltageError.status_flags.OVERVOLTAGE_FAULT_V325V )
    {
        SYS_PRINT("\nFault: V325V overvoltage\r\n");
        error_handlerData.voltageError.status_flags.OVERVOLTAGE_FAULT_V325V = 0;
    }
    if( error_handlerData.voltageError.status_flags.OVERVOLTAGE_FAULT_V18V )
    {
        SYS_PRINT("\nFault: V18V overvoltage\r\n");
        error_handlerData.voltageError.status_flags.OVERVOLTAGE_FAULT_V18V = 0;
    }    
    if( error_handlerData.voltageError.status_flags.OVERVOLTAGE_FAULT_V12V )
    {
        SYS_PRINT("\nFault: V12V overvoltage\r\n");
        error_handlerData.voltageError.status_flags.OVERVOLTAGE_FAULT_V12V = 0;
    }
    if( error_handlerData.voltageError.status_flags.OVERVOLTAGE_FAULT_V5V )
    {
        SYS_PRINT("\nFault: V5V overvoltage\r\n");
        error_handlerData.voltageError.status_flags.OVERVOLTAGE_FAULT_V5V = 0;
    }
    if( error_handlerData.voltageError.status_flags.OVERVOLTAGE_FAULT_V3V3_0 )
    {
        SYS_PRINT("\nFault: V3V3_0 overvoltage\r\n");
        error_handlerData.voltageError.status_flags.OVERVOLTAGE_FAULT_V3V3_0 = 0;
    }
    if( error_handlerData.voltageError.status_flags.OVERVOLTAGE_FAULT_V3V3_1 )
    {
        SYS_PRINT("\nFault: V3V3_1 overvoltage\r\n");
        error_handlerData.voltageError.status_flags.OVERVOLTAGE_FAULT_V3V3_1 = 0;
    }
    if( error_handlerData.voltageError.status_flags.OVERVOLTAGE_FAULT_V3V3_2 )
    {
        SYS_PRINT("\nFault: V3V3_2 overvoltage\r\n");
        error_handlerData.voltageError.status_flags.OVERVOLTAGE_FAULT_V3V3_2 = 0;
    }
    if( error_handlerData.voltageError.status_flags.OVERVOLTAGE_FAULT_V3V3AN1 )
    {
        SYS_PRINT("\nFault: V3V3AN1 overvoltage\r\n");
        error_handlerData.voltageError.status_flags.OVERVOLTAGE_FAULT_V3V3AN1 = 0;
    }
    if( error_handlerData.voltageError.status_flags.OVERVOLTAGE_FAULT_V3V3AN2 )
    {
        SYS_PRINT("\nFault: V3V3AN2 overvoltage\r\n");
        error_handlerData.voltageError.status_flags.OVERVOLTAGE_FAULT_V3V3AN2 = 0;
    }
    if( error_handlerData.voltageError.status_flags.OVERVOLTAGE_FAULT_V1V8_1 )
    {
        SYS_PRINT("\nFault: V1V8_1 overvoltage\r\n");
        error_handlerData.voltageError.status_flags.OVERVOLTAGE_FAULT_V1V8_1 = 0;
    }
    if( error_handlerData.voltageError.status_flags.OVERVOLTAGE_FAULT_V1V8_2 )
    {
        SYS_PRINT("\nFault: V1V8_2 overvoltage\r\n");
        error_handlerData.voltageError.status_flags.OVERVOLTAGE_FAULT_V1V8_2 = 0;
    }
}


/*
 * Current error handling functions
 */
void handleOvercurrentWarning( void )
{
    if( error_handlerData.currentError.status_flags.OVERCURRENT_WARNING_IL12 )
    {
        SYS_PRINT("\nWarning: IL12 on the RC13 pin overcurrent\r\n");
        error_handlerData.currentError.status_flags.OVERCURRENT_WARNING_IL12 = 0;
    }
    if( error_handlerData.currentError.status_flags.OVERCURRENT_WARNING_IL34 )
    {
        SYS_PRINT("\nWarning: IL34 on the RD6 pin overcurrent\r\n");
        error_handlerData.currentError.status_flags.OVERCURRENT_WARNING_IL34 = 0;        
    }
}

void handleOvercurrentFault( void )
{
    if( error_handlerData.currentError.status_flags.OVERCURRENT_FAULT_IL12 )
    {
        SYS_PRINT("\nFault: IL12 on the RC13 pin overcurrent\r\n");
        error_handlerData.currentError.status_flags.OVERCURRENT_FAULT_IL12 = 0;
    }
    if( error_handlerData.currentError.status_flags.OVERCURRENT_FAULT_IL34 )
    {
        SYS_PRINT("\nFault: IL34 on the RD6 pin overcurrent\r\n");
        error_handlerData.currentError.status_flags.OVERCURRENT_FAULT_IL34 = 0;
    }
}


/*
 * Temperature error handling functions
 */
void handleOverheatWarning( void )
{
    if( error_handlerData.temperatureError.status_flags.OVERHEAT_WARNING_M1 )
    {
        SYS_PRINT("\nWarning: Motor 1 overheat\r\n");
        error_handlerData.temperatureError.status_flags.OVERHEAT_WARNING_M1 = 0;
    }
    if( error_handlerData.temperatureError.status_flags.OVERHEAT_WARNING_M2 )
    {
        SYS_PRINT("\nWarning: Motor 2 overheat\r\n");
        error_handlerData.temperatureError.status_flags.OVERHEAT_WARNING_M2 = 0;
    }
    if( error_handlerData.temperatureError.status_flags.OVERHEAT_WARNING_PFC12 )
    {
        SYS_PRINT("\nWarning: PFC12 overheat\r\n");
        error_handlerData.temperatureError.status_flags.OVERHEAT_WARNING_PFC12 = 0;
    }
    if( error_handlerData.temperatureError.status_flags.OVERHEAT_WARNING_PFC34 )
    {
        SYS_PRINT("\nWarning: PFC34 overheat\r\n");
        error_handlerData.temperatureError.status_flags.OVERHEAT_WARNING_PFC34 = 0;
    }
    if( error_handlerData.temperatureError.status_flags.OVERHEAT_WARNING_ELCO )
    {
        SYS_PRINT("\nWarning: ELCO overheat\r\n");
        error_handlerData.temperatureError.status_flags.OVERHEAT_WARNING_ELCO = 0;
    }
    if( error_handlerData.temperatureError.status_flags.OVERHEAT_WARNING_BRUG )
    {
        SYS_PRINT("\nWarning: BRUG overheat\r\n");
        error_handlerData.temperatureError.status_flags.OVERHEAT_WARNING_BRUG = 0;
    }
    if( error_handlerData.temperatureError.status_flags.OVERHEAT_WARNING_VOED )
    {
        SYS_PRINT("\nWarning: VOED overheat\r\n");
        error_handlerData.temperatureError.status_flags.OVERHEAT_WARNING_VOED = 0;
    }
}

void handleOverheatFault( void )
{
    if( error_handlerData.temperatureError.status_flags.OVERHEAT_FAULT_M1 )
    {
        SYS_PRINT("\nFault: Motor 1 overheat\r\n");
        error_handlerData.temperatureError.status_flags.OVERHEAT_FAULT_M1 = 0;
    }
    if( error_handlerData.temperatureError.status_flags.OVERHEAT_FAULT_M2 )
    {
        SYS_PRINT("\nFault: Motor 2 overheat\r\n");
        error_handlerData.temperatureError.status_flags.OVERHEAT_FAULT_M2 = 0;
    }
    if( error_handlerData.temperatureError.status_flags.OVERHEAT_FAULT_PFC12 )
    {
        SYS_PRINT("\nFault: PFC12 overheat\r\n");
        error_handlerData.temperatureError.status_flags.OVERHEAT_FAULT_PFC12 = 0;
    }
    if( error_handlerData.temperatureError.status_flags.OVERHEAT_FAULT_PFC34 )
    {
        SYS_PRINT("\nFault: PFC34 overheat\r\n");
        error_handlerData.temperatureError.status_flags.OVERHEAT_FAULT_PFC34 = 0;
    }
    if( error_handlerData.temperatureError.status_flags.OVERHEAT_FAULT_ELCO )
    {
        SYS_PRINT("\nFault: ELCO overheat\r\n");
        error_handlerData.temperatureError.status_flags.OVERHEAT_FAULT_ELCO = 0;
    }
    if( error_handlerData.temperatureError.status_flags.OVERHEAT_FAULT_BRUG )
    {
        SYS_PRINT("\nFault: BRUG overheat\r\n");
        error_handlerData.temperatureError.status_flags.OVERHEAT_FAULT_BRUG = 0;
    }
    if( error_handlerData.temperatureError.status_flags.OVERHEAT_FAULT_VOED )
    {
        SYS_PRINT("\nFault: VOED overheat\r\n");
        error_handlerData.temperatureError.status_flags.OVERHEAT_FAULT_VOED = 0;
    }    
}


// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void ERROR_HANDLER_Initialize ( void )

  Remarks:
    See prototype in error_handler.h.
 */

void ERROR_HANDLER_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    error_handlerData.state = ERROR_HANDLER_STATE_INIT;
    
    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
    error_handlerData.voltageError.status = 0;
    error_handlerData.currentError.status = 0;
    error_handlerData.temperatureError.status = 0;
    error_handlerData.errorCheckSequence = 0;
}


/******************************************************************************
  Function:
    void ERROR_HANDLER_Tasks ( void )

  Remarks:
    See prototype in error_handler.h.
 */

void ERROR_HANDLER_Tasks ( void )
{

    /* Check the application's current state. */
    switch ( error_handlerData.state )
    {
        /* Application's initial state. */
        case ERROR_HANDLER_STATE_INIT:
        {        
            error_handlerData.state = ERROR_HANDLER_STATE_READY;
            error_handlerData.voltageError.status = 0;
            error_handlerData.currentError.status = 0;
            error_handlerData.temperatureError.status = 0;        
            error_handlerData.errorCheckSequence = 0;
            break;
        }
        
        /* TODO: implement your application state machine.*/
        case ERROR_HANDLER_STATE_READY:
        {
            if( error_handlerData.errorCheckSequence == 0 && \
                error_handlerData.voltageError.status != 0 )
            {
                error_handlerData.state = ERROR_HANDLER_STATE_HANDLE_VOLTAGE_ERROR;
            }
            else if( error_handlerData.errorCheckSequence == 1 && \
                     error_handlerData.currentError.status != 0 )
            {
                error_handlerData.state = ERROR_HANDLER_STATE_HANDLE_CURRENT_ERROR;
            }
            else if( error_handlerData.errorCheckSequence == 2 && \
                     error_handlerData.temperatureError.status != 0 )
            {
                error_handlerData.state = ERROR_HANDLER_STATE_HANDLE_TEMPERATURE_ERROR;
            }
            else
            {
                // Control LED blinking
                // LED1 indicates voltage states
                setLed1NormalBlink();
                // LED2 indicates current states.
                setLed2NormalBlink();
                // LED3 indicates temperature states.
                setLed3NormalBlink();
                error_handlerData.state = ERROR_HANDLER_STATE_READY;
            }
            // Increment error check sequence for next error check
            error_handlerData.errorCheckSequence++;
            // The error check sequence ranges from 0 ~ 2
            error_handlerData.errorCheckSequence = error_handlerData.errorCheckSequence % 3;
            break;
        }
                
        case ERROR_HANDLER_STATE_HANDLE_VOLTAGE_ERROR:
        {
//            SYS_DEBUG_BreakPoint();
            // Check voltage error: undervoltage warning
            if( error_handlerData.voltageError.status & 0x0000000000000FFF )
            {
                setLed1WarningBlink();
                handleUndervoltageWarning();
            }
            // Check voltage error: undervoltage fault
            if( error_handlerData.voltageError.status & 0x000000000FFF0000 )
            {
                setLed1FaultBlink();
                handleUndervoltageFault();
            }
            // Check voltage error: overvoltage warning
            if( error_handlerData.voltageError.status & 0x00000FFF00000000 )
            {
                setLed1WarningBlink();
                handleOvervoltageWarning();
            }
            // Check voltage error: overvoltage fault
            if( error_handlerData.voltageError.status & 0x0FFF000000000000 )
            {
                setLed1FaultBlink();
                handleOvervoltageFault();
            }
            error_handlerData.state = ERROR_HANDLER_STATE_READY;        
            break;
        }

        case ERROR_HANDLER_STATE_HANDLE_CURRENT_ERROR:
        {
//            SYS_DEBUG_BreakPoint();
            // Check current error: overcurrent warning
            if( error_handlerData.currentError.status & 0x03 )
            {
                setLed2WarningBlink();
                handleOvercurrentWarning();
            }
            // Check current error: overcurrent fault
            if( error_handlerData.currentError.status & 0x30 )
            {
                setLed2FaultBlink();
                handleOvercurrentFault();
            }
            error_handlerData.state = ERROR_HANDLER_STATE_READY;        
            break;
        }

        case ERROR_HANDLER_STATE_HANDLE_TEMPERATURE_ERROR:
        {
//            SYS_DEBUG_BreakPoint();
            // Check temperature error: overheat warning
            if( error_handlerData.temperatureError.status & 0x007F )
            {
                setLed3WarningBlink();
                handleOverheatWarning();
            }
            // Check temperature error: overheat fault
            if( error_handlerData.temperatureError.status & 0x7F00 )
            {
                setLed3FaultBlink();
                handleOverheatFault();
            }
            error_handlerData.state = ERROR_HANDLER_STATE_READY;        
            break;
        }        

        /* The default state should never be executed. */
        default:
        {
            /* TODO: Handle error in application's state machine. */
            SYS_PRINT("\nFault: Error handler in WRONG state!\r\n");
            // Force the state machine back to the initial state and restart again
            error_handlerData.state = ERROR_HANDLER_STATE_INIT;            
            break;
        }
    }
}

 

/*******************************************************************************
 End of File
 */
