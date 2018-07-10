/*******************************************************************************
  MPLAB Harmony Application Header File

  Company:
    Microchip Technology Inc.

  File Name:
    error_handler.h

  Summary:
    This header file provides prototypes and definitions for the application.

  Description:
    This header file provides function prototypes and data type definitions for
    the application.  Some of these are required by the system (such as the
    "APP_Initialize" and "APP_Tasks" prototypes) and some of them are only used
    internally by the application (such as the "APP_STATES" definition).  Both
    are defined here for convenience.
 *******************************************************************************/

//DOM-IGNORE-BEGIN
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
//DOM-IGNORE-END

#ifndef _ERROR_HANDLER_H
#define _ERROR_HANDLER_H

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include "system_config.h"
#include "system_definitions.h"

// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

extern "C" {

#endif
// DOM-IGNORE-END
    
#define YES     1
#define NO      0
    
// *****************************************************************************
// *****************************************************************************
// Section: Type Definitions
// *****************************************************************************
// *****************************************************************************

/*
 * Error status structure
 * 
 * It holds the status flags for errors that the error handler will handle.
 * 
 * If an error occurred, the corresponding error status flag bit is SET to 1.
 * 
 * Bit fields in the status are arranged in little-endianness.
 */    
typedef union
{
    uint64_t status;
    struct
    {
        // Undervoltage warning flags
        uint64_t UNDERVOLTAGE_WARNING_V380V: 1;
        uint64_t UNDERVOLTAGE_WARNING_V325V: 1;
        uint64_t UNDERVOLTAGE_WARNING_V18V: 1;
        uint64_t UNDERVOLTAGE_WARNING_V12V: 1;
        uint64_t UNDERVOLTAGE_WARNING_V5V: 1;
        uint64_t UNDERVOLTAGE_WARNING_V3V3_0: 1;
        uint64_t UNDERVOLTAGE_WARNING_V3V3_1: 1;
        uint64_t UNDERVOLTAGE_WARNING_V3V3_2: 1;
        uint64_t UNDERVOLTAGE_WARNING_V3V3AN1: 1;
        uint64_t UNDERVOLTAGE_WARNING_V3V3AN2: 1;
        uint64_t UNDERVOLTAGE_WARNING_V1V8_1: 1;
        uint64_t UNDERVOLTAGE_WARNING_V1V8_2: 1;
        uint64_t UNDERVOLTAGE_WARNING_UNUSED: 4;
        // Undervoltage fault flags
        uint64_t UNDERVOLTAGE_FAULT_V380V: 1;
        uint64_t UNDERVOLTAGE_FAULT_V325V: 1;
        uint64_t UNDERVOLTAGE_FAULT_V18V: 1;
        uint64_t UNDERVOLTAGE_FAULT_V12V: 1;
        uint64_t UNDERVOLTAGE_FAULT_V5V: 1;
        uint64_t UNDERVOLTAGE_FAULT_V3V3_0: 1;
        uint64_t UNDERVOLTAGE_FAULT_V3V3_1: 1;
        uint64_t UNDERVOLTAGE_FAULT_V3V3_2: 1;
        uint64_t UNDERVOLTAGE_FAULT_V3V3AN1: 1;
        uint64_t UNDERVOLTAGE_FAULT_V3V3AN2: 1;
        uint64_t UNDERVOLTAGE_FAULT_V1V8_1: 1;
        uint64_t UNDERVOLTAGE_FAULT_V1V8_2: 1;
        uint64_t UNDERVOLTAGE_FAULT_UNUSED: 4;
        // Overvoltage warning flags
        uint64_t OVERVOLTAGE_WARNING_V380V: 1;
        uint64_t OVERVOLTAGE_WARNING_V325V: 1;
        uint64_t OVERVOLTAGE_WARNING_V18V: 1;
        uint64_t OVERVOLTAGE_WARNING_V12V: 1;
        uint64_t OVERVOLTAGE_WARNING_V5V: 1;
        uint64_t OVERVOLTAGE_WARNING_V3V3_0: 1;
        uint64_t OVERVOLTAGE_WARNING_V3V3_1: 1;
        uint64_t OVERVOLTAGE_WARNING_V3V3_2: 1;
        uint64_t OVERVOLTAGE_WARNING_V3V3AN1: 1;
        uint64_t OVERVOLTAGE_WARNING_V3V3AN2: 1;
        uint64_t OVERVOLTAGE_WARNING_V1V8_1: 1;
        uint64_t OVERVOLTAGE_WARNING_V1V8_2: 1;
        uint64_t OVERVOLTAGE_WARNING_UNUSED: 4;
        // Overvoltage fault flags
        uint64_t OVERVOLTAGE_FAULT_V380V: 1;
        uint64_t OVERVOLTAGE_FAULT_V325V: 1;
        uint64_t OVERVOLTAGE_FAULT_V18V: 1;
        uint64_t OVERVOLTAGE_FAULT_V12V: 1;
        uint64_t OVERVOLTAGE_FAULT_V5V: 1;
        uint64_t OVERVOLTAGE_FAULT_V3V3_0: 1;
        uint64_t OVERVOLTAGE_FAULT_V3V3_1: 1;
        uint64_t OVERVOLTAGE_FAULT_V3V3_2: 1;
        uint64_t OVERVOLTAGE_FAULT_V3V3AN1: 1;
        uint64_t OVERVOLTAGE_FAULT_V3V3AN2: 1;
        uint64_t OVERVOLTAGE_FAULT_V1V8_1: 1;
        uint64_t OVERVOLTAGE_FAULT_V1V8_2: 1;
        uint64_t OVERVOLTAGE_FAULT_UNUSED: 4;
    } status_flags;
} VOLTAGE_ERROR_STATUS_TYPE;

typedef union
{
    uint8_t status;
    struct
    {
        // Overcurrent warning flags
        uint8_t OVERCURRENT_WARNING_IL12: 1;
        uint8_t OVERCURRENT_WARNING_IL34: 1;
        uint8_t OVERCURRENT_WARNING_UNUSED: 2;
        // Overcurrent fault flags
        uint8_t OVERCURRENT_FAULT_IL12: 1;
        uint8_t OVERCURRENT_FAULT_IL34: 1;
        uint8_t OVERCURRENT_FAULT_UNUSED: 2;
    } status_flags;
} CURRENT_ERROR_STATUS_TYPE;

typedef union
{   
    uint16_t status;
    struct
    {
        // Overheat warning flags
        uint16_t OVERHEAT_WARNING_M1: 1;
        uint16_t OVERHEAT_WARNING_M2: 1;
        uint16_t OVERHEAT_WARNING_PFC12: 1;
        uint16_t OVERHEAT_WARNING_PFC34: 1;
        uint16_t OVERHEAT_WARNING_ELCO: 1;
        uint16_t OVERHEAT_WARNING_BRUG: 1;
        uint16_t OVERHEAT_WARNING_VOED: 1;
        uint16_t OVERHEAT_WARNING_UNUSED: 1;
        // Overheat fault flags
        uint16_t OVERHEAT_FAULT_M1: 1;
        uint16_t OVERHEAT_FAULT_M2: 1;
        uint16_t OVERHEAT_FAULT_PFC12: 1;
        uint16_t OVERHEAT_FAULT_PFC34: 1;
        uint16_t OVERHEAT_FAULT_ELCO: 1;
        uint16_t OVERHEAT_FAULT_BRUG: 1;
        uint16_t OVERHEAT_FAULT_VOED: 1;
        uint16_t OVERHEAT_FAULT_UNUSED: 1;
    } status_flags;
} TEMPERATURE_ERROR_STATUS_TYPE;
    
// *****************************************************************************

/* Application states

  Summary:
    Application states enumeration

  Description:
    This enumeration defines the valid application states.  These states
    determine the behavior of the application at various times.
 */

typedef enum {
    /* Application's state machine's initial state. */
    ERROR_HANDLER_STATE_INIT = 0,

    /* TODO: Define states used by the application state machine. */
    ERROR_HANDLER_STATE_READY,
    ERROR_HANDLER_STATE_HANDLE_VOLTAGE_ERROR,
    ERROR_HANDLER_STATE_HANDLE_CURRENT_ERROR,
    ERROR_HANDLER_STATE_HANDLE_TEMPERATURE_ERROR

} ERROR_HANDLER_STATES;


// *****************************************************************************

/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    Application strings and buffers are be defined outside this structure.
 */

typedef struct {
    /* The application's current state */
    ERROR_HANDLER_STATES    state;

    /* TODO: Define any additional data used by the application. */
    VOLTAGE_ERROR_STATUS_TYPE   voltageError;
    CURRENT_ERROR_STATUS_TYPE   currentError;
    TEMPERATURE_ERROR_STATUS_TYPE   temperatureError;
    /*
     * errorCheckSequence = 0 for voltage error check
     * errorCheckSequence = 1 for current error check
     * errorCheckSequence = 2 for temperature error check
     */
    uint8_t     errorCheckSequence;

} ERROR_HANDLER_DATA;


// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Routines
// *****************************************************************************
// *****************************************************************************
/* These routines are called by drivers when certain events occur.
 */

/*
 * LED indication functions which are called in error_handler_tasks()
 * 
 * LED1 indicates voltage states
 * LED2 indicates current states
 * LED3 indicates temperature states 
 */
extern void setLed1NormalBlink(void);
extern void setLed1WarningBlink(void);
extern void setLed1FaultBlink(void);
extern void setLed2NormalBlink(void);
extern void setLed2WarningBlink(void);
extern void setLed2FaultBlink(void);
extern void setLed3NormalBlink(void);
extern void setLed3WarningBlink(void);
extern void setLed3FaultBlink(void);

// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void ERROR_HANDLER_Initialize ( void )

  Summary:
     MPLAB Harmony application initialization routine.

  Description:
    This function initializes the Harmony application.  It places the 
    application in its initial state and prepares it to run so that its 
    APP_Tasks function can be called.

  Precondition:
    All other system initialization routines should be called before calling
    this routine (in "SYS_Initialize").

  Parameters:
    None.

  Returns:
    None.

  Example:
    <code>
    ERROR_HANDLER_Initialize();
    </code>

  Remarks:
    This routine must be called from the SYS_Initialize function.
 */

void ERROR_HANDLER_Initialize(void);


/*******************************************************************************
  Function:
    void ERROR_HANDLER_Tasks ( void )

  Summary:
    MPLAB Harmony Demo application tasks function

  Description:
    This routine is the Harmony Demo application's tasks function.  It
    defines the application's state machine and core logic.

  Precondition:
    The system and application initialization ("SYS_Initialize") should be
    called before calling this.

  Parameters:
    None.

  Returns:
    None.

  Example:
    <code>
    ERROR_HANDLER_Tasks();
    </code>

  Remarks:
    This routine must be called from SYS_Tasks() routine.
 */

void ERROR_HANDLER_Tasks(void);


#endif /* _ERROR_HANDLER_H */

    //DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
//DOM-IGNORE-END

/*******************************************************************************
 End of File
 */

