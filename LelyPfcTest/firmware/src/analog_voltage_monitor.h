/*******************************************************************************
  MPLAB Harmony Application Header File

  Company:
    Microchip Technology Inc.

  File Name:
    analog_voltage_monitor.h

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

#ifndef _ANALOG_VOLTAGE_MONITOR_H
#define _ANALOG_VOLTAGE_MONITOR_H

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

// *****************************************************************************
// *****************************************************************************
// Section: Type Definitions
// *****************************************************************************
// *****************************************************************************
#define YES     1
#define NO      0

/*
 *  ADC update status structure
 *  It holds the update status of ADC scan data in the ADC data buffer.
 */
typedef union
{   
    uint32_t status;
    /* If an ADC datum is updated in the data buffer,
     * the corresponding update flag bit is SET to 1.             
     */
    struct
    {
        // Bit fields in the status are arranged in little-endianness.
        uint32_t V380V: 1;
        uint32_t V325V: 1;
        uint32_t VLIVE: 1;
        uint32_t VNEUTRAL: 1;
        uint32_t IL12: 1;
        uint32_t IL34: 1;   // --> AN5
        uint32_t TEMP_M1: 1;
        uint32_t TEMP_M2: 1;
        uint32_t TEMP_PFC12: 1;
        uint32_t TEMP_PFC34: 1;
        uint32_t TEMP_ELCO: 1;
        uint32_t TEMP_BRUG: 1;
        uint32_t TEMP_VOED: 1;
        uint32_t V18V: 1;
        uint32_t V12V: 1;
        uint32_t V5V: 1;
        uint32_t V3V3_0: 1;
        uint32_t V3V3_1: 1;
        uint32_t V3V3_2: 1;
        uint32_t V3V3AN1: 1;
        uint32_t V3V3AN2: 1;
        uint32_t V1V8_1: 1;
        uint32_t V1V8_2: 1;
        uint32_t UNUSED: 9;
    };
} ADC_SCAN_UPDATE_STATUS;
        
/* 
 * ADC data structure
 * It holds the ADC scan data for all 23 analog inputs. 
 */    
typedef union
{
    uint32_t buffer[24];
    struct
    {
        /* AC power */
        uint32_t V380V;
        uint32_t V325V;
        uint32_t VLIVE;     // AC live line
        uint32_t VNEUTRAL;  // AC neutral line
        uint32_t IL12;  // current load
        uint32_t IL34;
        /* Temperature */
        uint32_t TEMP_M1;   // Temperature for motor 1
        uint32_t TEMP_M2;
        uint32_t TEMP_PFC12; // Temperature for power factor corrector
        uint32_t TEMP_PFC34;
        uint32_t TEMP_ELCO;
        uint32_t TEMP_BRUG;
        uint32_t TEMP_VOED;
        /* DC power */
        uint32_t V18V;
        uint32_t V12V;
        uint32_t V5V;
        uint32_t V3V3_0;    // DC 3.3V
        uint32_t V3V3_1;
        uint32_t V3V3_2;
        uint32_t V3V3AN1;
        uint32_t V3V3AN2;
        uint32_t V1V8_1;    // DC 1.8V
        uint32_t V1V8_2;
        /* Update flags */
        ADC_SCAN_UPDATE_STATUS update;
    };
} ADC_SCAN_DATA_TYPE;

// *****************************************************************************
/* Application states

  Summary:
    Application states enumeration

  Description:
    This enumeration defines the valid application states.  These states
    determine the behavior of the application at various times.
*/

typedef enum
{
	/* Application's state machine's initial state. */
	ANALOG_VOLTAGE_MONITOR_STATE_INIT=0,

	/* TODO: Define states used by the application state machine. */
	ANALOG_VOLTAGE_MONITOR_STATE_SCAN,
    ANALOG_VOLTAGE_MONITOR_STATE_SCAN_DONE,
    ANALOG_VOLTAGE_MONITOR_STATE_DISPLAY

} ANALOG_VOLTAGE_MONITOR_STATES;


// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    Application strings and buffers are be defined outside this structure.
 */

typedef struct
{
    /* The application's current state */
    ANALOG_VOLTAGE_MONITOR_STATES state;

    /* TODO: Define any additional data used by the application. */
    ADC_SCAN_DATA_TYPE adc_data;

} ANALOG_VOLTAGE_MONITOR_DATA;


// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Routines
// *****************************************************************************
// *****************************************************************************
/* These routines are called by drivers when certain events occur.
*/
	
// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void ANALOG_VOLTAGE_MONITOR_Initialize ( void )

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
    ANALOG_VOLTAGE_MONITOR_Initialize();
    </code>

  Remarks:
    This routine must be called from the SYS_Initialize function.
*/

void ANALOG_VOLTAGE_MONITOR_Initialize ( void );


/*******************************************************************************
  Function:
    void ANALOG_VOLTAGE_MONITOR_Tasks ( void )

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
    ANALOG_VOLTAGE_MONITOR_Tasks();
    </code>

  Remarks:
    This routine must be called from SYS_Tasks() routine.
 */

void ANALOG_VOLTAGE_MONITOR_Tasks( void );


#endif /* _ANALOG_VOLTAGE_MONITOR_H */

//DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
//DOM-IGNORE-END

/*******************************************************************************
 End of File
 */

