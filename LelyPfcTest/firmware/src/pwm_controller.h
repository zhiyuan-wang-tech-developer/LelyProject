/*******************************************************************************
  MPLAB Harmony Application Header File

  Company:
    Microchip Technology Inc.

  File Name:
    pwm_controller.h

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

#ifndef _PWM_CONTROLLER_H
#define _PWM_CONTROLLER_H

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "system_config.h"
#include "system_definitions.h"

// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

extern "C" {

#endif
// DOM-IGNORE-END 


#define PWM_TDO_WORKAROUND

#define PWM_ChanBuck1 MCPWM_CHANNEL9    // 9H
#define PWM_ChanBuck2 MCPWM_CHANNEL3    // 3H
#define PWM_ChanBuck3 MCPWM_CHANNEL7    // 7H
#define PWM_ChanBuck4 MCPWM_CHANNEL1    // 1H
#define PWM_ChanBoost1 MCPWM_CHANNEL2   // 2H
#define PWM_ChanBoost2 MCPWM_CHANNEL8   // 8H
#define PWM_ChanBoost3 MCPWM_CHANNEL10  // 10H

#ifdef PWM_TDO_WORKAROUND
#define PWM_ChanBoost4 MCPWM_CHANNEL11   // 11H (instead of SPI Flash chip select)
#else
#define PWM_ChanBoost4 MCPWM_CHANNEL4   // 4H
#endif
    
    
    
// *****************************************************************************
// *****************************************************************************
// Section: Type Definitions
// *****************************************************************************
// *****************************************************************************
    
typedef union
{
    uint8_t update_status;
    struct
    {
        uint8_t PWM_BUCK1: 1;
        uint8_t PWM_BOOST1: 1;
        uint8_t PWM_BUCK2: 1;
        uint8_t PWM_BOOST2: 1;
        uint8_t PWM_BUCK3: 1;
        uint8_t PWM_BOOST3: 1;
        uint8_t PWM_BUCK4: 1;
        uint8_t PWM_BOOST4: 1;
    } updata_status_flag;
} UPDATE_STATUS_TYPE;

typedef union
{
    uint16_t update_array[8];
    struct
    {
        uint16_t PWM_BUCK1;
        uint16_t PWM_BOOST1;
        uint16_t PWM_BUCK2;
        uint16_t PWM_BOOST2;
        uint16_t PWM_BUCK3;
        uint16_t PWM_BOOST3;
        uint16_t PWM_BUCK4;
        uint16_t PWM_BOOST4;
    } update_value;
} UPDATE_DATA_TYPE;

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
	PWM_CONTROLLER_STATE_INIT=0,
            
	/* TODO: Define states used by the application state machine. */
    // In RUN state, stable PWM signals with fixed duty cycle and phase shift are output.
    PWM_CONTROLLER_STATE_RUN,
    // In UPDATE state, the duty cycle register and phase shift register are updated.
    PWM_CONTROLLER_STATE_UPDATE,
    // In STOP state, NO PWM signals are output on the channels.
    PWM_CONTROLLER_STATE_STOP
} PWM_CONTROLLER_STATES;

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
    PWM_CONTROLLER_STATES state;

    /* TODO: Define any additional data used by the application. */
    // If updateFlag is true, PWM controller's next state is UPDATE.
    bool updateFlag;
    // If stopFlag is true, PWM controller's next state is STOP.
    bool stopFlag;
    UPDATE_STATUS_TYPE dutyCycleStatus;
    UPDATE_STATUS_TYPE phaseShiftStatus;
    UPDATE_DATA_TYPE dutyCycle;
    UPDATE_DATA_TYPE phaseShift;
} PWM_CONTROLLER_DATA;


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
    void PWM_CONTROLLER_Initialize ( void )

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
    PWM_CONTROLLER_Initialize();
    </code>

  Remarks:
    This routine must be called from the SYS_Initialize function.
*/

void PWM_CONTROLLER_Initialize ( void );


/*******************************************************************************
  Function:
    void PWM_CONTROLLER_Tasks ( void )

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
    PWM_CONTROLLER_Tasks();
    </code>

  Remarks:
    This routine must be called from SYS_Tasks() routine.
 */

void PWM_CONTROLLER_Tasks( void );



// Added for tests
void PWM_SIGNAL_Update ( void );

void PWM_BOOST1_DutyCycleSet(uint16_t dutyCycle);
void PWM_BOOST2_DutyCycleSet(uint16_t dutyCycle);
void PWM_BOOST3_DutyCycleSet(uint16_t dutyCycle);
void PWM_BOOST4_DutyCycleSet(uint16_t dutyCycle);

void PWM_BUCK1_DutyCycleSet(uint16_t dutyCycle);
void PWM_BUCK2_DutyCycleSet(uint16_t dutyCycle);
void PWM_BUCK3_DutyCycleSet(uint16_t dutyCycle);
void PWM_BUCK4_DutyCycleSet(uint16_t dutyCycle);

void PWM_BOOST1_PhaseShiftSet(uint16_t phaseShift);
void PWM_BOOST2_PhaseShiftSet(uint16_t phaseShift);
void PWM_BOOST3_PhaseShiftSet(uint16_t phaseShift);
void PWM_BOOST4_PhaseShiftSet(uint16_t phaseShift);

void PWM_BUCK1_PhaseShiftSet(uint16_t phaseShift);
void PWM_BUCK2_PhaseShiftSet(uint16_t phaseShift);
void PWM_BUCK3_PhaseShiftSet(uint16_t phaseShift);
void PWM_BUCK4_PhaseShiftSet(uint16_t phaseShift);

#endif /* _PWM_CONTROLLER_H */

//DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
//DOM-IGNORE-END

/*******************************************************************************
 End of File
 */

