/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    pwm_controller.c

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

#include "pwm_controller.h"
#include "driver/mcpwm/drv_mcpwm_static.h"



/*
 * Macro Definitions for PWM Controller
 */
// PWM switching frequency FPWM = 150kHz
#define PWM_FREQ    150000

// one full period is 65535 in PWM time base register
// 1% duty cycle = 65535/100 = 655.35
float DUTY_CYCLE_STEP = 655.35;

// 1° phase shift =  65535/360 = 182.04
float PHASE_SHIFT_STEP = 182.04;

// Initial Duty Cycle (%)
uint16_t PWM_BUCK1_DUTY_CYCLE_INIT = 25;
uint16_t PWM_BUCK2_DUTY_CYCLE_INIT = 25;
uint16_t PWM_BUCK3_DUTY_CYCLE_INIT = 25;
uint16_t PWM_BUCK4_DUTY_CYCLE_INIT = 25;
uint16_t PWM_BOOST1_DUTY_CYCLE_INIT = 25;
uint16_t PWM_BOOST2_DUTY_CYCLE_INIT = 25;
uint16_t PWM_BOOST3_DUTY_CYCLE_INIT = 25;
uint16_t PWM_BOOST4_DUTY_CYCLE_INIT = 25;

// Initial Phase Shift (°)
uint16_t PWM_BUCK1_PHASE_SHIFT_INIT = 10;
uint16_t PWM_BUCK2_PHASE_SHIFT_INIT = 190;
uint16_t PWM_BUCK3_PHASE_SHIFT_INIT = 100;
uint16_t PWM_BUCK4_PHASE_SHIFT_INIT = 280;
uint16_t PWM_BOOST1_PHASE_SHIFT_INIT = 0;
uint16_t PWM_BOOST2_PHASE_SHIFT_INIT = 180;
uint16_t PWM_BOOST3_PHASE_SHIFT_INIT = 90;
uint16_t PWM_BOOST4_PHASE_SHIFT_INIT = 270;

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

PWM_CONTROLLER_DATA pwm_controllerData;

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
 * Function:
 *      to set PWM signal's duty cycle and phase shift
 * 
 * Remarks:
 *      PWM_BUCKx (x = 1...4) decreases the current supply from +325V
 *      PWM_BOOSTx (x = 1...4) increases the current supply from +325V
 *      dutyCycle = 0 ~ 100
 *      phaseShift = 0° ~ 359° 
 */
// PWM_BUCK1 <-> PWM9H 
void PWM_BUCK1_DutyCycleSet(uint16_t dutyCycle)
{
//    dutyCycle = dutyCycle * DUTY_CYCLE_STEP;
    PLIB_MCPWM_ChannelPrimaryDutyCycleSet (MCPWM_ID_0 , PWM_ChanBuck1 , dutyCycle);
}
void PWM_BUCK1_PhaseShiftSet(uint16_t phaseShift)
{
//    phaseShift = phaseShift * PHASE_SHIFT_STEP;
    PLIB_MCPWM_ChannelPhaseSet (MCPWM_ID_0 , PWM_ChanBuck1 , phaseShift);
}

// PWM_BOOST1 <-> PWM2H
void PWM_BOOST1_DutyCycleSet(uint16_t dutyCycle)
{
//    dutyCycle = dutyCycle * DUTY_CYCLE_STEP;
    PLIB_MCPWM_ChannelPrimaryDutyCycleSet (MCPWM_ID_0 , PWM_ChanBoost1 , dutyCycle);
}
void PWM_BOOST1_PhaseShiftSet(uint16_t phaseShift)
{
//    phaseShift = phaseShift * PHASE_SHIFT_STEP;
    PLIB_MCPWM_ChannelPhaseSet (MCPWM_ID_0 , PWM_ChanBoost1 , phaseShift);
}

// PWM_BUCK2 <-> PWM3H 
void PWM_BUCK2_DutyCycleSet(uint16_t dutyCycle)
{
//    dutyCycle = dutyCycle * DUTY_CYCLE_STEP;
    PLIB_MCPWM_ChannelPrimaryDutyCycleSet (MCPWM_ID_0 , PWM_ChanBuck2 , dutyCycle);
}
void PWM_BUCK2_PhaseShiftSet(uint16_t phaseShift)
{
//    phaseShift = phaseShift * PHASE_SHIFT_STEP;
    PLIB_MCPWM_ChannelPhaseSet (MCPWM_ID_0 , PWM_ChanBuck2 , phaseShift);
}

// PWM_BOOST2 <-> PWM8H
void PWM_BOOST2_DutyCycleSet(uint16_t dutyCycle)
{
//    dutyCycle = dutyCycle * DUTY_CYCLE_STEP;
    PLIB_MCPWM_ChannelPrimaryDutyCycleSet (MCPWM_ID_0 , PWM_ChanBoost2 , dutyCycle);
}
void PWM_BOOST2_PhaseShiftSet(uint16_t phaseShift)
{
    phaseShift = phaseShift * PHASE_SHIFT_STEP;
    PLIB_MCPWM_ChannelPhaseSet (MCPWM_ID_0 , PWM_ChanBoost2, phaseShift);
}

// PWM_BUCK3 <-> PWM7H 
void PWM_BUCK3_DutyCycleSet(uint16_t dutyCycle)
{
//    dutyCycle = dutyCycle * DUTY_CYCLE_STEP;
    PLIB_MCPWM_ChannelPrimaryDutyCycleSet (MCPWM_ID_0 , PWM_ChanBuck3 , dutyCycle);
}
void PWM_BUCK3_PhaseShiftSet(uint16_t phaseShift)
{
//    phaseShift = phaseShift * PHASE_SHIFT_STEP;
    PLIB_MCPWM_ChannelPhaseSet (MCPWM_ID_0 , PWM_ChanBuck3, phaseShift);
}

// PWM_BOOST3 <-> PWM10H
void PWM_BOOST3_DutyCycleSet(uint16_t dutyCycle)
{
//    dutyCycle = dutyCycle * DUTY_CYCLE_STEP;
    PLIB_MCPWM_ChannelPrimaryDutyCycleSet (MCPWM_ID_0 , PWM_ChanBoost3, dutyCycle);
}
void PWM_BOOST3_PhaseShiftSet(uint16_t phaseShift)
{
//    phaseShift = phaseShift * PHASE_SHIFT_STEP;
    PLIB_MCPWM_ChannelPhaseSet (MCPWM_ID_0 , PWM_ChanBoost3, phaseShift);
}

// PWM_BUCK4 <-> PWM1H 
void PWM_BUCK4_DutyCycleSet(uint16_t dutyCycle)
{
//    dutyCycle = dutyCycle * DUTY_CYCLE_STEP;
    PLIB_MCPWM_ChannelPrimaryDutyCycleSet (MCPWM_ID_0 , PWM_ChanBuck4, dutyCycle);
}
void PWM_BUCK4_PhaseShiftSet(uint16_t phaseShift)
{
    phaseShift = phaseShift * PHASE_SHIFT_STEP;
    PLIB_MCPWM_ChannelPhaseSet (MCPWM_ID_0 , PWM_ChanBuck4, phaseShift);
}

// PWM_BOOST4 <-> PWM4H
void PWM_BOOST4_DutyCycleSet(uint16_t dutyCycle)
{
//    dutyCycle = dutyCycle * DUTY_CYCLE_STEP;
    PLIB_MCPWM_ChannelPrimaryDutyCycleSet (MCPWM_ID_0 , PWM_ChanBoost4, dutyCycle);
}
void PWM_BOOST4_PhaseShiftSet(uint16_t phaseShift)
{
//    phaseShift = phaseShift * PHASE_SHIFT_STEP;
    PLIB_MCPWM_ChannelPhaseSet (MCPWM_ID_0 , PWM_ChanBoost4, phaseShift);
}

/*******************************************************************************
 * Function:
 *      To set the initial duty cycles and phase shifts for all PWM signals
 * 
 * Remarks:
 *      The PWM signals should be configured individually;
 *      The duty cycles are shared in the pairs PWM-BUCK1/2, PWM-BUCK3/4, PWM-BOOST1/2,  PWM-BOOST3/4; 
 *      The initial phase shifts are: 
 *          PWM-BUCK1: 0°,      PWM-BOOST1: 10° 
 *          PWM-BUCK2: 180°,    PWM-BOOST2: 190° 
 *          PWM-BUCK3: 90°,     PWM-BOOST3: 100° 
 *          PWM-BUCK4: 270°,    PWM-BOOST4: 280°
 *      	
 */
void PWM_SIGNAL_Initialize ( void )
{
    PWM_BUCK1_DutyCycleSet(PWM_BUCK1_DUTY_CYCLE_INIT);
    PWM_BUCK2_DutyCycleSet(PWM_BUCK2_DUTY_CYCLE_INIT);
    PWM_BUCK3_DutyCycleSet(PWM_BUCK3_DUTY_CYCLE_INIT);
    PWM_BUCK4_DutyCycleSet(PWM_BUCK4_DUTY_CYCLE_INIT);
    
    PWM_BOOST1_DutyCycleSet(PWM_BOOST1_DUTY_CYCLE_INIT);
    PWM_BOOST2_DutyCycleSet(PWM_BOOST2_DUTY_CYCLE_INIT);
    PWM_BOOST3_DutyCycleSet(PWM_BOOST3_DUTY_CYCLE_INIT);
    PWM_BOOST4_DutyCycleSet(PWM_BOOST4_DUTY_CYCLE_INIT);
    
    PWM_BUCK1_PhaseShiftSet(PWM_BUCK1_PHASE_SHIFT_INIT);
    PWM_BUCK2_PhaseShiftSet(PWM_BUCK2_PHASE_SHIFT_INIT);
    PWM_BUCK3_PhaseShiftSet(PWM_BUCK3_PHASE_SHIFT_INIT);    
    PWM_BUCK4_PhaseShiftSet(PWM_BUCK4_PHASE_SHIFT_INIT);

    PWM_BOOST1_PhaseShiftSet(PWM_BOOST1_PHASE_SHIFT_INIT);
    PWM_BOOST2_PhaseShiftSet(PWM_BOOST2_PHASE_SHIFT_INIT);
    PWM_BOOST3_PhaseShiftSet(PWM_BOOST3_PHASE_SHIFT_INIT);
    PWM_BOOST4_PhaseShiftSet(PWM_BOOST4_PHASE_SHIFT_INIT);
}

/*******************************************************************************
 * Function:
 *      To update the duty cycles and phase shifts for the requested PWM channels
 * 
 * Remarks:
 *      The duty cycles are shared in the pairs PWM-BUCK1/2, PWM-BUCK3/4, PWM-BOOST1/2,  PWM-BOOST3/4;
 *      	
 */
void PWM_SIGNAL_Update ( void )
{
    // update duty cycles
    if( pwm_controllerData.dutyCycleStatus.update_status != 0 )
    {
        if( pwm_controllerData.dutyCycleStatus.updata_status_flag.PWM_BUCK1 ) PWM_BUCK1_DutyCycleSet( pwm_controllerData.dutyCycle.update_value.PWM_BUCK1 );
        if( pwm_controllerData.dutyCycleStatus.updata_status_flag.PWM_BUCK2 ) PWM_BUCK2_DutyCycleSet( pwm_controllerData.dutyCycle.update_value.PWM_BUCK2 );
        if( pwm_controllerData.dutyCycleStatus.updata_status_flag.PWM_BUCK3 ) PWM_BUCK3_DutyCycleSet( pwm_controllerData.dutyCycle.update_value.PWM_BUCK3 );
        if( pwm_controllerData.dutyCycleStatus.updata_status_flag.PWM_BUCK4 ) PWM_BUCK4_DutyCycleSet( pwm_controllerData.dutyCycle.update_value.PWM_BUCK4 );
        if( pwm_controllerData.dutyCycleStatus.updata_status_flag.PWM_BOOST1 ) PWM_BOOST1_DutyCycleSet( pwm_controllerData.dutyCycle.update_value.PWM_BOOST1 );
        if( pwm_controllerData.dutyCycleStatus.updata_status_flag.PWM_BOOST2 ) PWM_BOOST2_DutyCycleSet( pwm_controllerData.dutyCycle.update_value.PWM_BOOST2 );
        if( pwm_controllerData.dutyCycleStatus.updata_status_flag.PWM_BOOST3 ) PWM_BOOST3_DutyCycleSet( pwm_controllerData.dutyCycle.update_value.PWM_BOOST3 );
        if( pwm_controllerData.dutyCycleStatus.updata_status_flag.PWM_BOOST4 ) PWM_BOOST4_DutyCycleSet( pwm_controllerData.dutyCycle.update_value.PWM_BOOST4 );
    }
    // update phase shifts
    if( pwm_controllerData.phaseShiftStatus.update_status != 0 )
    {
        if( pwm_controllerData.phaseShiftStatus.updata_status_flag.PWM_BUCK1 ) PWM_BUCK1_PhaseShiftSet( pwm_controllerData.phaseShift.update_value.PWM_BUCK1 );
        if( pwm_controllerData.phaseShiftStatus.updata_status_flag.PWM_BUCK2 ) PWM_BUCK2_PhaseShiftSet( pwm_controllerData.phaseShift.update_value.PWM_BUCK2 );
        if( pwm_controllerData.phaseShiftStatus.updata_status_flag.PWM_BUCK3 ) PWM_BUCK3_PhaseShiftSet( pwm_controllerData.phaseShift.update_value.PWM_BUCK3 );
        if( pwm_controllerData.phaseShiftStatus.updata_status_flag.PWM_BUCK4 ) PWM_BUCK4_PhaseShiftSet( pwm_controllerData.phaseShift.update_value.PWM_BUCK4 );
        if( pwm_controllerData.phaseShiftStatus.updata_status_flag.PWM_BOOST1 ) PWM_BOOST1_PhaseShiftSet( pwm_controllerData.phaseShift.update_value.PWM_BOOST1 );
        if( pwm_controllerData.phaseShiftStatus.updata_status_flag.PWM_BOOST2 ) PWM_BOOST2_PhaseShiftSet( pwm_controllerData.phaseShift.update_value.PWM_BOOST2 );
        if( pwm_controllerData.phaseShiftStatus.updata_status_flag.PWM_BOOST3 ) PWM_BOOST3_PhaseShiftSet( pwm_controllerData.phaseShift.update_value.PWM_BOOST3 );
        if( pwm_controllerData.phaseShiftStatus.updata_status_flag.PWM_BOOST4 ) PWM_BOOST4_PhaseShiftSet( pwm_controllerData.phaseShift.update_value.PWM_BOOST4 );
    }
    // clear duty cycle update status
    pwm_controllerData.dutyCycleStatus.update_status = 0;

    // clear phase shift update status
    pwm_controllerData.phaseShiftStatus.update_status = 0;
}

// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void PWM_CONTROLLER_Initialize ( void )

  Remarks:
    See prototype in pwm_controller.h.
 */

void PWM_CONTROLLER_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    pwm_controllerData.state = PWM_CONTROLLER_STATE_INIT;
    
    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
    pwm_controllerData.updateFlag = false;
    pwm_controllerData.stopFlag = false;
    pwm_controllerData.dutyCycleStatus.update_status = 0;
    pwm_controllerData.phaseShiftStatus.update_status = 0;
    memset(pwm_controllerData.dutyCycle.update_array, 0, 16); 
    memset(pwm_controllerData.phaseShift.update_array, 0, 16);
}


/******************************************************************************
  Function:
    void PWM_CONTROLLER_Tasks ( void )

  Remarks:
    See prototype in pwm_controller.h.
 */

void PWM_CONTROLLER_Tasks ( void )
{

    /* Check the application's current state. */
    switch ( pwm_controllerData.state )
    {
        /* Application's initial state. */
        case PWM_CONTROLLER_STATE_INIT:
        {
            bool appInitialized = true;
        
            if (appInitialized)
            {
            
                pwm_controllerData.state = PWM_CONTROLLER_STATE_RUN;
                // Initialize all PWM channels with duty cycles and phase shifts
                PWM_SIGNAL_Initialize();
                // Enable MCPWM module output
                DRV_MCPWM_Enable();
            }
            break;
        }

        /* TODO: implement your application state machine.*/
        case PWM_CONTROLLER_STATE_RUN:
        {
            // Temporary Test for Logic Analyzer. Now it is useless!
//            int i = 0;
//            V_LED1_GOn();
//            for(i=0; i<100; i++);
//            V_LED1_GOff();
//            for(i=0; i<200; i++);
//            SYS_DEBUG_BreakPoint();
            if( pwm_controllerData.updateFlag )
            {
                pwm_controllerData.state = PWM_CONTROLLER_STATE_UPDATE;
            }
            else if( pwm_controllerData.stopFlag )
            {
                pwm_controllerData.state = PWM_CONTROLLER_STATE_STOP;
            }
            else
            {
                pwm_controllerData.state = PWM_CONTROLLER_STATE_RUN;
            }
            break;
        }

        case PWM_CONTROLLER_STATE_UPDATE:
        {
            if( pwm_controllerData.updateFlag )
            {
                // update the MCPWM module's duty cycle and phase shift registers 
                PWM_SIGNAL_Update();
                pwm_controllerData.updateFlag = false;
            }
            pwm_controllerData.state = PWM_CONTROLLER_STATE_RUN;
//            SYS_DEBUG_BreakPoint();
            break;
        }

        case PWM_CONTROLLER_STATE_STOP:
        {
            // The stop flag is set somewhere externally!
            if( pwm_controllerData.stopFlag )
            {
                // Disable MCPWM module output
                DRV_MCPWM_Disable();
                pwm_controllerData.state = PWM_CONTROLLER_STATE_STOP;
            }
            else
            {
                // Enable MCPWM module output
                DRV_MCPWM_Enable();
                pwm_controllerData.state = PWM_CONTROLLER_STATE_RUN;
            }
        }        

        /* The default state should never be executed. */
        default:
        {
            /* TODO: Handle error in application's state machine. */
            SYS_PRINT("\nFault: PWM Controller in WRONG state!\r\n");
            /* Return to the initial state. */
            pwm_controllerData.state = PWM_CONTROLLER_STATE_INIT;
            break;
        }
    }
}


/*******************************************************************************
 End of File
 */
