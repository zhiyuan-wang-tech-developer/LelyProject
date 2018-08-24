/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    led_controller.c

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

#include "led_controller.h"
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

LED_CONTROLLER_DATA led_controllerData;

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

/* Application's Timer Callback Function */
static void TimerCallback (  uintptr_t context, uint32_t alarmCount )
{
    led_controllerData.timerCallbackCount++;

    if (led_controllerData.timerCallbackCount >= LED_CONTROLLER_TIMER_CALLBACKS_PER_EVENT)
    {
        led_controllerData.timerCallbackCount = 0;
        
        global_events.tmr_drv_led_200ms_blink_event = true;  
    }        
}

/* TODO:  Add any necessary callback functions.
*/

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************


/* Application's Timer Setup Function */
static void TimerSetup( void )
{
    DRV_TMR_AlarmRegister(
        led_controllerData.handleLedTimer0, 
        LED_CONTROLLER_TMR_DRV_PERIOD, 
        LED_CONTROLLER_TMR_DRV_IS_PERIODIC,
        (uintptr_t)NULL, 
        TimerCallback);
    DRV_TMR_Start(led_controllerData.handleLedTimer0);
}

/* TODO:  Add any necessary local functions.
*/

/*
 * Led control functions for LED1, LED2 and LED3
 */
// LED 1 functions
void LED1RedOn(void)
{
    V_LED1_ROn();
}
void LED1RedOff(void)
{
    V_LED1_ROff();
}
void LED1GreenOn(void)
{
    V_LED1_GOn();
}
void LED1GreenOff(void)
{
    V_LED1_GOff();
}
void LED1OrangeOn(void)
{
    V_LED1_ROn();
    V_LED1_GOn();
}
void LED1OrangeOff(void)
{
    V_LED1_ROff();
    V_LED1_GOff();
}
/*
 * It controls the LED blinking pattern.
 * 
 * LED 1 Blinking Pattern:
 *                  200ms | 200ms | 200ms | 200ms | 200ms
 * LED 1 Green  :   ON      OFF     OFF     OFF     OFF
 * LED 1 Red    :   ON      OFF     ON      ON      OFF
 * LED 1 Orange :   ON      OFF     ON      ON      OFF
 */
void LED1Blink(void)
{
    switch ( led_controllerData.led200msBlinkCount )
    {
        case 0:
        {
            if( led_controllerData.led1.status_flags.GreenIsBlinking )
            {
                LED1GreenOn();
            }
            if( led_controllerData.led1.status_flags.RedIsBlinking )
            {
                LED1RedOn();
            }
            if( led_controllerData.led1.status_flags.OrangeIsBlinking )
            {
                LED1OrangeOn();
            }           
            break;
        }
        
        case 1:
        {
            if( led_controllerData.led1.status_flags.GreenIsBlinking )
            {
                LED1GreenOff();
            }
            if( led_controllerData.led1.status_flags.RedIsBlinking )
            {
                LED1RedOff();
            }
            if( led_controllerData.led1.status_flags.OrangeIsBlinking )
            {
                LED1OrangeOff();
            }
            break;
        }
        
        case 2:
        {
            if( led_controllerData.led1.status_flags.GreenIsBlinking )
            {
                LED1GreenOff();
            }
            if( led_controllerData.led1.status_flags.RedIsBlinking )
            {
                LED1RedOn();
            }
            if( led_controllerData.led1.status_flags.OrangeIsBlinking )
            {
                LED1OrangeOn();
            }     
            break;
        }
        
        case 3:
        {
            if( led_controllerData.led1.status_flags.GreenIsBlinking )
            {
                LED1GreenOff();
            }
            if( led_controllerData.led1.status_flags.RedIsBlinking )
            {
                LED1RedOn();
            }
            if( led_controllerData.led1.status_flags.OrangeIsBlinking )
            {
                LED1OrangeOn();
            }                        
            break;
        }
        
        case 4:
        {
            if( led_controllerData.led1.status_flags.GreenIsBlinking )
            {
                LED1GreenOff();
            }
            if( led_controllerData.led1.status_flags.RedIsBlinking )
            {
                LED1RedOff();
            }
            if( led_controllerData.led1.status_flags.OrangeIsBlinking )
            {
                LED1OrangeOff();
            }                        
            break;
        }
        
        default:
        {
            led_controllerData.led200msBlinkCount = 0;
            break;
        }
    }
}

// LED 2 functions
void LED2RedOn(void)
{
    V_LED2_ROn();
}
void LED2RedOff(void)
{
    V_LED2_ROff();
}
void LED2GreenOn(void)
{
    V_LED2_GOn();
}
void LED2GreenOff(void)
{
    V_LED2_GOff();
}
void LED2OrangeOn(void)
{
    V_LED2_ROn();
    V_LED2_GOn();
}
void LED2OrangeOff(void)
{
    V_LED2_ROff();
    V_LED2_GOff();
}
/*
 * It controls the LED blinking pattern.
 * 
 * LED 2 Blinking Pattern:
 *                  200ms | 200ms | 200ms | 200ms | 200ms
 * LED 2 Green  :   ON      OFF     OFF     OFF     OFF
 * LED 2 Red    :   ON      OFF     ON      ON      OFF
 * LED 2 Orange :   ON      OFF     ON      ON      OFF
 */
void LED2Blink(void)
{
    switch ( led_controllerData.led200msBlinkCount )
    {
        case 0:
        {
            if( led_controllerData.led2.status_flags.GreenIsBlinking )
            {
                LED2GreenOn();
            }
            if( led_controllerData.led2.status_flags.RedIsBlinking )
            {
                LED2RedOn();
            }
            if( led_controllerData.led2.status_flags.OrangeIsBlinking )
            {
                LED2OrangeOn();
            }           
            break;
        }
        
        case 1:
        {
            if( led_controllerData.led2.status_flags.GreenIsBlinking )
            {
                LED2GreenOff();
            }
            if( led_controllerData.led2.status_flags.RedIsBlinking )
            {
                LED2RedOff();
            }
            if( led_controllerData.led2.status_flags.OrangeIsBlinking )
            {
                LED2OrangeOff();
            }
            break;
        }
        
        case 2:
        {
            if( led_controllerData.led2.status_flags.GreenIsBlinking )
            {
                LED2GreenOff();
            }
            if( led_controllerData.led2.status_flags.RedIsBlinking )
            {
                LED2RedOn();
            }
            if( led_controllerData.led2.status_flags.OrangeIsBlinking )
            {
                LED2OrangeOn();
            }     
            break;
        }
        
        case 3:
        {
            if( led_controllerData.led2.status_flags.GreenIsBlinking )
            {
                LED2GreenOff();
            }
            if( led_controllerData.led2.status_flags.RedIsBlinking )
            {
                LED2RedOn();
            }
            if( led_controllerData.led2.status_flags.OrangeIsBlinking )
            {
                LED2OrangeOn();
            }                        
            break;
        }
        
        case 4:
        {
            if( led_controllerData.led2.status_flags.GreenIsBlinking )
            {
                LED2GreenOff();
            }
            if( led_controllerData.led2.status_flags.RedIsBlinking )
            {
                LED2RedOff();
            }
            if( led_controllerData.led2.status_flags.OrangeIsBlinking )
            {
                LED2OrangeOff();
            }                        
            break;
        }
        
        default:
        {
            led_controllerData.led200msBlinkCount = 0;
            break;
        }
    }
}

// LED 3 functions
void LED3RedOn(void)
{
    V_LED3_ROn();
}
void LED3RedOff(void)
{
    V_LED3_ROff();
}
void LED3GreenOn(void)
{
    V_LED3_GOn();
}
void LED3GreenOff(void)
{
    V_LED3_GOff();
}
void LED3OrangeOn(void)
{
    V_LED3_ROn();
    V_LED3_GOn();
}
void LED3OrangeOff(void)
{
    V_LED3_ROff();
    V_LED3_GOff();
}
/*
 * It controls the LED blinking pattern.
 * 
 * LED 3 Blinking Pattern:
 *                  200ms | 200ms | 200ms | 200ms | 200ms
 * LED 3 Green  :   ON      OFF     OFF     OFF     OFF
 * LED 3 Red    :   ON      OFF     ON      ON      OFF
 * LED 3 Orange :   ON      OFF     ON      ON      OFF
 */
void LED3Blink(void)
{
    switch ( led_controllerData.led200msBlinkCount )
    {
        case 0:
        {
            if( led_controllerData.led3.status_flags.GreenIsBlinking )
            {
                LED3GreenOn();
            }
            if( led_controllerData.led3.status_flags.RedIsBlinking )
            {
                LED3RedOn();
            }
            if( led_controllerData.led3.status_flags.OrangeIsBlinking )
            {
                LED3OrangeOn();
            }           
            break;
        }
        
        case 1:
        {
            if( led_controllerData.led3.status_flags.GreenIsBlinking )
            {
                LED3GreenOff();
            }
            if( led_controllerData.led3.status_flags.RedIsBlinking )
            {
                LED3RedOff();
            }
            if( led_controllerData.led3.status_flags.OrangeIsBlinking )
            {
                LED3OrangeOff();
            }
            break;
        }
        
        case 2:
        {
            if( led_controllerData.led3.status_flags.GreenIsBlinking )
            {
                LED3GreenOff();
            }
            if( led_controllerData.led3.status_flags.RedIsBlinking )
            {
                LED3RedOn();
            }
            if( led_controllerData.led3.status_flags.OrangeIsBlinking )
            {
                LED3OrangeOn();
            }     
            break;
        }
        
        case 3:
        {
            if( led_controllerData.led3.status_flags.GreenIsBlinking )
            {
                LED3GreenOff();
            }
            if( led_controllerData.led3.status_flags.RedIsBlinking )
            {
                LED3RedOn();
            }
            if( led_controllerData.led3.status_flags.OrangeIsBlinking )
            {
                LED3OrangeOn();
            }                        
            break;
        }
        
        case 4:
        {
            if( led_controllerData.led3.status_flags.GreenIsBlinking )
            {
                LED3GreenOff();
            }
            if( led_controllerData.led3.status_flags.RedIsBlinking )
            {
                LED3RedOff();
            }
            if( led_controllerData.led3.status_flags.OrangeIsBlinking )
            {
                LED3OrangeOff();
            }                        
            break;
        }
        
        default:
        {
            led_controllerData.led200msBlinkCount = 0;
            break;
        }
    }
}

/*
 * Set LED blinking color to display system states (Normal, Warning, Fault)
 * 
 * Normal state -> Green is blinking
 * Warning state -> Orange is blinking
 * Fault state -> Red is blinking
 * 
 */
// For LED 1 blink setting
void setLed1NormalBlink(void)
{
    led_controllerData.led1.status_flags.GreenIsBlinking = YES;
    led_controllerData.led1.status_flags.OrangeIsBlinking = NO;
    led_controllerData.led1.status_flags.RedIsBlinking = NO;
}
void setLed1WarningBlink(void)
{
    led_controllerData.led1.status_flags.GreenIsBlinking = NO;
    led_controllerData.led1.status_flags.OrangeIsBlinking = YES;
    led_controllerData.led1.status_flags.RedIsBlinking = NO;    
}
void setLed1FaultBlink(void)
{
    led_controllerData.led1.status_flags.GreenIsBlinking = NO;
    led_controllerData.led1.status_flags.OrangeIsBlinking = NO;
    led_controllerData.led1.status_flags.RedIsBlinking = YES;    
}
// For LED 2 blink setting
void setLed2NormalBlink(void)
{
    led_controllerData.led2.status_flags.GreenIsBlinking = YES;
    led_controllerData.led2.status_flags.OrangeIsBlinking = NO;
    led_controllerData.led2.status_flags.RedIsBlinking = NO;
}
void setLed2WarningBlink(void)
{
    led_controllerData.led2.status_flags.GreenIsBlinking = NO;
    led_controllerData.led2.status_flags.OrangeIsBlinking = YES;
    led_controllerData.led2.status_flags.RedIsBlinking = NO;    
}
void setLed2FaultBlink(void)
{
    led_controllerData.led2.status_flags.GreenIsBlinking = NO;
    led_controllerData.led2.status_flags.OrangeIsBlinking = NO;
    led_controllerData.led2.status_flags.RedIsBlinking = YES;    
}
// For LED 3 blink setting
void setLed3NormalBlink(void)
{
    led_controllerData.led3.status_flags.GreenIsBlinking = YES;
    led_controllerData.led3.status_flags.OrangeIsBlinking = NO;
    led_controllerData.led3.status_flags.RedIsBlinking = NO;
}
void setLed3WarningBlink(void)
{
    led_controllerData.led3.status_flags.GreenIsBlinking = NO;
    led_controllerData.led3.status_flags.OrangeIsBlinking = YES;
    led_controllerData.led3.status_flags.RedIsBlinking = NO;    
}
void setLed3FaultBlink(void)
{
    led_controllerData.led3.status_flags.GreenIsBlinking = NO;
    led_controllerData.led3.status_flags.OrangeIsBlinking = NO;
    led_controllerData.led3.status_flags.RedIsBlinking = YES;    
}

// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void LED_CONTROLLER_Initialize ( void )

  Remarks:
    See prototype in led_controller.h.
 */

void LED_CONTROLLER_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    led_controllerData.state = LED_CONTROLLER_STATE_INIT;

    led_controllerData.handleLedTimer0 = DRV_HANDLE_INVALID;
    led_controllerData.timerCallbackCount = 0;
    
    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
    led_controllerData.led200msBlinkCount = 0;
    led_controllerData.led1.status = 0;
    led_controllerData.led2.status = 0;
    led_controllerData.led3.status = 0;
}


/******************************************************************************
  Function:
    void LED_CONTROLLER_Tasks ( void )

  Remarks:
    See prototype in led_controller.h.
 */

void LED_CONTROLLER_Tasks ( void )
{

    /* Check the application's current state. */
    switch ( led_controllerData.state )
    {
        /* Application's initial state. */
        case LED_CONTROLLER_STATE_INIT:
        {
            bool appInitialized = true;
       
            if (led_controllerData.handleLedTimer0 == DRV_HANDLE_INVALID)
            {
                led_controllerData.handleLedTimer0 = DRV_TMR_Open(LED_CONTROLLER_TMR_DRV, DRV_IO_INTENT_EXCLUSIVE);
                appInitialized &= ( DRV_HANDLE_INVALID != led_controllerData.handleLedTimer0 );
            }
        
            if (appInitialized)
            {
                TimerSetup();
                
                global_events.tmr_drv_led_200ms_blink_event = false;
                
                // After the initialization, all green LEDs are blinking. 
                setLed1NormalBlink();
                setLed2NormalBlink();
                setLed3NormalBlink();
                
                led_controllerData.state = LED_CONTROLLER_STATE_BLINK_LED;
            }
            break;
        }

        case LED_CONTROLLER_STATE_BLINK_LED:
        {
            if( global_event_triggered(&(global_events.tmr_drv_led_200ms_blink_event)) )
            {
                led_controllerData.led200msBlinkCount++;
                // The Led 200ms Blink Count increments from 0 ~ 4;
                // one blinking period = 1 second or 5*200ms
                led_controllerData.led200msBlinkCount = led_controllerData.led200msBlinkCount % 5;
//                SYS_DEBUG_BreakPoint();
            }
            LED1Blink();
            LED2Blink();
            LED3Blink();
            led_controllerData.state = LED_CONTROLLER_STATE_BLINK_LED;
            break;
        }

        /* TODO: implement your application state machine.*/
        

        /* The default state should never be executed. */
        default:
        {
            /* TODO: Handle error in application's state machine. */
            SYS_PRINT("\nFault: LED Controller in WRONG state!\r\n");
            /* Return to the initial state. */
            led_controllerData.state = LED_CONTROLLER_STATE_INIT;
            break;
        }
    }
}

 

/*******************************************************************************
 End of File
 */
