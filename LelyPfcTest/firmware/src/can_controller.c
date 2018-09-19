/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    can_controller.c

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

#include "can_controller.h"
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

CAN_CONTROLLER_DATA can_controllerData;
static uint8_t can_controller_can_tx_buffer[] = "DynaTron";
static uint8_t can_controller_can_rx_buffer[20];

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

/* state machine for the CAN */
static void CAN_Task(void)
{
    /* run the state machine here for CAN */
    switch (can_controllerData.canStateMachine)
    {
        default:
        case CAN_CONTROLLER_CAN_STATE_START:
            if (DRV_CAN_ChannelMessageTransmit(
                can_controllerData.handleCAN0,
                CAN_CHANNEL0,  /* default channel 0 */
                0,  /* given address */
                sizeof(can_controller_can_tx_buffer),
                can_controller_can_tx_buffer) == true)
            {
                can_controllerData.canStateMachine = CAN_CONTROLLER_CAN_STATE_RX;
            }
        break;

        case CAN_CONTROLLER_CAN_STATE_RX:
            if (DRV_CAN_ChannelMessageReceive(
                can_controllerData.handleCAN0,
                CAN_CHANNEL1,  /* default channel 0 */
                0,  /* given address */
                20,
                can_controller_can_rx_buffer) == true)
            {
                can_controllerData.canStateMachine = CAN_CONTROLLER_CAN_STATE_DONE;
            }
        break;

        case CAN_CONTROLLER_CAN_STATE_DONE:
        break;
    }
}


/* TODO:  Add any necessary local functions.
*/


// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void CAN_CONTROLLER_Initialize ( void )

  Remarks:
    See prototype in can_controller.h.
 */

void CAN_CONTROLLER_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    can_controllerData.state = CAN_CONTROLLER_STATE_INIT;

    can_controllerData.handleCAN0 = DRV_HANDLE_INVALID;

    
    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
}


/******************************************************************************
  Function:
    void CAN_CONTROLLER_Tasks ( void )

  Remarks:
    See prototype in can_controller.h.
 */

void CAN_CONTROLLER_Tasks ( void )
{

    /* Check the application's current state. */
    switch ( can_controllerData.state )
    {
        /* Application's initial state. */
        case CAN_CONTROLLER_STATE_INIT:
        {
            bool appInitialized = true;
       
            if (DRV_HANDLE_INVALID == can_controllerData.handleCAN0)
            {
                can_controllerData.handleCAN0 = DRV_CAN_Open(0, DRV_IO_INTENT_READWRITE);
                appInitialized &= (DRV_HANDLE_INVALID != can_controllerData.handleCAN0);
            }
        
            if (appInitialized)
            {
                /* initialize the CAN state machine */
                can_controllerData.canStateMachine = CAN_CONTROLLER_CAN_STATE_START;
            
                can_controllerData.state = CAN_CONTROLLER_STATE_SERVICE_TASKS;
            }
            break;
        }

        case CAN_CONTROLLER_STATE_SERVICE_TASKS:
        {
            /* run the state machine for servicing the CAN */
            CAN_Task();
        
            break;
        }

        /* TODO: implement your application state machine.*/
        

        /* The default state should never be executed. */
        default:
        {
            /* TODO: Handle error in application's state machine. */
            break;
        }
    }
}

 

/*******************************************************************************
 End of File
 */
