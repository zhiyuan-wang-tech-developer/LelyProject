/*******************************************************************************
  MPLAB Harmony Application Header File

  Company:
    Microchip Technology Inc.

  File Name:
    can_controller.h

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

#ifndef _CAN_CONTROLLER_H
#define _CAN_CONTROLLER_H

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

// CAN Message Identifier
#define     IdErrorReport                   0x400
#define     IdStatusRequest                 0x401
#define     IdVersionRequest                0x402
#define     IdProductionDataRequest         0x403
#define     IdMacAddressRequest             0x404
#define     IdParameterSetRequest           0x405
#define     IdParameterReadRequest          0x406
#define     IdGeneralMeasureRequest         0x409
#define     IdMainsMeasureRequest           0x40A
#define     IdTempMeasureRequest            0x40B

// Enumeration type for system error report message data byte 1
typedef enum 
{
    NoError = 0x00,
    MainVoltError = 0x01,
    OverloadError = 0x02,
    PFCError = 0x03,
    InternalPowerSupplyError = 0x04
} ERR_MSG;

// Data structure for system error report via CAN bus
typedef union
{
    uint8_t errStatus;
    struct 
    {
        uint8_t MainVolt: 1;
        uint8_t OverLoad: 1;
        uint8_t PFC: 1;
        uint8_t InternalPowerSupply: 1;
        uint8_t Other: 4;        
    } errStatusFlag;
} SYS_ERR_STATUS;

// Enumeration type for system parameter message date byte 1
typedef enum
{
    Umax = 0x00,
    Imax = 0x01,
    Tpfcmax = 0x02,
    Tmotormax = 0x03,
    Tbrugmax = 0x04,
    Tvoedmax = 0x05,
    Telcomax = 0x06,
    PFCreset = 0x07,
    U380typical = 0x08,
    reserved01 = 0x09,
    reserved02 = 0x0A
} PARAMETER_MSG;

// *****************************************************************************
// *****************************************************************************
// Section: Type Definitions
// *****************************************************************************
// *****************************************************************************

/* CAN message data structure for standard ID format */
typedef struct
{
    uint16_t Id; // 11 bits identifier
    uint8_t dataLength; // data length code
    uint8_t data[8]; // 8 bytes of data payload
} CAN_MSG_t;

// Data type for CAN Message FIFO in application software
typedef struct
{
    uint8_t inPosition;
    uint8_t outPosition;
    uint8_t countMsgs;
    uint8_t size;
    CAN_MSG_t *pFIFO;
} CAN_MSG_FIFO_BUFFER_t;
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
	CAN_CONTROLLER_STATE_INIT=0,
	CAN_CONTROLLER_STATE_WAIT,
    CAN_CONTROLLER_STATE_TX,
    CAN_CONTROLLER_STATE_RX
            
	/* TODO: Define states used by the application state machine. */

} CAN_CONTROLLER_STATES;


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
    CAN_CONTROLLER_STATES state;

    /* TODO: Define any additional data used by the application. */
    DRV_HANDLE handleTimer1forCANTx;

} CAN_CONTROLLER_DATA;


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
    void CAN_CONTROLLER_Initialize ( void )

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
    CAN_CONTROLLER_Initialize();
    </code>

  Remarks:
    This routine must be called from the SYS_Initialize function.
*/

void CAN_CONTROLLER_Initialize ( void );


/*******************************************************************************
  Function:
    void CAN_CONTROLLER_Tasks ( void )

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
    CAN_CONTROLLER_Tasks();
    </code>

  Remarks:
    This routine must be called from SYS_Tasks() routine.
 */

void CAN_CONTROLLER_Tasks( void );

bool CAN_SendMsg(CAN_MSG_t *pCanTxMsg);
bool CAN_ReceiveMsg(CAN_MSG_t *pCanRxMsg);

void CAN_parseRxMsg(CAN_MSG_t can_rx_msg);
void CAN_processErrorReportMsg(CAN_MSG_t can_msg_to_process);
bool isCanTxMsgFifoEmpty(void);
bool isCanTxMsgFifoFull(void);
bool CanTxMsgFifoPush(CAN_MSG_t TxMsgIn);
bool CanTxMsgFifoPop(CAN_MSG_t *pTxMsgOut);

#endif /* _CAN_CONTROLLER_H */

//DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
//DOM-IGNORE-END

/*******************************************************************************
 End of File
 */

