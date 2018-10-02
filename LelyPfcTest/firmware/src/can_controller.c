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

#define CAN_FIFO_SIZE   32
#define CAN_TX_CHANNEL  CAN_CHANNEL0
#define CAN_RX_CHANNEL  CAN_CHANNEL1

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************
static CAN_MSG_t CAN_TX_FIFO_Buffer_Array[CAN_FIFO_SIZE];
static CAN_MSG_FIFO_BUFFER_t CAN_TX_MSG_FIFO = {
                                                .inPosition = 0,
                                                .outPosition = 0,
                                                .countMsgs = 0,
                                                .size = CAN_FIFO_SIZE,
                                                .pFIFO = CAN_TX_FIFO_Buffer_Array
                                                };

//static CAN_MSG_t CAN_RX_FIFO_Buffer_Array[CAN_FIFO_SIZE];
//static CAN_MSG_FIFO_BUFFER_t CAN_RX_MSG_FIFO = {
//                                                .inPosition = 0,
//                                                .outPosition = 0,
//                                                .countMsgs = 0,
//                                                .size = CAN_FIFO_SIZE,
//                                                .pFIFO = CAN_RX_FIFO_Buffer_Array
//                                                };


// Hold one CAN standard tx message
static CAN_MSG_t can_tx_msg = {
                                .Id = 0x403,
                                .dataLength = 8,
                                .data = "DynaTron"
                                };
// Hold one CAN standard rx message 
static CAN_MSG_t can_rx_msg = {
                                .Id = 0x400,
                                .dataLength = 8,
                                .data = {'\0'}
                                };
// Hold CAN channel event
static CAN_CHANNEL_EVENT txChannelEvent;
static CAN_CHANNEL_EVENT rxChannelEvent;

// There global variables for CAN Bus Message Parser
// Hold system overall error status for error report via CAN bus
static SYS_ERR_STATUS system_error = {
                                        .errStatus = 0x0F
                                        };

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

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

/* Application's Timer Callback Function */
static void TimerCallback (  uintptr_t context, uint32_t alarmCount )
{
    global_events.start_can_tx = true;   
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
        can_controllerData.handleTimer1forCANTx, 
        CAN_CONTROLLER_TMR_DRV_PERIOD, 
        CAN_CONTROLLER_TMR_DRV_IS_PERIODIC,
        (uintptr_t)NULL, 
        TimerCallback);
    DRV_TMR_Start(can_controllerData.handleTimer1forCANTx);
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

    can_controllerData.handleTimer1forCANTx = DRV_HANDLE_INVALID;

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
       
            if (can_controllerData.handleTimer1forCANTx == DRV_HANDLE_INVALID)
            {
                can_controllerData.handleTimer1forCANTx = DRV_TMR_Open(CAN_CONTROLLER_TMR_DRV, DRV_IO_INTENT_EXCLUSIVE);
                appInitialized &= ( DRV_HANDLE_INVALID != can_controllerData.handleTimer1forCANTx );
            }
        
            if (appInitialized)
            {
                TimerSetup();
                DRV_TMR1_AlarmEnable(true);
                can_controllerData.state = CAN_CONTROLLER_STATE_WAIT;
            }
            break;
        }

        case CAN_CONTROLLER_STATE_WAIT:
        {
            // wait for Timer 6 interrupt for CAN Transmit
            if( global_event_triggered(&global_events.start_can_tx) )
            {
                printf("[CAN TX ...]\n");
//                PLIB_PORTS_PinToggle(PORTS_ID_0, PORT_CHANNEL_D, PORTS_BIT_POS_5);
//                V_CANTXToggle();
//                LATDbits.LATD5 ^= 1;
//                PORTDbits.RD5 ^= 1;
//                V_CANRXToggle();
                V_LED2_GToggle();
                can_controllerData.state = CAN_CONTROLLER_STATE_TX;
            }
            rxChannelEvent = PLIB_CAN_ChannelEventGet(CAN_ID_1, CAN_RX_CHANNEL);
            if( (rxChannelEvent & CAN_RX_CHANNEL_NOT_EMPTY) == CAN_RX_CHANNEL_NOT_EMPTY )
            {
                printf("[CAN RX ...]\n");
                can_controllerData.state = CAN_CONTROLLER_STATE_RX;
            }
            break;
        }

        /* TODO: implement your application state machine.*/

        case CAN_CONTROLLER_STATE_TX:
        {
//            printf("[CAN Baudrate %d kbps]\n", PLIB_CAN_BaudRateGet(CAN_ID_1, SYS_CLK_SystemFrequencyGet()));
            if( CAN_SendMsg(&can_tx_msg) )
            {
                printf("[CAN TX succeeded]\n");
            }
            else
            {
                printf("[CAN TX failed]\n");
                txChannelEvent = PLIB_CAN_ChannelEventGet(CAN_ID_1, CAN_TX_CHANNEL);
                printf("[CAN TX Channel Event %X]\n", txChannelEvent);
            }
            can_controllerData.state = CAN_CONTROLLER_STATE_WAIT;
            break;
        }

        case CAN_CONTROLLER_STATE_RX:
        {
            if( CAN_ReceiveMsg(&can_rx_msg) )
            {
                printf("[CAN RX succeeded]\n");
                printf("[CAN RX ID = 0x%3X]\n", can_rx_msg.Id);
                printf("[CAN RX DLC = %d]\n", can_rx_msg.dataLength);
                printf("[CAN RX MSG = %s]\n", can_rx_msg.data);
                printf("[CAN RX DATA = 0x%2X 0x%2X 0x%2X 0x%2X 0x%2X 0x%2X 0x%2X 0x%2X]\n", 
                                                                        can_rx_msg.data[0],
                                                                        can_rx_msg.data[1],
                                                                        can_rx_msg.data[2],
                                                                        can_rx_msg.data[3],
                                                                        can_rx_msg.data[4],
                                                                        can_rx_msg.data[5],
                                                                        can_rx_msg.data[6],
                                                                        can_rx_msg.data[7]);
            }
            else
            {
                printf("[CAN RX failed]\n");
            }
            can_controllerData.state = CAN_CONTROLLER_STATE_WAIT;
            break;
        }        

        /* The default state should never be executed. */
        default:
        {
            /* TODO: Handle error in application's state machine. */
            can_controllerData.state = CAN_CONTROLLER_STATE_INIT;
            break;
        }
    }
}

/*
 * @Description 
 *      Send a CAN standard ID message to the CAN bus
 * @Parameters 
 *      pCANTxMsg: a pointer to a CAN Message you want to transmit
 * @Return 
 *      true: if a CAN message is transmitted successfully
 *      false: failed to transmit CAN message
 */
bool CAN_SendMsg(CAN_MSG_t *pCanTxMsg)
{
//    bool retVal = false;
//    retVal = DRV_CAN0_ChannelMessageTransmit(CAN_TX_CHANNEL, pCanTxMsg->Id, pCanTxMsg->dataLength, pCanTxMsg->data);
//    return retVal;
    
    uint16_t txMsgId = pCanTxMsg->Id;
    uint8_t DLC = pCanTxMsg->dataLength; // Data Length Code
    uint8_t * pTxData = pCanTxMsg->data;
    uint8_t txByteCount = 0;
    CAN_TX_MSG_BUFFER * pTxMsgBuffer = NULL;
    
    if( (PLIB_CAN_ChannelEventGet(CAN_ID_1, CAN_TX_CHANNEL) & CAN_TX_CHANNEL_NOT_FULL) == CAN_TX_CHANNEL_NOT_FULL)
    {
        // Get a pointer to an empty transmit buffer
        pTxMsgBuffer = PLIB_CAN_TransmitBufferGet(CAN_ID_1, CAN_TX_CHANNEL);
        
        if( pTxMsgBuffer == NULL )
        {
            // There is not any empty transmit message buffer and hence quit with false
            return false;
        }
        // There is an empty transmit message buffer and hence load the message
        CAN_TX_MSG_BUFFER tempMsg = { .messageWord = {0} };
        
        /* Check whether the id is a Standard ID
         * The standard ID has 11 bits and its max limit is 0x7FF, so anything beyond that is Extended ID message */
        if( txMsgId > 0x7FF )
        {
            // It is an extended ID message and discarded!
            return false;
        }
        else
        {
            tempMsg.msgSID.sid = txMsgId;
            tempMsg.msgEID.eid = 0;
            tempMsg.msgEID.ide = 0;
        }
        
        if( DLC > 8 )
        {
            DLC = 8;
        }

        tempMsg.msgEID.data_length_code = DLC;
        
        while( txByteCount < DLC )
        {
            tempMsg.data[txByteCount++] = *pTxData++;
        }
        
        int i;
        for( i = 0; i < 4; i++ ){
            pTxMsgBuffer->messageWord[i] = tempMsg.messageWord[i];
        }
        

        // Update CAN module and then transmit data on the bus;
        // Update the CAN channel internal pointer 
        PLIB_CAN_ChannelUpdate(CAN_ID_1, CAN_TX_CHANNEL);
        // Transmit all messages in the TX channel
        PLIB_CAN_TransmitChannelFlush(CAN_ID_1, CAN_TX_CHANNEL);
        return(true);
    }
    // CAN TX Channel is full
    return (false);
}

/*
 * @Description 
 *      Receive a CAN standard ID message from CAN RX Message FIFO if there is at least one CAN message available in RX FIFO
 * @Parameters 
 *      pCanRxMsg: a pointer to a CAN message where the received message is held
 * @Return 
 *      true: succeed to read a message from the CAN bus RX FIFO buffer
 *      false: fail to read a message from the CAN bus RX FIFO buffer if the buffer is empty
 */
bool CAN_ReceiveMsg(CAN_MSG_t *pCanRxMsg)
{
//    bool retVal = false;
//    retVal = DRV_CAN0_ChannelMessageReceive(CAN_RX_CHANNEL, pCanRxMsg->Id, pCanRxMsg->dataLength, pCanRxMsg->data);
//    return retVal;
    
    uint8_t rxByteCount = 0;
    uint8_t * pRxData = pCanRxMsg->data;
    bool readStatus = false;    
    CAN_CHANNEL_EVENT rxChannelEvent = 0;
    CAN_RX_MSG_BUFFER *pRxMsgBuffer = NULL;
    
    /* Get the RX channel status */
    rxChannelEvent = PLIB_CAN_ChannelEventGet(CAN_ID_1, CAN_RX_CHANNEL);

    /* Check if there is a message available in RX channel. */
    if( (rxChannelEvent & CAN_RX_CHANNEL_NOT_EMPTY) == CAN_RX_CHANNEL_NOT_EMPTY )
    {
        /* There is a message available in the RX Channel FIFO. */

        /* Get a pointer to RX message buffer */
        pRxMsgBuffer = (CAN_RX_MSG_BUFFER *)PLIB_CAN_ReceivedMessageGet(CAN_ID_1, CAN_RX_CHANNEL);

        /* Process the message fields */

        /* Check if it is an extended message type */
        if( pRxMsgBuffer->msgEID.ide )
        {
            // It is not a standard ID message
            pCanRxMsg->Id = 0;
//            readStatus = false;
        }
        else
        {
            pCanRxMsg->Id = pRxMsgBuffer->msgSID.sid;
        }

        pCanRxMsg->dataLength = pRxMsgBuffer->msgEID.data_length_code;
        
        if( pRxMsgBuffer->msgEID.data_length_code > 0 )
        {
            while( rxByteCount < pRxMsgBuffer->msgEID.data_length_code )
            {
                 *pRxData++ = pRxMsgBuffer->data[rxByteCount++];
            }
        }

        /* Message processing is done, update the message buffer pointer. */
        PLIB_CAN_ChannelUpdate(CAN_ID_1, CAN_RX_CHANNEL);

        /* Message is processed successfully, so return true */
        readStatus = true;
    }
    else
    {
        /* There is no message to read ,so return false */
        readStatus = false;
    }

    return readStatus;
}

/*
 * @Description 
 *      parse a received CAN bus message and feed a response back to the CAN bus
 * @Parameters 
 *      can_rx_msg: the CAN Rx Message that you want to parse
 * @Return 
 *      None
 */
void CAN_parseRxMsg( CAN_MSG_t can_rx_msg )
{
    switch( can_rx_msg.Id )
    {
        case IdErrorReport:
            // Report Error
            
            break;
            
        case IdStatusRequest:
            // Request Status
            
            break;
            
        case IdVersionRequest:
            // Request Version
            
            break;
            
        case IdProductionDataRequest:
            
            break;
            
        case IdMacAddressRequest:
            
            break;
        
        case IdParameterSetRequest:
            
            break;
            
        case IdParameterReadRequest:
            
            break;
            
        case IdGeneralMeasureRequest:
            
            break;
        
        case IdMainsMeasureRequest:
            
            break;
            
        case IdTempMeasureRequest:
            
            break;

        default:
            
            break;
    }
}

/*
 * @Description 
 *      If a CAN message is parsed as system error report message, process this error report message 
 * @Parameters 
 *      can_msg_to_process: the system error report message to process 
 * @Return 
 *      None
 */
void CAN_processErrorReportMsg( CAN_MSG_t can_msg_to_process )
{
    CAN_MSG_t can_msg_response;
    can_msg_response.Id = can_msg_to_process.Id;
    can_msg_response.dataLength = 8;
    // Check the system error state
    if(system_error.errStatus == 0)
    {
        can_msg_response.data[0] = NoError;
        strcpy(&can_msg_response.data[1], "NoError");
        CanTxMsgFifoPush(can_msg_response);
    }
    
    if(system_error.errStatusFlag.MainVolt)
    {
        can_msg_response.data[0] = MainVoltError;
        strcpy(&can_msg_response.data[1], "MainVlt");
        CanTxMsgFifoPush(can_msg_response);
    }
    
    if(system_error.errStatusFlag.OverLoad)
    {
        can_msg_response.data[0] = OverloadError;
        strcpy(&can_msg_response.data[1], "Ovrload");
        CanTxMsgFifoPush(can_msg_response);
    }
    
    if(system_error.errStatusFlag.PFC)
    {
        can_msg_response.data[0] = PFCError;
        strcpy(&can_msg_response.data[1], "PFCErro");
        CanTxMsgFifoPush(can_msg_response);        
    }
     
    if(system_error.errStatusFlag.InternalPowerSupply)
    {
        can_msg_response.data[0] = InternalPowerSupplyError;
        strcpy(&can_msg_response.data[1], "PwerErr");
        CanTxMsgFifoPush(can_msg_response);
    }
}

void CAN_processStatusRequestMsg( CAN_MSG_t can_msg_to_process )
{
    uint16_t id = 0;
    uint8_t msg = 0;
    uint8_t data[5] = {0};
    if(can_msg_to_process.dataLength == 8)
    {
        id = can_msg_to_process.data[1]; // Hold high byte ID 0b0000 0xxx
        id = id << 8;
        id = id + can_msg_to_process.data[0]; // Hold low byte ID 0bxxxx xxxx
        msg = can_msg_to_process.data[2];
        memcpy(data, &can_msg_to_process.data[3], 5);
    }
    // Here handle id that you get
    
    // Here handle msg that you get 
    
    // Here handle data that you get
    
}

void CAN_processVersionRequestMsg( CAN_MSG_t can_msg_to_process )
{
    CAN_MSG_t can_msg_response;
    can_msg_response.Id = can_msg_to_process.Id;
    can_msg_response.dataLength = 8;
    memcpy(&can_msg_response.data[0], "T1", 2);     // Data 0.1 hold hardware type
    memcpy(&can_msg_response.data[2], "V1", 2);     // Data 2.3 hold hardware version
    memcpy(&can_msg_response.data[4], "V1", 2);     // Data 4.5 hold bootloader version
    memcpy(&can_msg_response.data[6], "V1", 2);     // Data 6.7 hold application version
    CanTxMsgFifoPush(can_msg_response);
}

void CAN_processProductionDataRequest( CAN_MSG_t can_msg_to_process )
{
    
}



/*
 * @Description 
 *      check if the CAN TX MSG FIFO buffer is empty
 * @Parameters 
 *      None
 * @Return 
 *      true: the CAN TX MSG FIFO buffer is empty
 *      false: the CAN TX MSG FIFO buffer has some TX messages
 */
bool isCanTxMsgFifoEmpty(void)
{
    if( CAN_TX_MSG_FIFO.countMsgs == 0 ) return true;
    else return false;
}

/*
 * @Description 
 *      check if the CAN TX MSG FIFO buffer is full
 * @Parameters 
 *      None
 * @Return 
 *      true: the CAN TX MSG FIFO buffer is full
 *      false: the CAN TX MSG FIFO buffer can store more messages
 */
bool isCanTxMsgFifoFull(void)
{
    if( CAN_TX_MSG_FIFO.countMsgs == CAN_TX_MSG_FIFO.size ) return true;
    else return false;
}

/*
 * @Description 
 *      push a tx message into the CAN TX MSG FIFO buffer
 * @Parameters 
 *      TxMsgIn: the tx message you want to push into the CAN TX MSG FIFO buffer
 * @Return 
 *      true: pushing a tx message into the CAN TX MSG FIFO buffer is successful
 *      false: can not push a tx message into the CAN TX MSG FIFO buffer
 */
bool CanTxMsgFifoPush(CAN_MSG_t TxMsgIn)
{
    if(isCanTxMsgFifoFull())
    {
        return false;
    }
    CAN_TX_MSG_FIFO.pFIFO[CAN_TX_MSG_FIFO.inPosition] = TxMsgIn;
    CAN_TX_MSG_FIFO.countMsgs++;
    CAN_TX_MSG_FIFO.inPosition = (CAN_TX_MSG_FIFO.inPosition + 1) % CAN_TX_MSG_FIFO.size; // round FIFO buffer
    return true;
}

/*
 * @Description 
 *      pop a tx message from the CAN TX MSG FIFO buffer
 * @Parameters 
 *      pTxMsgOut: pointer to a place where you store the tx message popped from the CAN TX MSG FIFO buffer
 * @Return 
 *      true: popping a tx message from the CAN TX FIFO buffer is successful
 *      false: can not pop a tx message from the CAN TX FIFO buffer
 */
bool CanTxMsgFifoPop(CAN_MSG_t *pTxMsgOut)
{
    if(isCanTxMsgFifoEmpty())
    {
        return false;
    }
    if(pTxMsgOut == NULL)
    {
        return false;
    }
    *pTxMsgOut = CAN_TX_MSG_FIFO.pFIFO[CAN_TX_MSG_FIFO.outPosition];
    CAN_TX_MSG_FIFO.countMsgs--;
    CAN_TX_MSG_FIFO.outPosition = (CAN_TX_MSG_FIFO.outPosition + 1) % CAN_TX_MSG_FIFO.size; // round FIFO buffer
    return true;
}
/*
 * @Description 
 *      
 * @Parameters 
 *      None
 * @Return 
 *      None
 */
/*******************************************************************************
 End of File
 */
