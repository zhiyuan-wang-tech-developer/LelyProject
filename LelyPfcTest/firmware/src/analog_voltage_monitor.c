/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    analog_voltage_monitor.c

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

#include "analog_voltage_monitor.h"
#include "system/debug/sys_debug.h"
#include <string.h>

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

ANALOG_VOLTAGE_MONITOR_DATA analog_voltage_monitorData;

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
/* Enable/Disable all DC-DC powers:
 * 12V
 * 3V3-1    3V3-2
 * 3V3AN1   3V3AN2
 * 1V8-1    1V8-2
 */
void EnableDCPower(void)
{
    V_EN_12VOn();
    V_EN_3V3_1On();
    V_EN_3V3_2On();
    V_EN_3V3_AN1On();
    V_EN_3V3_AN2On();
    V_EN_1V8_1On();
    V_EN_1V8_2On();
}

void DisableDCPower(void)
{
    V_EN_12VOff();
    V_EN_3V3_1Off();
    V_EN_3V3_2Off();
    V_EN_3V3_AN1Off();
    V_EN_3V3_AN2Off();
    V_EN_1V8_1Off();
    V_EN_1V8_2Off();
}

// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void ANALOG_VOLTAGE_MONITOR_Initialize ( void )

  Remarks:
    See prototype in analog_voltage_monitor.h.
 */

void ANALOG_VOLTAGE_MONITOR_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    analog_voltage_monitorData.state = ANALOG_VOLTAGE_MONITOR_STATE_INIT;

    
    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
    /* Clear the adc data buffer in initial state. */
    memset(analog_voltage_monitorData.adc_data.buffer, 0, sizeof(analog_voltage_monitorData.adc_data.buffer));
    /* Open all DC powers */
    EnableDCPower();
    SYS_PRINT("System is initialized!\r\n");
}


/******************************************************************************
  Function:
    void ANALOG_VOLTAGE_MONITOR_Tasks ( void )

  Remarks:
    See prototype in analog_voltage_monitor.h.
 */

void ANALOG_VOLTAGE_MONITOR_Tasks ( void )
{

    /* Check the application's current state. */
    switch ( analog_voltage_monitorData.state )
    {
        /* Application's initial state. */
        case ANALOG_VOLTAGE_MONITOR_STATE_INIT:
        {
            bool appInitialized = true;
     
            
//            ADC0CFG = DEVADC0;
//            ADC1CFG = DEVADC1;
//            ADC2CFG = DEVADC2;
//            ADC3CFG = DEVADC3;
//            ADC4CFG = DEVADC4;
//            ADC5CFG = DEVADC5;
//            
//            ADC7CFG = DEVADC7;
//            ADCCON1 = 0;
//            ADCCON2 = 0;
//            ADCANCON = 0;
//            ADCCON3 = 0;
            
//            ADCANCONbits.WKUPCLKCNT = 5;
//            ADCCON1bits.ON = 1;
            
//            while( !ADCCON2bits.BGVRRDY );
//            while( ADCCON2bits.REFFLT) ;
            
//            ADCANCON |= 0xFF;   //ANEN0-7 = 1
//            while( (ADCANCON & 0xFF00) == 0); //wait for wake ready
//            ADCCON3bits.TRGSUSP = 1;
//            ADCCON3bits.UPDRDY = 1;
//            ADCCON3bits.DIGEN0 = 1;
//            ADCCON3bits.DIGEN1 = 1;
//            ADCCON3bits.DIGEN2 = 1;
//            ADCCON3bits.DIGEN4 = 1;
//            ADCCON3bits.DIGEN5 = 1;
//            ADCCON3bits.DIGEN7 = 1;
            Nop();
            Nop();
            Nop();
            
            if (appInitialized)
            {            
                analog_voltage_monitorData.state = ANALOG_VOLTAGE_MONITOR_STATE_SCAN;
                /* Clear the adc data buffer in initial state. */
                memset(analog_voltage_monitorData.adc_data.buffer, 0, sizeof(analog_voltage_monitorData.adc_data.buffer));
                /* Start the ADC */
                Nop();
                Nop();
                DRV_ADC0_Open();
//                Nop();
//                Nop();
//                ADCCON3bits.DIGEN0 = 1;
//                Nop();
//                Nop();
//                DRV_ADC1_Open();
//                Nop();
//                Nop();
//                DRV_ADC2_Open();
//                DRV_ADC4_Open();
//                DRV_ADC5_Open();
//                DRV_ADC6_Open();

//                ADCCON3bits.TRGSUSP = 0;
//                ADCCON3bits.UPDRDY = 0; 
                DRV_ADC_Start();
            }
            break;
        }

        /* TODO: implement your application state machine.*/
        case ANALOG_VOLTAGE_MONITOR_STATE_SCAN:
        {
//            SYS_DEBUG_BreakPoint();
//            V_LED1_GOn();
            analog_voltage_monitorData.state = ANALOG_VOLTAGE_MONITOR_STATE_SCAN_DONE;
            break;
        }        

        case ANALOG_VOLTAGE_MONITOR_STATE_SCAN_DONE:
        {
            /* Check if every ADC data is updated in the ADC buffer */
            // AN5 can not be read out right now so its status flag is left ZERO. 
            if(analog_voltage_monitorData.adc_data.update.status == 0b00000000011111111111111111011111)
            {
                // If the ADC data are all updated, then go to next state for data display. 
                analog_voltage_monitorData.state = ANALOG_VOLTAGE_MONITOR_STATE_DISPLAY;
                DRV_ADC_Stop();
//                DRV_ADC0_Close();
//                DRV_ADC1_Close();
//                DRV_ADC2_Close();
//                DRV_ADC4_Close();
//                DRV_ADC5_Close();
//                DRV_ADC6_Close();
//                V_LED1_GOff();
            }
            else
            {
                // If the ADC data are not updated completely, then go to next state for scan.
                analog_voltage_monitorData.state = ANALOG_VOLTAGE_MONITOR_STATE_SCAN;                
            }
            break;
        }

        case ANALOG_VOLTAGE_MONITOR_STATE_DISPLAY:
        {
//            V_LED1_ROn();
            SYS_DEBUG_BreakPoint();
            SYS_PRINT("\r\nScan Result List: \r\n");
            SYS_PRINT("\t V_380V = %d \t V_325V = %d\n", analog_voltage_monitorData.adc_data.V380V, analog_voltage_monitorData.adc_data.V325V);
            SYS_PRINT("\t V_Live = %d \t V_Neutral = %d\n", analog_voltage_monitorData.adc_data.VLIVE, analog_voltage_monitorData.adc_data.VNEUTRAL);
            SYS_PRINT("\t IL12 = %d \t IL34 = %d\n", analog_voltage_monitorData.adc_data.IL12, analog_voltage_monitorData.adc_data.IL34);
            
            SYS_PRINT("\t T_M1 = %d \t T_M2 = %d\n", analog_voltage_monitorData.adc_data.TEMP_M1, analog_voltage_monitorData.adc_data.TEMP_M2);
            SYS_PRINT("\t T_PFC12 = %d \t T_PFC34 = %d\n", analog_voltage_monitorData.adc_data.TEMP_PFC12, analog_voltage_monitorData.adc_data.TEMP_PFC34);
            SYS_PRINT("\t T_ELCO = %d \t T_BRUG = %d \t T_VOED = %d\n", analog_voltage_monitorData.adc_data.TEMP_ELCO, analog_voltage_monitorData.adc_data.TEMP_BRUG, analog_voltage_monitorData.adc_data.TEMP_VOED);

            SYS_PRINT("\t V_18V = %d \t V_12V = %d \t V_5V = %d\n", analog_voltage_monitorData.adc_data.V18V, analog_voltage_monitorData.adc_data.V12V, analog_voltage_monitorData.adc_data.V5V);
            SYS_PRINT("\t V_3V3_0 = %d \t V_3V3_1 = %d \t V_3V3_2 = %d\n", analog_voltage_monitorData.adc_data.V3V3_0, analog_voltage_monitorData.adc_data.V3V3_1, analog_voltage_monitorData.adc_data.V3V3_2);
            SYS_PRINT("\t V_3V3_AN1 = %d \t V_3V3_AN2 = %d\n", analog_voltage_monitorData.adc_data.V3V3AN1, analog_voltage_monitorData.adc_data.V3V3AN2);
            SYS_PRINT("\t V_1V8_1 = %d \t V_1V8_2 = %d\n\n", analog_voltage_monitorData.adc_data.V1V8_1, analog_voltage_monitorData.adc_data.V1V8_2);
//            V_LED1_ROff();
            analog_voltage_monitorData.state = ANALOG_VOLTAGE_MONITOR_STATE_INIT;
            break;
        }
                
        /* The default state should never be executed. */
        default:
        {
            /* TODO: Handle error in application's state machine. */
            SYS_PRINT("Analog voltage monitor in wrong state!\r\n");
            /* Return to the initial state. */
            analog_voltage_monitorData.state = ANALOG_VOLTAGE_MONITOR_STATE_INIT;            
            break;
        }
    }
}

 

/*******************************************************************************
 End of File
 */
