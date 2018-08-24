/*******************************************************************************
 System Interrupts File

  File Name:
    system_interrupt.c

  Summary:
    Raw ISR definitions.

  Description:
    This file contains a definitions of the raw ISRs required to support the
    interrupt sub-system.

  Summary:
    This file contains source code for the interrupt vector functions in the
    system.

  Description:
    This file contains source code for the interrupt vector functions in the
    system.  It implements the system and part specific vector "stub" functions
    from which the individual "Tasks" functions are called for any modules
    executing interrupt-driven in the MPLAB Harmony system.

  Remarks:
    This file requires access to the systemObjects global data structure that
    contains the object handles to all MPLAB Harmony module objects executing
    interrupt-driven in the system.  These handles are passed into the individual
    module "Tasks" functions to identify the instance of the module to maintain.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2011-2014 released Microchip Technology Inc.  All rights reserved.

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

#include "system/common/sys_common.h"
#include "analog_voltage_monitor.h"
#include "error_handler.h"
#include "led_controller.h"
#include "pwm_controller.h"
#include "can_controller.h"
#include "system_definitions.h"

extern ANALOG_VOLTAGE_MONITOR_DATA analog_voltage_monitorData;
extern ERROR_HANDLER_DATA error_handlerData;

// *****************************************************************************
// *****************************************************************************
// Section: System Interrupt Vector Functions
// *****************************************************************************
// *****************************************************************************
void __ISR(_CHANGE_NOTICE_C_VECTOR, ipl1AUTO) _IntHandlerChangeNotification_PortC(void)
{
    if( V_PFC_OL12StateGet() )
    {
//        SYS_DEBUG_BreakPoint();
//        SYS_PRINT("\nFault: Overcurrent for IL12 on the RC13 pin\r\n");
        error_handlerData.currentError.status_flags.OVERCURRENT_FAULT_IL12 = YES;
    }
    PLIB_INT_SourceFlagClear(INT_ID_0,INT_SOURCE_CHANGE_NOTICE_C);
}

void __ISR(_CHANGE_NOTICE_D_VECTOR, ipl1AUTO) _IntHandlerChangeNotification_PortD(void)
{
    if( V_PFC_OL34StateGet() )
    {
//        SYS_DEBUG_BreakPoint();
//        SYS_PRINT("\nFault: Overcurrent for IL34 on the RD6 pin\r\n");
        error_handlerData.currentError.status_flags.OVERCURRENT_FAULT_IL34 = YES;
    }
    PLIB_INT_SourceFlagClear(INT_ID_0,INT_SOURCE_CHANGE_NOTICE_D);
}

void __ISR(_ADC_DATA0_VECTOR, ipl3AUTO) _IntHandlerDrvAdc_DATA0(void)
{
//    SYS_DEBUG_BreakPoint();
    if( DRV_ADC_SamplesAvailable(ADCHS_AN0) == true )
    {
        analog_voltage_monitorData.adc_raw_data.samples.V380V = DRV_ADC_SamplesRead(ADCHS_AN0);
        analog_voltage_monitorData.adc_raw_data.samples.update.status_bits.V380V = YES;
    }
    /* Clear ADC Interrupt Flag of INT_SOURCE_ADC_1_DATA0*/
    PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_ADC_1_DATA0);
}

void __ISR(_ADC_DATA1_VECTOR, ipl3AUTO) _IntHandlerDrvAdc_DATA1(void)
{
//    SYS_DEBUG_BreakPoint();
    if( DRV_ADC_SamplesAvailable(ADCHS_AN1) == true )
    {
        analog_voltage_monitorData.adc_raw_data.samples.TEMP_PFC34 = DRV_ADC_SamplesRead(ADCHS_AN1);
        analog_voltage_monitorData.adc_raw_data.samples.update.status_bits.TEMP_PFC34 = YES;
    }
    /* Clear ADC Interrupt Flag of INT_SOURCE_ADC_1_DATA1*/
    PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_ADC_1_DATA1);
}

void __ISR(_ADC_DATA2_VECTOR, ipl3AUTO) _IntHandlerDrvAdc_DATA2(void)
{
//    SYS_DEBUG_BreakPoint();
    if( DRV_ADC_SamplesAvailable(ADCHS_AN2) == true )
    {
        analog_voltage_monitorData.adc_raw_data.samples.V18V = DRV_ADC_SamplesRead(ADCHS_AN2);
        analog_voltage_monitorData.adc_raw_data.samples.update.status_bits.V18V = YES;
    }
   /* Clear ADC Interrupt Flag of INT_SOURCE_ADC_1_DATA2*/
   PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_ADC_1_DATA2);
}

void __ISR(_ADC_DATA4_VECTOR, ipl3AUTO) _IntHandlerDrvAdc_DATA4(void)
{
//    SYS_DEBUG_BreakPoint();
    if( DRV_ADC_SamplesAvailable(ADCHS_AN4) == true )
    {
        analog_voltage_monitorData.adc_raw_data.samples.VNEUTRAL = DRV_ADC_SamplesRead(ADCHS_AN4);
        analog_voltage_monitorData.adc_raw_data.samples.update.status_bits.VNEUTRAL = YES;
    }    
    /* Clear ADC Interrupt Flag of INT_SOURCE_ADC_1_DATA4*/
    PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_ADC_1_DATA4);
}

void __ISR(_ADC_DATA6_VECTOR, ipl3AUTO) _IntHandlerDrvAdc_DATA6(void)
{
    if( DRV_ADC_SamplesAvailable(ADCHS_AN6) == true )
    {
        analog_voltage_monitorData.adc_raw_data.samples.VLIVE = DRV_ADC_SamplesRead(ADCHS_AN6);
        analog_voltage_monitorData.adc_raw_data.samples.update.status_bits.VLIVE = YES;
    }
    /* Clear ADC Interrupt Flag of INT_SOURCE_ADC_1_DATA6*/
    PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_ADC_1_DATA6);
}

void __ISR(_ADC_DATA7_VECTOR, ipl3AUTO) _IntHandlerDrvAdc_DATA7(void)
{
    if( DRV_ADC_SamplesAvailable(ADCHS_AN7) == true )
    {
        analog_voltage_monitorData.adc_raw_data.samples.V3V3AN2 = DRV_ADC_SamplesRead(ADCHS_AN7);
        analog_voltage_monitorData.adc_raw_data.samples.update.status_bits.V3V3AN2 = YES;
    }   
    /* Clear ADC Interrupt Flag of INT_SOURCE_ADC_1_DATA7*/
    PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_ADC_1_DATA7);
}

void __ISR(_ADC_DATA8_VECTOR, ipl3AUTO) _IntHandlerDrvAdc_DATA8(void)
{
    if( DRV_ADC_SamplesAvailable(ADCHS_AN8) == true )
    {
        analog_voltage_monitorData.adc_raw_data.samples.V3V3_2 = DRV_ADC_SamplesRead(ADCHS_AN8);
        analog_voltage_monitorData.adc_raw_data.samples.update.status_bits.V3V3_2 = YES;
    }
    /* Clear ADC Interrupt Flag of INT_SOURCE_ADC_1_DATA8*/
    PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_ADC_1_DATA8);
}

void __ISR(_ADC_DATA9_VECTOR, ipl3AUTO) _IntHandlerDrvAdc_DATA9(void)
{
    if( DRV_ADC_SamplesAvailable(ADCHS_AN9) == true )
    {
        analog_voltage_monitorData.adc_raw_data.samples.V3V3_1 = DRV_ADC_SamplesRead(ADCHS_AN9);
        analog_voltage_monitorData.adc_raw_data.samples.update.status_bits.V3V3_1 = YES;
    }
    /* Clear ADC Interrupt Flag of INT_SOURCE_ADC_1_DATA9*/
    PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_ADC_1_DATA9);
}

void __ISR(_ADC_DATA10_VECTOR, ipl3AUTO) _IntHandlerDrvAdc_DATA10(void)
{
    if( DRV_ADC_SamplesAvailable(ADCHS_AN10) == true )
    {
        analog_voltage_monitorData.adc_raw_data.samples.V3V3AN1 = DRV_ADC_SamplesRead(ADCHS_AN10);
        analog_voltage_monitorData.adc_raw_data.samples.update.status_bits.V3V3AN1 = YES;
    }    
    /* Clear ADC Interrupt Flag of INT_SOURCE_ADC_1_DATA10*/
    PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_ADC_1_DATA10);
}

void __ISR(_ADC_DATA11_VECTOR, ipl3AUTO) _IntHandlerDrvAdc_DATA11(void)
{
    if( DRV_ADC_SamplesAvailable(ADCHS_AN11) == true )
    {
        analog_voltage_monitorData.adc_raw_data.samples.V1V8_2 = DRV_ADC_SamplesRead(ADCHS_AN11);
        analog_voltage_monitorData.adc_raw_data.samples.update.status_bits.V1V8_2 = YES;
    }
    /* Clear ADC Interrupt Flag of INT_SOURCE_ADC_1_DATA11*/
    PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_ADC_1_DATA11);
}
	
void __ISR(_ADC_DATA12_VECTOR, ipl3AUTO) _IntHandlerDrvAdc_DATA12(void)
{
    if( DRV_ADC_SamplesAvailable(ADCHS_AN12) == true )
    {
        analog_voltage_monitorData.adc_raw_data.samples.V5V = DRV_ADC_SamplesRead(ADCHS_AN12);
        analog_voltage_monitorData.adc_raw_data.samples.update.status_bits.V5V = YES;
    }
    /* Clear ADC Interrupt Flag of INT_SOURCE_ADC_1_DATA12*/
    PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_ADC_1_DATA12);
}
 
void __ISR(_ADC_DATA13_VECTOR, ipl3AUTO) _IntHandlerDrvAdc_DATA13(void)
{
    if( DRV_ADC_SamplesAvailable(ADCHS_AN13) == true )
    {
        analog_voltage_monitorData.adc_raw_data.samples.V3V3_0 = DRV_ADC_SamplesRead(ADCHS_AN13);
        analog_voltage_monitorData.adc_raw_data.samples.update.status_bits.V3V3_0 = YES;
    }
    /* Clear ADC Interrupt Flag of INT_SOURCE_ADC_1_DATA13*/
    PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_ADC_1_DATA13);
}
 
void __ISR(_ADC_DATA17_VECTOR, ipl3AUTO) _IntHandlerDrvAdc_DATA17(void)
{
    if( DRV_ADC_SamplesAvailable(ADCHS_AN17) == true )
    {
        analog_voltage_monitorData.adc_raw_data.samples.TEMP_PFC12 = DRV_ADC_SamplesRead(ADCHS_AN17);
        analog_voltage_monitorData.adc_raw_data.samples.update.status_bits.TEMP_PFC12 = YES;
    }
    /* Clear ADC Interrupt Flag of INT_SOURCE_ADC_1_DATA17*/
    PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_ADC_1_DATA17);
}
 
void __ISR(_ADC_DATA23_VECTOR, ipl3AUTO) _IntHandlerDrvAdc_DATA23(void)
{
    if( DRV_ADC_SamplesAvailable(ADCHS_AN23) == true )
    {
        analog_voltage_monitorData.adc_raw_data.samples.TEMP_M2 = DRV_ADC_SamplesRead(ADCHS_AN23);
        analog_voltage_monitorData.adc_raw_data.samples.update.status_bits.TEMP_M2 = YES;
    }
    /* Clear ADC Interrupt Flag of INT_SOURCE_ADC_1_DATA23*/
    PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_ADC_1_DATA23);
}
 
void __ISR(_ADC_DATA27_VECTOR, ipl3AUTO) _IntHandlerDrvAdc_DATA27(void)
{
    if( DRV_ADC_SamplesAvailable(ADCHS_AN27) == true )
    {
        analog_voltage_monitorData.adc_raw_data.samples.V12V = DRV_ADC_SamplesRead(ADCHS_AN27);
        analog_voltage_monitorData.adc_raw_data.samples.update.status_bits.V12V = YES;
    }
    /* Clear ADC Interrupt Flag of INT_SOURCE_ADC_1_DATA27*/
    PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_ADC_1_DATA27);
}
 
void __ISR(_ADC_DATA35_VECTOR, ipl3AUTO) _IntHandlerDrvAdc_DATA35(void)
{
    if( DRV_ADC_SamplesAvailable(ADCHS_AN35) == true )
    {
        analog_voltage_monitorData.adc_raw_data.samples.TEMP_ELCO = DRV_ADC_SamplesRead(ADCHS_AN35);
        analog_voltage_monitorData.adc_raw_data.samples.update.status_bits.TEMP_ELCO = YES;
    }    
    /* Clear ADC Interrupt Flag of INT_SOURCE_ADC_1_DATA35*/
    PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_ADC_1_DATA35);
}
 
void __ISR(_ADC_DATA36_VECTOR, ipl3AUTO) _IntHandlerDrvAdc_DATA36(void)
{
    if( DRV_ADC_SamplesAvailable(ADCHS_AN36) == true )
    {
        analog_voltage_monitorData.adc_raw_data.samples.TEMP_BRUG = DRV_ADC_SamplesRead(ADCHS_AN36);
        analog_voltage_monitorData.adc_raw_data.samples.update.status_bits.TEMP_BRUG = YES;
    }
    /* Clear ADC Interrupt Flag of INT_SOURCE_ADC_1_DATA36*/
    PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_ADC_1_DATA36);
}
 
void __ISR(_ADC_DATA37_VECTOR, ipl3AUTO) _IntHandlerDrvAdc_DATA37(void)
{
    if( DRV_ADC_SamplesAvailable(ADCHS_AN37) == true )
    {
        analog_voltage_monitorData.adc_raw_data.samples.TEMP_VOED = DRV_ADC_SamplesRead(ADCHS_AN37);
        analog_voltage_monitorData.adc_raw_data.samples.update.status_bits.TEMP_VOED = YES;
    }
    /* Clear ADC Interrupt Flag of INT_SOURCE_ADC_1_DATA37*/
    PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_ADC_1_DATA37);
}
 
void __ISR(_ADC_DATA39_VECTOR, ipl3AUTO) _IntHandlerDrvAdc_DATA39(void)
{
    if( DRV_ADC_SamplesAvailable(ADCHS_AN39) == true )
    {
        analog_voltage_monitorData.adc_raw_data.samples.V325V = DRV_ADC_SamplesRead(ADCHS_AN39);
        analog_voltage_monitorData.adc_raw_data.samples.update.status_bits.V325V = YES;
    }
    /* Clear ADC Interrupt Flag of INT_SOURCE_ADC_1_DATA39*/
    PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_ADC_1_DATA39);
}
 
void __ISR(_ADC_DATA45_VECTOR, ipl3AUTO) _IntHandlerDrvAdc_DATA45(void)
{
    if( DRV_ADC_SamplesAvailable(ADCHS_AN45) == true )
    {
        analog_voltage_monitorData.adc_raw_data.samples.V1V8_1 = DRV_ADC_SamplesRead(ADCHS_AN45);
        analog_voltage_monitorData.adc_raw_data.samples.update.status_bits.V1V8_1 = YES;
    }    
    /* Clear ADC Interrupt Flag of INT_SOURCE_ADC_1_DATA45*/
    PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_ADC_1_DATA45);
}
 
void __ISR(_ADC_DATA46_VECTOR, ipl3AUTO) _IntHandlerDrvAdc_DATA46(void)
{
    if( DRV_ADC_SamplesAvailable(ADCHS_AN46) == true )
    {
        analog_voltage_monitorData.adc_raw_data.samples.IL12 = DRV_ADC_SamplesRead(ADCHS_AN46);
        analog_voltage_monitorData.adc_raw_data.samples.update.status_bits.IL12 = YES;
    }    
    /* Clear ADC Interrupt Flag of INT_SOURCE_ADC_1_DATA46*/
    PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_ADC_1_DATA46);
}
 
void __ISR(_ADC_DATA47_VECTOR, ipl3AUTO) _IntHandlerDrvAdc_DATA47(void)
{
    if( DRV_ADC_SamplesAvailable(ADCHS_AN47) == true )
    {
        analog_voltage_monitorData.adc_raw_data.samples.TEMP_M1 = DRV_ADC_SamplesRead(ADCHS_AN47);
        analog_voltage_monitorData.adc_raw_data.samples.update.status_bits.TEMP_M1 = YES;
    }
    /* Clear ADC Interrupt Flag of INT_SOURCE_ADC_1_DATA47*/
    PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_ADC_1_DATA47);
}
     
     

void __ISR(_TIMER_1_VECTOR, ipl4AUTO) IntHandlerDrvTmrInstance0(void)
{
    DRV_TMR_Tasks(sysObj.drvTmr0);
}
/*******************************************************************************
 End of File
*/
