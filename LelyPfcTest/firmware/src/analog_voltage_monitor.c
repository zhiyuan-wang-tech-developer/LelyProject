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
#include "peripheral/nvm/processor/nvm_p32mk0512mcf100.h"
#include <string.h>
#include <stdlib.h>

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************
extern ERROR_HANDLER_DATA error_handlerData;

/*
 * NTC (Negative Temperature Coefficient) Thermal Resistor
 * Temperature Characteristics Table
 * 
 * Thermistor Type: NC P 18 XH 103 J 03 RB
 * Resistance:  10K ohm @ 25?
 * B-Constant:  3380K           
 * Ref. Part Number: NCU  XH103
 *                      U --- High Reliability Series
 *                     XH --- Nominal B-Constant 3350-3399K
 *                    103 --- 10K ohm Resistance  
 */
const NTC_LOOKUP_TABLE_TYPE   NTC_LookupTable[NTC_LOOKUP_TABLE_SIZE] = {
    { .resistance =    531, .temperature = 125 },  //#01
    { .resistance =    596, .temperature = 120 },  //#02
    { .resistance =    672, .temperature = 115 },  //#03
    { .resistance =    758, .temperature = 110 },  //#04
    { .resistance =    858, .temperature = 105 },  //#05
    { .resistance =    974, .temperature = 100 },  //#06
    { .resistance =   1110, .temperature =  95 },  //#07
    { .resistance =   1268, .temperature =  90 },  //#08
    { .resistance =   1452, .temperature =  85 },  //#09
    { .resistance =   1669, .temperature =  80 },  //#10
    { .resistance =   1925, .temperature =  75 },  //#11
    { .resistance =   2228, .temperature =  70 },  //#12
    { .resistance =   2586, .temperature =  65 },  //#13
    { .resistance =   3014, .temperature =  60 },  //#14
    { .resistance =   3535, .temperature =  55 },  //#15
    { .resistance =   4161, .temperature =  50 },  //#16
    { .resistance =   4917, .temperature =  45 },  //#17
    { .resistance =   5834, .temperature =  40 },  //#18
    { .resistance =   6948, .temperature =  35 },  //#19
    { .resistance =   8315, .temperature =  30 },  //#20
    { .resistance =  10000, .temperature =  25 },  //#21
    { .resistance =  12081, .temperature =  20 },  //#22
    { .resistance =  14674, .temperature =  15 },  //#23
    { .resistance =  17926, .temperature =  10 },  //#24
    { .resistance =  22021, .temperature =   5 },  //#25
    { .resistance =  27219, .temperature =   0 },  //#26
    { .resistance =  33892, .temperature =  -5 },  //#27
    { .resistance =  42506, .temperature = -10 },  //#28
    { .resistance =  53650, .temperature = -15 },  //#29
    { .resistance =  68237, .temperature = -20 },  //#30
    { .resistance =  87559, .temperature = -25 },  //#31
    { .resistance = 113347, .temperature = -30 },  //#32
    { .resistance = 148171, .temperature = -35 },  //#33
    { .resistance = 195652, .temperature = -40 }   //#34
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


/*
 * Print all ADC samples via UART communication port
 */
void ADCScanResultPrint(void)
{
    /* Critical section begins. */
    /* Disable all the interrupts so that ADC scan results can be printed without interruption! */
    /* User can remove the interrupt disable and the interrupt enable pair but the printing would be deformed! */
    PLIB_INT_Disable(INT_ID_0);
    SYS_PRINT("\n\t           ADC SCAN RESULT LIST         \n");
    SYS_PRINT("\t -------------------------------------------------- \n");
    SYS_PRINT("\t INPUT     \t  RAW (DEC)  \t  CONVERTED  \n");
    SYS_PRINT("\t -------------------------------------------------- \n");
    SYS_PRINT("\t V_380V    \t  %d  \t  %.1f mV  \n", analog_voltage_monitorData.adc_raw_data.samples.V380V, analog_voltage_monitorData.adc_converted_data.samples.V380V);
    SYS_PRINT("\t V_325V    \t  %d  \t  %.1f mV  \n", analog_voltage_monitorData.adc_raw_data.samples.V325V, analog_voltage_monitorData.adc_converted_data.samples.V325V);
    SYS_PRINT("\t V_Live    \t  %d  \t  %.1f mV  \n", analog_voltage_monitorData.adc_raw_data.samples.VLIVE, analog_voltage_monitorData.adc_converted_data.samples.VLIVE);
    SYS_PRINT("\t V_Neutral \t  %d  \t  %.1f mV  \n", analog_voltage_monitorData.adc_raw_data.samples.VNEUTRAL, analog_voltage_monitorData.adc_converted_data.samples.VNEUTRAL);
    SYS_PRINT("\t V_18V     \t  %d  \t  %.1f mV  \n", analog_voltage_monitorData.adc_raw_data.samples.V18V, analog_voltage_monitorData.adc_converted_data.samples.V18V);
    SYS_PRINT("\t V_12V     \t  %d  \t  %.1f mV  \n", analog_voltage_monitorData.adc_raw_data.samples.V12V, analog_voltage_monitorData.adc_converted_data.samples.V12V);
    SYS_PRINT("\t V_5V      \t  %d  \t  %.1f mV  \n", analog_voltage_monitorData.adc_raw_data.samples.V5V, analog_voltage_monitorData.adc_converted_data.samples.V5V);
    SYS_PRINT("\t V_3V3_0   \t  %d  \t  %.1f mV  \n", analog_voltage_monitorData.adc_raw_data.samples.V3V3_0, analog_voltage_monitorData.adc_converted_data.samples.V3V3_0);
    SYS_PRINT("\t V_3V3_1   \t  %d  \t  %.1f mV  \n", analog_voltage_monitorData.adc_raw_data.samples.V3V3_1, analog_voltage_monitorData.adc_converted_data.samples.V3V3_1);
    SYS_PRINT("\t V_3V3_2   \t  %d  \t  %.1f mV  \n", analog_voltage_monitorData.adc_raw_data.samples.V3V3_2, analog_voltage_monitorData.adc_converted_data.samples.V3V3_2);
    SYS_PRINT("\t V_3V3_AN1 \t  %d  \t  %.1f mV  \n", analog_voltage_monitorData.adc_raw_data.samples.V3V3AN1, analog_voltage_monitorData.adc_converted_data.samples.V3V3AN1);
    SYS_PRINT("\t V_3V3_AN2 \t  %d  \t  %.1f mV  \n", analog_voltage_monitorData.adc_raw_data.samples.V3V3AN2, analog_voltage_monitorData.adc_converted_data.samples.V3V3AN2);
    SYS_PRINT("\t V_1V8_1   \t  %d  \t  %.1f mV  \n", analog_voltage_monitorData.adc_raw_data.samples.V1V8_1, analog_voltage_monitorData.adc_converted_data.samples.V1V8_1);
    SYS_PRINT("\t V_1V8_2   \t  %d  \t  %.1f mV  \n", analog_voltage_monitorData.adc_raw_data.samples.V1V8_2, analog_voltage_monitorData.adc_converted_data.samples.V1V8_2);
    SYS_PRINT("\t -------------------------------------------------- \n");
    SYS_PRINT("\t IL12      \t  %d  \t  %.1f mA  \n", analog_voltage_monitorData.adc_raw_data.samples.IL12, analog_voltage_monitorData.adc_converted_data.samples.IL12);
    SYS_PRINT("\t IL34      \t  %d  \t  %.1f mA  \n", analog_voltage_monitorData.adc_raw_data.samples.IL34, analog_voltage_monitorData.adc_converted_data.samples.IL34);
    SYS_PRINT("\t -------------------------------------------------- \n");
    SYS_PRINT("\t T_M1      \t  %d  \t  %.1f DegC  \n", analog_voltage_monitorData.adc_raw_data.samples.TEMP_M1, analog_voltage_monitorData.adc_converted_data.samples.TEMP_M1);
    SYS_PRINT("\t T_M2      \t  %d  \t  %.1f DegC  \n", analog_voltage_monitorData.adc_raw_data.samples.TEMP_M2, analog_voltage_monitorData.adc_converted_data.samples.TEMP_M2);
    SYS_PRINT("\t T_PFC12   \t  %d  \t  %.1f DegC  \n", analog_voltage_monitorData.adc_raw_data.samples.TEMP_PFC12, analog_voltage_monitorData.adc_converted_data.samples.TEMP_PFC12);
    SYS_PRINT("\t T_PFC34   \t  %d  \t  %.1f DegC  \n", analog_voltage_monitorData.adc_raw_data.samples.TEMP_PFC34, analog_voltage_monitorData.adc_converted_data.samples.TEMP_PFC34);
    SYS_PRINT("\t T_ELCO    \t  %d  \t  %.1f DegC  \n", analog_voltage_monitorData.adc_raw_data.samples.TEMP_ELCO, analog_voltage_monitorData.adc_converted_data.samples.TEMP_ELCO);
    SYS_PRINT("\t T_BRUG    \t  %d  \t  %.1f DegC  \n", analog_voltage_monitorData.adc_raw_data.samples.TEMP_BRUG, analog_voltage_monitorData.adc_converted_data.samples.TEMP_BRUG);
    SYS_PRINT("\t T_VOED    \t  %d  \t  %.1f DegC  \n", analog_voltage_monitorData.adc_raw_data.samples.TEMP_VOED, analog_voltage_monitorData.adc_converted_data.samples.TEMP_VOED);
    SYS_PRINT("\t -------------------------------------------------- \n");
    /* Critical section ends. */
    PLIB_INT_Enable(INT_ID_0);
}


/*******************************************************************************
 * 
 * The following functions used for ADC raw value to Voltage (mV) 
 * 
 *******************************************************************************/
/*
 * It converts the ADC raw sample value (0 ~ 4095) to the corresponding voltage in mV.
 * 
 * Parameter:
 *      ADC_Raw_Value: ADC raw value read out of the ADC data buffer
 * 
 * Return:
 *      The voltage in mV that corresponds to the ADC raw value       
 */
float ConvertADCRawSampleToVoltage(uint32_t ADC_Raw_Value)
{
    return (float)( ADC_LSB_VOLTAGE_mV * ADC_Raw_Value );
}

/*
 * It converts all ADC raw samples in ADC raw data structure into corresponding voltages. 
 */
void ConvertAllADCRawSamplesToVoltages(void)
{
//    SYS_DEBUG_BreakPoint();
    uint8_t i = 0;
    for( i = 0; i < 23; i++ )
    {
        analog_voltage_monitorData.adc_converted_data.buffer[i] = ConvertADCRawSampleToVoltage(analog_voltage_monitorData.adc_raw_data.buffer[i]);
    }
}


/*******************************************************************************
 * 
 * The following function used for ADC raw value to Current (mA) 
 * 
 *******************************************************************************/
/*
 * It converts the ADC raw sample value (0 ~ 4095) to the corresponding current in mA.
 * 
 * The current sensor IC type is ACS733KLATR-20AB-T, whose output voltage is
 * linearly proportional to input AC/DC current. 
 * 
 * Parameter:
 *      ADC_Raw_Value: ADC raw value read out of the ADC data buffer
 * 
 * Return:
 *      The current in mA that corresponds to the ADC raw value       
 */
float ConvertADCRawSampleToCurrent(uint32_t ADC_Raw_Value)
{
    // The current sensor voltage output in mV
    float viout = 0;
    // The primary input current in mA
    float ipr = 0;
    viout = ConvertADCRawSampleToVoltage(ADC_Raw_Value);
    if( viout >= ZERO_CURRENT_OUTPUT_VOLTAGE )
    {
        ipr = ((viout - ZERO_CURRENT_OUTPUT_VOLTAGE) / CURRENT_SENSITIVIY) * 1000;
    }
    else
    {
        ipr = ((ZERO_CURRENT_OUTPUT_VOLTAGE - viout) / CURRENT_SENSITIVIY) * 1000;
    }
    return ipr;
}

/*******************************************************************************
 * 
 * The following functions used for ADC raw value to Temperature (Celsius Degrees) 
 * 
 *******************************************************************************/
/*
 * It converts ADC raw sample into corresponding thermal resistance value.
 * 
 * Parameter:
 *      ADC_Raw_Value: ADC raw value read out of the ADC data buffer
 * 
 * Return:
 *      The thermal resistance value in ohm that corresponds to the ADC raw value 
 */
float ConvertADCRawSampleToThermalResistance(uint32_t ADC_Raw_Value)
{
    float Rt = 0;   // Thermal Resistance Value
    if( ADC_Raw_Value < ADC_MAX_VALUE )
    {
        Rt = ( (float) ADC_Raw_Value / (float) (ADC_MAX_VALUE - ADC_Raw_Value) ) * REF_RESISTANCE_Ohm;
    }
    else
    {
        Rt = 13513500;  // Rt = 4095 * 3300
    }
    return Rt;
}

/*
 * It compares the thermal resistance value from search element with the thermal resistance value from lookup table.
 * 
 * Precondition:
 *      This function is called repeatedly by bsearch() to compare the search element against the element in lookup table.
 * 
 * Parameter:
 *      pSearchElement: pointer to the element for search
 *      pTableElement:  pointer to the element in the lookup table 
 * 
 * Return:
 *      -1: The element is not found and less than the current table element.
 *       0: The element is found! It is equal to the current table element or in between the current table element and the next table element.
 *       1: The element is not found and bigger than the current table element. 
 */
int compareThermalResistanceInLookupTable(const void * pSearchElement, const void * pTableElement)
{
    if( (*(NTC_LOOKUP_TABLE_TYPE *)pSearchElement).resistance < (*(NTC_LOOKUP_TABLE_TYPE *)pTableElement).resistance )
    {
        // The search element resistance is smaller than the table element resistance.
        // Check if the table element is the first element in the lookup table.
        if( (NTC_LOOKUP_TABLE_TYPE *)pTableElement == (NTC_LOOKUP_TABLE_TYPE *)&NTC_LookupTable[0] )
        {
            // The element is found! The found element is specified as the first element.
            return 0;
        }
        else
        {
            // The element is NOT found!
            return -1;
        }
    }
    
    if( (*(NTC_LOOKUP_TABLE_TYPE *)pSearchElement).resistance == (*(NTC_LOOKUP_TABLE_TYPE *)pTableElement).resistance )
    {
        // The search element resistance is exactly equal to the table element resistance. The element is found!
        return 0;
    }

    if( (*(NTC_LOOKUP_TABLE_TYPE *)pSearchElement).resistance > (*(NTC_LOOKUP_TABLE_TYPE *)pTableElement).resistance )
    {
        // The search element resistance is bigger than the current table element resistance.
        // Check if the table element is the last element in the lookup table.
        if( (NTC_LOOKUP_TABLE_TYPE *)pTableElement == (NTC_LOOKUP_TABLE_TYPE *)&NTC_LookupTable[NTC_LOOKUP_TABLE_SIZE - 1] )
        {
            // The element is found! The found element is specified as the last element. 
            return 0;
        }
        
        if( (*(NTC_LOOKUP_TABLE_TYPE *)pSearchElement).resistance < (*(((NTC_LOOKUP_TABLE_TYPE *)pTableElement) + 1)).resistance )
        {
            // The search element resistance is smaller than the next table element resistance.
            // The search element is in between the current table element and the next table element.
            // The element is found!
            return 0;
        }
        else
        {
            // The search element is NOT in between the current table element and the next table element.
            // The element is NOT found!
            return 1;
        }
    }
    return -1;
}

/*
 * It converts ADC raw sample from the thermal resistor into the corresponding temperature.
 * The NTC lookup table is binary-searched in this function. 
 * 
 * Parameter:
 *      ADC_Raw_Value: ADC raw value read out of the ADC data buffer
 * 
 * Return:
 *      The temperature in celsius degree 
 */
float ConvertADCRawSampleToTemperature(uint32_t ADC_Raw_Value)
{
    // This element is going to be compared with the lookup table elements.
    NTC_LOOKUP_TABLE_TYPE searchElement = {.resistance = 0, .temperature = 0};
    searchElement.resistance = ConvertADCRawSampleToThermalResistance(ADC_Raw_Value);
    if( searchElement.resistance <= NTC_LookupTable[0].resistance )
    {         
        // If the search element resistance is less than the smallest resistance in the lookup table,
        // then the search element temperature is the temperature of the first element in the lookup table.
        searchElement.temperature = NTC_LookupTable[0].temperature;
    }
    else if( searchElement.resistance >= NTC_LookupTable[NTC_LOOKUP_TABLE_SIZE-1].resistance )
    {
        // If the search element resistance is bigger than the biggest resistance in the lookup table,
        // then the search element temperature is the temperature of the last element in the lookup table.
        searchElement.temperature = NTC_LookupTable[NTC_LOOKUP_TABLE_SIZE-1].temperature;
    }
    else
    {
        NTC_LOOKUP_TABLE_TYPE *pFoundTableElement = NULL;
        float slope = 0; // slope for linear interpolation
        // binary search for element in the lookup table
        // bsearch() function is from stdlib.h
        pFoundTableElement = (NTC_LOOKUP_TABLE_TYPE *)bsearch((void *)&searchElement, (void *)NTC_LookupTable, NTC_LOOKUP_TABLE_SIZE, sizeof(NTC_LOOKUP_TABLE_TYPE), compareThermalResistanceInLookupTable);
        if( pFoundTableElement != NULL )
        {
            // calculate slope
            slope = (pFoundTableElement->temperature - (pFoundTableElement + 1)->temperature) / (pFoundTableElement->resistance - (pFoundTableElement + 1)->resistance);
            // interpolate
            searchElement.temperature = (searchElement.resistance - pFoundTableElement->resistance) * slope + pFoundTableElement->temperature;
        }
    }
    return searchElement.temperature;
}


/*******************************************************************************
 * 
 * It converts all ADC raw samples in ADC raw data structure into 
 * corresponding Voltages, Currents and Temperatures.
 *  
 *******************************************************************************/
void ConvertAllADCRawSamples(void)
{
//    SYS_DEBUG_BreakPoint();
    uint8_t i = 0;
    for( i = 0; i < 23; i++ )
    {
        if( i < 14 )
        {
            // The first 14 elements (i = 0 ~ 13) in data buffer are voltages.
            analog_voltage_monitorData.adc_converted_data.buffer[i] = ConvertADCRawSampleToVoltage(analog_voltage_monitorData.adc_raw_data.buffer[i]);
        }
        else if( i < 16 )
        {
            // The elements (i = 14 ~ 15) in data buffer are currents.
            analog_voltage_monitorData.adc_converted_data.buffer[i] = ConvertADCRawSampleToCurrent(analog_voltage_monitorData.adc_raw_data.buffer[i]);            
        }
        else
        {
            // The rest 7 elements (i = 16 ~ 22) in data buffer are temperatures.
            analog_voltage_monitorData.adc_converted_data.buffer[i] = ConvertADCRawSampleToTemperature(analog_voltage_monitorData.adc_raw_data.buffer[i]);
        }
    }
}


/*******************************************************************************
 * 
 * Check whether or not the ADC converted data samples exceed the warning or
 * the fault threshold.
 * 
 *******************************************************************************/
void CheckErrors(void)
{
    /*
     * Compare DC power voltages against voltage warning and fault thresholds.
     */
    // For V-AN-380V
    if( analog_voltage_monitorData.adc_converted_data.samples.V380V <= V380V_UNDERVOLTAGE_FAULT_THRESHOLD )
    {
        error_handlerData.voltageError.status_flags.UNDERVOLTAGE_FAULT_V380V = YES;
    }
    else if( analog_voltage_monitorData.adc_converted_data.samples.V380V <= V380V_UNDERVOLTAGE_WARNING_THRESHOLD )
    {
        error_handlerData.voltageError.status_flags.UNDERVOLTAGE_WARNING_V380V = YES;
    }
    else if( analog_voltage_monitorData.adc_converted_data.samples.V380V >= V380V_OVERVOLTAGE_FAULT_THRESHOLD )
    {
        error_handlerData.voltageError.status_flags.OVERVOLTAGE_FAULT_V380V = YES;
    }
    else if( analog_voltage_monitorData.adc_converted_data.samples.V380V >= V380V_OVERVOLTAGE_WARNING_THRESHOLD )
    {
        error_handlerData.voltageError.status_flags.OVERVOLTAGE_WARNING_V380V = YES;
    }
    else
    {
        error_handlerData.voltageError.status_flags.UNDERVOLTAGE_FAULT_V380V = NO;
        error_handlerData.voltageError.status_flags.UNDERVOLTAGE_WARNING_V380V = NO;
        error_handlerData.voltageError.status_flags.OVERVOLTAGE_FAULT_V380V = NO;
        error_handlerData.voltageError.status_flags.OVERVOLTAGE_WARNING_V380V = NO;
    }
    
    // For V-AN-325V
    if( analog_voltage_monitorData.adc_converted_data.samples.V325V <= V325V_UNDERVOLTAGE_FAULT_THRESHOLD )
    {
        error_handlerData.voltageError.status_flags.UNDERVOLTAGE_FAULT_V325V = YES;
    }
    else if( analog_voltage_monitorData.adc_converted_data.samples.V325V <= V325V_UNDERVOLTAGE_WARNING_THRESHOLD )
    {
        error_handlerData.voltageError.status_flags.UNDERVOLTAGE_WARNING_V325V = YES;
    }
    else if( analog_voltage_monitorData.adc_converted_data.samples.V325V >= V325V_OVERVOLTAGE_FAULT_THRESHOLD )
    {
        error_handlerData.voltageError.status_flags.OVERVOLTAGE_FAULT_V325V = YES;
    }
    else if( analog_voltage_monitorData.adc_converted_data.samples.V325V >= V325V_OVERVOLTAGE_WARNING_THRESHOLD )
    {
        error_handlerData.voltageError.status_flags.OVERVOLTAGE_WARNING_V325V = YES;
    }
    else
    {
        error_handlerData.voltageError.status_flags.UNDERVOLTAGE_FAULT_V325V = NO;
        error_handlerData.voltageError.status_flags.UNDERVOLTAGE_WARNING_V325V = NO;
        error_handlerData.voltageError.status_flags.OVERVOLTAGE_FAULT_V325V = NO;
        error_handlerData.voltageError.status_flags.OVERVOLTAGE_WARNING_V325V = NO;
    }

    // For V-AN-18V
    if( analog_voltage_monitorData.adc_converted_data.samples.V18V <= V18V_UNDERVOLTAGE_FAULT_THRESHOLD )
    {
        error_handlerData.voltageError.status_flags.UNDERVOLTAGE_FAULT_V18V = YES;
    }
    else if( analog_voltage_monitorData.adc_converted_data.samples.V18V <= V18V_UNDERVOLTAGE_WARNING_THRESHOLD )
    {
        error_handlerData.voltageError.status_flags.UNDERVOLTAGE_WARNING_V18V = YES;
    }
    else if( analog_voltage_monitorData.adc_converted_data.samples.V18V >= V18V_OVERVOLTAGE_FAULT_THRESHOLD )
    {
        error_handlerData.voltageError.status_flags.OVERVOLTAGE_FAULT_V18V = YES;
    }
    else if( analog_voltage_monitorData.adc_converted_data.samples.V18V >= V18V_OVERVOLTAGE_WARNING_THRESHOLD )
    {
        error_handlerData.voltageError.status_flags.OVERVOLTAGE_WARNING_V18V = YES;
    }
    else
    {
        error_handlerData.voltageError.status_flags.UNDERVOLTAGE_FAULT_V18V = NO;
        error_handlerData.voltageError.status_flags.UNDERVOLTAGE_WARNING_V18V = NO;
        error_handlerData.voltageError.status_flags.OVERVOLTAGE_FAULT_V18V = NO;
        error_handlerData.voltageError.status_flags.OVERVOLTAGE_WARNING_V18V = NO;
    }    

    // For V-AN-12V
    if( analog_voltage_monitorData.adc_converted_data.samples.V12V <= V12V_UNDERVOLTAGE_FAULT_THRESHOLD )
    {
        error_handlerData.voltageError.status_flags.UNDERVOLTAGE_FAULT_V12V = YES;
    }
    else if( analog_voltage_monitorData.adc_converted_data.samples.V12V <= V12V_UNDERVOLTAGE_WARNING_THRESHOLD )
    {
        error_handlerData.voltageError.status_flags.UNDERVOLTAGE_WARNING_V12V = YES;
    }
    else if( analog_voltage_monitorData.adc_converted_data.samples.V12V >= V12V_OVERVOLTAGE_FAULT_THRESHOLD )
    {
        error_handlerData.voltageError.status_flags.OVERVOLTAGE_FAULT_V12V = YES;
    }
    else if( analog_voltage_monitorData.adc_converted_data.samples.V12V >= V12V_OVERVOLTAGE_WARNING_THRESHOLD )
    {
        error_handlerData.voltageError.status_flags.OVERVOLTAGE_WARNING_V12V = YES;
    }
    else
    {
        error_handlerData.voltageError.status_flags.UNDERVOLTAGE_FAULT_V12V = NO;
        error_handlerData.voltageError.status_flags.UNDERVOLTAGE_WARNING_V12V = NO;
        error_handlerData.voltageError.status_flags.OVERVOLTAGE_FAULT_V12V = NO;
        error_handlerData.voltageError.status_flags.OVERVOLTAGE_WARNING_V12V = NO;
    } 

    // For V-AN-5V
    if( analog_voltage_monitorData.adc_converted_data.samples.V5V <= V5V_UNDERVOLTAGE_FAULT_THRESHOLD )
    {
        error_handlerData.voltageError.status_flags.UNDERVOLTAGE_FAULT_V5V = YES;
    }
    else if( analog_voltage_monitorData.adc_converted_data.samples.V5V <= V5V_UNDERVOLTAGE_WARNING_THRESHOLD )
    {
        error_handlerData.voltageError.status_flags.UNDERVOLTAGE_WARNING_V5V = YES;
    }
    else if( analog_voltage_monitorData.adc_converted_data.samples.V5V >= V5V_OVERVOLTAGE_FAULT_THRESHOLD )
    {
        error_handlerData.voltageError.status_flags.OVERVOLTAGE_FAULT_V5V = YES;
    }
    else if( analog_voltage_monitorData.adc_converted_data.samples.V5V >= V5V_OVERVOLTAGE_WARNING_THRESHOLD )
    {
        error_handlerData.voltageError.status_flags.OVERVOLTAGE_WARNING_V5V = YES;
    }
    else
    {
        error_handlerData.voltageError.status_flags.UNDERVOLTAGE_FAULT_V5V = NO;
        error_handlerData.voltageError.status_flags.UNDERVOLTAGE_WARNING_V5V = NO;
        error_handlerData.voltageError.status_flags.OVERVOLTAGE_FAULT_V5V = NO;
        error_handlerData.voltageError.status_flags.OVERVOLTAGE_WARNING_V5V = NO;
    } 

    // For V-AN-3V3-0
    if( analog_voltage_monitorData.adc_converted_data.samples.V3V3_0 <= V3V3_0_UNDERVOLTAGE_FAULT_THRESHOLD )
    {
        error_handlerData.voltageError.status_flags.UNDERVOLTAGE_FAULT_V3V3_0 = YES;
    }
    else if( analog_voltage_monitorData.adc_converted_data.samples.V3V3_0 <= V3V3_0_UNDERVOLTAGE_WARNING_THRESHOLD )
    {
        error_handlerData.voltageError.status_flags.UNDERVOLTAGE_WARNING_V3V3_0 = YES;
    }
    else if( analog_voltage_monitorData.adc_converted_data.samples.V3V3_0 >= V3V3_0_OVERVOLTAGE_FAULT_THRESHOLD )
    {
        error_handlerData.voltageError.status_flags.OVERVOLTAGE_FAULT_V3V3_0 = YES;
    }
    else if( analog_voltage_monitorData.adc_converted_data.samples.V3V3_0 >= V3V3_0_OVERVOLTAGE_WARNING_THRESHOLD )
    {
        error_handlerData.voltageError.status_flags.OVERVOLTAGE_WARNING_V3V3_0 = YES;
    }
    else
    {
        error_handlerData.voltageError.status_flags.UNDERVOLTAGE_FAULT_V3V3_0 = NO;
        error_handlerData.voltageError.status_flags.UNDERVOLTAGE_WARNING_V3V3_0 = NO;
        error_handlerData.voltageError.status_flags.OVERVOLTAGE_FAULT_V3V3_0 = NO;
        error_handlerData.voltageError.status_flags.OVERVOLTAGE_WARNING_V3V3_0 = NO;
    }

    // For V-AN-3V3-1
    if( analog_voltage_monitorData.adc_converted_data.samples.V3V3_1 <= V3V3_1_UNDERVOLTAGE_FAULT_THRESHOLD )
    {
        error_handlerData.voltageError.status_flags.UNDERVOLTAGE_FAULT_V3V3_1 = YES;
    }
    else if( analog_voltage_monitorData.adc_converted_data.samples.V3V3_1 <= V3V3_1_UNDERVOLTAGE_WARNING_THRESHOLD )
    {
        error_handlerData.voltageError.status_flags.UNDERVOLTAGE_WARNING_V3V3_1 = YES;
    }
    else if( analog_voltage_monitorData.adc_converted_data.samples.V3V3_1 >= V3V3_1_OVERVOLTAGE_FAULT_THRESHOLD )
    {
        error_handlerData.voltageError.status_flags.OVERVOLTAGE_FAULT_V3V3_1 = YES;
    }
    else if( analog_voltage_monitorData.adc_converted_data.samples.V3V3_1 >= V3V3_1_OVERVOLTAGE_WARNING_THRESHOLD )
    {
        error_handlerData.voltageError.status_flags.OVERVOLTAGE_WARNING_V3V3_1 = YES;
    }
    else
    {
        error_handlerData.voltageError.status_flags.UNDERVOLTAGE_FAULT_V3V3_1 = NO;
        error_handlerData.voltageError.status_flags.UNDERVOLTAGE_WARNING_V3V3_1 = NO;
        error_handlerData.voltageError.status_flags.OVERVOLTAGE_FAULT_V3V3_1 = NO;
        error_handlerData.voltageError.status_flags.OVERVOLTAGE_WARNING_V3V3_1 = NO;
    }

    // For V-AN-3V3-2
    if( analog_voltage_monitorData.adc_converted_data.samples.V3V3_2 <= V3V3_2_UNDERVOLTAGE_FAULT_THRESHOLD )
    {
        error_handlerData.voltageError.status_flags.UNDERVOLTAGE_FAULT_V3V3_2 = YES;
    }
    else if( analog_voltage_monitorData.adc_converted_data.samples.V3V3_2 <= V3V3_2_UNDERVOLTAGE_WARNING_THRESHOLD )
    {
        error_handlerData.voltageError.status_flags.UNDERVOLTAGE_WARNING_V3V3_2 = YES;
    }
    else if( analog_voltage_monitorData.adc_converted_data.samples.V3V3_2 >= V3V3_2_OVERVOLTAGE_FAULT_THRESHOLD )
    {
        error_handlerData.voltageError.status_flags.OVERVOLTAGE_FAULT_V3V3_2 = YES;
    }
    else if( analog_voltage_monitorData.adc_converted_data.samples.V3V3_2 >= V3V3_2_OVERVOLTAGE_WARNING_THRESHOLD )
    {
        error_handlerData.voltageError.status_flags.OVERVOLTAGE_WARNING_V3V3_2 = YES;
    }
    else
    {
        error_handlerData.voltageError.status_flags.UNDERVOLTAGE_FAULT_V3V3_2 = NO;
        error_handlerData.voltageError.status_flags.UNDERVOLTAGE_WARNING_V3V3_2 = NO;
        error_handlerData.voltageError.status_flags.OVERVOLTAGE_FAULT_V3V3_2 = NO;
        error_handlerData.voltageError.status_flags.OVERVOLTAGE_WARNING_V3V3_2 = NO;
    }
    
    // For V-AN-3V3AN1
    if( analog_voltage_monitorData.adc_converted_data.samples.V3V3AN1 <= V3V3AN1_UNDERVOLTAGE_FAULT_THRESHOLD )
    {
        error_handlerData.voltageError.status_flags.UNDERVOLTAGE_FAULT_V3V3AN1 = YES;
    }
    else if( analog_voltage_monitorData.adc_converted_data.samples.V3V3AN1 <= V3V3AN1_UNDERVOLTAGE_WARNING_THRESHOLD )
    {
        error_handlerData.voltageError.status_flags.UNDERVOLTAGE_WARNING_V3V3AN1 = YES;
    }
    else if( analog_voltage_monitorData.adc_converted_data.samples.V3V3AN1 >= V3V3AN1_OVERVOLTAGE_FAULT_THRESHOLD )
    {
        error_handlerData.voltageError.status_flags.OVERVOLTAGE_FAULT_V3V3AN1 = YES;
    }
    else if( analog_voltage_monitorData.adc_converted_data.samples.V3V3AN1 >= V3V3AN1_OVERVOLTAGE_WARNING_THRESHOLD )
    {
        error_handlerData.voltageError.status_flags.OVERVOLTAGE_WARNING_V3V3AN1 = YES;
    }
    else
    {
        error_handlerData.voltageError.status_flags.UNDERVOLTAGE_FAULT_V3V3AN1 = NO;
        error_handlerData.voltageError.status_flags.UNDERVOLTAGE_WARNING_V3V3AN1 = NO;
        error_handlerData.voltageError.status_flags.OVERVOLTAGE_FAULT_V3V3AN1 = NO;
        error_handlerData.voltageError.status_flags.OVERVOLTAGE_WARNING_V3V3AN1 = NO;
    }

    // For V-AN-3V3AN2
    if( analog_voltage_monitorData.adc_converted_data.samples.V3V3AN2 <= V3V3AN2_UNDERVOLTAGE_FAULT_THRESHOLD )
    {
        error_handlerData.voltageError.status_flags.UNDERVOLTAGE_FAULT_V3V3AN2 = YES;
    }
    else if( analog_voltage_monitorData.adc_converted_data.samples.V3V3AN2 <= V3V3AN2_UNDERVOLTAGE_WARNING_THRESHOLD )
    {
        error_handlerData.voltageError.status_flags.UNDERVOLTAGE_WARNING_V3V3AN2 = YES;
    }
    else if( analog_voltage_monitorData.adc_converted_data.samples.V3V3AN2 >= V3V3AN2_OVERVOLTAGE_FAULT_THRESHOLD )
    {
        error_handlerData.voltageError.status_flags.OVERVOLTAGE_FAULT_V3V3AN2 = YES;
    }
    else if( analog_voltage_monitorData.adc_converted_data.samples.V3V3AN2 >= V3V3AN2_OVERVOLTAGE_WARNING_THRESHOLD )
    {
        error_handlerData.voltageError.status_flags.OVERVOLTAGE_WARNING_V3V3AN2 = YES;
    }
    else
    {
        error_handlerData.voltageError.status_flags.UNDERVOLTAGE_FAULT_V3V3AN2 = NO;
        error_handlerData.voltageError.status_flags.UNDERVOLTAGE_WARNING_V3V3AN2 = NO;
        error_handlerData.voltageError.status_flags.OVERVOLTAGE_FAULT_V3V3AN2 = NO;
        error_handlerData.voltageError.status_flags.OVERVOLTAGE_WARNING_V3V3AN2 = NO;
    }    

    // For V-AN-1V8-1
    if( analog_voltage_monitorData.adc_converted_data.samples.V1V8_1 <= V1V8_1_UNDERVOLTAGE_FAULT_THRESHOLD )
    {
        error_handlerData.voltageError.status_flags.UNDERVOLTAGE_FAULT_V1V8_1 = YES;
    }
    else if( analog_voltage_monitorData.adc_converted_data.samples.V1V8_1 <= V1V8_1_UNDERVOLTAGE_WARNING_THRESHOLD )
    {
        error_handlerData.voltageError.status_flags.UNDERVOLTAGE_WARNING_V1V8_1 = YES;
    }
    else if( analog_voltage_monitorData.adc_converted_data.samples.V1V8_1 >= V1V8_1_OVERVOLTAGE_FAULT_THRESHOLD )
    {
        error_handlerData.voltageError.status_flags.OVERVOLTAGE_FAULT_V1V8_1 = YES;
    }
    else if( analog_voltage_monitorData.adc_converted_data.samples.V1V8_1 >= V1V8_1_OVERVOLTAGE_WARNING_THRESHOLD )
    {
        error_handlerData.voltageError.status_flags.OVERVOLTAGE_WARNING_V1V8_1 = YES;
    }
    else
    {
        error_handlerData.voltageError.status_flags.UNDERVOLTAGE_FAULT_V1V8_1 = NO;
        error_handlerData.voltageError.status_flags.UNDERVOLTAGE_WARNING_V1V8_1 = NO;
        error_handlerData.voltageError.status_flags.OVERVOLTAGE_FAULT_V1V8_1 = NO;
        error_handlerData.voltageError.status_flags.OVERVOLTAGE_WARNING_V1V8_1 = NO;
    }

    // For V-AN-1V8-2
    if( analog_voltage_monitorData.adc_converted_data.samples.V1V8_2 <= V1V8_2_UNDERVOLTAGE_FAULT_THRESHOLD )
    {
        error_handlerData.voltageError.status_flags.UNDERVOLTAGE_FAULT_V1V8_2 = YES;
    }
    else if( analog_voltage_monitorData.adc_converted_data.samples.V1V8_2 <= V1V8_2_UNDERVOLTAGE_WARNING_THRESHOLD )
    {
        error_handlerData.voltageError.status_flags.UNDERVOLTAGE_WARNING_V1V8_2 = YES;
    }
    else if( analog_voltage_monitorData.adc_converted_data.samples.V1V8_2 >= V1V8_2_OVERVOLTAGE_FAULT_THRESHOLD )
    {
        error_handlerData.voltageError.status_flags.OVERVOLTAGE_FAULT_V1V8_2 = YES;
    }
    else if( analog_voltage_monitorData.adc_converted_data.samples.V1V8_2 >= V1V8_2_OVERVOLTAGE_WARNING_THRESHOLD )
    {
        error_handlerData.voltageError.status_flags.OVERVOLTAGE_WARNING_V1V8_2 = YES;
    }
    else
    {
        error_handlerData.voltageError.status_flags.UNDERVOLTAGE_FAULT_V1V8_2 = NO;
        error_handlerData.voltageError.status_flags.UNDERVOLTAGE_WARNING_V1V8_2 = NO;
        error_handlerData.voltageError.status_flags.OVERVOLTAGE_FAULT_V1V8_2 = NO;
        error_handlerData.voltageError.status_flags.OVERVOLTAGE_WARNING_V1V8_2 = NO;
    }

    /*
     * Compare current load with current warning and fault thresholds.
     */
    // For current load IL12
    if( analog_voltage_monitorData.adc_converted_data.samples.IL12 < OVERCURRENT_WARNING_THRESHOLD )
    {
        error_handlerData.currentError.status_flags.OVERCURRENT_WARNING_IL12 = NO;
        error_handlerData.currentError.status_flags.OVERCURRENT_FAULT_IL12 = NO;
    }
    else if( analog_voltage_monitorData.adc_converted_data.samples.IL12 < OVERCURRENT_FAULT_THRESHOLD )
    {
        error_handlerData.currentError.status_flags.OVERCURRENT_WARNING_IL12 = YES;
    }
    else
    {
        //NOTE: This overcurrent fault IL12 flag is also set by ISR for change notification on Port C13 
        error_handlerData.currentError.status_flags.OVERCURRENT_FAULT_IL12 = YES;
    }
    
    // For current load IL34
    if( analog_voltage_monitorData.adc_converted_data.samples.IL34 < OVERCURRENT_WARNING_THRESHOLD )
    {
        error_handlerData.currentError.status_flags.OVERCURRENT_WARNING_IL34 = NO;
        error_handlerData.currentError.status_flags.OVERCURRENT_FAULT_IL34 = NO;
    }
    else if( analog_voltage_monitorData.adc_converted_data.samples.IL34 < OVERCURRENT_FAULT_THRESHOLD )
    {
        error_handlerData.currentError.status_flags.OVERCURRENT_WARNING_IL34 = YES;
    }
    else
    {
        //NOTE: This overcurrent fault IL34 flag is also set by ISR for change notification on Port D6 
        error_handlerData.currentError.status_flags.OVERCURRENT_FAULT_IL34 = YES;
    }    

    /*
     * Compare the temperature from the NTC resistor with overheat warning and fault thresholds.
     */
    // For overheat on Motor 1
    if( analog_voltage_monitorData.adc_converted_data.samples.TEMP_M1 < MOTOR_OVERHEAT_WARNING_THRESHOLD )
    {
        error_handlerData.temperatureError.status_flags.OVERHEAT_WARNING_M1 = NO;
        error_handlerData.temperatureError.status_flags.OVERHEAT_FAULT_M1 = NO;
    }
    else if( analog_voltage_monitorData.adc_converted_data.samples.TEMP_M1 < MOTOR_OVERHEAT_FAULT_THRESHOLD )
    {
        error_handlerData.temperatureError.status_flags.OVERHEAT_WARNING_M1 = YES;
    }
    else
    {
        error_handlerData.temperatureError.status_flags.OVERHEAT_FAULT_M1 = YES;
    }
    
    // For overheat on Motor 2
    if( analog_voltage_monitorData.adc_converted_data.samples.TEMP_M2 < MOTOR_OVERHEAT_WARNING_THRESHOLD )
    {
        error_handlerData.temperatureError.status_flags.OVERHEAT_WARNING_M2 = NO;
        error_handlerData.temperatureError.status_flags.OVERHEAT_FAULT_M2 = NO;        
    }
    else if( analog_voltage_monitorData.adc_converted_data.samples.TEMP_M2 < MOTOR_OVERHEAT_FAULT_THRESHOLD )
    {
        error_handlerData.temperatureError.status_flags.OVERHEAT_WARNING_M2 = YES;
    }
    else
    {
        error_handlerData.temperatureError.status_flags.OVERHEAT_FAULT_M2 = YES;
    }
    
    // For overheat on PFC 12
    if( analog_voltage_monitorData.adc_converted_data.samples.TEMP_PFC12 < PFC_OVERHEAT_WARNING_THRESHOLD )
    {
        error_handlerData.temperatureError.status_flags.OVERHEAT_WARNING_PFC12 = NO;
        error_handlerData.temperatureError.status_flags.OVERHEAT_FAULT_PFC12 = NO; 
    }
    else if( analog_voltage_monitorData.adc_converted_data.samples.TEMP_PFC12 < PFC_OVERHEAT_FAULT_THRESHOLD )
    {
        error_handlerData.temperatureError.status_flags.OVERHEAT_WARNING_PFC12 = YES;
    }
    else
    {
        error_handlerData.temperatureError.status_flags.OVERHEAT_FAULT_PFC12 = YES;
    }
    
    // For overheat on PFC 34
    if( analog_voltage_monitorData.adc_converted_data.samples.TEMP_PFC34 < PFC_OVERHEAT_WARNING_THRESHOLD )
    {
        error_handlerData.temperatureError.status_flags.OVERHEAT_WARNING_PFC34 = NO;
        error_handlerData.temperatureError.status_flags.OVERHEAT_FAULT_PFC34 = NO; 
    }
    else if( analog_voltage_monitorData.adc_converted_data.samples.TEMP_PFC34 < PFC_OVERHEAT_FAULT_THRESHOLD )
    {
        error_handlerData.temperatureError.status_flags.OVERHEAT_WARNING_PFC34 = YES;
    }
    else
    {
        error_handlerData.temperatureError.status_flags.OVERHEAT_FAULT_PFC34 = YES;
    }
    
    // For overheat on ELCO
    if( analog_voltage_monitorData.adc_converted_data.samples.TEMP_ELCO < ELCO_OVERHEAT_WARNING_THRESHOLD )
    {
        error_handlerData.temperatureError.status_flags.OVERHEAT_WARNING_ELCO = NO;
        error_handlerData.temperatureError.status_flags.OVERHEAT_FAULT_ELCO = NO;         
    }
    else if( analog_voltage_monitorData.adc_converted_data.samples.TEMP_ELCO < ELCO_OVERHEAT_FAULT_THRESHOLD )
    {
        error_handlerData.temperatureError.status_flags.OVERHEAT_WARNING_ELCO = YES;
    }
    else
    {
        error_handlerData.temperatureError.status_flags.OVERHEAT_FAULT_ELCO = YES;       
    }
    
    // For overheat on BRUG
    if( analog_voltage_monitorData.adc_converted_data.samples.TEMP_BRUG < BRUG_OVERHEAT_WARNING_THRESHOLD )
    {
        error_handlerData.temperatureError.status_flags.OVERHEAT_WARNING_BRUG = NO;
        error_handlerData.temperatureError.status_flags.OVERHEAT_FAULT_BRUG = NO;         
    }
    else if( analog_voltage_monitorData.adc_converted_data.samples.TEMP_BRUG < BRUG_OVERHEAT_FAULT_THRESHOLD )
    {
        error_handlerData.temperatureError.status_flags.OVERHEAT_WARNING_BRUG = YES;
    }
    else
    {
        error_handlerData.temperatureError.status_flags.OVERHEAT_FAULT_BRUG = YES;        
    }
    
    // For overheat on VOED
    if( analog_voltage_monitorData.adc_converted_data.samples.TEMP_VOED < VOED_OVERHEAT_WARNING_THRESHOLD )
    {
        error_handlerData.temperatureError.status_flags.OVERHEAT_WARNING_VOED = NO;
        error_handlerData.temperatureError.status_flags.OVERHEAT_FAULT_VOED = NO;         
    }
    else if( analog_voltage_monitorData.adc_converted_data.samples.TEMP_VOED < VOED_OVERHEAT_FAULT_THRESHOLD )
    {
        error_handlerData.temperatureError.status_flags.OVERHEAT_WARNING_VOED = YES;
    }
    else
    {
        error_handlerData.temperatureError.status_flags.OVERHEAT_FAULT_VOED = YES;        
    }
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
    /* Clear the ADC data buffer in initial state. */
    memset(analog_voltage_monitorData.adc_raw_data.buffer, 0, sizeof(analog_voltage_monitorData.adc_raw_data.buffer));
    
    /* Open all DC powers */
//    EnableDCPower();
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
//            SYS_DEBUG_BreakPoint();           
            if (appInitialized)
            {            
                analog_voltage_monitorData.state = ANALOG_VOLTAGE_MONITOR_STATE_SCAN;
                /* Clear the ADC data buffer in initial state. */
                memset(analog_voltage_monitorData.adc_raw_data.buffer, 0, sizeof(analog_voltage_monitorData.adc_raw_data.buffer));
                memset(analog_voltage_monitorData.adc_converted_data.buffer, 0, sizeof(analog_voltage_monitorData.adc_converted_data.buffer));
                /* Start the ADC */
//                ADCCON3bits.DIGEN0 = 1;
                /* Enable the channel digital features. */
                DRV_ADC0_Open();
                DRV_ADC1_Open();
                DRV_ADC2_Open();
                DRV_ADC4_Open();
                DRV_ADC5_Open();
                DRV_ADC6_Open();
                /* Enable the global software EDGE trigger for analog input scanning. */
                /* The global software trigger bit is cleared automatically in the next ADC clock cycle. */
                DRV_ADC_Start();
            }
            break;
        }

        /* TODO: implement your application state machine.*/
        case ANALOG_VOLTAGE_MONITOR_STATE_SCAN:
        {
//            V_LED1_GOn();
//            SYS_DEBUG_BreakPoint();
            analog_voltage_monitorData.state = ANALOG_VOLTAGE_MONITOR_STATE_SCAN_DONE;
            break;
        }        

        case ANALOG_VOLTAGE_MONITOR_STATE_SCAN_DONE:
        {
            /* Check if every ADC data is updated in the ADC buffer */
            // AN5 can not be read out right now so its status flag is left ZERO.
            // Note: 0x007F7FFF == 0b00000000011111110111111111111111
            if (analog_voltage_monitorData.adc_raw_data.samples.update.status == 0x007F7FFF)
            {
//                V_LED1_GOff();
                // If the ADC data are all updated, then go to next state for data conversion. 
                analog_voltage_monitorData.state = ANALOG_VOLTAGE_MONITOR_STATE_CONVERT;
            }
            else
            {
                // If the ADC data are not updated completely, then go to next state for scan.
                analog_voltage_monitorData.state = ANALOG_VOLTAGE_MONITOR_STATE_SCAN;                
            }
            break;
        }

        case ANALOG_VOLTAGE_MONITOR_STATE_CONVERT:
        {
            ConvertAllADCRawSamples();
            analog_voltage_monitorData.state = ANALOG_VOLTAGE_MONITOR_STATE_ERROR_CHECK;
            break;
        }

        case ANALOG_VOLTAGE_MONITOR_STATE_ERROR_CHECK:
        {
            CheckErrors();
            analog_voltage_monitorData.state = ANALOG_VOLTAGE_MONITOR_STATE_DISPLAY;
            break;
        }        
        
        case ANALOG_VOLTAGE_MONITOR_STATE_DISPLAY:
        {
//            V_LED1_ROn();
//            SYS_DEBUG_BreakPoint();
            ADCScanResultPrint();
//            V_LED1_ROff();
            analog_voltage_monitorData.state = ANALOG_VOLTAGE_MONITOR_STATE_INIT;
            break;
        }
                
        /* The default state should never be executed. */
        default:
        {
            /* TODO: Handle error in application's state machine. */
            SYS_PRINT("\nFault: Analog voltage monitor in WRONG state!\r\n");
            /* Return to the initial state. */
            analog_voltage_monitorData.state = ANALOG_VOLTAGE_MONITOR_STATE_INIT;            
            break;
        }
    }
}


/*******************************************************************************
 End of File
 */
