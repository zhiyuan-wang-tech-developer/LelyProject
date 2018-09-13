#include <stddef.h>                     // Defines NULL
#include <stdbool.h>                    // Defines true
#include <stdlib.h>                     // Defines EXIT_FAILURE
#include "system/common/sys_module.h"   // SYS function prototypes
#include "system/debug/sys_debug.h"
#include "system_config.h"
#include "analog_voltage_monitor.h"
#include <peripheral/peripheral.h>

#include "test_adc.h"
#include "test_pwm.h"
#include "test_params.h"
#include "test_uart.h"



extern volatile ANALOG_VOLTAGE_MONITOR_DATA analog_voltage_monitorData;
extern PWM_CONTROLLER_DATA pwm_controllerData;

static void test_Piccolo1_PowerOn(){
    //Enable 3V3-1
    V_EN_3V3_1On();    
    //Wait for 3V3-1 to be fully powered up    
    test_wait_3V3_1(V3V3_1_UNDERVOLTAGE_WARNING_THRESHOLD);
    
            
    //Enable 3V3AN-1
    V_EN_3V3_AN1On();    
    //Wait for 3V3AN-1 to be fully powered up
    test_wait_3V3AN_1(V3V3AN1_UNDERVOLTAGE_WARNING_THRESHOLD);
        
    //Enable 1V8-1
    V_EN_1V8_1On();    
    //Wait for 1V8-1 to be fully powered up
    test_wait_1V8_1(V1V8_1_UNDERVOLTAGE_WARNING_THRESHOLD);
}

static void test_Piccolo1_PowerOff(){
    V_EN_1V8_1Off();
    V_EN_3V3_AN1Off();
    V_EN_3V3_1Off();
}

static void test_Piccolo2_PowerOn(){
    
    //Enable 3V3-2
    V_EN_3V3_2On();        
    //Wait for 3V3-2 to be fully powered up
    test_wait_3V3_2(V3V3_2_UNDERVOLTAGE_WARNING_THRESHOLD);
                
    //Enable 3V3AN-2
    V_EN_3V3_AN2On();    
    //Wait for 3V3AN-2 to be fully powered up
    test_wait_3V3AN_2(V3V3AN2_UNDERVOLTAGE_WARNING_THRESHOLD);
    
    
    //Enable 1V8-2
    V_EN_1V8_2On();    
    //Wait for 1V8-2 to be fully powered up
    test_wait_1V8_2( V1V8_2_UNDERVOLTAGE_WARNING_THRESHOLD );    
}

static void test_Piccolo2_PowerOff(){
    V_EN_1V8_2Off();
    V_EN_3V3_AN2Off();
    V_EN_3V3_2Off();
}

static void test_Piccolo_Power(){
    //Enable Piccolo power
    test_Piccolo1_PowerOn();
    test_Piccolo2_PowerOn();

    SYS_DEBUG_BreakPoint();
    
    //Disable Piccolo power
    test_Piccolo1_PowerOff();
    test_Piccolo2_PowerOff();

}


void test_comparator(){
     uint16_t cdac_threshold = ((3300.0 / 2.0) + (-11.0 * 66.0)) * (4095.0 / 3300.0);

    //Disable CDAC before (re-)configuring
    PLIB_CDAC_Disable(CDAC_ID_3);
    
    //Configure Comparator 1 threshold (CDAC3)
    PLIB_CDAC_OutputDisable(CDAC_ID_3, CDAC_OUTPUT1);               //No output to pin
    PLIB_CDAC_ReferenceVoltageSelect(CDAC_ID_3, CDAC_VREF_AVDD);    //Positive reference = AVdd
    PLIB_CDAC_DataWrite(CDAC_ID_3, cdac_threshold);                 //Set target value
    
    //Enable CDAC
    PLIB_CDAC_Enable(CDAC_ID_3);

    //Disable Comparator before (re-)configuring
    PLIB_CMP_Disable(CMP_ID_1);
    
    //Disable outputs
    PLIB_CMP_OpAmpOutputDisable(CMP_ID_1);

    //Enable outputs
    PLIB_CMP_OutputEnable(CMP_ID_1);
        
    //Disable OpAmp mode
    PLIB_CMP_OpAmpDisable(CMP_ID_1);
    
    //Compare CDAC3 to C1IN1- = V-AN-IL34    
    PLIB_CMP_NonInvertingInputChannelSelect(CMP_ID_1, CMP_NON_INVERTING_INPUT_CDAC);
    PLIB_CMP_InvertingInputChannelSelect(CMP_ID_1, CMP_INVERTING_INPUT_1);    
    
    //Configure polarity
    //Invert output: COUT = Vin+ < Vin- => COUT = V-AN-IL34 > CDAC3
    //Normal output: COUT = Vin+ > Vin- => COUT = V-AN-IL34 < CDAC3
    PLIB_CMP_OutputInvertDisable(CMP_ID_1);
//    PLIB_CMP_OutputInvertEnable(CMP_ID_1);
    
    PLIB_CMP_Enable(CMP_ID_1);
    
    

    // Remap C1OUT -> LED 2 Green
    PLIB_PORTS_RemapOutput(PORTS_ID_0, OUTPUT_FUNC_C1OUT, OUTPUT_PIN_RPE15);
}

void test_startup(){
    volatile bool flag = true;
    
    SYS_DEBUG_BreakPoint();
    
    // Initialize drivers
    SYS_Initialize ( NULL );

    //Disable pre-fetch
    CHECONbits.PREFEN = 0;
    
    test_init_adc();

    
    
    ADC_SCAN_UPDATE_STATUS updateAll;
    updateAll.status = -1;
    updateAll.status_bits.UNUSED = 0;    
    const uint32_t allUpdated = updateAll.status;
    
    
    
    
    test_comparator();
    

    SYS_DEBUG_BreakPoint();
    
    
    //Try single ADC    
    while( PLIB_ADCHS_AnalogInputDataIsReady(ADCHS_ID_0, ADCHS_AN17) ){
        PLIB_ADCHS_AnalogInputResultGet(ADCHS_ID_0, ADCHS_AN17);
    }
    PLIB_ADCHS_SoftwareConversionInputSelect(ADCHS_ID_0, ADCHS_AN17);
    PLIB_ADCHS_SoftwareConversionStart(ADCHS_ID_0);    
    while( !PLIB_ADCHS_AnalogInputDataIsReady(ADCHS_ID_0, ADCHS_AN17) );
    
    SYS_DEBUG_BreakPoint();
    
    
    analog_voltage_monitorData.adc_raw_data.samples.update.status = 0;

    test_adc_enableSlowADC(true);
    test_adc_enableFastADC(true);
    
    while( flag ){
        if( analog_voltage_monitorData.adc_raw_data.samples.update.status == allUpdated ){
            //wait for first conversion
            SYS_DEBUG_BreakPoint();
            flag = false;
//            V_LED1_GToggle();
//            test_adc_convertValues(&analog_voltage_monitorData);
//            analog_voltage_monitorData.adc_raw_data.samples.update.status = 0;
//            DRV_ADC_Start();
        }
    }
    
    SYS_DEBUG_BreakPoint();

    test_uart_init();
  
    SYS_DEBUG_BreakPoint();
    
    T2CON = 0;
    IFS0bits.T2IF = 0;
    TMR2 = 0;
    
    SYS_DEBUG_BreakPoint();
    
    
    //Wait for 5V to be fully powered up
    test_wait_5V( V5V_UNDERVOLTAGE_WARNING_THRESHOLD );
    
    //Enable 12V
    V_EN_12VOn();    
    //Wait for 12V to be fully powered up
    test_wait_12V( V12V_UNDERVOLTAGE_WARNING_THRESHOLD );    

    test_Piccolo_Power();

    test_pwm();
}


    

