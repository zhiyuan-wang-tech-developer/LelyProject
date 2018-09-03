#include <stddef.h>                     // Defines NULL
#include <stdbool.h>                    // Defines true
#include <stdlib.h>                     // Defines EXIT_FAILURE
#include "system/common/sys_module.h"   // SYS function prototypes
#include "system/debug/sys_debug.h"
#include "system_config.h"
#include "analog_voltage_monitor.h"
#include <peripheral/peripheral.h>


extern void ConvertAllADCRawSamplesToVoltages(void);
extern ANALOG_VOLTAGE_MONITOR_DATA analog_voltage_monitorData;

static ANALOG_VOLTAGE_MONITOR_DATA adcData;


static void test_init_adc(){
    DRV_ADC_Initialize();
    
    DRV_ADC0_Open();
    DRV_ADC1_Open();
    DRV_ADC2_Open();
    DRV_ADC4_Open();
    DRV_ADC5_Open();
    DRV_ADC6_Open();
    
    /* Enable the global software EDGE trigger for analog input scanning. */
    /* The global software trigger bit is cleared automatically in the next ADC clock cycle. */
    DRV_ADC_Start();

    //Set ADC0 = Read AN5 == IL34
    PLIB_ADCHS_ChannelInputSelect(ADCHS_ID_0, ADCHS_CHANNEL_0, ADCHS_CHANNEL_0_ALTERNATE_INP_AN5);

    //Set ADC1 = Read AN4 == VN
    PLIB_ADCHS_ChannelInputSelect(ADCHS_ID_0, ADCHS_CHANNEL_1, ADCHS_CHANNEL_1_ALTERNATE_INP_AN4);

    //Set ADC2 = Read AN6 == VL
    PLIB_ADCHS_ChannelInputSelect(ADCHS_ID_0, ADCHS_CHANNEL_2, ADCHS_CHANNEL_2_ALTERNATE_INP_AN6);
}

static void test_update_adcValues(ANALOG_VOLTAGE_MONITOR_DATA* data){
    ConvertAllADCRawSamplesToVoltages();
    memcpy(data, &analog_voltage_monitorData, sizeof(ANALOG_VOLTAGE_MONITOR_DATA));
}

static void test_Piccolo1_PowerOn(){
    //Enable 3V3-1
    V_EN_3V3_1On();    
    //Wait for 3V3-1 to be fully powered up
    do{    
        test_update_adcValues(&adcData);
    }while( adcData.adc_raw_data.samples.V3V3_1 < V3V3_1_UNDERVOLTAGE_WARNING_THRESHOLD );
    
            
    //Enable 3V3AN-1
    V_EN_3V3_AN1On();    
    //Wait for 3V3AN-1 to be fully powered up
    do{    
        test_update_adcValues(&adcData);
    }while( adcData.adc_raw_data.samples.V3V3AN1 < V3V3AN1_UNDERVOLTAGE_WARNING_THRESHOLD );
    
    
    //Enable 1V8-1
    V_EN_1V8_1On();    
    //Wait for 1V8-1 to be fully powered up
    do{    
        test_update_adcValues(&adcData);
    }while( adcData.adc_raw_data.samples.V1V8_1 < V1V8_1_UNDERVOLTAGE_WARNING_THRESHOLD );
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
    do{    
        test_update_adcValues(&adcData);
    }while( adcData.adc_raw_data.samples.V3V3_2 < V3V3_2_UNDERVOLTAGE_WARNING_THRESHOLD );
    
            
    //Enable 3V3AN-2
    V_EN_3V3_AN2On();    
    //Wait for 3V3AN-2 to be fully powered up
    do{    
        test_update_adcValues(&adcData);
    }while( adcData.adc_raw_data.samples.V3V3AN2 < V3V3AN2_UNDERVOLTAGE_WARNING_THRESHOLD );
    
    
    //Enable 1V8-2
    V_EN_1V8_2On();    
    //Wait for 1V8-2 to be fully powered up
    do{    
        test_update_adcValues(&adcData);
    }while( adcData.adc_raw_data.samples.V1V8_2 < V1V8_2_UNDERVOLTAGE_WARNING_THRESHOLD );    
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


void test_startup(){
    // Initialize drivers
    SYS_Initialize ( NULL );
    
    test_init_adc();
    
    DRV_ADC_Start();

    while( false == PLIB_ADCHS_AnalogInputDataIsReady(ADCHS_ID_0, ADCHS_AN0) );
                
    //Wait for 5V to be fully powered up
    do{
        test_update_adcValues(&adcData);
    }while( adcData.adc_raw_data.samples.V5V < V5V_UNDERVOLTAGE_WARNING_THRESHOLD );

    
    //Enable 12V
    V_EN_12VOn();    
    //Wait for 12V to be fully powered up
    do{    
        test_update_adcValues(&adcData);
    }while( adcData.adc_raw_data.samples.V12V < V12V_UNDERVOLTAGE_WARNING_THRESHOLD );    
    
    
    test_Piccolo_Power();
}


