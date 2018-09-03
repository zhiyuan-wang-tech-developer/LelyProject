#include "test_adc.h"
#include "analog_voltage_monitor.h"

extern void ConvertAllADCRawSamplesToVoltages(void);
extern ANALOG_VOLTAGE_MONITOR_DATA analog_voltage_monitorData;

static ANALOG_VOLTAGE_MONITOR_DATA adcData;


void test_init_adc(){
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

void test_update_adcValues(ANALOG_VOLTAGE_MONITOR_DATA* data){
    ConvertAllADCRawSamplesToVoltages();
    memcpy(data, &analog_voltage_monitorData, sizeof(ANALOG_VOLTAGE_MONITOR_DATA));
}


//------------------------------------------------------------------------------
// Common power
void test_wait_18V(uint16_t targetADC){
    do{
        test_update_adcValues(&adcData);
    }while( adcData.adc_raw_data.samples.V18V < targetADC );
}
void test_wait_12V(uint16_t targetADC){
    do{
        test_update_adcValues(&adcData);
    }while( adcData.adc_raw_data.samples.V12V < targetADC );
}
void test_wait_5V(uint16_t targetADC){
    do{
        test_update_adcValues(&adcData);
    }while( adcData.adc_raw_data.samples.V5V< targetADC );
}


//------------------------------------------------------------------------------
// Net power
void test_wait_380V(uint16_t targetADC){
    do{
        test_update_adcValues(&adcData);
    }while( adcData.adc_raw_data.samples.V380V < targetADC );
}
void test_wait_325V(uint16_t targetADC){
    do{
        test_update_adcValues(&adcData);
    }while( adcData.adc_raw_data.samples.V325V < targetADC );
}


//------------------------------------------------------------------------------
// Piccolo 1 power
void test_wait_3V3_1(uint16_t targetADC){
    do{
        test_update_adcValues(&adcData);
    }while( adcData.adc_raw_data.samples.V3V3_1 < targetADC );
}
void test_wait_3V3AN_1(uint16_t targetADC){
    do{
        test_update_adcValues(&adcData);
    }while( adcData.adc_raw_data.samples.V3V3AN1 < targetADC );
}
void test_wait_1V8_1(uint16_t targetADC){
    do{
        test_update_adcValues(&adcData);
    }while( adcData.adc_raw_data.samples.V1V8_1 < targetADC );
}

//------------------------------------------------------------------------------
// Piccolo 2 power
void test_wait_3V3_2(uint16_t targetADC){
    do{
        test_update_adcValues(&adcData);
    }while( adcData.adc_raw_data.samples.V3V3_2 < targetADC );
    
}
void test_wait_3V3AN_2(uint16_t targetADC){
    do{
        test_update_adcValues(&adcData);
    }while( adcData.adc_raw_data.samples.V3V3AN2 < targetADC );
}
void test_wait_1V8_2(uint16_t targetADC){
    do{
        test_update_adcValues(&adcData);
    }while( adcData.adc_raw_data.samples.V1V8_2 < targetADC );
}

