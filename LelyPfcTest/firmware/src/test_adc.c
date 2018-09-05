#include "test_adc.h"
#include "analog_voltage_monitor.h"
#include "led_controller.h"


extern float ConvertADCRawSampleToThermalResistance(uint32_t ADC_Raw_Value);
extern float ConvertADCRawSampleToCurrent(uint32_t ADC_Raw_Value);

extern ANALOG_VOLTAGE_MONITOR_DATA analog_voltage_monitorData;

static ANALOG_VOLTAGE_MONITOR_DATA adcData;

void __ISR(_ADC_DF1_VECTOR, ipl3AUTO) _IntHandlerDrvAdc_Filter1(void){
    V_LED1_GOn();
    
    //Read filtered value
    analog_voltage_monitorData.adc_raw_data.samples.IL34 = PLIB_ADCHS_DigitalFilterDataGet(ADCHS_ID_0, ADCHS_DIGITAL_FILTER_1);
       
    //Clear interrupt flag
    IFS3bits.AD1DF1IF = 0;
    
    V_LED1_GOff();
}



void test_init_adc(){
    DRV_ADC_Initialize();
    
    DRV_ADC0_Open();
    DRV_ADC1_Open();
    DRV_ADC2_Open();
    DRV_ADC4_Open();
    DRV_ADC5_Open();
    DRV_ADC6_Open();
    
    //Set ADC0 = Read AN5 == IL34
    //Pass through digital averaging filter
    PLIB_ADCHS_ChannelInputSelect(ADCHS_ID_0, ADCHS_CHANNEL_0, ADCHS_CHANNEL_0_ALTERNATE_INP_AN5);
    PLIB_ADCHS_DigitalFilterAveragingModeSetup(ADCHS_ID_0, ADCHS_DIGITAL_FILTER_1, ADCHS_AN5, ADCHS_DIGITAL_FILTER_SIGNIFICANT_FIRST_12BITS, ADCHS_DIGITAL_FILTER_AVERAGE_SAMPLE_COUNT_4, true);
    
    //Triggered by TMR3
    PLIB_ADCHS_AnalogInputTriggerSourceSelect(ADCHS_ID_0, ADCHS_CLASS12_AN5, ADCHS_TRIGGER_SOURCE_TMR3_MATCH);
    PLIB_ADCHS_ChannelTriggerSampleSelect(ADCHS_ID_0, ADCHS_CHANNEL_0, ADCHS_CHANNEL_UNSYNC_TRIGGER_UNSYNC_SAMPLING);
    
    //Use Filter Ready interrupt instead of data ready
    PLIB_ADCHS_AnalogInputDataReadyInterruptDisable(ADCHS_ID_0, ADCHS_AN5);
    PLIB_ADCHS_DigitalFilterDataReadyInterruptEnable(ADCHS_ID_0, ADCHS_DIGITAL_FILTER_1);
    
    //PWM period = 1200 = 100 KHz
    //ADC period = 4* PWM = 300 = 400 KHz
    T3CON = 0;
    PR3 = 300;
    T3CON = 0x8000;
        
    //Set ADC1 = Read AN4 == VN
    PLIB_ADCHS_ChannelInputSelect(ADCHS_ID_0, ADCHS_CHANNEL_1, ADCHS_CHANNEL_1_ALTERNATE_INP_AN4);

    //Set ADC2 = Read AN6 == VL
    PLIB_ADCHS_ChannelInputSelect(ADCHS_ID_0, ADCHS_CHANNEL_2, ADCHS_CHANNEL_2_ALTERNATE_INP_AN6);
    
    
    
    /* Enable the global software EDGE trigger for analog input scanning. */
    /* The global software trigger bit is cleared automatically in the next ADC clock cycle. */
    DRV_ADC_Start();
}

void test_adc_convertValues(ANALOG_VOLTAGE_MONITOR_DATA* data){    
    data->adc_converted_data.samples.IL12 = ConvertADCRawSampleToCurrent(data->adc_raw_data.samples.IL12);
    data->adc_converted_data.samples.IL34 = ConvertADCRawSampleToCurrent(data->adc_raw_data.samples.IL34);
    
    data->adc_converted_data.samples.V12V = data->adc_raw_data.samples.V12V * ADC_LSB_VOLTAGE_mV * data->dividers.an_12V;
    data->adc_converted_data.samples.V18V = data->adc_raw_data.samples.V18V * ADC_LSB_VOLTAGE_mV * data->dividers.an_18V;
    data->adc_converted_data.samples.V5V = data->adc_raw_data.samples.V5V * ADC_LSB_VOLTAGE_mV * data->dividers.an_5V;
    
    data->adc_converted_data.samples.V1V8_1 = data->adc_raw_data.samples.V1V8_1 * ADC_LSB_VOLTAGE_mV * data->dividers.an_1V8;
    data->adc_converted_data.samples.V1V8_2 = data->adc_raw_data.samples.V1V8_2 * ADC_LSB_VOLTAGE_mV * data->dividers.an_1V8;
    
    data->adc_converted_data.samples.V3V3_0 = data->adc_raw_data.samples.V3V3_0 * ADC_LSB_VOLTAGE_mV * data->dividers.an_3V3;
    data->adc_converted_data.samples.V3V3_1 = data->adc_raw_data.samples.V3V3_1 * ADC_LSB_VOLTAGE_mV * data->dividers.an_3V3;
    data->adc_converted_data.samples.V3V3_2 = data->adc_raw_data.samples.V3V3_2 * ADC_LSB_VOLTAGE_mV * data->dividers.an_3V3;
    
    data->adc_converted_data.samples.V3V3AN1 = data->adc_raw_data.samples.V3V3AN1 * ADC_LSB_VOLTAGE_mV * data->dividers.an_3V3AN;
    data->adc_converted_data.samples.V3V3AN2 = data->adc_raw_data.samples.V3V3AN2 * ADC_LSB_VOLTAGE_mV * data->dividers.an_3V3AN;
    
    data->adc_converted_data.samples.V325V = data->adc_raw_data.samples.V325V * ADC_LSB_VOLTAGE_mV * data->dividers.an_325V;
    data->adc_converted_data.samples.V380V = data->adc_raw_data.samples.V380V * ADC_LSB_VOLTAGE_mV * data->dividers.an_380V;
    
    data->adc_converted_data.samples.VLIVE = data->adc_raw_data.samples.VLIVE * ADC_LSB_VOLTAGE_mV * data->dividers.an_net;
    data->adc_converted_data.samples.VNEUTRAL = data->adc_raw_data.samples.VNEUTRAL * ADC_LSB_VOLTAGE_mV * data->dividers.an_net;
    
    
    data->adc_converted_data.samples.TEMP_BRUG = ConvertADCRawSampleToThermalResistance(data->adc_raw_data.samples.TEMP_BRUG);
    data->adc_converted_data.samples.TEMP_VOED = ConvertADCRawSampleToThermalResistance(data->adc_raw_data.samples.TEMP_VOED);
    data->adc_converted_data.samples.TEMP_ELCO = ConvertADCRawSampleToThermalResistance(data->adc_raw_data.samples.TEMP_ELCO);
    data->adc_converted_data.samples.TEMP_M1 = ConvertADCRawSampleToThermalResistance(data->adc_raw_data.samples.TEMP_M1);
    data->adc_converted_data.samples.TEMP_M2 = ConvertADCRawSampleToThermalResistance(data->adc_raw_data.samples.TEMP_M2);
    data->adc_converted_data.samples.TEMP_PFC12 = ConvertADCRawSampleToThermalResistance(data->adc_raw_data.samples.TEMP_PFC12);
    data->adc_converted_data.samples.TEMP_PFC34 = ConvertADCRawSampleToThermalResistance(data->adc_raw_data.samples.TEMP_PFC34);

}

void test_update_adcValues(ANALOG_VOLTAGE_MONITOR_DATA* data){
    //Read all adcs

    //Convert ADC values to mV 
    test_adc_convertValues(&analog_voltage_monitorData);
    
    //Return values
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
    }while( adcData.adc_raw_data.samples.V5V < targetADC );
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

bool test_adc_get(unsigned int index, uint16_t* adcVal, float* convVal){
    switch( index ){
        case 1: *adcVal = adcData.adc_raw_data.samples.V325V; *convVal = adcData.adc_converted_data.samples.V325V; break;
        case 2: *adcVal = adcData.adc_raw_data.samples.V380V; *convVal = adcData.adc_converted_data.samples.V380V; break;
        case 3: *adcVal = adcData.adc_raw_data.samples.TEMP_BRUG; *convVal = adcData.adc_converted_data.samples.TEMP_BRUG; break;
        case 4: *adcVal = adcData.adc_raw_data.samples.TEMP_VOED; *convVal = adcData.adc_converted_data.samples.TEMP_VOED; break;
        case 5: *adcVal = adcData.adc_raw_data.samples.TEMP_ELCO; *convVal = adcData.adc_converted_data.samples.TEMP_ELCO; break;
        case 6: *adcVal = adcData.adc_raw_data.samples.TEMP_PFC12; *convVal = adcData.adc_converted_data.samples.TEMP_PFC12; break;
        case 7: *adcVal = adcData.adc_raw_data.samples.TEMP_PFC34; *convVal = adcData.adc_converted_data.samples.TEMP_PFC34; break;
        case 8: *adcVal = adcData.adc_raw_data.samples.TEMP_M1; *convVal = adcData.adc_converted_data.samples.TEMP_M1; break;
        case 9: *adcVal = adcData.adc_raw_data.samples.TEMP_M2; *convVal = adcData.adc_converted_data.samples.TEMP_M2; break;
        case 10: *adcVal = adcData.adc_raw_data.samples.V18V; *convVal = adcData.adc_converted_data.samples.V18V; break;
        case 11: *adcVal = adcData.adc_raw_data.samples.V12V; *convVal = adcData.adc_converted_data.samples.V12V; break;
        case 12: *adcVal = adcData.adc_raw_data.samples.V5V; *convVal = adcData.adc_converted_data.samples.V5V; break;
        case 13: *adcVal = adcData.adc_raw_data.samples.V3V3_0; *convVal = adcData.adc_converted_data.samples.V3V3_0; break;
        case 14: *adcVal = adcData.adc_raw_data.samples.V3V3_1; *convVal = adcData.adc_converted_data.samples.V3V3_1; break;
        case 15: *adcVal = adcData.adc_raw_data.samples.V3V3AN1; *convVal = adcData.adc_converted_data.samples.V3V3AN1; break;
        case 16: *adcVal = adcData.adc_raw_data.samples.V1V8_1; *convVal = adcData.adc_converted_data.samples.V1V8_1; break;
        case 17: *adcVal = adcData.adc_raw_data.samples.V3V3_2; *convVal = adcData.adc_converted_data.samples.V3V3_2; break;
        case 18: *adcVal = adcData.adc_raw_data.samples.V3V3AN2; *convVal = adcData.adc_converted_data.samples.V3V3AN2; break;
        case 19: *adcVal = adcData.adc_raw_data.samples.V1V8_2; *convVal = adcData.adc_converted_data.samples.V1V8_2; break;
        default: return false;
    }
    return true;
}
