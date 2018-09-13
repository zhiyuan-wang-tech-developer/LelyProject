#include "test_adc.h"
#include "analog_voltage_monitor.h"
#include "led_controller.h"
#include "test_pwm.h"

#define ADC_Channel_380V     ADCHS_AN0
#define ADC_Channel_TPFC_34  ADCHS_AN1
#define ADC_Channel_18V      ADCHS_AN2
#define ADC_Channel_NEUTRAL  ADCHS_AN4
#define ADC_Channel_IL_34    ADCHS_AN5
#define ADC_Channel_LIVE     ADCHS_AN6
#define ADC_Channel_3V3AN_2  ADCHS_AN7
#define ADC_Channel_3V3_2    ADCHS_AN8
#define ADC_Channel_3V3_1    ADCHS_AN9
#define ADC_Channel_3V3AN_1  ADCHS_AN10
#define ADC_Channel_1V8_2    ADCHS_AN11
#define ADC_Channel_5V       ADCHS_AN12
#define ADC_Channel_3V3_0    ADCHS_AN13
#define ADC_Channel_TPFC_12  ADCHS_AN17
#define ADC_Channel_TM_2     ADCHS_AN23
#define ADC_Channel_12V      ADCHS_AN27
#define ADC_Channel_TELCO    ADCHS_AN35
#define ADC_Channel_TBRUG    ADCHS_AN36
#define ADC_Channel_TVOED    ADCHS_AN37
#define ADC_Channel_325V     ADCHS_AN39
#define ADC_Channel_1V8_1    ADCHS_AN45
#define ADC_Channel_IL_12    ADCHS_AN46
#define ADC_Channel_TM_1     ADCHS_AN47


#define _setSample(__SAMPLE_NAME__, __CHANNEL_NAME__)  \
    do{ \
        if( PLIB_ADCHS_AnalogInputDataIsReady( ADCHS_ID_0, __CHANNEL_NAME__ ) ) { \
            analog_voltage_monitorData.adc_raw_data.samples.__SAMPLE_NAME__ = PLIB_ADCHS_AnalogInputResultGet(ADCHS_ID_0, __CHANNEL_NAME__); \
            analog_voltage_monitorData.adc_raw_data.samples.update.status_bits.__SAMPLE_NAME__ = true; \
        } \
    } while(0)

#define _setFilteredSample(__SAMPLE_NAME__, __FILTER_NAME__)  \
    do{ \
        if( PLIB_ADCHS_DigitalFilterDataIsReady( ADCHS_ID_0, __FILTER_NAME__ ) ) { \
            analog_voltage_monitorData.adc_raw_data.samples.__SAMPLE_NAME__ = PLIB_ADCHS_DigitalFilterDataGet(ADCHS_ID_0, __FILTER_NAME__); \
            analog_voltage_monitorData.adc_raw_data.samples.update.status_bits.__SAMPLE_NAME__ = true; \
        } \
    } while(0)


extern float ConvertADCRawSampleToThermalResistance(uint32_t ADC_Raw_Value);
extern float ConvertADCRawSampleToCurrent(uint32_t ADC_Raw_Value);

extern volatile ANALOG_VOLTAGE_MONITOR_DATA analog_voltage_monitorData;

static ANALOG_VOLTAGE_MONITOR_DATA adcData;




// adc difference from mid -> mV -> A
const float adcToA = (3300.0 / 4095.0) / 66.0;

// adc -> mV adc -> V380V
const float adcToV = (3.300 / 4095.0) * (47e3+1e6+4.7e3)/(4.7e3);





void __attribute__(( at_vector(_ADC_DF1_VECTOR), interrupt(ipl5SRS), aligned(16) )) _IntHandlerDrvAdc_Filter1Shadow(void)
{    
    //V_LED1_GOn();
    PORTESET = _PORTE_RE0_MASK;    

//    static const ADC_SCAN_UPDATE_STATUS flagMask = {
//        .status_bits.IL34 = 1,
//        .status_bits.V380V = 1,
//        .status_bits.VLIVE = 1,
//        .status_bits.VNEUTRAL = 1,
//        .status_bits.TEMP_PFC34 = 1
//    };
//    
//    
    //Read the dedicated channels, interrupting on ADC Filter, because this triggers after the normal conversions are complete
    analog_voltage_monitorData.adc_raw_data.samples.IL34 = PLIB_ADCHS_DigitalFilterDataGet(ADCHS_ID_0, ADCHS_DIGITAL_FILTER_1);
//    analog_voltage_monitorData.adc_raw_data.samples.update.status_bits.IL34 = true;
    
    analog_voltage_monitorData.adc_raw_data.samples.V380V = PLIB_ADCHS_AnalogInputResultGet(ADCHS_ID_0, ADC_Channel_380V);
//    analog_voltage_monitorData.adc_raw_data.samples.update.status_bits.V380V = true;

    analog_voltage_monitorData.adc_raw_data.samples.VLIVE = PLIB_ADCHS_AnalogInputResultGet(ADCHS_ID_0, ADC_Channel_LIVE);
//    analog_voltage_monitorData.adc_raw_data.samples.update.status_bits.VLIVE = true;

    analog_voltage_monitorData.adc_raw_data.samples.VNEUTRAL = PLIB_ADCHS_AnalogInputResultGet(ADCHS_ID_0, ADC_Channel_NEUTRAL);
//    analog_voltage_monitorData.adc_raw_data.samples.update.status_bits.VNEUTRAL = true;

    analog_voltage_monitorData.adc_raw_data.samples.TEMP_PFC34 = PLIB_ADCHS_AnalogInputResultGet(ADCHS_ID_0, ADC_Channel_TPFC_34);
//    analog_voltage_monitorData.adc_raw_data.samples.update.status_bits.TEMP_PFC34 = true;
    
    analog_voltage_monitorData.adc_raw_data.samples.update.status |= 0x1f;//flagMask.status;
    
    
//    //data->adc_converted_data.samples.V380V = data->adc_raw_data.samples.V380V * ADC_LSB_VOLTAGE_mV * data->dividers.an_380V;
//    float Uinductor = analog_voltage_monitorData.adc_raw_data.samples.V380V * adcToV;
//    float current_A = (float)(analog_voltage_monitorData.adc_raw_data.samples.IL34 - 2048) * adcToA;
//    float time = test_pwm_calc_time(Uinductor, current_A);
    
//    V_LED1_GOff();
    PORTECLR = _PORTE_RE0_MASK;

    //Clear interrupt flag
//    IFS3bits.AD1DF1IF = 0;    
    IFS3CLR = _IFS3_AD1DF1IF_MASK;
}


void __ISR(_ADC_DATA5_VECTOR, ipl3AUTO) _IntHandlerDrvAdc_DATA5(void)
{
    _setSample(IL34, ADCHS_AN5);    
    IFS3bits.AD1D5IF = 0;
}




void __ISR(_ADC_EOS_VECTOR, ipl3AUTO) _IntHandlerDrvAdc_END_OF_SCAN(void)
{
    if( !PLIB_ADCHS_AnalogInputScanIsComplete(ADCHS_ID_0)){
        return;
    }

    V_LED1_ROn();
    
    _setSample( IL12, ADC_Channel_IL_12 );    
    _setSample( IL34, ADC_Channel_IL_34);    
    _setSample( TEMP_BRUG, ADC_Channel_TBRUG);    
    _setSample( TEMP_ELCO, ADC_Channel_TELCO);    
    _setSample( TEMP_M1, ADC_Channel_TM_1);
    _setSample( TEMP_M2, ADC_Channel_TM_2);
    _setSample( TEMP_PFC12, ADC_Channel_TPFC_12);
    _setSample( TEMP_PFC34, ADC_Channel_TPFC_34);
    _setSample( TEMP_VOED, ADC_Channel_TVOED);
    _setSample( V12V, ADC_Channel_12V);
    _setSample( V18V, ADC_Channel_18V);
    _setSample( V1V8_1, ADC_Channel_1V8_1);
    _setSample( V1V8_2, ADC_Channel_1V8_2);
    _setSample( V325V, ADC_Channel_325V);
    _setSample( V380V, ADC_Channel_380V);
    _setSample( V3V3AN1, ADC_Channel_3V3AN_1);
    _setSample( V3V3AN2, ADC_Channel_3V3AN_2);
    _setSample( V3V3_0, ADC_Channel_3V3_0);
    _setSample( V3V3_1, ADC_Channel_3V3_1);
    _setSample( V3V3_2, ADC_Channel_3V3_2);
    _setSample( V5V, ADC_Channel_5V);
    _setSample( VLIVE, ADC_Channel_LIVE);
    _setSample( VNEUTRAL, ADC_Channel_NEUTRAL);

    /* Clear ADC Interrupt Flag of INT_SOURCE_ADC_END_OF_SCAN */
    PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_ADC_END_OF_SCAN);
    
    V_LED1_ROff();    
}

void __ISR(_TIMER_3_VECTOR, ipl4AUTO) _IntHandler_Timer3(void)
{
    V_LED3_RToggle();
    IFS0bits.T3IF = 0;
}



void test_init_adc(){
    DRV_ADC_Initialize();

    //dedicated shadow register set for fast ADC ISR    
    PLIB_INT_ShadowRegisterAssign(INT_ID_0, INT_PRIORITY_LEVEL5, INT_SHADOW_REGISTER_1);
    
    //Configure ADC5 in the same way other ADCs are configured
    if (DEVADC5 != 0xFFFFFFFF)
    PLIB_ADCHS_ChannelConfigurationSet(DRV_ADC_ID_1, ADCHS_CHANNEL_5, DEVADC5);
    
    
    PLIB_ADCHS_ChannelSetup( DRV_ADC_ID_1, ADCHS_CHANNEL_5, ADCHS_DATA_RESOLUTION_12BIT, 1, 1, ADCHS_EARLY_INTERRUPT_PRIOR_CLOCK_4 );
    PLIB_ADCHS_ChannelTriggerSampleSelect( DRV_ADC_ID_1, ADCHS_CHANNEL_5, ADCHS_CHANNEL_UNSYNC_TRIGGER_UNSYNC_SAMPLING );
    PLIB_ADCHS_AnalogInputDataReadyInterruptEnable( DRV_ADC_ID_1, ADCHS_CLASS12_AN5 );
    PLIB_ADCHS_AnalogInputModeSelect( DRV_ADC_ID_1, ADCHS_AN5, ADCHS_INPUT_MODE_SINGLE_ENDED_UNIPOLAR );
    PLIB_ADCHS_AnalogInputEdgeTriggerSet( DRV_ADC_ID_1, ADCHS_CLASS12_AN5 );
    PLIB_ADCHS_AnalogInputTriggerSourceSelect( DRV_ADC_ID_1, ADCHS_CLASS12_AN5, ADCHS_TRIGGER_SOURCE_GLOBAL_SOFTWARE_EDGE );

    
    //Set ADC5 = Read AN5 == IL34
    //Pass through digital averaging filter
    PLIB_ADCHS_ChannelInputSelect(ADCHS_ID_0, ADCHS_CHANNEL_5, ADCHS_CHANNEL_5_DEFAULT_INP_AN5);
    PLIB_ADCHS_DigitalFilterAveragingModeSetup(ADCHS_ID_0, ADCHS_DIGITAL_FILTER_1, ADCHS_AN5, ADCHS_DIGITAL_FILTER_SIGNIFICANT_FIRST_12BITS, ADCHS_DIGITAL_FILTER_AVERAGE_SAMPLE_COUNT_16, true);
    PLIB_ADCHS_DigitalFilterEnable(ADCHS_ID_0, ADCHS_DIGITAL_FILTER_1);
    
    
    //Triggered by TMR3
    PLIB_ADCHS_AnalogInputScanRemove(ADCHS_ID_0, ADCHS_AN5);
    PLIB_ADCHS_ChannelTriggerSampleSelect(ADCHS_ID_0, ADCHS_CHANNEL_5, ADCHS_CHANNEL_UNSYNC_TRIGGER_UNSYNC_SAMPLING);
    
    //Use Filter Ready interrupt instead of data ready
    PLIB_ADCHS_DigitalFilterDataReadyInterruptEnable(ADCHS_ID_0, ADCHS_DIGITAL_FILTER_1);
    
    
    PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_ADC_1_DF1);
    PLIB_INT_SourceEnable(INT_ID_0, INT_SOURCE_ADC_1_DF1);
    PLIB_INT_VectorPrioritySet(INT_ID_0, INT_VECTOR_ADC1_DF1, INT_PRIORITY_LEVEL5);
    PLIB_INT_VectorSubPrioritySet(INT_ID_0, INT_VECTOR_ADC1_DF1, INT_SUBPRIORITY_LEVEL0);
    
        
    PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_ADC_END_OF_SCAN);
    PLIB_INT_SourceEnable(INT_ID_0, INT_SOURCE_ADC_END_OF_SCAN);
    PLIB_INT_VectorPrioritySet(INT_ID_0, INT_VECTOR_ADC_END_OF_SCAN, INT_PRIORITY_LEVEL3);
    PLIB_INT_VectorSubPrioritySet(INT_ID_0, INT_VECTOR_ADC_END_OF_SCAN, INT_SUBPRIORITY_LEVEL0);         
        
    // Override timing parameters
    //     16.667ns < TAD < 6250 ns
    //     ADC 0-5:       sample >= 3 TAD
    //     ADC 7:         sample >= 4 TAD

    ADCCON3bits.ADCSEL = 3;     //Tclk = SYSCLK = 120 MHz
    ADCCON3bits.CONCLKDIV = 0;  //TQ   = TCLK
    
    ADCCON2bits.ADCDIV  = 1;    //TAD = TQ * 2 
    ADCCON2bits.SAMC    = 2;    //Sample = 4 TAD
    
    ADC0TIMEbits.ADCDIV = 1;    //TAD = TQ * 2
    ADC0TIMEbits.SAMC = 1;      //Sample = 3 TAD

    ADC1TIMEbits.ADCDIV = 1;    //TAD = TQ * 2
    ADC1TIMEbits.SAMC = 1;      //Sample = 3 TAD
    
    ADC2TIMEbits.ADCDIV = 1;    //TAD = TQ * 2
    ADC2TIMEbits.SAMC = 1;      //Sample = 3 TAD
    
    ADC3TIMEbits.ADCDIV = 1;    //TAD = TQ * 2
    ADC3TIMEbits.SAMC = 1;      //Sample = 3 TAD
    
    ADC4TIMEbits.ADCDIV = 1;    //TAD = TQ * 2
    ADC4TIMEbits.SAMC = 1;      //Sample = 3 TAD
    
    ADC5TIMEbits.ADCDIV = 1;    //TAD = TQ * 2
    ADC5TIMEbits.SAMC = 1;      //Sample = 3 TAD
    
    
    DRV_ADC0_Open();
    DRV_ADC1_Open();
    DRV_ADC2_Open();
    DRV_ADC4_Open();
    DRV_ADC5_Open();
    DRV_ADC6_Open();
    
    //PWM period = 1200 = 100 KHz
    //ADC period = 4* PWM = 300 = 400 KHz

    //disable Fast ADC timer interrupts
    IEC0bits.T3IE = 0;
    
    //disable Slow ADC timer interrupts
    IEC0bits.T5IE = 0;
    
    //Set ADC0 = Read AN0 = +380V
    PLIB_ADCHS_ChannelInputSelect(ADCHS_ID_0, ADCHS_CHANNEL_0, ADCHS_CHANNEL_0_DEFAULT_INP_AN0);
    
    //Set ADC1 = Read AN1 = T-PFC34
    PLIB_ADCHS_ChannelInputSelect(ADCHS_ID_0, ADCHS_CHANNEL_1, ADCHS_CHANNEL_1_DEFAULT_INP_AN1);
    
    //Set ADC2 = Read AN6 == VL
    PLIB_ADCHS_ChannelInputSelect(ADCHS_ID_0, ADCHS_CHANNEL_2, ADCHS_CHANNEL_2_ALTERNATE_INP_AN6);
    
    //Set ADC4 = Read AN4 == VN
    PLIB_ADCHS_ChannelInputSelect(ADCHS_ID_0, ADCHS_CHANNEL_4, ADCHS_CHANNEL_4_DEFAULT_INP_AN4);

    PLIB_ADCHS_AnalogInputTriggerSourceSelect(ADCHS_ID_0, ADCHS_CLASS12_AN0, ADCHS_TRIGGER_SOURCE_TMR3_MATCH);
    PLIB_ADCHS_AnalogInputTriggerSourceSelect(ADCHS_ID_0, ADCHS_CLASS12_AN1, ADCHS_TRIGGER_SOURCE_TMR3_MATCH);    
    PLIB_ADCHS_AnalogInputTriggerSourceSelect(ADCHS_ID_0, ADCHS_CLASS12_AN4, ADCHS_TRIGGER_SOURCE_TMR3_MATCH);
    PLIB_ADCHS_AnalogInputTriggerSourceSelect(ADCHS_ID_0, ADCHS_CLASS12_AN5, ADCHS_TRIGGER_SOURCE_TMR3_MATCH);
    PLIB_ADCHS_AnalogInputTriggerSourceSelect(ADCHS_ID_0, ADCHS_CLASS12_AN6, ADCHS_TRIGGER_SOURCE_TMR3_MATCH);
    
    
    //Remove dedicated lines from scan list
    PLIB_ADCHS_AnalogInputScanRemove(ADCHS_ID_0, ADCHS_AN0);
    PLIB_ADCHS_AnalogInputScanRemove(ADCHS_ID_0, ADCHS_AN1);
    PLIB_ADCHS_AnalogInputScanRemove(ADCHS_ID_0, ADCHS_AN6);
    PLIB_ADCHS_AnalogInputScanRemove(ADCHS_ID_0, ADCHS_AN4);
    PLIB_ADCHS_AnalogInputScanRemove(ADCHS_ID_0, ADCHS_AN5);
    
    //Set up scan complete interrupt
    PLIB_ADCHS_ExternalConversionRequestEnable(ADCHS_ID_0);
    PLIB_ADCHS_ScanCompleteInterruptEnable(ADCHS_ID_0);

    //Remap analog inputs to SCAN trigger
#define ADCHS_TRIGGER_SOURCE_SCAN 3
    PLIB_ADCHS_AnalogInputTriggerSourceSelect(ADCHS_ID_0, ADCHS_CLASS12_AN2, ADCHS_TRIGGER_SOURCE_SCAN);   //18v
    PLIB_ADCHS_AnalogInputTriggerSourceSelect(ADCHS_ID_0, ADCHS_CLASS12_AN7, ADCHS_TRIGGER_SOURCE_SCAN);   //3v3an2
    PLIB_ADCHS_AnalogInputTriggerSourceSelect(ADCHS_ID_0, ADCHS_CLASS12_AN8, ADCHS_TRIGGER_SOURCE_SCAN);   //3v3-2
    PLIB_ADCHS_AnalogInputTriggerSourceSelect(ADCHS_ID_0, ADCHS_CLASS12_AN9, ADCHS_TRIGGER_SOURCE_SCAN);   //3v3-1
    PLIB_ADCHS_AnalogInputTriggerSourceSelect(ADCHS_ID_0, ADCHS_CLASS12_AN10, ADCHS_TRIGGER_SOURCE_SCAN);   //3v3an1
    PLIB_ADCHS_AnalogInputTriggerSourceSelect(ADCHS_ID_0, ADCHS_CLASS12_AN11, ADCHS_TRIGGER_SOURCE_SCAN);   //1v8-2
    PLIB_ADCHS_AnalogInputTriggerSourceSelect(ADCHS_ID_0, ADCHS_CLASS12_AN12, ADCHS_TRIGGER_SOURCE_SCAN);   //5v
    PLIB_ADCHS_AnalogInputTriggerSourceSelect(ADCHS_ID_0, ADCHS_CLASS12_AN13, ADCHS_TRIGGER_SOURCE_SCAN);   //3v3-0
    PLIB_ADCHS_AnalogInputTriggerSourceSelect(ADCHS_ID_0, ADCHS_CLASS12_AN17, ADCHS_TRIGGER_SOURCE_SCAN);   //t-pfc12
    PLIB_ADCHS_AnalogInputTriggerSourceSelect(ADCHS_ID_0, ADCHS_CLASS12_AN23, ADCHS_TRIGGER_SOURCE_SCAN);   //t-m2
    PLIB_ADCHS_AnalogInputTriggerSourceSelect(ADCHS_ID_0, ADCHS_CLASS12_AN27, ADCHS_TRIGGER_SOURCE_SCAN);   //12v
    
    //Use T5 as trigger source for analog input scan
    PLIB_ADCHS_AnalogInputScanSetup(ADCHS_ID_0, ADCHS_AN2, ADCHS_SCAN_TRIGGER_SENSITIVE_EDGE, ADCHS_SCAN_TRIGGER_SOURCE_TMR5_MATCH);
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

void test_adc_update_adcValues(ANALOG_VOLTAGE_MONITOR_DATA* data){
    //Read all adcs
    analog_voltage_monitorData.adc_raw_data.samples.update.status = 0;
    if( !T5CONbits.ON ){
        test_adc_enableSlowADC(true);
    }
    if( !T3CONbits.ON ){
        test_adc_enableFastADC(true);   
    }
    
    
//    DRV_ADC_Start();
    while( analog_voltage_monitorData.adc_raw_data.samples.update.status == 0){
        Nop();
    }
    
    //Convert ADC values to mV 
    test_adc_convertValues( (ANALOG_VOLTAGE_MONITOR_DATA*) &analog_voltage_monitorData);
    
    //Return values
    memcpy((void*)data, (void*)&analog_voltage_monitorData, sizeof(ANALOG_VOLTAGE_MONITOR_DATA));
}


//------------------------------------------------------------------------------
// Common power
void test_wait_18V(uint16_t targetADC){
    do{
        test_adc_update_adcValues(&adcData);
    }while( adcData.adc_raw_data.samples.V18V < targetADC );
}
void test_wait_12V(uint16_t targetADC){
    do{
        test_adc_update_adcValues(&adcData);
    }while( adcData.adc_raw_data.samples.V12V < targetADC );
}
void test_wait_5V(uint16_t targetADC){
    do{
        test_adc_update_adcValues(&adcData);
    }while( adcData.adc_raw_data.samples.V5V < targetADC );
}


//------------------------------------------------------------------------------
// Net power
void test_wait_380V(uint16_t targetADC){
    do{
        test_adc_update_adcValues(&adcData);
    }while( adcData.adc_raw_data.samples.V380V < targetADC );
}
void test_wait_325V(uint16_t targetADC){
    do{
        test_adc_update_adcValues(&adcData);
    }while( adcData.adc_raw_data.samples.V325V < targetADC );
}


//------------------------------------------------------------------------------
// Piccolo 1 power
void test_wait_3V3_1(uint16_t targetADC){
    do{
        test_adc_update_adcValues(&adcData);
    }while( adcData.adc_raw_data.samples.V3V3_1 < targetADC );
}
void test_wait_3V3AN_1(uint16_t targetADC){
    do{
        test_adc_update_adcValues(&adcData);
    }while( adcData.adc_raw_data.samples.V3V3AN1 < targetADC );
}
void test_wait_1V8_1(uint16_t targetADC){
    do{
        test_adc_update_adcValues(&adcData);
    }while( adcData.adc_raw_data.samples.V1V8_1 < targetADC );
}

//------------------------------------------------------------------------------
// Piccolo 2 power
void test_wait_3V3_2(uint16_t targetADC){
    do{
        test_adc_update_adcValues(&adcData);
    }while( adcData.adc_raw_data.samples.V3V3_2 < targetADC );
    
}
void test_wait_3V3AN_2(uint16_t targetADC){
    do{
        test_adc_update_adcValues(&adcData);
    }while( adcData.adc_raw_data.samples.V3V3AN2 < targetADC );
}
void test_wait_1V8_2(uint16_t targetADC){
    do{
        test_adc_update_adcValues(&adcData);
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

void test_adc_enableFastADC(bool val){
    PR3 = 150-1;
    T3CON = val ? 0x8000 : 0;
}

void test_adc_enableSlowADC(bool val){
    PR5 = 12*1000 -1;     //0.1 ms
    T5CON = val ? 0x8000 : 0;
}
