#include "test_pwm.h"
#include "test_params.h"
#include "test_adc.c"
#include <string.h>

#include <peripheral/cmp/plib_cmp.h>
#include <peripheral/cdac/plib_cdac.h>

extern PWM_CONTROLLER_DATA pwm_controllerData;
static PWM_CONTROLLER_DATA pwmData;

static void test_pwm_setCurrentLimit();


#define C1OUT_ON_LED
void test_pwm(){
    memcpy(&pwmData, &pwm_controllerData, sizeof(PWM_CONTROLLER_DATA));

    test_pwm_setCurrentLimit();
    
    while( !PLIB_MCPWM_ChannelCurrentLimitIsAsserted(MCPWM_ID_0, MCPWM_CHANNEL1) && !PLIB_CMP_OutputStatusGet(CMP_ID_1) ){
        Nop();
    }
    
    SYS_DEBUG_BreakPoint();
}


static void test_pwm_setCurrentLimit(){
    // Setup IL34 as current limit for PWMS
    
    // Use Comparator 1 with CDAC3 and IL34 as inputs
    
    // Current is being sensed with    
    // ACS733KLATR-20AB = 66 mV/ A, Vout = 0.5 * Vcc + ([-20 .. +20 A] * 66 mV)        
    
    // Polarity is reversed, so looking for a maximum negative current
    
    // Overcurrent @ -11A = Vcc/2 - (11 * 66) mV
    // 12-bit CDAC = ((3300 / 2) - 726) mV * (4095 / 3300 mV) = 1146
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
    PLIB_CMP_OutputDisable(CMP_ID_1);    
    PLIB_CMP_OpAmpOutputDisable(CMP_ID_1);

    //Disable OpAmp mode
    PLIB_CMP_OpAmpDisable(CMP_ID_1);
    
    //Compare CDAC3 to C1IN1- = V-AN-IL34    
    PLIB_CMP_NonInvertingInputChannelSelect(CMP_ID_1, CMP_NON_INVERTING_INPUT_CDAC);
    PLIB_CMP_InvertingInputChannelSelect(CMP_ID_1, CMP_INVERTING_INPUT_1);    
    
    //Configure polarity
    //Invert output: COUT = Vin+ < Vin- => COUT = V-AN-IL34 > CDAC3
    //Normal output: COUT = Vin+ > Vin- => COUT = V-AN-IL34 < CDAC3
//    PLIB_CMP_OutputInvertDisable(CMP_ID_1);
    PLIB_CMP_OutputInvertEnable(CMP_ID_1);
    
#ifdef C1OUT_ON_LED    
     // Remap C1OUT -> LED 2 Green
    PLIB_PORTS_RemapOutput(PORTS_ID_0, OUTPUT_FUNC_C1OUT, OUTPUT_PIN_RPE15);
    PLIB_CMP_OutputEnable(CMP_ID_1);
#endif
    
    //Disable PWM to set current limits
    PLIB_MCPWM_Disable(MCPWM_ID_0);
    
    //Use output of Comparator 1 as CurrentLimit for PWM Channels 1-8
    PLIB_MCPWM_ChannelCurrentLimitSetup(MCPWM_ID_0, PWM_ChanBoost1, MCPWM_CURRENTLIMIT_SOURCE_IS_COMPARATOR1, MCPWM_CURRENTLIMIT_INPUT_POLARITY_ACTIVE_LOW, MCPWM_CURRENTLIMIT_OVERRIDE_PWMxH_0, MCPWM_CURRENTLIMIT_OVERRIDE_PWMxL_0, MCPWM_STANDARD_CURRENT_LIMIT_MODE_ENABLE);
    PLIB_MCPWM_ChannelCurrentLimitSetup(MCPWM_ID_0, PWM_ChanBoost2, MCPWM_CURRENTLIMIT_SOURCE_IS_COMPARATOR1, MCPWM_CURRENTLIMIT_INPUT_POLARITY_ACTIVE_LOW, MCPWM_CURRENTLIMIT_OVERRIDE_PWMxH_0, MCPWM_CURRENTLIMIT_OVERRIDE_PWMxL_0, MCPWM_STANDARD_CURRENT_LIMIT_MODE_ENABLE);
    PLIB_MCPWM_ChannelCurrentLimitSetup(MCPWM_ID_0, PWM_ChanBoost3, MCPWM_CURRENTLIMIT_SOURCE_IS_COMPARATOR1, MCPWM_CURRENTLIMIT_INPUT_POLARITY_ACTIVE_LOW, MCPWM_CURRENTLIMIT_OVERRIDE_PWMxH_0, MCPWM_CURRENTLIMIT_OVERRIDE_PWMxL_0, MCPWM_STANDARD_CURRENT_LIMIT_MODE_ENABLE);
    PLIB_MCPWM_ChannelCurrentLimitSetup(MCPWM_ID_0, PWM_ChanBoost4, MCPWM_CURRENTLIMIT_SOURCE_IS_COMPARATOR1, MCPWM_CURRENTLIMIT_INPUT_POLARITY_ACTIVE_LOW, MCPWM_CURRENTLIMIT_OVERRIDE_PWMxH_0, MCPWM_CURRENTLIMIT_OVERRIDE_PWMxL_0, MCPWM_STANDARD_CURRENT_LIMIT_MODE_ENABLE);
    PLIB_MCPWM_ChannelCurrentLimitSetup(MCPWM_ID_0, PWM_ChanBuck1, MCPWM_CURRENTLIMIT_SOURCE_IS_COMPARATOR1, MCPWM_CURRENTLIMIT_INPUT_POLARITY_ACTIVE_LOW, MCPWM_CURRENTLIMIT_OVERRIDE_PWMxH_0, MCPWM_CURRENTLIMIT_OVERRIDE_PWMxL_0, MCPWM_STANDARD_CURRENT_LIMIT_MODE_ENABLE);
    PLIB_MCPWM_ChannelCurrentLimitSetup(MCPWM_ID_0, PWM_ChanBuck2, MCPWM_CURRENTLIMIT_SOURCE_IS_COMPARATOR1, MCPWM_CURRENTLIMIT_INPUT_POLARITY_ACTIVE_LOW, MCPWM_CURRENTLIMIT_OVERRIDE_PWMxH_0, MCPWM_CURRENTLIMIT_OVERRIDE_PWMxL_0, MCPWM_STANDARD_CURRENT_LIMIT_MODE_ENABLE);
    PLIB_MCPWM_ChannelCurrentLimitSetup(MCPWM_ID_0, PWM_ChanBuck3, MCPWM_CURRENTLIMIT_SOURCE_IS_COMPARATOR1, MCPWM_CURRENTLIMIT_INPUT_POLARITY_ACTIVE_LOW, MCPWM_CURRENTLIMIT_OVERRIDE_PWMxH_0, MCPWM_CURRENTLIMIT_OVERRIDE_PWMxL_0, MCPWM_STANDARD_CURRENT_LIMIT_MODE_ENABLE);
    PLIB_MCPWM_ChannelCurrentLimitSetup(MCPWM_ID_0, PWM_ChanBuck4, MCPWM_CURRENTLIMIT_SOURCE_IS_COMPARATOR1, MCPWM_CURRENTLIMIT_INPUT_POLARITY_ACTIVE_LOW, MCPWM_CURRENTLIMIT_OVERRIDE_PWMxH_0, MCPWM_CURRENTLIMIT_OVERRIDE_PWMxL_0, MCPWM_STANDARD_CURRENT_LIMIT_MODE_ENABLE);
    
#ifdef PWM_CURRENTLIMIT_INTERRUPTS
//    Enable current limit interrupts
    PLIB_MCPWM_ChannelCurrentLimitInterruptEnable(MCPWM_ID_0, PWM_ChanBoost1);  
    PLIB_MCPWM_ChannelCurrentLimitInterruptEnable(MCPWM_ID_0, PWM_ChanBoost2);  
    PLIB_MCPWM_ChannelCurrentLimitInterruptEnable(MCPWM_ID_0, PWM_ChanBoost3);  
    PLIB_MCPWM_ChannelCurrentLimitInterruptEnable(MCPWM_ID_0, PWM_ChanBoost4);  
    PLIB_MCPWM_ChannelCurrentLimitInterruptEnable(MCPWM_ID_0, PWM_ChanBuck1);  
    PLIB_MCPWM_ChannelCurrentLimitInterruptEnable(MCPWM_ID_0, PWM_ChanBuck2);  
    PLIB_MCPWM_ChannelCurrentLimitInterruptEnable(MCPWM_ID_0, PWM_ChanBuck3);  
    PLIB_MCPWM_ChannelCurrentLimitInterruptEnable(MCPWM_ID_0, PWM_ChanBuck4);
#endif
    
    PLIB_MCPWM_Enable(MCPWM_ID_0);
    
    //Enable Comparator
    PLIB_CMP_Enable(CMP_ID_1);
}

void test_pwm_SetBuckPhase(pwm_channel_t channel, uint16_t val, bool update){
    if( isHigherThanBuckMaxDC(val) ){
        val = getBuckMaxDC();
    }
    
    if( channel == PWM_ALL || channel == PWM_Pair_1 || channel == PWM_Group_12 ){
        pwmData.phaseShift.update_value.PWM_BUCK1 = val;
        pwmData.phaseShiftStatus.updata_status_flag.PWM_BUCK1 = true;
    }
    
    if( channel == PWM_ALL || channel == PWM_Pair_2 || channel == PWM_Group_12 ){
        pwmData.phaseShift.update_value.PWM_BUCK2 = val;
        pwmData.phaseShiftStatus.updata_status_flag.PWM_BUCK2 = true;        
    }
    
    if( channel == PWM_ALL || channel == PWM_Pair_3 || channel == PWM_Group_34 ){
        pwmData.phaseShift.update_value.PWM_BUCK3 = val;
        pwmData.phaseShiftStatus.updata_status_flag.PWM_BUCK3 = true;
    }
    
    if( channel == PWM_ALL || channel == PWM_Pair_4 || channel == PWM_Group_34 ){
        pwmData.phaseShift.update_value.PWM_BUCK4 = val;
        pwmData.phaseShiftStatus.updata_status_flag.PWM_BUCK4 = true;
    }
    
    if( update ){
        memcpy(&pwm_controllerData, &pwmData, sizeof(PWM_CONTROLLER_DATA));
        PWM_SIGNAL_Update();
    }
}

void test_pwm_SetBoostPhase(pwm_channel_t channel, uint16_t val, bool update){
    if( isHigherThanBoostMaxDC(val) ){
        val = getBoostMaxDC();
    }else if( isLowerThanBoostMinDC(val)){
        val = getBoostMinDC();
    }
    
    if( channel == PWM_ALL || channel == PWM_Pair_1 || channel == PWM_Group_12 ){
        pwmData.phaseShift.update_value.PWM_BOOST1 = val;
        pwmData.phaseShiftStatus.updata_status_flag.PWM_BOOST1 = true;
    }
    
    if( channel == PWM_ALL || channel == PWM_Pair_2 || channel == PWM_Group_12 ){
        pwmData.phaseShift.update_value.PWM_BOOST2 = val;
        pwmData.phaseShiftStatus.updata_status_flag.PWM_BOOST2 = true;
    }
    
    if( channel == PWM_ALL || channel == PWM_Pair_3 || channel == PWM_Group_34 ){
        pwmData.phaseShift.update_value.PWM_BOOST3 = val;
        pwmData.phaseShiftStatus.updata_status_flag.PWM_BOOST3 = true;
    }
    
    if( channel == PWM_ALL || channel == PWM_Pair_4 || channel == PWM_Group_34 ){
        pwmData.phaseShift.update_value.PWM_BOOST4 = val;
        pwmData.phaseShiftStatus.updata_status_flag.PWM_BOOST4 = true;
    }
    

    if( update ){
        memcpy(&pwm_controllerData, &pwmData, sizeof(PWM_CONTROLLER_DATA));
        PWM_SIGNAL_Update();
    }
}

void test_pwm_SetBuckBoostPhase(pwm_channel_t channel, uint16_t buck, uint16_t boost){
    test_pwm_SetBuckPhase(channel, buck, false);
    test_pwm_SetBoostPhase(channel, boost, true);
}


void test_pwm_SetBuckDC(pwm_channel_t channel, uint16_t val, bool update){
    if( isHigherThanBuckMaxDC(val) ){
        val = getBuckMaxDC();
    }
    
    if( channel == PWM_ALL || channel == PWM_Pair_1 || channel == PWM_Group_12 ){
        pwmData.dutyCycle.update_value.PWM_BUCK1 = val;
        pwmData.dutyCycleStatus.updata_status_flag.PWM_BUCK1 = true;
    }
    
    if( channel == PWM_ALL || channel == PWM_Pair_2 || channel == PWM_Group_12 ){
        pwmData.dutyCycle.update_value.PWM_BUCK2 = val;
        pwmData.dutyCycleStatus.updata_status_flag.PWM_BUCK2 = true;        
    }
    
    if( channel == PWM_ALL || channel == PWM_Pair_3 || channel == PWM_Group_34 ){
        pwmData.dutyCycle.update_value.PWM_BUCK3 = val;
        pwmData.dutyCycleStatus.updata_status_flag.PWM_BUCK3 = true;
    }
    
    if( channel == PWM_ALL || channel == PWM_Pair_4 || channel == PWM_Group_34 ){
        pwmData.dutyCycle.update_value.PWM_BUCK4 = val;
        pwmData.dutyCycleStatus.updata_status_flag.PWM_BUCK4 = true;
    }
    
    if( update ){
        memcpy(&pwm_controllerData, &pwmData, sizeof(PWM_CONTROLLER_DATA));
        PWM_SIGNAL_Update();
    }
}

void test_pwm_SetBoostDC(pwm_channel_t channel, uint16_t val, bool update){
    if( isHigherThanBoostMaxDC(val) ){
        val = getBoostMaxDC();
    }else if( isLowerThanBoostMinDC(val)){
        val = getBoostMinDC();
    }
    
    if( channel == PWM_ALL || channel == PWM_Pair_1 || channel == PWM_Group_12 ){
        pwmData.dutyCycle.update_value.PWM_BOOST1 = val;
        pwmData.dutyCycleStatus.updata_status_flag.PWM_BOOST1 = true;
    }
    
    if( channel == PWM_ALL || channel == PWM_Pair_2 || channel == PWM_Group_12 ){
        pwmData.dutyCycle.update_value.PWM_BOOST2 = val;
        pwmData.dutyCycleStatus.updata_status_flag.PWM_BOOST2 = true;
    }
    
    if( channel == PWM_ALL || channel == PWM_Pair_3 || channel == PWM_Group_34 ){
        pwmData.dutyCycle.update_value.PWM_BOOST3 = val;
        pwmData.dutyCycleStatus.updata_status_flag.PWM_BOOST3 = true;
    }
    
    if( channel == PWM_ALL || channel == PWM_Pair_4 || channel == PWM_Group_34 ){
        pwmData.dutyCycle.update_value.PWM_BOOST4 = val;
        pwmData.dutyCycleStatus.updata_status_flag.PWM_BOOST4 = true;
    }
    

    if( update ){
        memcpy(&pwm_controllerData, &pwmData, sizeof(PWM_CONTROLLER_DATA));
        PWM_SIGNAL_Update();
    }
}

void test_pwm_SetBuckBoostDC(pwm_channel_t channel, uint16_t buck, uint16_t boost){
    test_pwm_SetBuckDC(channel, buck, false);
    test_pwm_SetBoostDC(channel, boost, true);
}


void test_pwm_RampUp( pwm_channel_t pwms, uint16_t target, uint16_t dc_step){
    uint16_t dc = 0;

    do{
        //delay 10ms, half-sine of net power
        test_delay_10us(1000);  
        
        //set dc
        test_pwm_SetBuckBoostDC(pwms, dc, dc);
        
        dc += dc_step;
    }while( analog_voltage_monitorData.adc_raw_data.samples.V380V < target );

}


bool test_pwm_Get( pwm_channel_t channel, pwm_type_t type, uint16_t* dc, uint16_t* phase){    
    switch( type ){
        case PWM_BUCK:
            if( channel == PWM_ALL || channel == PWM_Pair_1 || channel == PWM_Group_12 ){
                *dc = pwm_controllerData.dutyCycle.update_value.PWM_BUCK1;
                *phase = pwm_controllerData.phaseShift.update_value.PWM_BUCK1;
            }else if( channel == PWM_Pair_2 ){
                *dc = pwm_controllerData.dutyCycle.update_value.PWM_BUCK2;
                *phase = pwm_controllerData.phaseShift.update_value.PWM_BUCK2;
            }else if( channel == PWM_Pair_3 || channel == PWM_Group_34 ){
                *dc = pwm_controllerData.dutyCycle.update_value.PWM_BUCK3;
                *phase = pwm_controllerData.phaseShift.update_value.PWM_BUCK3;
            }else if( channel == PWM_Pair_4 ){
                *dc = pwm_controllerData.dutyCycle.update_value.PWM_BUCK4;
                *phase = pwm_controllerData.phaseShift.update_value.PWM_BUCK4;
            }else{
                return false;
            }
            break;
            
            
        case PWM_BOOST:
            if( channel == PWM_ALL || channel == PWM_Pair_1 || channel == PWM_Group_12 ){
                *dc = pwm_controllerData.dutyCycle.update_value.PWM_BOOST1;
                *phase = pwm_controllerData.phaseShift.update_value.PWM_BOOST1;
            }else if( channel == PWM_Pair_2 ){
                *dc = pwm_controllerData.dutyCycle.update_value.PWM_BOOST2;
                *phase = pwm_controllerData.phaseShift.update_value.PWM_BOOST2;
            }else if( channel == PWM_Pair_3  || channel == PWM_Group_34 ){
                *dc = pwm_controllerData.dutyCycle.update_value.PWM_BOOST3;
                *phase = pwm_controllerData.phaseShift.update_value.PWM_BOOST3;
            }else if( channel == PWM_Pair_4 ){
                *dc = pwm_controllerData.dutyCycle.update_value.PWM_BOOST4;
                *phase = pwm_controllerData.phaseShift.update_value.PWM_BOOST4;
            }else{
                return false;
            }
            break;
        default:
            return false;
    }
    return true;
}