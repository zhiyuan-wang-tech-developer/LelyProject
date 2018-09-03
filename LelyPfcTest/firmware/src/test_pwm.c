#include "test_pwm.h"
#include "test_params.h"
#include "test_adc.c"
#include <string.h>

#include <peripheral/cmp/plib_cmp.h>
#include <peripheral/cdac/plib_cdac.h>

extern PWM_CONTROLLER_DATA pwm_controllerData;
static PWM_CONTROLLER_DATA pwmData;

static void test_pwm_setCurrentLimit();


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
    PLIB_CMP_OutputInvertDisable(CMP_ID_1);
    
    
    //Use output of Comparator 1 as CurrentLimit for PWM Channels 1-8
    PLIB_MCPWM_ChannelCurrentLimitSetup(MCPWM_ID_0, MCPWM_CHANNEL1, MCPWM_CURRENTLIMIT_SOURCE_IS_COMPARATOR1, MCPWM_CURRENTLIMIT_INPUT_POLARITY_ACTIVE_HIGH, MCPWM_CURRENTLIMIT_OVERRIDE_PWMxH_0, MCPWM_CURRENTLIMIT_OVERRIDE_PWMxL_0, MCPWM_STANDARD_CURRENT_LIMIT_MODE_ENABLE);
    PLIB_MCPWM_ChannelCurrentLimitSetup(MCPWM_ID_0, MCPWM_CHANNEL2, MCPWM_CURRENTLIMIT_SOURCE_IS_COMPARATOR1, MCPWM_CURRENTLIMIT_INPUT_POLARITY_ACTIVE_HIGH, MCPWM_CURRENTLIMIT_OVERRIDE_PWMxH_0, MCPWM_CURRENTLIMIT_OVERRIDE_PWMxL_0, MCPWM_STANDARD_CURRENT_LIMIT_MODE_ENABLE);
    PLIB_MCPWM_ChannelCurrentLimitSetup(MCPWM_ID_0, MCPWM_CHANNEL3, MCPWM_CURRENTLIMIT_SOURCE_IS_COMPARATOR1, MCPWM_CURRENTLIMIT_INPUT_POLARITY_ACTIVE_HIGH, MCPWM_CURRENTLIMIT_OVERRIDE_PWMxH_0, MCPWM_CURRENTLIMIT_OVERRIDE_PWMxL_0, MCPWM_STANDARD_CURRENT_LIMIT_MODE_ENABLE);
    PLIB_MCPWM_ChannelCurrentLimitSetup(MCPWM_ID_0, MCPWM_CHANNEL4, MCPWM_CURRENTLIMIT_SOURCE_IS_COMPARATOR1, MCPWM_CURRENTLIMIT_INPUT_POLARITY_ACTIVE_HIGH, MCPWM_CURRENTLIMIT_OVERRIDE_PWMxH_0, MCPWM_CURRENTLIMIT_OVERRIDE_PWMxL_0, MCPWM_STANDARD_CURRENT_LIMIT_MODE_ENABLE);
    PLIB_MCPWM_ChannelCurrentLimitSetup(MCPWM_ID_0, MCPWM_CHANNEL5, MCPWM_CURRENTLIMIT_SOURCE_IS_COMPARATOR1, MCPWM_CURRENTLIMIT_INPUT_POLARITY_ACTIVE_HIGH, MCPWM_CURRENTLIMIT_OVERRIDE_PWMxH_0, MCPWM_CURRENTLIMIT_OVERRIDE_PWMxL_0, MCPWM_STANDARD_CURRENT_LIMIT_MODE_ENABLE);
    PLIB_MCPWM_ChannelCurrentLimitSetup(MCPWM_ID_0, MCPWM_CHANNEL6, MCPWM_CURRENTLIMIT_SOURCE_IS_COMPARATOR1, MCPWM_CURRENTLIMIT_INPUT_POLARITY_ACTIVE_HIGH, MCPWM_CURRENTLIMIT_OVERRIDE_PWMxH_0, MCPWM_CURRENTLIMIT_OVERRIDE_PWMxL_0, MCPWM_STANDARD_CURRENT_LIMIT_MODE_ENABLE);
    PLIB_MCPWM_ChannelCurrentLimitSetup(MCPWM_ID_0, MCPWM_CHANNEL7, MCPWM_CURRENTLIMIT_SOURCE_IS_COMPARATOR1, MCPWM_CURRENTLIMIT_INPUT_POLARITY_ACTIVE_HIGH, MCPWM_CURRENTLIMIT_OVERRIDE_PWMxH_0, MCPWM_CURRENTLIMIT_OVERRIDE_PWMxL_0, MCPWM_STANDARD_CURRENT_LIMIT_MODE_ENABLE);
    PLIB_MCPWM_ChannelCurrentLimitSetup(MCPWM_ID_0, MCPWM_CHANNEL8, MCPWM_CURRENTLIMIT_SOURCE_IS_COMPARATOR1, MCPWM_CURRENTLIMIT_INPUT_POLARITY_ACTIVE_HIGH, MCPWM_CURRENTLIMIT_OVERRIDE_PWMxH_0, MCPWM_CURRENTLIMIT_OVERRIDE_PWMxL_0, MCPWM_STANDARD_CURRENT_LIMIT_MODE_ENABLE);
    
    PLIB_MCPWM_ChannelCurrentLimitInterruptEnable(MCPWM_ID_0, MCPWM_CHANNEL1);
    PLIB_MCPWM_ChannelCurrentLimitInterruptEnable(MCPWM_ID_0, MCPWM_CHANNEL2);
    PLIB_MCPWM_ChannelCurrentLimitInterruptEnable(MCPWM_ID_0, MCPWM_CHANNEL3);
    PLIB_MCPWM_ChannelCurrentLimitInterruptEnable(MCPWM_ID_0, MCPWM_CHANNEL4);
    PLIB_MCPWM_ChannelCurrentLimitInterruptEnable(MCPWM_ID_0, MCPWM_CHANNEL5);
    PLIB_MCPWM_ChannelCurrentLimitInterruptEnable(MCPWM_ID_0, MCPWM_CHANNEL6);
    PLIB_MCPWM_ChannelCurrentLimitInterruptEnable(MCPWM_ID_0, MCPWM_CHANNEL7);
    PLIB_MCPWM_ChannelCurrentLimitInterruptEnable(MCPWM_ID_0, MCPWM_CHANNEL8);
    
    //Enable Comparator
    PLIB_CMP_Enable(CMP_ID_1);
}

void test_pwm_SetBuck(uint16_t val, bool update){
    if( isHigherThanBuckMaxDC(val) ){
        val = getBuckMaxDC();
    }
    
    pwmData.dutyCycle.update_value.PWM_BUCK1 = val;
    pwmData.dutyCycle.update_value.PWM_BUCK2 = val;
    pwmData.dutyCycle.update_value.PWM_BUCK3 = val;
    pwmData.dutyCycle.update_value.PWM_BUCK4 = val;
    
    pwmData.dutyCycleStatus.updata_status_flag.PWM_BUCK1 = true;
    pwmData.dutyCycleStatus.updata_status_flag.PWM_BUCK2 = true;
    pwmData.dutyCycleStatus.updata_status_flag.PWM_BUCK3 = true;
    pwmData.dutyCycleStatus.updata_status_flag.PWM_BUCK4 = true;
    
    if( update ){
        memcpy(&pwm_controllerData, &pwmData, sizeof(PWM_CONTROLLER_DATA));
        PWM_SIGNAL_Update();
    }
}

void test_pwm_SetBoost(uint16_t val, bool update){
    if( isHigherThanBoostMaxDC(val) ){
        val = getBoostMaxDC();
    }else if( isLowerThanBoostMinDC(val)){
        val = getBoostMinDC();
    }
    
    pwmData.dutyCycle.update_value.PWM_BOOST1 = val;
    pwmData.dutyCycle.update_value.PWM_BOOST2 = val;
    pwmData.dutyCycle.update_value.PWM_BOOST3 = val;
    pwmData.dutyCycle.update_value.PWM_BOOST4 = val;
    
    pwmData.dutyCycleStatus.updata_status_flag.PWM_BOOST1 = true;
    pwmData.dutyCycleStatus.updata_status_flag.PWM_BOOST2 = true;
    pwmData.dutyCycleStatus.updata_status_flag.PWM_BOOST3 = true;
    pwmData.dutyCycleStatus.updata_status_flag.PWM_BOOST4 = true;

    if( update ){
        memcpy(&pwm_controllerData, &pwmData, sizeof(PWM_CONTROLLER_DATA));
        PWM_SIGNAL_Update();
    }
}

void test_pwm_SetBuckBoost(uint16_t buck, uint16_t boost){
    test_pwm_SetBuck(buck, false);
    test_pwm_SetBoost(boost, true);
}




void test_pwm_RampUp( uint16_t target){
    uint16_t dc = 0;
    
    
    do{
        //delay 10ms, half-sine of net power
        test_delay_10us(1000);  
        
        //set dc
        test_pwm_SetBuckBoost(dc, dc);
    }while( analog_voltage_monitorData.adc_raw_data.samples.V380V < target );

}
