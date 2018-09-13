#include "test_pwm.h"
#include "test_params.h"
#include "test_adc.c"
#include "system/devcon/sys_devcon.h"
#include <string.h>

#include <peripheral/cmp/plib_cmp.h>
#include <peripheral/cdac/plib_cdac.h>
#include <peripheral/devcon/plib_devcon.h>

extern PWM_CONTROLLER_DATA pwm_controllerData;
static PWM_CONTROLLER_DATA pwmData;

static void test_pwm_setCurrentLimit();


#define C1OUT_ON_LED


void test_pwm_init(){
    uint16_t period = 1200;
    
    //Enable current limit
    test_pwm_setCurrentLimit();
    
    //Disable for reconfiguration
    PLIB_MCPWM_Disable(MCPWM_ID_0);
    
    //Enable PWM outputs 7H - 10H
    SYS_DEVCON_SystemUnlock();   
    
//    PLIB_DEVCON_2WireJTAGDisableTDO(DEVCON_ID_0);       //disable JTAG TDO
    PLIB_DEVCON_JTAGPortDisable(DEVCON_ID_0);           //disable JTAG
    
    CFGCONbits.PWMAPIN1 = 1;    //output 7H on pin 1L
    CFGCONbits.PWMAPIN2 = 1;    //output 8H on pin 2L
    CFGCONbits.PWMAPIN3 = 1;    //output 9H on pin 3L
    CFGCONbits.PWMAPIN4 = 1;    //output 10H on pin 4L    
    SYS_DEVCON_SystemLock();
    
    
    
    PLIB_MCPWM_PrimaryTimerSetup( MCPWM_ID_0, MCPWM_CLOCK_DIVIDE_BY_1, period);
    
    PLIB_MCPWM_ChannelSetup(MCPWM_ID_0, PWM_ChanBuck1, MCPWM_TIME_BASE_SOURCE_PRIMARY, MCPWM_TIME_BASE_SYNCHRONIZED, MCPWM_EDGE_ALIGNED, MCPWM_OUTPUT_REDUNDANT_MODE, MCPWM_PWMxH_ACTIVEHIGH, MCPWM_PWMxL_ACTIVELOW, MCPWM_DEADTIME_DISABLE, MCPWM_DEADTIME_COMPENSATION_POLARITY_ACTIVE_LOW);
    PLIB_MCPWM_ChannelSetup(MCPWM_ID_0, PWM_ChanBuck2, MCPWM_TIME_BASE_SOURCE_PRIMARY, MCPWM_TIME_BASE_SYNCHRONIZED, MCPWM_EDGE_ALIGNED, MCPWM_OUTPUT_REDUNDANT_MODE, MCPWM_PWMxH_ACTIVEHIGH, MCPWM_PWMxL_ACTIVELOW, MCPWM_DEADTIME_DISABLE, MCPWM_DEADTIME_COMPENSATION_POLARITY_ACTIVE_LOW);
    PLIB_MCPWM_ChannelSetup(MCPWM_ID_0, PWM_ChanBuck3, MCPWM_TIME_BASE_SOURCE_PRIMARY, MCPWM_TIME_BASE_SYNCHRONIZED, MCPWM_EDGE_ALIGNED, MCPWM_OUTPUT_REDUNDANT_MODE, MCPWM_PWMxH_ACTIVEHIGH, MCPWM_PWMxL_ACTIVELOW, MCPWM_DEADTIME_DISABLE, MCPWM_DEADTIME_COMPENSATION_POLARITY_ACTIVE_LOW);
    PLIB_MCPWM_ChannelSetup(MCPWM_ID_0, PWM_ChanBuck4, MCPWM_TIME_BASE_SOURCE_PRIMARY, MCPWM_TIME_BASE_SYNCHRONIZED, MCPWM_EDGE_ALIGNED, MCPWM_OUTPUT_REDUNDANT_MODE, MCPWM_PWMxH_ACTIVEHIGH, MCPWM_PWMxL_ACTIVELOW, MCPWM_DEADTIME_DISABLE, MCPWM_DEADTIME_COMPENSATION_POLARITY_ACTIVE_LOW);
    PLIB_MCPWM_ChannelSetup(MCPWM_ID_0, PWM_ChanBoost1, MCPWM_TIME_BASE_SOURCE_PRIMARY, MCPWM_TIME_BASE_SYNCHRONIZED, MCPWM_EDGE_ALIGNED, MCPWM_OUTPUT_REDUNDANT_MODE, MCPWM_PWMxH_ACTIVEHIGH, MCPWM_PWMxL_ACTIVELOW, MCPWM_DEADTIME_DISABLE, MCPWM_DEADTIME_COMPENSATION_POLARITY_ACTIVE_LOW);
    PLIB_MCPWM_ChannelSetup(MCPWM_ID_0, PWM_ChanBoost2, MCPWM_TIME_BASE_SOURCE_PRIMARY, MCPWM_TIME_BASE_SYNCHRONIZED, MCPWM_EDGE_ALIGNED, MCPWM_OUTPUT_REDUNDANT_MODE, MCPWM_PWMxH_ACTIVEHIGH, MCPWM_PWMxL_ACTIVELOW, MCPWM_DEADTIME_DISABLE, MCPWM_DEADTIME_COMPENSATION_POLARITY_ACTIVE_LOW);
    PLIB_MCPWM_ChannelSetup(MCPWM_ID_0, PWM_ChanBoost3, MCPWM_TIME_BASE_SOURCE_PRIMARY, MCPWM_TIME_BASE_SYNCHRONIZED, MCPWM_EDGE_ALIGNED, MCPWM_OUTPUT_REDUNDANT_MODE, MCPWM_PWMxH_ACTIVEHIGH, MCPWM_PWMxL_ACTIVELOW, MCPWM_DEADTIME_DISABLE, MCPWM_DEADTIME_COMPENSATION_POLARITY_ACTIVE_LOW);
    PLIB_MCPWM_ChannelSetup(MCPWM_ID_0, PWM_ChanBoost4, MCPWM_TIME_BASE_SOURCE_PRIMARY, MCPWM_TIME_BASE_SYNCHRONIZED, MCPWM_EDGE_ALIGNED, MCPWM_OUTPUT_REDUNDANT_MODE, MCPWM_PWMxH_ACTIVEHIGH, MCPWM_PWMxL_ACTIVELOW, MCPWM_DEADTIME_DISABLE, MCPWM_DEADTIME_COMPENSATION_POLARITY_ACTIVE_LOW);    
    
    
    //Disable faults    
    PLIB_MCPWM_ChannelFaultSetup(MCPWM_ID_0, PWM_ChanBuck1, MCPWM_FAULT_SOURCE_IS_FLT1, MCPWM_FAULT_INPUT_POLARITY_ACTIVE_HIGH, MCPWM_FAULT_OVERRIDE_PWMxH_0, MCPWM_FAULT_OVERRIDE_PWMxL_0, MCPWM_FAULT_MODE_DISABLED);
    PLIB_MCPWM_ChannelFaultSetup(MCPWM_ID_0, PWM_ChanBuck2, MCPWM_FAULT_SOURCE_IS_FLT1, MCPWM_FAULT_INPUT_POLARITY_ACTIVE_HIGH, MCPWM_FAULT_OVERRIDE_PWMxH_0, MCPWM_FAULT_OVERRIDE_PWMxL_0, MCPWM_FAULT_MODE_DISABLED);
    PLIB_MCPWM_ChannelFaultSetup(MCPWM_ID_0, PWM_ChanBuck3, MCPWM_FAULT_SOURCE_IS_FLT1, MCPWM_FAULT_INPUT_POLARITY_ACTIVE_HIGH, MCPWM_FAULT_OVERRIDE_PWMxH_0, MCPWM_FAULT_OVERRIDE_PWMxL_0, MCPWM_FAULT_MODE_DISABLED);
    PLIB_MCPWM_ChannelFaultSetup(MCPWM_ID_0, PWM_ChanBuck4, MCPWM_FAULT_SOURCE_IS_FLT1, MCPWM_FAULT_INPUT_POLARITY_ACTIVE_HIGH, MCPWM_FAULT_OVERRIDE_PWMxH_0, MCPWM_FAULT_OVERRIDE_PWMxL_0, MCPWM_FAULT_MODE_DISABLED);
    PLIB_MCPWM_ChannelFaultSetup(MCPWM_ID_0, PWM_ChanBoost1, MCPWM_FAULT_SOURCE_IS_FLT1, MCPWM_FAULT_INPUT_POLARITY_ACTIVE_HIGH, MCPWM_FAULT_OVERRIDE_PWMxH_0, MCPWM_FAULT_OVERRIDE_PWMxL_0, MCPWM_FAULT_MODE_DISABLED);
    PLIB_MCPWM_ChannelFaultSetup(MCPWM_ID_0, PWM_ChanBoost2, MCPWM_FAULT_SOURCE_IS_FLT1, MCPWM_FAULT_INPUT_POLARITY_ACTIVE_HIGH, MCPWM_FAULT_OVERRIDE_PWMxH_0, MCPWM_FAULT_OVERRIDE_PWMxL_0, MCPWM_FAULT_MODE_DISABLED);
    PLIB_MCPWM_ChannelFaultSetup(MCPWM_ID_0, PWM_ChanBoost3, MCPWM_FAULT_SOURCE_IS_FLT1, MCPWM_FAULT_INPUT_POLARITY_ACTIVE_HIGH, MCPWM_FAULT_OVERRIDE_PWMxH_0, MCPWM_FAULT_OVERRIDE_PWMxL_0, MCPWM_FAULT_MODE_DISABLED);
    PLIB_MCPWM_ChannelFaultSetup(MCPWM_ID_0, PWM_ChanBoost4, MCPWM_FAULT_SOURCE_IS_FLT1, MCPWM_FAULT_INPUT_POLARITY_ACTIVE_HIGH, MCPWM_FAULT_OVERRIDE_PWMxH_0, MCPWM_FAULT_OVERRIDE_PWMxL_0, MCPWM_FAULT_MODE_DISABLED);

    //Set duty cycles to 0
    PLIB_MCPWM_ChannelPrimaryDutyCycleSet(MCPWM_ID_0, PWM_ChanBuck1, 0);
    PLIB_MCPWM_ChannelPrimaryDutyCycleSet(MCPWM_ID_0, PWM_ChanBuck2, 0);
    PLIB_MCPWM_ChannelPrimaryDutyCycleSet(MCPWM_ID_0, PWM_ChanBuck3, 0);
    PLIB_MCPWM_ChannelPrimaryDutyCycleSet(MCPWM_ID_0, PWM_ChanBuck4, 0);
    PLIB_MCPWM_ChannelPrimaryDutyCycleSet(MCPWM_ID_0, PWM_ChanBoost1, 0);
    PLIB_MCPWM_ChannelPrimaryDutyCycleSet(MCPWM_ID_0, PWM_ChanBoost2, 0);
    PLIB_MCPWM_ChannelPrimaryDutyCycleSet(MCPWM_ID_0, PWM_ChanBoost3, 0);
    PLIB_MCPWM_ChannelPrimaryDutyCycleSet(MCPWM_ID_0, PWM_ChanBoost4, 0);

    //Set phase shift to 0
    PLIB_MCPWM_ChannelPhaseSet(MCPWM_ID_0, PWM_ChanBuck1, period * 10 / 360.0);
    PLIB_MCPWM_ChannelPhaseSet(MCPWM_ID_0, PWM_ChanBuck2, period * 190 / 360.0);
    
    PLIB_MCPWM_ChannelPhaseSet(MCPWM_ID_0, PWM_ChanBuck3, period * 100 / 360.0);
    PLIB_MCPWM_ChannelPhaseSet(MCPWM_ID_0, PWM_ChanBuck4, period * 280 / 360.0);
    
    PLIB_MCPWM_ChannelPhaseSet(MCPWM_ID_0, PWM_ChanBoost1, period * 0 / 360.0);
    PLIB_MCPWM_ChannelPhaseSet(MCPWM_ID_0, PWM_ChanBoost2, period * 180 / 360.0);
    
    PLIB_MCPWM_ChannelPhaseSet(MCPWM_ID_0, PWM_ChanBoost3, period * 90 / 360.0);
    PLIB_MCPWM_ChannelPhaseSet(MCPWM_ID_0, PWM_ChanBoost4, period * 270 / 360.0);
    
    //Enable PWM_H outputs
    PLIB_MCPWM_ChannelPWMxHEnable(MCPWM_ID_0, PWM_ChanBuck1);
    PLIB_MCPWM_ChannelPWMxHEnable(MCPWM_ID_0, PWM_ChanBuck2);
    PLIB_MCPWM_ChannelPWMxHEnable(MCPWM_ID_0, PWM_ChanBuck3);
    PLIB_MCPWM_ChannelPWMxHEnable(MCPWM_ID_0, PWM_ChanBuck4);
    PLIB_MCPWM_ChannelPWMxHEnable(MCPWM_ID_0, PWM_ChanBoost1);
    PLIB_MCPWM_ChannelPWMxHEnable(MCPWM_ID_0, PWM_ChanBoost2);
    PLIB_MCPWM_ChannelPWMxHEnable(MCPWM_ID_0, PWM_ChanBoost3);
    PLIB_MCPWM_ChannelPWMxHEnable(MCPWM_ID_0, PWM_ChanBoost4);

    //Disable PMW_H overrides
    PLIB_MCPWM_ChannelPWMxHOverrideDisable(MCPWM_ID_0, PWM_ChanBuck1);
    PLIB_MCPWM_ChannelPWMxHOverrideDisable(MCPWM_ID_0, PWM_ChanBuck2);
    PLIB_MCPWM_ChannelPWMxHOverrideDisable(MCPWM_ID_0, PWM_ChanBuck3);
    PLIB_MCPWM_ChannelPWMxHOverrideDisable(MCPWM_ID_0, PWM_ChanBuck4);
    PLIB_MCPWM_ChannelPWMxHOverrideDisable(MCPWM_ID_0, PWM_ChanBoost1);
    PLIB_MCPWM_ChannelPWMxHOverrideDisable(MCPWM_ID_0, PWM_ChanBoost2);
    PLIB_MCPWM_ChannelPWMxHOverrideDisable(MCPWM_ID_0, PWM_ChanBoost3);
    PLIB_MCPWM_ChannelPWMxHOverrideDisable(MCPWM_ID_0, PWM_ChanBoost4);
    
    
//    //Disable current limites
//    PLIB_MCPWM_ChannelCurrentLimitSetup(MCPWM_ID_0, PWM_ChanBoost1, MCPWM_CURRENTLIMIT_SOURCE_IS_COMPARATOR1, MCPWM_CURRENTLIMIT_INPUT_POLARITY_ACTIVE_LOW, MCPWM_CURRENTLIMIT_OVERRIDE_PWMxH_0, MCPWM_CURRENTLIMIT_OVERRIDE_PWMxL_0, MCPWM_CURRENTLIMIT_DISABLE);
//    PLIB_MCPWM_ChannelCurrentLimitSetup(MCPWM_ID_0, PWM_ChanBoost2, MCPWM_CURRENTLIMIT_SOURCE_IS_COMPARATOR1, MCPWM_CURRENTLIMIT_INPUT_POLARITY_ACTIVE_LOW, MCPWM_CURRENTLIMIT_OVERRIDE_PWMxH_0, MCPWM_CURRENTLIMIT_OVERRIDE_PWMxL_0, MCPWM_CURRENTLIMIT_DISABLE);
//    PLIB_MCPWM_ChannelCurrentLimitSetup(MCPWM_ID_0, PWM_ChanBoost3, MCPWM_CURRENTLIMIT_SOURCE_IS_COMPARATOR1, MCPWM_CURRENTLIMIT_INPUT_POLARITY_ACTIVE_LOW, MCPWM_CURRENTLIMIT_OVERRIDE_PWMxH_0, MCPWM_CURRENTLIMIT_OVERRIDE_PWMxL_0, MCPWM_CURRENTLIMIT_DISABLE);
//    PLIB_MCPWM_ChannelCurrentLimitSetup(MCPWM_ID_0, PWM_ChanBoost4, MCPWM_CURRENTLIMIT_SOURCE_IS_COMPARATOR1, MCPWM_CURRENTLIMIT_INPUT_POLARITY_ACTIVE_LOW, MCPWM_CURRENTLIMIT_OVERRIDE_PWMxH_0, MCPWM_CURRENTLIMIT_OVERRIDE_PWMxL_0, MCPWM_CURRENTLIMIT_DISABLE);
//    PLIB_MCPWM_ChannelCurrentLimitSetup(MCPWM_ID_0, PWM_ChanBuck1, MCPWM_CURRENTLIMIT_SOURCE_IS_COMPARATOR1, MCPWM_CURRENTLIMIT_INPUT_POLARITY_ACTIVE_LOW, MCPWM_CURRENTLIMIT_OVERRIDE_PWMxH_0, MCPWM_CURRENTLIMIT_OVERRIDE_PWMxL_0, MCPWM_CURRENTLIMIT_DISABLE);
//    PLIB_MCPWM_ChannelCurrentLimitSetup(MCPWM_ID_0, PWM_ChanBuck2, MCPWM_CURRENTLIMIT_SOURCE_IS_COMPARATOR1, MCPWM_CURRENTLIMIT_INPUT_POLARITY_ACTIVE_LOW, MCPWM_CURRENTLIMIT_OVERRIDE_PWMxH_0, MCPWM_CURRENTLIMIT_OVERRIDE_PWMxL_0, MCPWM_CURRENTLIMIT_DISABLE);
//    PLIB_MCPWM_ChannelCurrentLimitSetup(MCPWM_ID_0, PWM_ChanBuck3, MCPWM_CURRENTLIMIT_SOURCE_IS_COMPARATOR1, MCPWM_CURRENTLIMIT_INPUT_POLARITY_ACTIVE_LOW, MCPWM_CURRENTLIMIT_OVERRIDE_PWMxH_0, MCPWM_CURRENTLIMIT_OVERRIDE_PWMxL_0, MCPWM_CURRENTLIMIT_DISABLE);
//    PLIB_MCPWM_ChannelCurrentLimitSetup(MCPWM_ID_0, PWM_ChanBuck4, MCPWM_CURRENTLIMIT_SOURCE_IS_COMPARATOR1, MCPWM_CURRENTLIMIT_INPUT_POLARITY_ACTIVE_LOW, MCPWM_CURRENTLIMIT_OVERRIDE_PWMxH_0, MCPWM_CURRENTLIMIT_OVERRIDE_PWMxL_0, MCPWM_CURRENTLIMIT_DISABLE);
//    
    
    //Enable PWM
    PLIB_MCPWM_Enable(MCPWM_ID_0);
}


void test_pwm_manual(){
    //Re-initialize
    test_pwm_init();

    volatile pwm_channel_t pwmChannels = PWM_ALL;
    volatile uint16_t buck_dc = 0;
    volatile uint16_t boost_dc = 0;
    
    volatile uint16_t boost1_phase = 0;         //= 0%
    volatile uint16_t boost2_phase = 600;       //= 50%
    volatile uint16_t boost3_phase = 300;       //= 25%
    volatile uint16_t boost4_phase = 900;       //= 75%
        
    volatile uint16_t buck_boost_phase = 120;   //= 10% shift
    
    volatile uint16_t buck1_phase = boost1_phase + buck_boost_phase;
    volatile uint16_t buck2_phase = boost2_phase + buck_boost_phase;
    volatile uint16_t buck3_phase = boost3_phase + buck_boost_phase;
    volatile uint16_t buck4_phase = boost4_phase + buck_boost_phase;
    

    //Enable FPCs
    V_PFC_STOP_12_NOn();
    V_PFC_STOP_34_NOn();
    
    //Flag to clear overload error state
    volatile bool resetOverloads = false;
    
    //Flag to stop the manual test
    volatile bool sw_exit = false;
    
    do {
        // Period = 1200
        // Change buck/boost phase and buck/boost dc to test
        test_pwm_SetBuckBoostDC(pwmChannels, buck_dc, boost_dc);

        buck1_phase = boost1_phase + buck_boost_phase;
        buck2_phase = boost2_phase + buck_boost_phase;
        buck3_phase = boost3_phase + buck_boost_phase;
        buck4_phase = boost4_phase + buck_boost_phase;

        test_pwm_SetBuckBoostPhase(PWM_Pair_1, buck1_phase, boost1_phase);
        test_pwm_SetBuckBoostPhase(PWM_Pair_2, buck2_phase, boost2_phase);
        test_pwm_SetBuckBoostPhase(PWM_Pair_3, buck3_phase, boost3_phase);
        test_pwm_SetBuckBoostPhase(PWM_Pair_4, buck4_phase, boost4_phase);

        if( resetOverloads ){
            resetOverloads = false;

            V_PFC_STOP_12_NOff();
            V_PFC_STOP_34_NOff();

            V_PFC_STOP_12_NOn();
            V_PFC_STOP_34_NOn();
        }

        
        SYS_DEBUG_BreakPoint();
    }while( !sw_exit );
}

void test_pwm_cooldown(){
                        // s  -> ms  -> 10us
    uint32_t delay_10us = 60U * 1000U * 100U;
     
    //Disable PWM, allow capacitors to drain
    PLIB_MCPWM_Disable(MCPWM_ID_0);
    test_delay_10us( delay_10us );
}

void test_pwm(){
    SYS_DEBUG_BreakPoint();

    //Do manual testing
    test_pwm_manual();
    test_pwm_cooldown();
    
    
    //Try with a single pair
    test_pwm_RampUp(PWM_Pair_3);
        
    test_pwm_cooldown();
    
    //Now try it with a whole group
    test_pwm_RampUp(PWM_Group_34);
    
    test_pwm_cooldown();
        
    //Now try it with all PWMs
    test_pwm_RampUp(PWM_ALL);

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


void test_pwm_RampUp( pwm_channel_t pwms){    
    volatile bool sw_exit = false;

    enum {
        Init,
        RampBuck,
        RampBoost,
        Settle
    } step = Init;
        
    volatile uint16_t buck_dc = 0;
    volatile uint16_t boost_dc = 150;           //Initialize with 150 / 120Mhz = 1.25 us overlap
    
    const uint16_t boost1_phase = 0;         //= 0%
    const uint16_t boost2_phase = 600;       //= 50%
    const uint16_t boost3_phase = 300;       //= 25%
    const uint16_t boost4_phase = 900;       //= 75%
        
    volatile uint16_t buck_boost_phase = 120;   //= 10% shift
    
    volatile uint16_t buck1_phase = boost1_phase + buck_boost_phase;
    volatile uint16_t buck2_phase = boost2_phase + buck_boost_phase;
    volatile uint16_t buck3_phase = boost3_phase + buck_boost_phase;
    volatile uint16_t buck4_phase = boost4_phase + buck_boost_phase;
    
    
    volatile uint32_t buck_delay_step = 10 * 100;           //10ms between steps
    volatile uint32_t boost_delay_step = 1 * 200 * 100;    //500ms between steps


    do{
        uint16_t target = getTarget380V();
        
        switch( step ){
            case Init:
                //Off during init
                V_LED3_GOff();
                V_LED3_ROff();                
                
                //Initialize driver
                test_pwm_init();
                
                //Set safe initial values                
                buck_dc = 200;
                boost_dc = 150;
                buck_boost_phase = 120;   //= 10% shift
                boost_delay_step = 1 * 200 * 100;    //200ms between steps
                 
                buck1_phase = boost1_phase + buck_boost_phase;
                buck2_phase = boost2_phase + buck_boost_phase;
                buck3_phase = boost3_phase + buck_boost_phase;
                buck4_phase = boost4_phase + buck_boost_phase;

                test_pwm_SetBuckBoostPhase(PWM_Pair_1, buck1_phase, boost1_phase);
                test_pwm_SetBuckBoostPhase(PWM_Pair_2, buck2_phase, boost2_phase);
                test_pwm_SetBuckBoostPhase(PWM_Pair_3, buck3_phase, boost3_phase);
                test_pwm_SetBuckBoostPhase(PWM_Pair_4, buck4_phase, boost4_phase);

                test_pwm_SetBuckBoostDC(pwms, buck_dc, boost_dc);
                
                //Reset overloads
                V_PFC_STOP_12_NOff();
                V_PFC_STOP_34_NOff();
    
                V_PFC_STOP_12_NOn();
                V_PFC_STOP_34_NOn();                
                
                step = RampBuck;
                break;
                
            case RampBuck:
                //Orange during Buck
                V_LED3_GOn();
                V_LED3_ROn();                

                
                //Ramp up to target buck DC
                test_delay_10us(buck_delay_step);   //delay between steps
                
                if( buck_dc >= getBuckMaxDC() ){
                    step = RampBoost;
                }else{
                    buck_dc += getBuckStepPlus();
                }
                
                test_pwm_SetBuckBoostDC(pwms, buck_dc, boost_dc);
                break;
                
            case RampBoost:
                //Green when ramping boost up
                V_LED3_GOn();
                V_LED3_ROff();                

                
                test_delay_10us(boost_delay_step);   //delay between steps
                
                if( analog_voltage_monitorData.adc_raw_data.samples.V380V < (target - 100) ){
                    //Target -20V
                    boost_dc += 10;
                    boost_delay_step = 1 * 1000 * 100;    //1000ms between steps;
                    
                }else if(analog_voltage_monitorData.adc_raw_data.samples.V380V < (target - 50) ){
                    //Target -10V
                    boost_dc += 5;
                    boost_delay_step = 500 * 100;    //500ms between steps;
                    
                }else if(analog_voltage_monitorData.adc_raw_data.samples.V380V < (target - 10) ){
                    //Target -5V
                    boost_dc += 2;
                    boost_delay_step = 200 * 100;    //200ms between steps;
                }else if(analog_voltage_monitorData.adc_raw_data.samples.V380V < (target - 5) ){
                    //Target -5V
                    boost_dc += 1;
                    boost_delay_step = 100 * 100;       //100ms between steps;                    
                }
                
                if( analog_voltage_monitorData.adc_raw_data.samples.V380V >= (target - 10) ){
                    //Target -5V
                    step = Settle;
                    boost_delay_step = 100 * 100;       //100ms between steps;
                }
                                
                if( boost_dc > getBoostMaxDC() ){
                    boost_dc = getBoostMaxDC();
                }
                
                test_pwm_SetBuckBoostDC(pwms, buck_dc, boost_dc);
                break;

            case Settle:
                //Red when ramping boost down
                V_LED3_GOff();
                V_LED3_ROn();                

                
                test_delay_10us(boost_delay_step);   //delay between steps
                
                if( analog_voltage_monitorData.adc_raw_data.samples.V380V > (target + 50) ){
                    //Target +20V
                    boost_dc -= 10;
                    boost_delay_step = 200 * 100;  //200ms
                }
                else if( analog_voltage_monitorData.adc_raw_data.samples.V380V > (target + 10) ){
                    //Target +5V
                    boost_dc -= 1;
                    boost_delay_step = 200 * 100;  //200ms
                }else{
                    step = RampBoost;
                }
                
                test_pwm_SetBuckBoostDC(pwms, buck_dc, boost_dc);

                break;
        }
    }while( !sw_exit );
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