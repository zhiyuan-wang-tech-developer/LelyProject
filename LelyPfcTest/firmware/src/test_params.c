#include <proc/p32mk0512mcf100.h>

#include "test_params.h"

test_params_t params = { 
    .boost = {
        .dc_step_err = 10,  // -10 in error
        .dc_step_min = 1,   // -1  at overshoot
        .dc_step_plus = 1,
        .max_dc = 600, //max dc = 50%
        .min_dc = 0
    }, 
    .buck = {
        .dc_step_plus = 1,
        .max_dc = 1100, //max dc = 91%
    },
    .pwr_380V = { 
        .TARGET = (uint16_t) ((380.0 * (4.7e3/(1e6+47e3+4.7e3)) ) * (4095.0/3.300))
    },
    .net = { 
        .current = { 0 },
        .voltage = { 
            .HIGH = { 0 },
            .LOW = { 0 }
        }
    },
    .thermo = {
        .brug = { 0 },
        .elco = { 0 },
        .motor = { 0 },
        .pfc = { 0 },
        .voed = { 0 }
    }
};

void test_delay_10us(uint32_t delay_us){
    PR2 = 1200;
    
    T2CON = 0x8000; //ON, 1:1 = 120 MHz
    
    while( delay_us ){
        if( IFS0bits.T2IF ){
            IFS0bits.T2IF = 0;
            --delay_us;
        }
    }
    
    T2CON = 0;
}


bool test_param_set(unsigned int param, uint16_t value){
    switch( param ){
        case 1: setTarget380V(value); break;
        case 2: setMaxError380V(value); break;
        case 3: setMaxWarn380V(value); break;
        case 4: setMinWarn380V(value); break;
        case 5: setMinError380V(value); break;
        case 6: setMaxErrorIL(value); break;
        case 7: setMaxWarnIL(value); break;
        case 8: setMaxErrorVL(value); break;
        case 9: setMaxWarnVL(value); break;
        case 10: setMinWarnVL(value); break;
        case 11: setMinErrorVL(value); break;
        case 12: setTolerance380V(value); break;
        case 13: setBoostMaxDC(value); break;
        case 14: setBoostMinDC(value); break;
        case 15: setBoostStepPlus(value); break;
        case 16: setBoostStepMin(value); break;
        case 17: setBoostStepError(value); break;
        case 18: setBuckStepPlus(value); break;
        case 19: setTBrugWarn(value); break;
        case 20: setTBrugError(value); break;
        case 21: setTVoedError(value); break;
        case 22: setTVoedWarn(value); break;
        case 23: setTPfcError(value); break;
        case 24: setTPfcWarn(value); break;
        case 25: setTElcoError(value); break;
        case 26: setTElcoWarn(value); break;
        case 27: setTMotorError(value); break;
        case 28: setTMotorWarn(value); break;

        default: return false;
    }
    
    return true;
}

bool test_param_get(unsigned int param, uint16_t* value){
    switch( param ){
        case 1: *value = getTarget380V(); break;
        case 2: *value = getMaxError380V(); break;
        case 3: *value = getMaxWarn380V(); break;
        case 4: *value = getMinWarn380V(); break;
        case 5: *value = getMinError380V(); break;
        case 6: *value = getMaxErrorIL(); break;
        case 7: *value = getMaxWarnIL(); break;
        case 8: *value = getMaxErrorVL(); break;
        case 9: *value = getMaxWarnVL(); break;
        case 10: *value = getMinWarnVL(); break;
        case 11: *value = getMinErrorVL(); break;
        case 12: *value = getTolerance380V(); break;
        case 13: *value = getBoostMaxDC(); break;
        case 14: *value = getBoostMinDC(); break;
        case 15: *value = getBoostStepPlus(); break;
        case 16: *value = getBoostStepMin(); break;
        case 17: *value = getBoostStepError(); break;
        case 18: *value = getBuckStepPlus(); break;
        case 19: *value = getTBrugWarn(); break;
        case 20: *value = getTBrugError(); break;
        case 21: *value = getTVoedError(); break;
        case 22: *value = getTVoedWarn(); break;
        case 23: *value = getTPfcError(); break;
        case 24: *value = getTPfcWarn(); break;
        case 25: *value = getTElcoError(); break;
        case 26: *value = getTElcoWarn(); break;
        case 27: *value = getTMotorError(); break;
        case 28: *value = getTMotorWarn(); break;

        default: return false;
    }
    return true;
}