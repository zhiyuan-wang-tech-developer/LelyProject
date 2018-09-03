/* 
 * File:   test_params.h
 * Author: thijs
 *
 * Created on 3 september 2018, 15:32
 */

#ifndef TEST_PARAMS_H
#define	TEST_PARAMS_H

#include <stdint.h>
#include <stdbool.h>


typedef struct {
    uint16_t WARN;
    uint16_t ERROR;
} thresholds_t;

typedef struct {
    struct {
        uint16_t TARGET;
        thresholds_t HIGH;
        thresholds_t LOW;
        uint16_t TOLERANCE;
    } pwr_380V;
    
    struct{
        thresholds_t current;
        struct{
            thresholds_t HIGH;
            thresholds_t LOW;
        } voltage;
    } net;

    struct { 
        thresholds_t brug;
        thresholds_t voed;
        thresholds_t pfc;
        thresholds_t elco;
        thresholds_t motor;
    } thermo;
    
    struct {
        uint16_t max_dc;
        uint16_t min_dc;
        uint16_t dc_step_plus;
        uint16_t dc_step_min;
        uint16_t dc_step_err;
    } boost;
    
    struct {
        uint16_t max_dc;
        uint16_t dc_step_plus;
    } buck;
    
} test_params_t;


extern test_params_t params;


//Macro for creating parameter functions
// Creates getParam, setParam, isHigherThanParam and isLowerThanParam functions
#define GET_SET_PARAM(name,var) \
    __inline uint16_t get ## name (){ \
        return var; \
    } \
    __inline void set ## name (uint16_t val){ \
        var = val; \
    } \
    __inline bool isHigherThan ## name(uint16_t val){ \
        return val > get ## name(); \
    }\
    __inline bool isLowerThan ## name(uint16_t val){ \
        return val < get ## name(); \
    }


GET_SET_PARAM(Target380V, params.pwr_380V.TARGET);
GET_SET_PARAM(Tolerance380V, params.pwr_380V.TOLERANCE);
GET_SET_PARAM(MaxWarn380V, params.pwr_380V.HIGH.WARN);
GET_SET_PARAM(MaxError380V, params.pwr_380V.HIGH.ERROR);
GET_SET_PARAM(MinWarn380V, params.pwr_380V.LOW.WARN);
GET_SET_PARAM(MinError380V, params.pwr_380V.LOW.ERROR);

GET_SET_PARAM(MaxWarnVL, params.net.voltage.HIGH.WARN);
GET_SET_PARAM(MaxErrorVL, params.net.voltage.HIGH.ERROR);
GET_SET_PARAM(MinWarnVL, params.net.voltage.LOW.WARN);
GET_SET_PARAM(MinErrorVL, params.net.voltage.LOW.ERROR);
GET_SET_PARAM(MaxWarnIL, params.net.current.WARN);
GET_SET_PARAM(MaxErrorIL, params.net.current.ERROR);

GET_SET_PARAM(BoostMaxDC, params.boost.max_dc);
GET_SET_PARAM(BoostMinDC, params.boost.min_dc);
GET_SET_PARAM(BoostStepPlus, params.boost.dc_step_plus);
GET_SET_PARAM(BoostStepMin, params.boost.dc_step_min);
GET_SET_PARAM(BoostStepError, params.boost.dc_step_err);

GET_SET_PARAM(BuckMaxDC, params.buck.max_dc);
GET_SET_PARAM(BuckStepPlus, params.buck.dc_step_plus);

GET_SET_PARAM(TBrugError, params.thermo.brug.ERROR);
GET_SET_PARAM(TBrugWarn, params.thermo.brug.WARN);
GET_SET_PARAM(TVoedError, params.thermo.voed.ERROR);
GET_SET_PARAM(TVoedWarn, params.thermo.voed.WARN);
GET_SET_PARAM(TPfcError, params.thermo.pfc.ERROR);
GET_SET_PARAM(TPfcWarn, params.thermo.pfc.WARN);
GET_SET_PARAM(TElcoError, params.thermo.elco.ERROR);
GET_SET_PARAM(TElcoWarn, params.thermo.elco.WARN);
GET_SET_PARAM(TMotorError, params.thermo.motor.ERROR);
GET_SET_PARAM(TMotorWarn, params.thermo.motor.WARN);


void test_delay_10us(uint32_t delay_us);

#endif	/* TEST_PARAMS_H */

