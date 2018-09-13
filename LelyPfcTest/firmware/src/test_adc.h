/* 
 * File:   test_adc.h
 * Author: thijs
 *
 * Created on 3 september 2018, 10:34
 */

#ifndef TEST_ADC_H
#define	TEST_ADC_H

#ifdef	__cplusplus
extern "C" {
#endif

    
#include <stdint.h>
#include "analog_voltage_monitor.h"
    
void test_init_adc();
void test_adc_update_adcValues(ANALOG_VOLTAGE_MONITOR_DATA* data);
void test_adc_convertValues(ANALOG_VOLTAGE_MONITOR_DATA* data);

void test_wait_18V(uint16_t targetADC);
void test_wait_12V(uint16_t targetADC);
void test_wait_5V(uint16_t targetADC);

void test_wait_380V(uint16_t targetADC);
void test_wait_325V(uint16_t targetADC);

void test_wait_3V3_1(uint16_t targetADC);
void test_wait_3V3AN_1(uint16_t targetADC);
void test_wait_1V8_1(uint16_t targetADC);

void test_wait_3V3_2(uint16_t targetADC);
void test_wait_3V3AN_2(uint16_t targetADC);
void test_wait_1V8_2(uint16_t targetADC);

bool test_adc_get(unsigned int index, uint16_t* adcVal, float* convVal);


void test_adc_enableFastADC(bool val);
void test_adc_enableSlowADC(bool val);

#ifdef	__cplusplus
}
#endif

#endif	/* TEST_ADC_H */

