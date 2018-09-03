/* 
 * File:   test_pwm.h
 * Author: thijs
 *
 * Created on 3 september 2018, 10:52
 */

#ifndef TEST_PWM_H
#define	TEST_PWM_H

#ifdef	__cplusplus
extern "C" {
#endif

#include "pwm_controller.h"

void test_pwm();
void test_pwm_SetBuckBoost(uint16_t buck, uint16_t boost);
void test_pwm_SetBuck(uint16_t val, bool update);
void test_pwm_SetBoost(uint16_t val, bool update);

#ifdef	__cplusplus
}
#endif

#endif	/* TEST_PWM_H */

