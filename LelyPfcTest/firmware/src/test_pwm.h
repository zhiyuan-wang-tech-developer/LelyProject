/* 
 * File:   test_pwm.h
 * Author: thijs
 *
 * Created on 3 september 2018, 10:52
 * 
 * Reviser: Zhiyuan
 * Changed on 21 september 2018, 11:56
 */

#ifndef TEST_PWM_H
#define	TEST_PWM_H

#ifdef	__cplusplus
extern "C" {
#endif

#include "pwm_controller.h"

// change PWM channel index
typedef enum {
    PWM_Pair_1 = 1,
    PWM_Pair_2 = 2,
    PWM_Pair_3 = 3,
    PWM_Pair_4 = 4,
    PWM_Group_12 = 5,
    PWM_Group_34 = 6,
    PWM_ALL = 7,
} pwm_channel_t;

typedef enum {
    PWM_BUCK = 'B',
    PWM_BOOST = 'b'
} pwm_type_t;

    
void test_pwm();
void test_pwm_SetBuckBoostDC(pwm_channel_t channel, uint16_t buck, uint16_t boost);
void test_pwm_SetBuckDC(pwm_channel_t channel,uint16_t val, bool update);
void test_pwm_SetBoostDC(pwm_channel_t channel, uint16_t val, bool update);
void test_pwm_SetBuckBoostPhase(pwm_channel_t channel, uint16_t buck, uint16_t boost);
void test_pwm_SetBuckPhase(pwm_channel_t channel,uint16_t val, bool update);
void test_pwm_SetBoostPhase(pwm_channel_t channel, uint16_t val, bool update);

bool test_pwm_Get( pwm_channel_t channel, pwm_type_t type, uint16_t* dc, uint16_t* phase);

void test_pwm_RampUp( pwm_channel_t pwms);



#ifdef	__cplusplus
}
#endif

#endif	/* TEST_PWM_H */

