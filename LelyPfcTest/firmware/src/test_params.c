#include <proc/p32mk0512mcf100.h>

#include "test_params.h"

test_params_t params = { {0} };

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