#include "test_uart.h"
#include "test_pwm.h"
#include "test_pwm.c"

#include <stdio.h>

#define TEST_START_CHAR '{'
#define TEST_END_CHAR   '}'

#define TEST_CMD_LEN 64
char cmd[TEST_CMD_LEN];
uint16_t cmd_pos = 0;


static enum {
    ProtoFindStart,
    ProtoFindEnd
} protoState = ProtoFindStart;



void test_uart_parseSetPWM(char* c){
    char type, param;
    unsigned int channel, value;
    
    bool set = false;
    
    if( sscanf(c, "%c%u%c%u", &type, &channel, &param, &value) == 4){
        //all
        if( type == 'b'){
            if( param == 'D' || param == 'd' ){
                test_pwm_SetBoostDC(channel, value, true);
                set = true;
            }
            else if( param == 'P' || param == 'p' ){
                test_pwm_SetBoostPhase(channel, value, true);  
                set = true;
            }
        }else if(type == 'B'){
            if( param == 'D' || param == 'd' ){
                test_pwm_SetBuckDC(channel, value, true);
                set = true;
            }
            else if( param == 'P' || param == 'p' ){
                test_pwm_SetBuckPhase(channel, value, true);
                set = true;
            }
        }
        
        if( set ){
            //PWM set, send current status as response
            uint16_t dc, phase;
            
            test_pwm_Get( channel, type, &dc, &phase);
            u2_write("%cs%c%uP%uD%u%c", TEST_START_CHAR, type, channel, phase, dc, TEST_END_CHAR);
        }
    }
}



void test_uart_parseSetParameter(char* c){
    unsigned int param, value;
    
    if( sscanf(c, "%*c%u=%u", &param, &value) == 2 ){
        if( test_param_set(param, value) ){
            test_param_get(param, (uint16_t*) &value);
            u2_write("%csP%u=%u%c", TEST_START_CHAR, param, value, TEST_END_CHAR);
        }
    }
}


void test_uart_parseAdc(char* c){
    
}


void test_uart_parsGet(char* c){
    
}




void test_uart_parsSet(char* c){
    switch( *c ){
        case 'b': 
        case 'B':
            test_uart_parseSetPWM(c);
            break;
            
        case 'p':
        case 'P':
            test_uart_parseSetParameter(c);
            break;
    }
}

void test_uart_parseCommand(char* c){
    switch( *c ){
        case 's':
        case 'S':
            test_uart_parsSet(++c);
            break;
        case 'g':
        case 'G':
            test_uart_parsGet(++c);
            break;
            
        case 'a':
        case 'A':
            test_uart_parseAdc(++c);
            break;
            
        default:
            break;
    }
}


bool test_uart_findCommand(char c){
    switch( protoState ){
        case ProtoFindStart:
            if( c == TEST_START_CHAR ){
                cmd_pos = 0;
                cmd[cmd_pos++] = c;
            }
            break;
            
        case ProtoFindEnd:
            cmd[cmd_pos++] = c;
            
            if( (c == TEST_END_CHAR) || (cmd_pos >= TEST_CMD_LEN) ){
                protoState = ProtoFindStart;
                return true;
            }
            break;
        default:
            break;
    }
    return false;
}
