#include "test_uart.h"
#include "test_pwm.h"
#include "test_params.h"
#include "test_adc.h"

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




bool test_io_get(unsigned int index, bool* value){
    switch( index ){
        case 1: *value = V_PFC_OL12StateGet(); break;
        case 2: *value = V_PFC_OL34StateGet(); break;
        
        case 3: *value = V_EN_12VStateGet(); break;
        case 4: *value = V_EN_3V3_1StateGet(); break;        
        case 5: *value = V_EN_3V3_AN1StateGet(); break;
        case 6: *value = V_EN_1V8_1StateGet(); break;        
        
        case 7: *value = V_EN_3V3_2StateGet(); break;        
        case 8: *value = V_EN_3V3_AN2StateGet(); break;
        case 9: *value = V_EN_1V8_2StateGet(); break;                
        default: return false;
    }
    
    return true;
}


bool test_io_set(unsigned int index, bool value){
    switch( index ){
        case 1: V_PFC_STOP_12_NStateSet(value); break;
        case 2: V_PFC_STOP_34_NStateSet(value); break;
        
        case 3: V_EN_12VStateSet(value); break;
        case 4: V_EN_3V3_1StateSet(value); break;        
        case 5: V_EN_3V3_AN1StateSet(value); break;
        case 6: V_EN_1V8_1StateSet(value); break;        
        
        case 7: V_EN_3V3_2StateSet(value); break;        
        case 8: V_EN_3V3_AN2StateSet(value); break;
        case 9: V_EN_1V8_2StateSet(value); break;                
        
        default: return false;
    }
    
    return true;
}



void test_uart_parseGetIO(char* c){
    unsigned int index;
    if( sscanf(c, "%*c%u", &index) == 1){
        bool value;
        if( test_io_get(index, &value) ){
            u2_write("%gI%u=%u", TEST_START_CHAR, index, value, TEST_END_CHAR);
        }
    }
}

void test_uart_parseSetIO(char* c){
    unsigned int index;
    char value;
        
    if( sscanf(c, "%*c%u=%c", &index, &value) == 2){
        bool bval = (value != '0');
        if( test_io_set(index, bval) ){
            test_io_get(index, &bval);
            u2_write("%sI%u=%u", TEST_START_CHAR, index, bval, TEST_END_CHAR);
        }
    }
}




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
    unsigned int index;
    if( sscanf(c, "%*c%u", &index) == 1){
        uint16_t adcVal;
        float convVal;
        if( test_adc_get(index, &adcVal, &convVal) ){
            u2_write("%cA%u=%u;%f", TEST_START_CHAR, index, adcVal, convVal, TEST_END_CHAR);
        }
    }
}

void test_uart_parseGetPWM(char* c){
    unsigned int channel;
    char type;
    
    
    if( sscanf(c, "%c%u", &type, &channel ) == 2){        
        uint16_t dc, phase;

        test_pwm_Get( channel, type, &dc, &phase);
        u2_write("%cg%c%uP%uD%u%c", TEST_START_CHAR, type, channel, phase, dc, TEST_END_CHAR);
    }
}

void test_uart_parseGetParameter(char* c){
    unsigned int param, value;
    
    if( sscanf(c, "%*c%u", &param) == 1 ){
        test_param_get(param, (uint16_t*) &value);
        u2_write("%cgP%u=%u%c", TEST_START_CHAR, param, value, TEST_END_CHAR);
    }
}

void test_uart_parsGet(char* c){
    switch( *c ){
        case 'b': 
        case 'B':
            test_uart_parseGetPWM(c);
            break;
            
        case 'p':
        case 'P':
            test_uart_parseGetParameter(c);
            break;
            
        case 'i':
        case 'I':
            test_uart_parseGetIO(c);
            break;
            
        default:
            break;
    }
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
            
        case 'i':
        case 'I':
            test_uart_parseSetIO(c);
        default:
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
            test_uart_parseAdc(c);
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
                protoState = ProtoFindEnd;
            }
            break;
            
        case ProtoFindEnd:
            cmd[cmd_pos++] = c;
            
            if( (c == TEST_END_CHAR) ){
                protoState = ProtoFindStart;
                return true;
            }else if( c == TEST_START_CHAR ){
                cmd_pos = 0;
                cmd[cmd_pos++] = c;
            }else if( (cmd_pos >= TEST_CMD_LEN) ){
                //Too long, abort
                protoState = ProtoFindStart;
                return false;
            }
            break;
        default:
            break;
    }
    return false;
}

