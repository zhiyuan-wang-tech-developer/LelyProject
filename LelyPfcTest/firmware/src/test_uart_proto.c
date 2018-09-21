#include "test_uart.h"
#include "test_pwm.h"
#include "test_params.h"
#include "test_adc.h"

#include <stdio.h>

#define TEST_START_CHAR '['
#define TEST_END_CHAR   ']'

#define TEST_CMD_LEN 64
char cmd[TEST_CMD_LEN];
uint16_t cmd_pos = 0;


static enum {
    ProtoFindStart,
    ProtoFindEnd
} protoState = ProtoFindStart;




bool test_io_get(unsigned int index, bool* pvalue){
    switch( index ){
        case 1: *pvalue = V_PFC_OL12StateGet(); break;
        case 2: *pvalue = V_PFC_OL34StateGet(); break;
        
        case 3: *pvalue = V_EN_12VStateGet(); break;
        case 4: *pvalue = V_EN_3V3_1StateGet(); break;        
        case 5: *pvalue = V_EN_3V3_AN1StateGet(); break;
        case 6: *pvalue = V_EN_1V8_1StateGet(); break;        
        
        case 7: *pvalue = V_EN_3V3_2StateGet(); break;        
        case 8: *pvalue = V_EN_3V3_AN2StateGet(); break;
        case 9: *pvalue = V_EN_1V8_2StateGet(); break;                
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
    // Ignore the first character 'I'
    if( sscanf(c, "%*c%u", &index) == 1){
        bool value;
        if( test_io_get(index, &value) ){
            // After get IO, the Lely response is: [gIx=x] 
//            test_uart_write("%cgI%u=%u%c", TEST_START_CHAR, index, value, TEST_END_CHAR);
            printf("%cgI%u=%u%c", TEST_START_CHAR, index, value, TEST_END_CHAR);
        }
    }
}

void test_uart_parseSetIO(char* c){
    unsigned int index;
    char value;
    // * means that the first char is ignored and not put into the corresponding parameter.    
    if( sscanf(c, "%*c%u=%c", &index, &value) == 2){
        bool bval = (value != '0');
        if( test_io_set(index, bval) ){
            test_io_get(index, &bval);
            // After set IO, the Lely response is: [sIx=x] 
//            test_uart_write("%csI%u=%u%c", TEST_START_CHAR, index, bval, TEST_END_CHAR);
            printf("%csI%u=%u%c", TEST_START_CHAR, index, bval, TEST_END_CHAR);
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
                // set boost duty cycle command: [sbxDx]
                test_pwm_SetBoostDC(channel, value, true);
                set = true;
            }
            else if( param == 'P' || param == 'p' ){
                // set boost phase command: [sbxPx]
                test_pwm_SetBoostPhase(channel, value, true);  
                set = true;
            }
        }else if(type == 'B'){
            if( param == 'D' || param == 'd' ){
                // set buck duty cycle command: [sBxDx]
                test_pwm_SetBuckDC(channel, value, true);
                set = true;
            }
            else if( param == 'P' || param == 'p' ){
                // set buck phase command: [sBxPx]
                test_pwm_SetBuckPhase(channel, value, true);
                set = true;
            }
        }
        
        if( set ){
            //PWM set, send current status as response
            uint16_t dc, phase;
            
            test_pwm_Get( channel, type, &dc, &phase);
            // After set PWM, the Lely response is: [sbxDxPx] or [sBxDxPx] 
            // Respond with duty cycle first and phase second so that it conforms to my PC debugger. 
//            test_uart_write("%cs%c%uD%uP%u%c", TEST_START_CHAR, type, channel, dc, phase, TEST_END_CHAR);
            printf("%cs%c%uD%uP%u%c", TEST_START_CHAR, type, channel, dc, phase, TEST_END_CHAR);
        }
    }
}



void test_uart_parseSetParameter(char* c){
    unsigned int param, value;
    // %*c here * means that the char data is ignored and not put into the corresponding parameter.
    if( sscanf(c, "%*c%u=%u", &param, &value) == 2 ){
        if( test_param_set(param, value) ){
            test_param_get(param, (uint16_t*) &value);
            // After set Parameter, the Lely response is: [sPx=x]
//            test_uart_write("%csP%u=%u%c", TEST_START_CHAR, param, value, TEST_END_CHAR);
            printf("%csP%u=%u%c", TEST_START_CHAR, param, value, TEST_END_CHAR);
        }
    }
}


void test_uart_parseGetAdc(char* c){
    unsigned int index;
    // Ignore the first character 'A'
    if( sscanf(c, "%*c%u", &index) == 1){
        uint16_t adcVal;
        float convVal;
        if( test_adc_get(index, &adcVal, &convVal) ){
            // After get ADC, the Lely response is: [gAx=x;x]
//            test_uart_write("%cgA%u=%u;%f%c", TEST_START_CHAR, index, adcVal, convVal, TEST_END_CHAR);
            printf("%cgA%u=%u;%.2f%c", TEST_START_CHAR, index, adcVal, convVal, TEST_END_CHAR);
        }
    }
}

void test_uart_parseGetPWM(char* c){
    unsigned int channel;
    char type;
    
    
    if( sscanf(c, "%c%u", &type, &channel ) == 2){        
        uint16_t dc, phase;

        test_pwm_Get( channel, type, &dc, &phase);
        // After get PWM, the Lely response is: [gbxDxPx] or [gBxDxPx]
//        test_uart_write("%cg%c%uD%uP%u%c", TEST_START_CHAR, type, channel, dc, phase, TEST_END_CHAR);
        printf("%cg%c%uD%uP%u%c", TEST_START_CHAR, type, channel, dc, phase, TEST_END_CHAR);
    }
}

void test_uart_parseGetParameter(char* c){
    unsigned int param, value;
    // Ignore first character 'P'
    if( sscanf(c, "%*c%u", &param) == 1 ){
        test_param_get(param, (uint16_t*) &value);
        // After get Parameter, the Lely response is: [gPx=x]
//        test_uart_write("%cgP%u=%u%c", TEST_START_CHAR, param, value, TEST_END_CHAR);
        printf("%cgP%u=%u%c", TEST_START_CHAR, param, value, TEST_END_CHAR);
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
                                
        case 'a':
        case 'A':
            test_uart_parseGetAdc(c);
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
    if( *c == TEST_START_CHAR ){
        c++;
    }
    
    switch( *c ){
        case 's':
        case 'S':
            // skip start char
            test_uart_parsSet(++c);
            break;
        case 'g':
        case 'G':
            // skip start char
            test_uart_parsGet(++c);
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
                // If a complete command is found, it returns ture. 
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

void test_uart_processCommands(const char* str, bool reset){
    if( reset ){
        cmd_pos = 0;
        protoState = ProtoFindStart;
    }
    // check if the character is NULL char '\0', whose ASCII value is 0.
    while( *str ){
        if( test_uart_findCommand(*str++) ){
            // The command is found, parse it.
            test_uart_parseCommand(cmd);
        }
    }
}
    