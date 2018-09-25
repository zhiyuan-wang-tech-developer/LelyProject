/* 
 * File:   test_uart.h
 * Author: thijs
 *
 * Created on 4 september 2018, 11:01
 */

#ifndef TEST_UART_H
#define	TEST_UART_H

#ifdef	__cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdarg.h>
#include <stdint.h>

#include <stdlib.h>
#include <string.h>
    
#define UART2_RX_BUF_LEN  128
    
typedef struct {
    uint16_t inIndex;
    uint16_t outIndex;
    uint16_t countUsedCells;
    uint16_t size;
    char * pFIFO;
} FIFO_Buffer_t;    

bool isRxFifoEmpty(void);
bool isRxFifoFull(void);
bool RxFifoPush(char charIn);
bool RxFifoPop(char *pcharOut);

void extractCmdFromRxFifo(void);

bool test_uart_findCommand(char c);
void test_uart_parseCommand(char* c);
void test_uart_processCommands(const char* str, bool reset);

void test_uart_init();
int test_uart_write(char* fmt, ...);


#ifdef	__cplusplus
}
#endif

#endif	/* TEST_UART_H */

