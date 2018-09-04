#include "test_uart.h"
#include "system/clk/sys_clk.h"

#define U2_BUF_LEN  128

uint16_t u2_pos = 0;
uint8_t u2_buf[U2_BUF_LEN];



void __ISR(_UART2_RX_VECTOR, ipl3AUTO) _IntHandlerUART2Rx(void){
    //Buffer data
    while( U2STAbits.URXDA ){
        u2_buf[u2_pos++] = U2RXREG;
    }
    
    //Clear flag
    IFS1bits.U2RXIF = 0;
}

void test_uart_init(){
    
    //Reset registers
    U2MODE = 0;
    U2STA = 0;
    
    float baud = 9600;
    
    U2MODEbits.BRGH = 1;        //High speed baud rate generator
        
    U2BRG = SYS_CLK_PeripheralFrequencyGet(CLK_BUS_PERIPHERAL_2) / (baud * (U2MODEbits.BRGH ? 4 : 16) ) -1;
    
    
     U2MODEbits.UARTEN = 1;     //Enable UART
       
     U2STAbits.URXEN = 1;       //Enable receiver
     U2STAbits.UTXEN = 1;       //Enable transmitter

     IEC1bits.U2RXIE = 1;       //Enable Rx interrupt
}