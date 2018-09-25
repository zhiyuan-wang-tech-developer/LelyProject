#include <stdio.h>

#include "test_uart.h"
#include "system/clk/sys_clk.h"
#include "system/debug/sys_debug.h"

//uint16_t uart2_pos = 0;
static char uart2_buffer[UART2_RX_BUF_LEN];

static FIFO_Buffer_t uart2_rx_fifo_buffer = { .inIndex = 0,
                                       .outIndex = 0,
                                       .countUsedCells = 0,
                                       .size = UART2_RX_BUF_LEN,
                                       .pFIFO = uart2_buffer
                                        };

/*
 * @Desciption 
 *      check if the RX FIFO buffer is empty
 * @Parameters 
 *      None
 * @Return 
 *      true: the RX FIFO buffer is empty
 *      false: the RX FIFO buffer has RX characters
 */
bool isRxFifoEmpty(void)
{
    if( uart2_rx_fifo_buffer.countUsedCells == 0 ) return true;
    else return false;
}

/*
 * @Desciption 
 *      check if the RX FIFO buffer is full
 * @Parameters 
 *      None
 * @Return 
 *      true: the RX FIFO buffer is full
 *      false: the RX FIFO buffer can store more characters
 */
bool isRxFifoFull(void)
{
    if( uart2_rx_fifo_buffer.countUsedCells == uart2_rx_fifo_buffer.size ) return true;
    else return false;
}

/*
 * @Desciption 
 *      push a character into the RX FIFO buffer
 * @Parameters 
 *      charIn: the character you want to push into the FIFO buffer
 * @Return 
 *      true: pushing a character into the RX FIFO buffer is successful
 *      false: can not push a character into the RX FIFO buffer
 */
bool RxFifoPush(char charIn)
{
    if(isRxFifoFull()){
        return false;
    }
    uart2_rx_fifo_buffer.pFIFO[uart2_rx_fifo_buffer.inIndex] = charIn;
    uart2_rx_fifo_buffer.countUsedCells++;
    uart2_rx_fifo_buffer.inIndex = (uart2_rx_fifo_buffer.inIndex + 1) % uart2_rx_fifo_buffer.size; // round FIFO buffer
    return true;
}

/*
 * @Desciption 
 *      pop a character from the RX FIFO buffer
 * @Parameters 
 *      pcharOut: pointer to a place where you store the character popped from the FIFO buffer
 * @Return 
 *      true: popping a character from the RX FIFO buffer is successful
 *      false: can not pop a character from the RX FIFO buffer
 */
bool RxFifoPop(char *pcharOut)
{
    if(isRxFifoEmpty()){
        return false;
    }
    if(pcharOut == NULL){
        return false;
    }
    *pcharOut = uart2_rx_fifo_buffer.pFIFO[uart2_rx_fifo_buffer.outIndex];
    uart2_rx_fifo_buffer.countUsedCells--;
    uart2_rx_fifo_buffer.outIndex = (uart2_rx_fifo_buffer.outIndex + 1) % uart2_rx_fifo_buffer.size; // round FIFO buffer
    return true;
}

void __ISR(_UART2_RX_VECTOR, ipl3AUTO) _IntHandlerUART2Rx(void){
    //Buffer data
    char rxChar = 0;
    while( U2STAbits.URXDA ){
//        uart2_buffer[uart2_pos++] = U2RXREG;
//        if( uart2_pos >= UART2_RX_BUF_LEN ) uart2_pos = 0;
        rxChar = U2RXREG;
        if(RxFifoPush(rxChar)) continue;
        else break;
    }
    //Clear flag
    IFS1bits.U2RXIF = 0;
}


int test_uart_write(char* fmt, ...){
    
    va_list vl;
    va_start(vl, fmt);

    char buf[256] = {0};
    // It transmits many null characters... resulting in heavy transmission overhead. 
    int nwrite = vsprintf(buf, fmt, vl);
    int i = 0;
    
    for( i = 0; i < nwrite; i++ ){
        while( U2STAbits.UTXBF );   //wait for buffer not full
        U2TXREG = buf[i];
    }
    
    va_end(vl);
    
    while( !U2STAbits.TRMT );
    return nwrite;
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

