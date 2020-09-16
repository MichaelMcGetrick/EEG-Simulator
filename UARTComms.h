/*
 * UARTCOmms.h
 *
 *  First created on: Aug 21, 2018
 *      AUTHOR: Michael McGetrick
 *      DESCRIPTION: Provide communication for comment/debug statements between AVR M/C and PC
 *      This code is specific to the AVR M/C ATmega328P
 *      For direct communication between M/C and PC, use the functions
 *      USART_Receive()
 *      USART_Transmit()
 *      for sending single bytes (char)
 *      For sending an entire string use the function
 *      debug()
 *      For the purposes of dealing with formatted strings, use the functions
 *      printf()
 *      scanf()
 *      We need to find a stream pathway for these standard i/o functions
 *      We use
 *      FDEV_SETUP_STREAM
 *      to map these standard functions to uart_putchar and uart_getchar
 *      and configure this to the standard stream.
 *      Then  we can implement printf and scanf functions (which allow for simple string/integer
 *      format combinations) - these functions will call up the relevant uart_*** functions allowing
 *      for formatted data on the TX/RX of the UART.
 *
 *      NB: Above formatting does not provide for floating point. This needs to be handled differently -
 *      special library and linker options need to be set for this.
 *
 *      Using printf() with format specifiers:
 *      Ensure that the specifier matches the variable type:
 *      e.g.
 *      uint32_t (unsigned long int) => use "%lu"     (32 bit is long)
 *      uint8_t (unsigned int)       => use "u"       (8 bits)
 *      etc......
 *
 */

#ifndef UARTCOMMS_H_
#define UARTCOMMS_H_

#include <avr/io.h>
#include <stdio.h>

//FUNCTION DECLARATIONS HERE -----------------------------------------------------------------------
void ioinit(void);      // initializes IO
int uart_putchar(char c, FILE *stream);
int uart_getchar(FILE *stream);
uint8_t USART_Receive( void );
void USART_Transmit( int8_t data );
void debug(char *str_data);
void uart_sendStr(char *str_data);
void printf_16bit(uint16_t val);
//--------------------------------------------------------------------------------------------------



//ATTRIBUTES ---------------------------------------------------------------------------------------
extern uint32_t BAUD;
extern uint32_t MYUBRR;
extern FILE usart_str;


#endif /* UARTCOMMS_H_ */
