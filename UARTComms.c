/*
 * UARTComms.c
 *
 *  First created on: Aug 21, 2018
 *      AUTHOR: Michael McGetrick
 *      DECRIPTION: See debug.h
 */


#include "UARTComms.h"

//ATTRIBUTES  --------------------------------------------------------------------------------------
uint32_t BAUD = 115200;    
uint32_t MYUBRR = 0;  //Initialised value


//Define a stream to be redirected to standard input/output (so as to use formatted function printf()
//Output stream: printf()  is mapped to uart_putchar()
//Input stream:  scanf() is mapped to uart_getchar()
//static FILE usart_str = FDEV_SETUP_STREAM(uart_putchar, uart_getchar, _FDEV_SETUP_RW);
FILE usart_str = FDEV_SETUP_STREAM(uart_putchar, uart_getchar, _FDEV_SETUP_RW);

    

//FUNCTION DEFINITIONS HERE: -----------------------------------------------------------------------

void ioinit()
{
	//Set up the UART Registers:
	//Override define definition to avoid rounding errors (See datasheet)

	if(BAUD == 4800)
	{
		MYUBRR = 207;
	}
	if(BAUD == 9600)
	{
		MYUBRR = 103;
	}
	if(BAUD == 14400)
	{
		MYUBRR = 68;
	}
	if(BAUD == 19200)
	{
		MYUBRR = 51;
	}
	if(BAUD == 38400)
	{
		MYUBRR = 25;
	}
	if(BAUD == 57600)
	{
		MYUBRR = 16;
	}
	if(BAUD == 115200)
	{
		MYUBRR = 8;
	}
	if(BAUD == 230400)
	{
		MYUBRR = 3;
	}


	UBRR0H = MYUBRR >> 8;  //(Essentially, sets high bits to zero)
	UBRR0L =  MYUBRR;
	UCSR0B = (1<<RXEN0)|(1<<TXEN0); //Sets the required bits to enable the UART Tx and Rx

	UCSR0A = (1<<UDRE0); //Set flag to empty data register - then ready to accept new data

	UCSR0C =  (1 << UCSZ01) | (1 << UCSZ00);    // Set character size to 8 bit,
												//NB:1 stop bit (USBS0 bit)  0 by default
												//Parity bit (UPM0/OPM1) zero by default

	//For standard comms (e.g. settings in Putty)
	//Baud rate:  9600
	//Bits: 8
	//Stop: 1
	//Parity: None

	stdout = &usart_str; //Required for printf() init
	stdin = &usart_str;  //Required for scanf() init

}//ioinit


 
int uart_putchar(char c, FILE *stream) {

    //REGISTER DESCRIPTIONS
    //UDR0 is the data register for current data (either receive or transmit) - see Datasheet
    //If written here, it will then be transferred to transfer Data register TXB
    //UCSR0A - Control and data register
    //UDRE0 - flag (bit 5) of UCSR0A - if flagged to 1, then it means the data register isready for the
    //next value

    if (c == '\n')
    uart_putchar('\r', stream);
    loop_until_bit_is_set(UCSR0A, UDRE0);


    UDR0 = c;

    return 0;
}
 


int uart_getchar(FILE *stream)
{
    //REGISTER DESCRIPTIONS:
	//UDR0 is the data register for current data (either receive or transmit) - see Datasheet
	//UCSR0A - Control and data register
	// RXC0 flag (bit 7) is the flag to indicate that the data has been read (and buffer is now empty)

	while( !(UCSR0A & (1<<RXC0)) );

    return(UDR0);
}
 


//Send/Receive functions without use of standard input/output streams (for use with printf()
uint8_t USART_Receive( void )
{
    // Wait for data to be received 
    while ( !(UCSR0A & (1<<RXC0)) )
    ;
    // Get and return received data from buffer 
    return UDR0;
}


void USART_Transmit(int8_t data )
{
    // Wait for empty transmit buffer 
    while ( !( UCSR0A & (1<<UDRE0)) )
    ;
    // Put data into buffer, sends the data 
    UDR0 = data;
}


 
void debug(char *str_data)   //Send string to PC console
{
    while(*str_data)
    {
    	USART_Transmit(*str_data);
        str_data++;
    }
}



void uart_sendStr(char *str_data)   //Send string data (same as debug() )
{
    while(*str_data)
    {
    	USART_Transmit(*str_data);
        str_data++;
    }
}
 


//Use for sending 16 bit value via UART
void printf_16bit(uint16_t val)
{
	char valStr[10];
	sprintf(valStr," %u\r\n", val);
	printf(valStr);

}


//-----------------------------------------------------------------------------------------------------

