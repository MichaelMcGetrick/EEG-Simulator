/*
 * MAIN.C
 *
 *	AUTHOR: 	Michael McGetrick
 *	CREATED:	June 26, 2020
 *
 *  DESCRIPTION:
 *  This is a project to simulate an EEG signal.
 *
 *  Part One
 *  --------
 *  The signal will be generated from a signal generator with an amplitude
 *  between 0-5V. (No need for amplification at this stage).
 *  The h/w will have the following characteristics:
 *  i) Analog signal from generator to be fed to an ADC chip (MCP3008).
 *     Although the Atmega328p has analog inputs, we will use the external
 *     ADC chip so as to keep development as close as possible to Raspberry
 *     Pi development (the Raspberry does not have analog GPIO, and so we
 *     will need external ADC).
 *  ii) The external ADC has an SPI interface - we will use this to receive the
 *      digital value from the signal generator/ADC
 *  iii) We will then sample the signal at an appropriate sampling frequency
 *      through the SPI interface.
 *
 *  NB: Ensure we connect signal generator ground to breadboard ground - otherwise
 *      we do not get correct signal.
 *
 * Part Two
 * --------
 * Develop FFT code to get the frequency distribution of the signal.
 * We will develop this as a separate C module (which we can also use for the
 * Raspberry Pi/Linux system.
 *
 * Part Three
 * ----------
 * Add the amplification and filtering components needed for the genuine
 * EEG signal. Initially, we can use the signal generator to provide mV
 * (or micro-V) signals, if possible
 *
 *  Miscellaneous
 *  -------------
 *  Add facility to perform auto testing of the frequency response of the
 *  HPF (high pass filter) sub-circuit. The results will be sent by UART
 *  to  external processing for processing and creating a frequency response graphics
 *
 *  Use of floating point numbers:
 *  We need to add the following linker options:
 *  General
 *  -u, vfprintf,  (ensure last comma is there)
 *   Libraries: Add
 *   printf_flt
 *
 *   Code compiles and links, but still does not output float with printf()
 *   Looking at the size of the program, there is no difference when the float library
 *   is omitted. Clearly, the linker is not linking it - For further investigation
 *
 *
 * PIN CONFIGURATION:
 * -----------------
 *	SPI connections:
 *	SPI PIN		Atmega328p #		Arduino #				MCP3008
 *	-------		------------		---------				-------
 *	SS			PB2					10
 *	MOSI		PB3					11
 *	MISO		PB4					12
 *	SCK			PB5					13
 *
 *
 *
 *
 *
 *
 *---------------------------------------------------------------------------------------------------------*/


#include <avr/io.h>
#include <stdio.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>			//NB: Need to add m library in Library section


//#include "debug.h"

//DEFINES: ------------------------
#define BLINK_DELAY_MS 100
//UART
#define FOSC 16000000
//#define BAUD 9600
uint32_t BAUD = 115200; //9600
//#define MYUBRR FOSC/16/BAUD-1   //Needed to define the UBRR (UART Baud rate) register
uint32_t MYUBRR;



//TIMER - DATA SAMPLING
//NB: If SAMPLE_RATE Set to 0, use SECS_PER_SAMPLE for Timer configuration
//	 For 1 sample/sec, use SAMPLE_RATE = 0; SECS_PER_SAMPLE = 1
#define SAMPLE_RATE			1000 //1000   //(samples per sec)
#define SECS_PER_SAMPLE		1
#define PRESCALER			8  //64  //1024: Sample rate = 0; Others: USe Timer1_Calcs.xls table
//Sample buffer
#define BYTES_PER_SAMPLE	2	//For MSB, LSB and control flags
#define SAMPLE_BUF_LEN		500  //Data buffer to collect samples before UART transmission


//SPI
#define SPI_SS_GPIO PB2
#define SPI_SS_PORT PORTB
#define SPI_SS_DDR DDRB

#define SPI_MOSI_GPIO PB3
#define SPI_MOSI_PORT PORTB
#define SPI_MOSI_DDR DDRB


#define SPI_MISO_GPIO PB4
#define SPI_MISO_PORT PORTB
#define SPI_MISO_DDR DDRB

#define SPI_SCK_GPIO PB5
#define SPI_SCK_PORT PORTB
#define SPI_SCK_DDR DDRB

#define SPI_SLAVE_SELECTED SPI_SS_PORT &= ~(1 << SPI_SS_GPIO)
#define SPI_SLAVE_DESELECTED SPI_SS_PORT |= (1 << SPI_SS_GPIO)


//SPI Comms configuration:
#define START_BIT	1
//MPC3008
#define RD_MODE 	  1   //Single:1; Differential: 0
#define INPUT_CHAN    0 //7   //Analog input channel index
#define ARRAY_LEN	  3	//Array length for incoming data
#define SPI_CTRL_CFG (RD_MODE << 7) | (INPUT_CHAN << 4)

//FREQUENCY RESPONSE
#define FREQ_RESP_PRESCALER	8


//FUNCTION DECLARATIONS: --------------------------
//UART:
void ioinit(void);
void USART_Transmit( int8_t data );
void debug(char *str_data);
static int uart_putchar(char c, FILE *stream);
uint8_t uart_getchar(void);


//Timer functions for data sampling (16 bit timer)
void timer1_init(void);
void start_timer1(void);
void stop_timer1(void);

//Timer functions for generating square wave signals
//(8 bit timer)
void timer0_init(void);
void start_timer0(void);
void stop_timer0(void);
volatile uint32_t timer0_cnt, timer0_comp;
//uint32_t CURR_FREQ;
float CURR_FREQ;
bool sig_level_high;
bool freq_response;
uint32_t fr_sampleRate, fr_secsPerSam;
uint32_t fr_prescaler = 8;



//SPI
void SPI_MasterInit(void);
void SPI_Transmit( int8_t data );
void SPI_ReadMPC3008();
void simulation(void);
void readSimulation(void);

//Analogue Pin prototypes
void adc_init(void);
uint16_t readAnalogIn(uint8_t pin);


//Data Processing:
int8_t checkByte(int8_t byte, int8_t n);
int16_t getVal(int8_t msb,int8_t lsb);


//Diagnostics:
void doBaudTest(void);
void freqResponse(void);


static FILE usart_str = FDEV_SETUP_STREAM(uart_putchar, uart_getchar, _FDEV_SETUP_RW);
//-------------------------------------------------

//Define flag ('s': send substitute; 'n': send unchanged)
char cf_msb, cf_lsb;

//SPI ATTRIBUTES:
uint8_t SPI_MSB, SPI_LSB;
uint8_t SPIData[ARRAY_LEN];
//uint8_t dataBuf[BYTES_PER_SAMPLE*SAMPLE_BUF_LEN]; //Accommodate 4 bytes per sample
int sampleCnt = 0;
int byteCnt = 0;
char data[5];
uint16_t dataBuf[SAMPLE_BUF_LEN];

int32_t cnt;
bool m_bSimulation	= false;
char* AnaInpDev = "MPC3008"; //MPC3008;ADC PIN


volatile bool sample = false;
volatile bool chg_val = false;
volatile int timer_cnt = 0;

//Time diagnostics
long t_s, t;



int main (void)
{

	/* set pin 7 of PORTD for output*/
	 DDRD |= (1 << DDD7);


	 //Enable global interrupts:
 	 sei();


	 //Set up for UART communications:
	 ioinit(); //Setup IO pins and defaults

	 bool baudtest = false;
	 freq_response = false;
	 //Test for Baud rate ----------------------------------------------------
	 if(baudtest)
	 {
		 doBaudTest();

	 }//if(baudtest)
	 //Do frequency response test---------------------------------------------
	 	 if(freq_response)
	 	 {
	 		 freqResponse();

	 	 }//if(baudtest)

     //------------------------------------------------------------------------

	 if(m_bSimulation)
	 {
		 simulation();

	 }//if(m_bSimulation)
	 else
	 {

         //Normal sampling for transfer to external PC via UART
		 if( strcmp(AnaInpDev,"ADCPIN") == 0)
		 {
			 //Initialise ADC pins:
			 adc_init();
			 //printf("Setting up for Arduino ADC input\n");
		 }
		 if( strcmp(AnaInpDev,"MPC3008") == 0)
		 {
			 //Set up for SPI communications
			 SPI_MasterInit();
			 //printf("Setting up for MPC3008 input\n");
		 }



		 //Initialise databuffer
		 /*
		 for(int i=0;i<SAMPLE_BUF_LEN;i++){
			 dataBuf[i] = 0;
		 }
		 */

		 // set pin 7 high to turn led on
		 PORTD |= (1 << PORTD7);


		 //Initialise timer:
		 timer1_init();


		 cnt = 0;


		 //printf("Starting timer..\n");
		 start_timer1();
		 while(1)
		 {

			 if(!sample)
			 {
				 //Wait until flagged to sample
				 //printf("Waiting for flag change\n");
			 }
			 else
			 {
				 // set pin 7 high to turn led on
				 //PORTD |= (1 << PORTD7);

				 sample = false;
				 stop_timer1();
				 //printf("Have caught flag change\n");

				 //Read value:

				 SPI_ReadMPC3008();


				 //dataBuf[sampleCnt] = getVal(SPI_MSB,SPI_LSB);
				 //dataBuf[sampleCnt] = readAnalogIn(0);


				 itoa(getVal(SPI_MSB,SPI_LSB),data,10);
				 printf("%s\r",data); //Need /r for line feed for serial port program

				 /*
				 if(sampleCnt == SAMPLE_BUF_LEN -1)
				 {
					 sampleCnt = 0;
					 byteCnt = 0;
					 while(sampleCnt < SAMPLE_BUF_LEN)
					 {


						 //itoa(dataBuf[sampleCnt],data,10);
						 //printf("%s\r",data); //Need /r for line feed for serial port program

						 sampleCnt = sampleCnt + 1;
					 }//while


					 sampleCnt = 0;
					 byteCnt = 0;
					 //printf("Transmission completed!\n");
					 break;


				 }
				 else
				 {
					 sampleCnt = sampleCnt + 1;
					 //Start timer:
					start_timer1();


				 }
				*/
				 //Start timer:
				start_timer1();

			 }


		 }//while

	 }//if(!m_bSimulation

     //printf("Am out of while - just before program end!\n");
	 return 0;

}//main


void simulation(void)
{

	 // set pin 7 high to turn led on
	 PORTD |= (1 << PORTD7);


	 //Initialise timer:
	 timer1_init();


	 cnt = 0;
	 start_timer1();

	 while(1)
	 {

		 if(!sample)
		 {
			 //Wait until flagged to sample
			 //printf("Waiting for flag change\n");
		 }
		 else
		 {
			 // set pin 7 high to turn led on
			 PORTD |= (1 << PORTD7);

			 sample = false;
			 stop_timer1();
			 //printf("Have caught flag change\n");

			 //Read value:
			 readSimulation();

			 //Check bytes:
			 cf_msb = 'n';
			 cf_lsb = 'n';
			 SPI_MSB = checkByte(SPI_MSB,0);
			 SPI_LSB = checkByte(SPI_LSB,1);

			 //uint16_t volts =  getVal(SPI_MSB,SPI_LSB);
			 //printf("cf_msb: %c cf_lsb: %c Voltage: %d\n", cf_msb,cf_lsb,volts);


			 USART_Transmit(cf_msb);
			 USART_Transmit(SPI_MSB);
			 USART_Transmit(cf_lsb);
			 USART_Transmit(SPI_LSB);
			 USART_Transmit('\n');


			 // set pin 7 low to turn led off
			 PORTD &= ~(1 << PORTD7);

			 //Start timer:
			start_timer1();

		 }


	 }//while



}//simulation


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

}

void USART_Transmit(int8_t data )
{
    /* Wait for empty transmit buffer */
    while ( !( UCSR0A & (1<<UDRE0)) )
    ;
    /* Put data into buffer, sends the data */
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

static int uart_putchar(char c, FILE *stream) {

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

uint8_t uart_getchar(void)
{
    //REGISTER DESCRIPTIONS:
	//UDR0 is the data register for current data (either receive or transmit) - see Datasheet
	//UCSR0A - Control and data register
	// RXC0 flag (bit 7) is the flag to indicate that the data has been read (and buffer is now empty)

	while( !(UCSR0A & (1<<RXC0)) );

    return(UDR0);
}



//TIMER FUNCTIONS -----------------------------------------------
void timer1_init()   //16 bit timer
{

	//Configure timer:
	TCCR1B |= (1 << WGM12) ;  //CTC mode
	//Set timer interrupt:
	TIMSK1 |= (1 << OCIE1A);

	//Define counter compare value:
	uint32_t val;
	if(!freq_response)   //Use in normal trace mode
	{
		if(SAMPLE_RATE == 0)
		{
			val = (uint32_t)FOSC/((uint32_t)PRESCALER);

		}
		else
		{

			val = (uint32_t)FOSC/((uint32_t)SAMPLE_RATE*(uint32_t)PRESCALER);

		}
	}
	else		//Use in frequency response mode
	{

		uint32_t sample_rate = 100.0*CURR_FREQ;
		fr_sampleRate = sample_rate;
		if(sample_rate < 40 && sample_rate >= 4)
		{
			fr_prescaler = 64;
			val = (uint32_t)FOSC/((uint32_t)sample_rate*(uint32_t)fr_prescaler);
		}
		if(sample_rate < 3)
		{
			fr_prescaler = 256;
			val = (uint32_t)FOSC/((uint32_t)sample_rate*(uint32_t)fr_prescaler);
		}
		else
		{
			//Use default value
			val = (uint32_t)FOSC/((uint32_t)sample_rate*(uint32_t)fr_prescaler);

		}


		if(sample_rate <= 1)
		{
			fr_sampleRate = 0;
			float sps = 1.0/(float) sample_rate;
			fr_secsPerSam = (int) sps;
		}
		//printf("val: %lu\n",val);


	}


	OCR1A = (uint16_t) val;






	//printf("The CTC value is: %d\n",OCR1A);



}//timer1_init


void start_timer1()
{

	//Start the timer:
	TCNT1= 0;

	//Set the prescaler
	if(!freq_response)
	{
		if(SAMPLE_RATE == 0)
		{
			TCCR1B |= (1 << CS12) | (1 << CS10) ;   //1024 pre-scale (slow timer clock down a bit)
		}
		else   //Set for SAMPLE_RATE > 1 Hz
		{
			if(PRESCALER == 1)
			{
				TCCR1B |= (1 << CS10) ;
			}
			if(PRESCALER == 8)
			{
				TCCR1B |= (1 << CS11);
			}
			if(PRESCALER == 64)
			{
				TCCR1B |= (1 << CS11) | (1 << CS10);
			}
			if(PRESCALER == 256)
			{
				TCCR1B |= (1 << CS12);
			}
			if(PRESCALER == 1024)
			{
				TCCR1B |= (1 << CS12) | (1 << CS10);
			}

		}
	}
	else   //Start timer in frequency response  mode
	{
		if(fr_prescaler == 8)
		{
			TCCR1B |= (1 << CS11);
		}
		if(fr_prescaler == 64)
		{
			TCCR1B |= (1 << CS11) | (1 << CS10);
		}
		if(fr_prescaler == 256)
		{
			TCCR1B |= (1 << CS12);
		}


	}
}//start_timer1


void stop_timer1()
{
	TCCR1B &= ~(1 << CS10);
	TCCR1B &= ~(1 << CS11);
	TCCR1B &= ~(1 << CS12);

}
//Handler for end of timer clock pulse period
ISR(TIMER1_COMPA_vect)
{
	//printf("In interrupt\n");

	if(!freq_response)
	{
		//Set flag to sample:
		if(SAMPLE_RATE == 0)
		{
			timer_cnt = timer_cnt + 1;
			if(timer_cnt == SECS_PER_SAMPLE)
			{
				timer_cnt = 0;
				sample = true;
			}

		}
		else
		{
			sample = true;
		}
	}
	else   //Process for frequency response analysis
	{
		//Set flag to sample:
		if(fr_sampleRate == 0)
		{
			timer_cnt = timer_cnt + 1;
			if(timer_cnt == fr_secsPerSam)
			{
				timer_cnt = 0;
				sample = true;
			}

		}
		else
		{
			sample = true;
		}

	}

}


void timer0_init()  //8 bit timer
{

	//Configure timer:
	//TCCR1B |= (1 << WGM12) ;  //CTC mode
	TCCR0A |= (1 << WGM01) ;  //CTC mode

	//Set timer interrupt:
	TIMSK0 |= (1 << OCIE0A);

	//Define counter compare value:
	OCR0A = 250;   //Gives integral multiples for all frequencies required

	timer0_cnt = 0;

	long TIMER_CLK = FOSC/FREQ_RESP_PRESCALER;
	timer0_comp = TIMER_CLK/2;
	//timer0_comp = timer0_comp/CURR_FREQ;

	float val = (float)timer0_comp/CURR_FREQ;
    timer0_comp = (long) val;


	//printf("time0_comp %lu\n",(long) timer0_comp);


}//timer0_init


void start_timer0()
{

	//Start the timer:
	TCNT0= 0;

	//Set the prescaler
	if(FREQ_RESP_PRESCALER == 8)
	{
		TCCR0B |= (1 << CS01);
	}
	if(FREQ_RESP_PRESCALER == 64)
	{
		TCCR0B |= (1 << CS01);
		TCCR0B |= (1 << CS00);
	}
	if(FREQ_RESP_PRESCALER == 256)
	{
		TCCR0B |= (1 << CS02);
	}
	if(FREQ_RESP_PRESCALER == 1024)
	{
		TCCR0B |= (1 << CS02);
		TCCR0B |= (1 << CS00);
	}


}//start_timer0


void stop_timer0()
{
	TCCR0B &= ~(1 << CS00);
	TCCR0B &= ~(1 << CS01);
	TCCR0B &= ~(1 << CS02);

}//stop_timer0

//Handler for end of timer clock pulse period
ISR(TIMER0_COMPA_vect)
{

	timer0_cnt = timer0_cnt + OCR0A;
	//printf("timer0_cnt: %d\n",timer0_cnt);

	if(timer0_cnt == timer0_comp)
	{
		chg_val = true;

	}

}


//END --TIMER FUNCTIONS ------------------------------------




int8_t checkByte(int8_t byte, int8_t n)
{
	/*
	NB: The C program SerialPort has been configured using termios structure.
	However, it is not disabling some of the ASCII control characters 0-31 and 127.
	The data is read erroneously for these values.
	As a temporary fix, we will replace these ascii values with the following mapping
	*/

	int chg = 0;

	switch(byte)
	{
		case 3:   //ETX
			chg = 1;
			byte = 'A';
			break;
		case 4:   //EOT
			chg = 1;
			byte = 'B';
			break;
		case 10:   //LF
			chg = 1;
			byte = 'C';
			break;
		case 13:   //CR
			chg = 1;
			byte = 'D';
			break;
		case 17:   //DC1

			chg = 1;
			byte = 'E';
			break;
		case 18:   //DC2
			chg = 1;
			byte = 'F';
			break;
		case 19:   //DC3
			chg = 1;
			byte = 'G';
			break;
		case 21:   //NAK
			chg = 1;
			byte = 'H';
			break;
		case 22:   //SYN
			chg = 1;
			byte = 'I';
			break;
		case 23:   //ETB
			chg = 1;
			byte = 'J';
			break;
		case 26:   //SUB
			chg = 1;
			byte = 'K';
			break;
		case 28:   //FS
			chg = 1;
			byte = 'L';
			break;
		case 127:   //FS
			chg = 1;
			byte = 'M';
			break;

	}//switch


	if(chg == 1)  //We have done substitution
	{
		if(n == 0)
		{
			cf_msb = 's';
		}
		if(n == 1)
		{
		  cf_lsb = 's';
		}
	}

	return byte;

}//checkByte

//SPI Functions:
void SPI_MasterInit()
{

	/*
	// Set MOSI and SCK output, all others input
	DDR_SPI = (1<<DD_MOSI)|(1<<DD_SCK);
	// Enable SPI, Master, set clock rate fck/16
	SPCR = (1<<SPE)|(1<<MSTR)|(1<<SPR0);
	*/


	SPCR = 0;  //Set all bits to zero
	//Enable SPI
	SPCR |= (1 << SPE);
	//Set to Master Mode
	SPCR |= (1 << MSTR);
	//Define Clock frequency
	//NB: MPC3008 has max.clock frequency of 3.6MHz for 5V supply (see datasheet)
	SPCR |= (1 << SPR0); //Prescaler fosc/16 (1MHz)
	//Define Clock Polarity and phase
	//Mode (0,0): CPOL =0; CPHA =0; (Already set to zero)
	// This mode used when CLK is 0 for idle

	//Set MOSI, SCK and SS to output, all others input
	SPI_MOSI_DDR |= (1 << SPI_MOSI_GPIO);
	SPI_SS_DDR |= (1 << SPI_SS_GPIO);
	SPI_SCK_DDR |= (1 << SPI_SCK_GPIO);

	//Ensure slave is unselected (high)
	SPI_SS_PORT |= (1 << SPI_SS_GPIO);
	//Set pullup on MISO (input by default)
	SPI_MISO_PORT |= (1 << SPI_MISO_GPIO);




}

void SPI_Transmit( int8_t data )
{
	//NB: As we send bits to slave, the slave sends bits back
	//    So, bit for bit, the SPI shift register gets redefined with
	//    data from slave. We need to ensure we send as many bits to ensure we
	//    get the actual data required (depends on the slave - see data sheet).


	SPDR = data;            //Start the transmission process

	//Wait until we get a response back
	while( !(SPSR & (1 << SPIF)))
	{
		//Wait for sent data
	}
	//Switch off light to show we got here
	//PORTD &= ~(1 << PORTD7);


}//SPI_Transmit

void SPI_ReadMPC3008()
{


	SPI_SLAVE_SELECTED;     //Select Slave (line goes to LOW)

	//Send START bit
	SPI_Transmit(START_BIT);
	SPIData[0] = SPDR;  //Receive first byte from Slave

	//Send control configuration data:
	SPI_Transmit(SPI_CTRL_CFG);
	SPIData[1] = SPDR;
	SPIData[1] &= 0x03;  //Only need the last two digits

	//Send miscellaneous data (to keep slave sending back bits)
	SPI_Transmit(0x00);
	SPIData[2] = SPDR;


	SPI_SLAVE_DESELECTED;     //De-select Slave (line goes HIGH)


	//Get the MSB and LSB:
	SPI_MSB = SPIData[1];
	//Handle to omit the spurious data
	SPI_LSB = SPIData[2];




}//SPI_Read


void adc_init(void)
{
	//Set data direction for PORT C:
	DDRC = 0b00000000; //Set PORT C to be inputs


	//Set up registers:
	ADMUX |= (1 << REFS0);  //Set reference voltage as AVREF (default supply voltage)
							//Analog input channel pin 0

	//Enable ADC:
	//ADCSRA = 0b10000111; //Use prescaler 128
	ADCSRA |= (1<< ADEN);  //Enable ADC
	//Set prescaler (8):
	ADCSRA |= (1<< ADPS0);
	ADCSRA |= (1<< ADPS1);
	//ADCSRA |= (1<< ADPS2);


	// Enable the ADC and set the prescaler to max value (128)
	//NB: Would not work without a precaler - presumably, clock going too fast to work


}


uint16_t readAnalogIn(uint8_t pin)
{

	 uint8_t low, high;

	 ADMUX |= (pin & 0x0f);  //Set bits for input pin (MUX[3.0])

	 ADCSRA |= (1<<ADSC);  //Start the conversion process
	 while (ADCSRA & (1<<ADSC))
	 {
	    //Wait for end of conversion (ADSC goes to 0 when conversion complete)

	 }

	 //Retrieve ADCL first (see datasheet)
	 low = ADCL;
	 high = ADCH;


	 // combine the two bytes
	 return (high << 8) | low;

}




void readSimulation()
{

	int mode = 1; //0: Raw count; 1: Sine wave

	int16_t mask = 0xff;

	if(mode == 0)
	{
		int16_t val1 = cnt;
		SPI_MSB =  val1 >> 8;
		//val_lsb =  (uint8_t) mask & val;
		SPI_LSB =  mask & cnt;
	}
	if(mode == 1)	//Generate Sine wave:
	{
		float T = 1.0;   //Period
		float A = 100.0;   //Amplitude
		int16_t DC_OFFSET = 100;
		int16_t val1 =(int16_t) A*sin(2*3.412*cnt/T) + DC_OFFSET;
		 val1 = 125; //temp override
		SPI_MSB =  val1 >> 8;
		SPI_LSB =  mask & val1;

	}



   if(cnt == 32767)
   {
	   cnt = 0;
   }
   else
   {
	 cnt = cnt + 1;
   }


}//readSimulation


int16_t getVal(int8_t msb,int8_t lsb)
{

	int16_t val_msb,val_lsb;
	val_msb = msb;
	val_msb = val_msb << 8;



	int i;
	for(i=0;i<8;i++)  //Mask lower byte with 1
	{
		val_msb |= (1 << i);
	}


	val_lsb = lsb;


	for(i=8;i<16;i++)  //Mask upper byte with 1
	{
		val_lsb |= (1 << i);
	}


	return  val_msb & val_lsb;



}//getVal

//DIAGNOSTIC UTILITIES -----------------------------------------
void doBaudTest(void)
{
	// set pin 7 high to turn led on
	 PORTD |= (1 << PORTD7);


	 printf("We are checking transmission with this Baud rate: %lu\n", (unsigned long)BAUD);


	 if(BAUD == 115200)
	 {
		printf("We have identified 115200 rate!\n");
		printf("MYUBRR: %lu\n",(unsigned long)MYUBRR);
	 }
	 if(BAUD == 9600)
	 {
		printf("We have identified 9600 rate!\n");
		printf("MYUBRR: %lu\n",(unsigned long)MYUBRR);
	 }
	 if(BAUD == 4800)
	 {
		printf("We have identified 4800 rate!\n");
		printf("MYUBRR: %lu\n",(unsigned long)MYUBRR);
	 }

	 char data[5];
	 for(uint16_t i=0;i<1024;i++)
	 {
		 itoa(i,data,10);
		 printf("%s\n",data);
	 }

	  _delay_ms(5000);
	 // set pin 7 low to turn led off
	 PORTD &= ~(1 << PORTD7);
	 
	 //Check libmathlibrary operational:
	 //int16_t val1 = pow(4,0.5);
	 //printf("Square root is:   %d\n", val1); 

	 printf("Exiting program...\n");
	 exit(0);

}//doBaudTest


void freqResponse(void)
{

	//GENERATE FREQUENCY RESPONSE DATA -----------------------------------
	// Instructions for use:
	//   i) Use Arduino ADC Pin 0 for reading sample values
	//  ii) Use Arduino digital pin 3 (PD3) for output of generated square wave
	// iii) Use GND bias to remove voltage divider bias to input signal
	//  iv) Define the time in seconds to keep generating a given signal
	//      (useful for inspecting with the DSO)
	//   v) Sampling frequencies (only for use for range 0.01Hz - 10000Hz!):
	//		- For all signal frequencies sampling freq = 100*signal freq
	//		- Use Prescaler 8 (this gives integral count values for 100-10000 sps)
	//      - may need to use another for lower frequencies
	//      - only use frequencies that provide integral counts for the prescaler values
	//      - The voltage value will be the average voltage of the square wave over
	//        integral number of cycles. For a square wave between 0-1024, this would
	//        be 512.
	//
	//NOTES: The sampling appears to degrade for signal frequencies > 50Hz
	//		 More cycles per data buffer appear than should - which is an indication
	//       of under-sampling.
	//       This probably due to the fact we are hammering the MCU with the two timers
	//       and getting near to the maximum ADC sampling rate.
	//       At this point, it might just be better to do the sampling without the
	//       timer and just do straight loop read. The ADC Pin should provide a maximum
	//       sampling rate (theoretically) o around
	//       150,000 sps with ADC prescaler 8
	//		 300,000 sps with prescaler 4
	//       However, it is not recommended to go to such low prescaler values
	//
	//       This should not really a problem for our project - we are only interested
	//       in brain wave frequencies/
	//       Also, the undersampling should not really affect the measurement of the
	//       voltage metric (average, rms or peak)
	//
	//---------------------------------------------------------------------

	uint16_t Vref;   //Highest value out of digital PIN 3;


	//Define test frequencies:
	int NUM = 23;
	float SIG_FREQS[] =
	{
		0.01,0.05,0.1,0.2,0.4,0.5,0.8,
		1,2,4,5,8,10,
		20,40,50,80,100,
		200,400,500,800,1000
	};

	uint32_t Vavg[NUM];  //Average value of signal (averaging over all positive values)
	uint32_t samCnt[NUM];  //Sample count for each signal


	//Define time (in seconds) to generate each signal
	uint32_t NUM_SECS = 1*30;



	//printf("Setting up for frequency response auto-testing...\n");


	//Set up output port for test signal (square wave)
	// set pin 3 of PORTD for output
	 DDRD |= (1 << DDD3);


	 //Initialise ADC pins://Initialise ADC pin:
 	 adc_init();


 	 //PERFORM REFERENCE VOLTAGE CHECK: ----------------------------------------
	 // Get actual value of Pin3 output (to be used as reference to HPF output)
	 // set pin 3 high
	 PORTD |= (1 << PORTD3);
	 Vref = readAnalogIn(0);
	 //set Pin 3 low
	 PORTD &= ~(1 << PORTD3);
	  //-------------------------------------------------------------------------

 	 //NUM = 5; //temp
 	 //Send number of datasets, sample count and reference voltage
 	 char numStr[10],refStr[10];
 	 sprintf(numStr,"%d",NUM);
 	 sprintf(refStr,"%d",(int) Vref);

 	 //Send vi UART:
 	 printf("%s %s\r",numStr,refStr);


 	 for(int i=0;i<NUM;i++)
 	 {
 		 if(SIG_FREQS[i] == 0.01)   //Ensure we keep signal for entire cycle
 		 {
 			 NUM_SECS = 1*100;
 			//NUM_SECS = 1*30; //temp
 		 }
 		 else
 		 {
 			 NUM_SECS = 1*30;
 		 }

		 //Start sampling:
		  CURR_FREQ = SIG_FREQS[i]; //Initialise signal frequency
		  Vavg[i] = 0.0;

		  if(CURR_FREQ >= 1.0)
		  {
			  printf("Testing signal frequency %d Hz .......\r",(int)CURR_FREQ);
		  }
		  else
		  {
			  printf("Testing signal frequency of %d seconds per sample .......\r",(int)(1.0/CURR_FREQ));
		  }
		 //Initialise level:
		 PORTD |= (1 << PORTD3);
		 sig_level_high = true;

		 //Set up timer for signal generation
		 timer0_init();
		 start_timer0();
		 uint32_t cnt = 0;

		 //Set up sampling timer:
		 fr_prescaler = 8; //Initialise prescaler for freq response analysis
		 timer1_init();
		 start_timer1();
		 uint32_t sampleCnt = 0;

		 uint32_t max_cnt = NUM_SECS*CURR_FREQ*2; //NB: interrupt occurs every half cycle
		 while(1)
		 {

			if(chg_val)  //Change level of test square wave
			{
				stop_timer0();
				chg_val = false;
				//printf("timer0_cnt: %lu\n",timer0_cnt);
				//printf("Cnt: %d, changing level\n",cnt);


				timer0_cnt = 0;
				if(sig_level_high)
				{
					sig_level_high = false;
					PORTD &= ~(1 << PORTD3);
					//printf("Switching off\n");
				}
				else
				{
					sig_level_high = true;
					PORTD |= (1 << PORTD3);
					//printf("Switching on\n");
				}
				cnt = cnt + 1;
				if(cnt == max_cnt)
				{
					//Stop sampling timer and do re-initialisation (in case still active)
					stop_timer1();
					sample = false;
					sampleCnt = 0;

					break;
				}
				else
				{
					start_timer0();
				}


			}
			if(sample)   //Take a sample of present signal
			{
				stop_timer1();
				sample  = false;
				dataBuf[sampleCnt] = readAnalogIn(1);
				//printf("Have caught interrupt\n");

				Vavg[i] += dataBuf[sampleCnt];
				if(sampleCnt < SAMPLE_BUF_LEN - 1)
				{
					start_timer1();
					sampleCnt = sampleCnt + 1;
					//printf("Sample count: %d\n",sampleCnt);
				}
				else
				{


				}


			}
			//Save the sample  count:
			samCnt[i] = sampleCnt+1;

		 }//while

 	 }//for

	//Save to data array


	//At end of test push results to externals PC via UART

	char valStr[10],freqStr[10],cntStr[10];
	for(int i=0;i< NUM;i++)
 	{
 		//printf("%lu\r",Vavg[i]); //Need /r for line feed for serial port program

 		if(SIG_FREQS[i] < 1)
 		{
 			sprintf(valStr,"%lu",Vavg[i]);
 			sprintf(cntStr,"%d",(int) samCnt[i]);
 			sprintf(freqStr,"%d",(int)(1.0/SIG_FREQS[i]));
 			printf("%s %s %s %s\r","<1Hz",cntStr,freqStr,valStr);
 		}
 		else
 		{
 			sprintf(valStr,"%lu",Vavg[i]);
 			sprintf(cntStr,"%d",(int) samCnt[i]);
 			sprintf(freqStr,"%d",(int)SIG_FREQS[i]);
 			printf("%s %s %s %s\r",">1Hz",cntStr,freqStr,valStr);
 		}




 	}



 	//Reset test signal amplitude to zero:
 	PORTD |= (1 << PORTD3);
	printf("Exiting program!\n");

	exit(0);

}////freqResponse
