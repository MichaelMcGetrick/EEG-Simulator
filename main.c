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
 *	SCK		PB5					13
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
#include <time.h>
#include <stdbool.h>
#include <math.h>			//NB: Need to add m library in Library section


#include "UARTComms.h"
#include "Timer0.h"		
#include "Timer1.h"
#include "spi.h"
#include "i2c.h"


//DEFINES: ------------------------
#define BLINK_DELAY_MS 100

//Sample buffer
#define BYTES_PER_SAMPLE	2	//For MSB, LSB and control flags
#define SAMPLE_BUF_LEN		500  //Data buffer to collect samples before UART transmission


//SPI


//SPI Comms configuration:
#define START_BIT			1
//MPC3008
#define RD_MODE 	  		1   //Single:1; Differential: 0
#define INPUT_CHAN    	0 //7   //Analog input channel index
#define ARRAY_LEN	  		3	//Array length for incoming data
#define SPI_CTRL_CFG 	(RD_MODE << 7) | (INPUT_CHAN << 4)

#define SAMPLE_MODE		1  //0: Normal; 1: sampling metrics


//I2C RTC (DS3231) defines:
#define SLAVE_ADDR 0x68        
//RTC register addresses:
#define RTC_SECOND 0x00



//FUNCTION DECLARATIONS: --------------------------
void SPI_ReadMPC3008();
void simulation(void);
void readSimulation(void);

//Analogue Pin prototypes
void adc_init(void);
uint16_t readAnalogIn(uint8_t pin);


//Data Processing:
int8_t checkByte(int8_t byte, int8_t n);
uint16_t getVal(uint8_t msb,uint8_t lsb);


//Diagnostics:
void doBaudTest(void);
void freqResponse(void);
void doMetricTest_FastLoop(void);
void doMetricTest_Normal(void);
uint32_t getHMSTimestamp();


//-------------------------------------------------

//Define flag ('s': send substitute; 'n': send unchanged)
char cf_msb, cf_lsb;

//SPI ATTRIBUTES:
uint8_t SPI_MSB, SPI_LSB;
uint8_t SPIData[ARRAY_LEN];
int sampleCnt = 0;
int byteCnt = 0;
char data[5];
uint16_t dataBuf[SAMPLE_BUF_LEN];

int32_t cnt,maxCnt;
bool m_bSimulation	= false;
bool m_bSamplingMetrics = true;
bool m_bSamplingMetrics_Fastloop = false;
char* AnaInpDev = "MPC3008"; //MPC3008;ADC PIN

//Frequency response attributes:
float CURR_FREQ;
bool sig_level_high;

//Time diagnostics
long t_s, t;

//RTC attributes:
uint8_t digit1, digit2;
uint8_t mask = 0xf;
char retStr[5];



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
	 if(m_bSamplingMetrics_Fastloop)
	 {
		 //Initialise I2C for RTC
		 i2c_init();
		 
		 doMetricTest_FastLoop();

	 }//if
	 if(m_bSamplingMetrics)	
	 {
		 //Initialise I2C for RTC
		 i2c_init();
		 
		 //doMetricTest_Normal();
		 doMetricTest_Normal();

	 }//if
	 
	 
	 //Do frequency response test---------------------------------------------
	 	 if(freq_response)
	 	 {
	 		 _delay_ms(2000);
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


		 // set pin 7 high to turn led on
		 PORTD |= (1 << PORTD7);


		 //Initialise timer:
		 timer1_init(0.0); //Provide dummy signal frequency

		 cnt = 0;
		 maxCnt = 10;


		 //printf("Starting timer..\n");
		 //Get timestamp1
		 
		 //start_t = clock();
		 
		 
       
      
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
				 
				 sample = false;
				 stop_timer1();
				 //printf("Have caught flag change\n");

              
				 //Read value:
				 SPI_ReadMPC3008();


				 itoa(getVal(SPI_MSB,SPI_LSB),data,10);
				 printf("%s\r",data); //Need /r for line feed for serial port program

              			
              cnt++;				 
		  	   //Start timer:
     	 		 start_timer1();
     	 		 

			 }


		 }//while
		 //get timestamp 2
		 //Send data to UART
		 
		 		 		 

	 }//if(!m_bSimulation

     printf("Am out of while - just before program end!\n");
	 return 0;

}//main


void simulation(void)
{

	 // set pin 7 high to turn led on
	 PORTD |= (1 << PORTD7);


	 //Initialise timer:
	 timer1_init(0.0);  //Provide dummy frequency


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



}//SPI_ReadMCP3008


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


uint16_t getVal(uint8_t msb,uint8_t lsb)
{

	uint16_t val_msb = msb;
    uint16_t val_lsb = lsb;
	val_msb = val_msb << 8;
   
    return  val_msb | val_lsb;
	

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
	//	i) Use Arduino ADC Pin 0 for reading the ouput voltage for VRef   
	// ii) Use Arduino ADC Pin 1 for reading sample values
	//iii) Use Arduino digital pin 3 (PD3) for output of generated square wave
	// iv) Use GND bias to remove voltage divider bias to input signal
	//  v) Define the time in seconds to keep generating a given signal
	//      (useful for inspecting with the DSO)
	// vi) Sampling frequencies (only for use for range 0.01Hz - 10000Hz!):
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
	/*		
	int NUM_SAMPLES = 13;
	float SIG_FREQS[] =
	{
		0.01, 0.05, 0.1, 0.2, 0.4, 0.5, 0.8,
		1.0, 2.0, 4.0, 5.0, 8.0, 10.0
		
	};
	*/
	
	int NUM_SAMPLES = 10;  
	float SIG_FREQS[] =
	{
		20.0, 40.0, 50.0, 80.0, 100.0, 200.0,
		400.0, 500.0, 800.0, 1000.0
	};
	
		
	char freqFmtStr[NUM_SAMPLES][10];

	uint32_t Vavg[NUM_SAMPLES];  //Average value of signal (averaging over all positive values)
	uint32_t samCnt[NUM_SAMPLES];  //Sample count for each signal


	//Define time (in seconds) to generate each signal
	uint32_t NUM_SECS = 1*30;



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

 	 //Send number of datasets, sample count and reference voltage
 	 char numStr[10],refStr[10];
 	 sprintf(numStr,"%d",NUM_SAMPLES);
 	 sprintf(refStr,"%d",(int) Vref);
    

 	 //Send via UART:
 	 printf("%s %s\r",numStr,refStr);
 	 
 	 
 	 
 	 for(int i=0;i<NUM_SAMPLES;i++)
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
			  float num = CURR_FREQ;
			  int inum = (int) CURR_FREQ;
			  float fraction = num - (float) inum;
			 	  
			  sprintf(freqFmtStr[i],"%d%c%d", (int) inum,'.',(int) (fraction*100));
			  printf("Testing signal frequency %s Hz ....... \r",freqFmtStr[i]);
			  
		  }
		  else
		  {
			  float num = (1.0/CURR_FREQ);
			  int inum = (int) (1.0/CURR_FREQ);
			  float fraction = num - (float) inum;
			 
			  sprintf(freqFmtStr[i],"%d%c%d", (int) inum,'.',(int) (fraction*100));
			  printf("Testing signal frequency of %s seconds per sample ....... \r",freqFmtStr[i]);
			  
	     }
		  
		 //Initialise level:
		 PORTD |= (1 << PORTD3);
		 sig_level_high = true;

		 //Set up timer for signal generation
		 timer0_init(CURR_FREQ);
		 start_timer0();
		 uint32_t cnt = 0;

		 //Set up sampling timer:
		 fr_prescaler = 8; //Initialise prescaler for freq response analysis
		 timer1_init(CURR_FREQ);
		 start_timer1();
		 uint32_t sampleCnt = 0;

		 uint32_t max_cnt = NUM_SECS*CURR_FREQ*2; //NB: interrupt occurs every half cycle
		 while(1)
		 {

			if(chg_val)  //Change level of test square wave
			{
				stop_timer0();
				chg_val = false;
				
				timer0_cnt = 0;
				if(sig_level_high)
				{
					sig_level_high = false;
					PORTD &= ~(1 << PORTD3);
					
				}
				else
				{
					sig_level_high = true;
					PORTD |= (1 << PORTD3);
					
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
				
				Vavg[i] += dataBuf[sampleCnt];
				if(sampleCnt < SAMPLE_BUF_LEN - 1)
				{
					start_timer1();
					sampleCnt = sampleCnt + 1;
					
				}
				else
				{


				}


			}
			//Save the sample  count:
			samCnt[i] = sampleCnt+1;

		 }//while

 	 }//for

	

	//At end of test push results to externals PC via UART

	char valStr[10],cntStr[10];
	for(int i=0;i< NUM_SAMPLES;i++)
 	{
 		//printf("%lu\r",Vavg[i]); //Need /r for line feed for serial port program
		if(SIG_FREQS[i] < 1.0)
 		{
 			sprintf(valStr,"%lu",Vavg[i]);
 			sprintf(cntStr,"%d",(int) samCnt[i]);
 			//sprintf(freqStr,"%d",(int)(1.0/SIG_FREQS[i]));
 			printf("%s %s %s %s\r","<1Hz",cntStr,freqFmtStr[i],valStr);
 		}
 		else
 		{
 			sprintf(valStr,"%lu",Vavg[i]);
 			sprintf(cntStr,"%d",(int) samCnt[i]);
 			//sprintf(freqStr,"%d",(int)SIG_FREQS[i]);
 			printf("%s %s %s %s\r",">1Hz",cntStr,freqFmtStr[i],valStr);
 		}


 	}



 	//Reset test signal amplitude to zero:
 	PORTD |= (1 << PORTD3);
	printf("Exiting program!\n");

	exit(0);

}////freqResponse


void doMetricTest_Normal(void)
{

	 uint32_t ts_st,ts_end,time_diff;
	 uint32_t MAX_CNT;
	 

    printf("Doing NORMAL Sampling Metric Test ...\n");
    
	 //Set up for SPI communications
	 SPI_MasterInit();
	  

	int numSamples = 1; //16;
	float sample_rates[] = {1.0f,10.0f,50.0f,100.0f,200.0f,250.0f,
		                     500.0f,1000.0f,1250.0f,2000.0f,4000.0f, 
		                     5000.0f,8000.0f,10000.0f,16000.0f,20000.0f};
	//NB: Above sampling frequencies give integral counts for timer after prescaling 
   //    The correct prescaler value must be set for each sampling frequency
   
   //Start test:
	uint32_t TIME_INT = 60*3; //Time interval for sampling (secs) 
	MAX_CNT = sample_rates[15]*TIME_INT;
   cnt = 0;

	 // set pin 7 high to turn led on
	 PORTD |= (1 << PORTD7);


	 //Initialise timer:
	 timer1_init(0.0); //Provide dummy signal frequency
	 	 
   
	SPI_MasterInit();  //Need to do twice (check why?)
	
	
	 
	 ts_st = getHMSTimestamp();
	 start_timer1();
	 while(cnt < MAX_CNT)
	 {

       if(!sample)
		 {
			 //Wait until flagged to sample
			 //printf("Waiting for flag change\n");
		 }
		 else
		 {
			 
			 sample = false;
			 stop_timer1();
			 //printf("Have caught flag change\n");
			  
			 //Read value:
			 SPI_ReadMPC3008();

			 //itoa(getVal(SPI_MSB,SPI_LSB),data,10);
			 //printf("%s\n",data); //Need /r for line feed for serial port program
						
			  cnt++;				 
			//Start timer:
			 start_timer1();
			 
		 }

	 }//while
	 //get timestamp 2
	 stop_timer1();
	 ts_end = getHMSTimestamp();
	 
	 time_diff = ts_end - ts_st;
	
    printf("Time difference for loop (secs):  %lu\n",time_diff);
	 printf("Max count: %lu\n",MAX_CNT);
	 uint32_t eff_rate = (MAX_CNT/time_diff);
	 printf("Effective sampling rate:  %lu (Hz)\n",eff_rate) ; //to nearest 1Hz
	 
	
	//set pin 7 low to turn led off
   PORTD &= ~(1 << PORTD7);
	
	printf("Exiting NORMAL Sampling Metric test ...\n");

	exit(0);
    
    

}//doMetricTest_Normal


void doMetricTest_FastLoop(void)
{
	 uint32_t ts_st,ts_end,time_diff;
	 uint32_t MAX_CNT;
	 
	 printf("Performing FASTLOOP metric test on MCP3008....\n");
	 
	
	 //set pin 7 high to turn led on
	 PORTD |= (1 << PORTD7);
		 
 	
 	//Start test:
	MAX_CNT = 1*1000000; //1000000; //Fast loop mode
	
 	//Set up for SPI communications
	SPI_MasterInit();
	 		
	ts_st = getHMSTimestamp();
	SPI_MasterInit();  //Need to do twice (check why?)
	
	for(uint32_t i=0;i<MAX_CNT;i++)
	{
		SPI_ReadMPC3008();
		
	}	
	 
	ts_end = getHMSTimestamp();
	time_diff = ts_end - ts_st;
	
	printf("Time difference for loop (secs):  %lu\n",time_diff);
	printf("Max count: %lu\n",MAX_CNT);
	//printf("Time difference per sample (secs):  %f\n",(float)time_diff/(float)MAX_CNT);
	uint32_t eff_rate = (MAX_CNT/time_diff);
	printf("Effective sampling rate:  %lu\n",eff_rate) ;
	
		
	
	//set pin 7 low to turn led off
   PORTD &= ~(1 << PORTD7);
	
	printf("Exiting Fastloop Sampling Metric test ...\n");

	exit(0);
	
	
}//doMetricTest_FastLoop	


uint32_t getHMSTimestamp()
{
	//Get total timestamp in seconds for Hrs/Min/Sec:
	char numStr[3];
	
	uint32_t ts_hr,ts_min,ts_sec;
	
	uint8_t nBytes = 3;  //Hrs/Min/Sec
	if( i2c_readbyte(SLAVE_ADDR,RTC_SECOND,nBytes) == -1 )
	{
		printf("\r\nThere was a problem reading from the DS3231 register.");
	}
	
	//Get secs from hrs
	//Get first digit:
	digit1 = dataBuffer[2] >> 4;
	//Get second digit:
	digit2 = dataBuffer[2] & mask;
   sprintf(numStr,"%d%d",digit1,digit2);
	ts_hr =  60*60*((uint32_t)atoi(numStr));
	//ts_hr =  atoi(numStr);
	
		
	//Get secs from mins
	//Get first digit:
	digit1 = dataBuffer[1] >> 4;
	//Get second digit:
	digit2 = dataBuffer[1] & mask;
   sprintf(numStr,"%d%d",digit1,digit2);
	ts_min =  60*((uint32_t)atoi(numStr));
	
	//Get first digit:
	digit1 = dataBuffer[0] >> 4;
	//Get second digit:
	digit2 = dataBuffer[0] & mask;
   sprintf(numStr,"%d%d",digit1,digit2);
	ts_sec =  (uint32_t) atoi(numStr);
	
		
	return ts_hr+ts_min+ts_sec;
		
			
}//getHMSTimestamp	



