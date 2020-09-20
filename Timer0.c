/*
 ======================================================================================
 Name        : Timer0.h
 Author      : Michael McGetrick
 Version     :
 Copyright   : 
 Description : C Module to access Timer0 (8 bit) of the Atmega328p MCU 
               
 	 	 	   
 =====================================================================================
*/
#include "Timer0.h"
#include <avr/interrupt.h>


//ATTRIBUTES  --------------------------------------------------------------------------------------
//float CURR_FREQ = 0.0;
volatile bool chg_val = false;    
volatile uint32_t timer0_cnt = 0;
volatile uint32_t timer0_comp=0;

//FUNCTION DEFINITIONS HERE: -----------------------------------------------------------------------

void timer0_init(float curr_freq)  //8 bit timer
{

	//Configure timer:
	//TCCR1B |= (1 << WGM12) ;  //CTC mode#include <avr/interrupt.h>
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


	CURR_FREQ = curr_freq;

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
 
//-----------------------------------------------------------------------------------------------------

