/*
 ======================================================================================
 Name        : Timer1.h
 Author      : Michael McGetrick
 Version     :
 Copyright   : 
 Description : C Module to access Timer1 (16 bit) of the Atmega328p MCU 
               
 	 	 	   
 =====================================================================================
*/
#include "Timer1.h"
#include <avr/interrupt.h>


//ATTRIBUTES  --------------------------------------------------------------------------------------
uint32_t fr_prescaler = 8;
volatile int timer_cnt = 0;
volatile bool sample = false;
bool freq_response = false;
uint32_t fr_sampleRate = 0;
uint32_t fr_secsPerSam = 0;


//FUNCTION DEFINITIONS HERE: -----------------------------------------------------------------------
void timer1_init(float curr_freq)   //16 bit timer
{

	CURR_FREQ = curr_freq;
	
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
 
//-----------------------------------------------------------------------------------------------------

