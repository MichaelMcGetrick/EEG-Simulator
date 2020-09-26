/*
 ======================================================================================
 Name        : Timer1.h
 Author      : Michael McGetrick
 Version     :
 Copyright   : 
 Description : C Module to access Timer1 (16 bit) of the Atmega328p MCU 
               
 	 	 	   
 =====================================================================================
*/
  

#ifndef TIMER1_H_
#define TIMER1_H_

#include <avr/io.h>
#include <stdio.h>
#include <stdbool.h>

//DEFINES HERE ------------------------------------------------------------------------------------
#define FOSC 16000000    				//CPU clock frequency
#define SAMPLE_RATE			1000 //1000   //(samples per sec)
#define PRESCALER				8  //64  //1024: Sample rate = 0; Others: USe Timer1_Calcs.xls table
#define PRESCALER_METRICS	8//1024
#define SECS_PER_SAMPLE		1

//Defines for conditional compilation: 
#define	REGULAR				0
#define	METRIC_TEST			1
#define PROGRAM_MODE			REGULAR	//METRIC_TEST				


//FUNCTION DECLARATIONS HERE -----------------------------------------------------------------------
//(16 bit timer)
void timer1_init(float curr_freq);
void start_timer1(void);
void stop_timer1(void);


//ATTRIBUTES ---------------------------------------------------------------------------------------
float CURR_FREQ;
extern uint32_t fr_prescaler;
extern uint32_t fr_sampleRate, fr_secsPerSam;
extern bool freq_response;
extern volatile bool sample;
extern volatile int timer_cnt;

#endif /* TIMER1_H_ */
