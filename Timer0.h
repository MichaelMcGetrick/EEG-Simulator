/*
 ======================================================================================
 Name        : Timer0.h
 Author      : Michael McGetrick
 Version     :
 Copyright   : 
 Description : C Module to access Timer0 (8 bit) of the Atmega328p MCU 
               
 	 	 	   
 =====================================================================================
*/
  

#ifndef TIMER0_H_
#define TIMER0_H_

#include <avr/io.h>
#include <stdio.h>
#include <stdbool.h>

//DEFINES HERE ------------------------------------------------------------------------------------
#define FOSC 16000000    				//CPU clock frequency
#define FREQ_RESP_PRESCALER	8		//MCU prescaler

//FUNCTION DECLARATIONS HERE -----------------------------------------------------------------------
//(8 bit timer)
void timer0_init(float curr_freq);
void start_timer0(void);
void stop_timer0(void);




//ATTRIBUTES ---------------------------------------------------------------------------------------
float CURR_FREQ;
extern volatile bool chg_val;
extern volatile uint32_t timer0_cnt, timer0_comp;

#endif /* TIMER0_H_ */
