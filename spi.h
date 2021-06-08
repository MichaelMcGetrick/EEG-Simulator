/*
 ======================================================================================
 Name        : spi.h
 Author      : Michael McGetrick
 Version     :
 Copyright   : 
 Description : C Module for SPI communication with peripheral devices 
               
 	 	 	   
 =====================================================================================
*/
  

#ifndef SPI_H_
#define SPI_H_

#include <avr/io.h>
#include <stdio.h>
#include <stdbool.h>

//DEFINES HERE ------------------------------------------------------------------------------------
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

#define SPI_CLK_FREQ		3 //MHz: Highest Options: 1MHz or 2MHz (4MHz above 3.6MHz maximum rating for MCP3008)
									//1: 1MHz; 2: 2MHz; 3: 0.5MHz



//FUNCTION DECLARATIONS HERE -----------------------------------------------------------------------
void SPI_MasterInit(void);
void SPI_Transmit( int8_t data );



//ATTRIBUTES ---------------------------------------------------------------------------------------


#endif /* SPI_H_ */
