/*
 ======================================================================================
 Name        : spi.c
 Author      : Michael McGetrick
 Version     :
 Copyright   : 
 Description : C Module for SPI communication with peripheral devices 
               
 	 	 	   
 =====================================================================================
*/
#include "spi.h"


//ATTRIBUTES  --------------------------------------------------------------------------------------

//FUNCTION DEFINITIONS HERE: -----------------------------------------------------------------------
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
	if(SPI_CLK_FREQ == 1)
	{
		SPCR |= (1 << SPR0); //Prescaler fosc/16 (1MHz)
	}
	if(SPI_CLK_FREQ == 2)
	{
		//Prescaler fosc/8 (1MHz)
		SPCR |= (1 << SPR0);
		SPSR |= (1 << SPI2X); //Bit to double the clock speed 
	}
	
	
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
	

}//SPI_Transmit

 
//-----------------------------------------------------------------------------------------------------

