/*
  	i2c.c - I2C Module for ATmega328p MCU

	AUTHOR: MICHAEL MCGETRICK
	DESCRIPTION: SET OF FUNCTIONS to implement the I2C Bus Communications on the ATmega328p MCU
				//Functionality  is specific for the GY-521 I2C Device.



	Copyright (c) 2018 Michael McGetrick.  All right reserved.

*/

#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#include "i2c.h"

//DEFINES HERE ------------------------------------------------------------------------
static int8_t reg;

//DEFINE FUNCTIONS HERE ---------------------------------------------------------------
void i2c_init(void)	//Ready the I2C pins and set the bit rate
{

	//NB: The DS3231 board (with EEPROM chip) has pullup resistros connected
	//    to the SDA/CLK pins of DS3231. Hence, we do not need to provide external
	//    pullups in our circuit

	//Set prescaler to 1:
	TWSR &= ~(1 << TWPS0);
	TWSR &= ~(1 << TWPS1);

	//Set the SCL frequency

	TWBR = ((F_CPU / TWI_CLOCK_FREQ) - 16) / 2;
	//This provides 72 for F_CPU = 16MHz and SLC = 100000Hz

	//Enable I2C module, Acks, and I2C interrupt
	//TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWEA);
	//Activate without interrupt:
	TWCR = _BV(TWEN) | _BV(TWEA);

}

unsigned char i2c_transmit(unsigned char type)
{
  switch(type)
  {
     case I2C_START:    // Send Start Condition
       TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
       break;
     case I2C_DATA:     // Send Data
       TWCR = (1 << TWINT) | (1 << TWEN);
       break;
     case I2C_ACK:     // Send ACK
	   TWCR = (1 << TWINT) | (1 << TWEA) | (1 << TWEN);
	   break;
     case I2C_NACK:     // Send NACK
    	reg = TWCR;
	   reg &= ~(1 << TWEA);
	   reg = (1 << TWINT) | (1 << TWEN);
       TWCR = reg;
	   break;
	 case I2C_STOP:     // Send Stop Condition
       TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);
       return 0;
  }

  // Wait for TWINT flag set in TWCR Register
  while (!(TWCR & (1 << TWINT)));

  // Return TWI Status Register, mask the prescaler bits (TWPS1,TWPS0)
  return (TWSR & 0xF8);

}//i2c_transmit


int i2c_writebyte(unsigned int dev_addr,unsigned int reg_addr,char data)
{
  unsigned char n = 0;
  unsigned char twi_status;
  char r_val = -1;

i2c_retry:
  if (n++ >= MAX_TRIES) return r_val;

  // Transmit Start Condition
  twi_status=i2c_transmit(I2C_START);

  // Check the TWI Status
  if (twi_status == TW_MT_ARB_LOST) goto i2c_retry;
  if ((twi_status != TW_START) && (twi_status != TW_REP_START)) goto i2c_quit;

  // Set up slave address (SLA_W) in data register
  //TWDR = (dev_id & 0xF0) | ((dev_addr & 0x07) << 1) | TW_WRITE;
  TWDR = (dev_addr << 1) | TW_WRITE;

  // Transmit I2C Address Data
  twi_status=i2c_transmit(I2C_DATA);

  // Check the TWSR status
  if ((twi_status == TW_MT_SLA_NACK) || (twi_status == TW_MT_ARB_LOST)) goto i2c_retry;
  if (twi_status != TW_MT_SLA_ACK) goto i2c_quit;

  //Send the internal register address required:
  TWDR = reg_addr;
  // Transmit I2C Data
  twi_status=i2c_transmit(I2C_DATA);
  // Check the TWSR status
  if (twi_status != TW_MT_DATA_ACK) goto i2c_quit;


  // Put data into data register and start transmission
  TWDR = data;

  // Transmit I2C Data
  twi_status=i2c_transmit(I2C_DATA);

  // Check the TWSR status
  if (twi_status != TW_MT_DATA_ACK) goto i2c_quit;

  // TWI Transmit Ok
  r_val=1;

i2c_quit:
  // Transmit I2C Data
  twi_status=i2c_transmit(I2C_STOP);
  return r_val;


}//i2c_writebyte


//int i2c_readbyte(unsigned int i2c_address, unsigned int dev_id,
//                 unsigned int dev_addr, char *data)
int i2c_readbyte(unsigned int dev_addr, unsigned int reg_addr, unsigned int numbytes)
{

  unsigned char n = 0;
  unsigned char twi_status;
  char r_val = -1;


i2c_retry:
  if (n++ >= MAX_TRIES) return r_val;

    // Send start Condition
  twi_status=i2c_transmit(I2C_START);

  // Check the TWSR status
  if (twi_status == TW_MT_ARB_LOST) goto i2c_retry;
  if ((twi_status != TW_START) && (twi_status != TW_REP_START)) goto i2c_quit;

  // Send slave address (SLA_W)
  TWDR = (dev_addr << 1) | TW_WRITE;
  // Transmit I2C Data
  twi_status=i2c_transmit(I2C_DATA);

  // Check the TWSR status
  if ((twi_status == TW_MT_SLA_NACK) || (twi_status == TW_MT_ARB_LOST)) goto i2c_retry;
  if (twi_status != TW_MT_SLA_ACK) goto i2c_quit;

  // Send the required register address:
   TWDR = reg_addr;

   // Transmit I2C Data
   twi_status=i2c_transmit(I2C_DATA);

   // Check the TWSR status
   if (twi_status != TW_MT_DATA_ACK) goto i2c_quit;

    // Send start Condition
    twi_status=i2c_transmit(I2C_START);

    // Check the TWSR status
    if (twi_status == TW_MT_ARB_LOST) goto i2c_retry;
    if ((twi_status != TW_START) && (twi_status != TW_REP_START)) goto i2c_quit;

    // Send slave address (SLA_R)
    TWDR = (dev_addr << 1) | TW_READ;

    // Transmit I2C Data
    twi_status=i2c_transmit(I2C_DATA);

    // Check the TWSR status
    if ((twi_status == TW_MR_SLA_NACK) || (twi_status == TW_MR_ARB_LOST)) goto i2c_retry;
    if (twi_status != TW_MR_SLA_ACK) goto i2c_quit;

    //Get the required bytes:
    for(int i=0; i< numbytes; i++)
    {
		if( i < (numbytes-1))
		{
			twi_status=i2c_transmit(I2C_ACK);
			if (twi_status == TW_MR_DATA_ACK)
			{
				// Get the Data
				dataBuffer[i] = TWDR;

			}
			else
			{
				goto i2c_retry;
			}
		}
		else
		{
			//Get the last byte:
			twi_status=i2c_transmit(I2C_NACK);
			if (twi_status == TW_MR_DATA_NACK)   //This should be the last byte requested
			{
				// Get the Data
				dataBuffer[i] = TWDR;

			}
			else
			{
				printf("\r\nDis NOT receive the last byte with NACK returned");
			}
		}

    }

    r_val=1;

  i2c_quit:
    // Send Stop Condition
    twi_status=i2c_transmit(I2C_STOP);
    return r_val;


}


