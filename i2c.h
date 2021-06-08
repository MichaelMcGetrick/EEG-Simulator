/*
  i2c.h - I2C Module for Atmega328p
  AUTHOR: MICHAEL MCGETRICK
	DESCRIPTION: SET OF FUNCTIONS to implement the I2C Bus Communications on the ATmega328p MCU
				//Functionality  is specific for the GY-521 I2C Device.



	Copyright (c) 2018 Michael McGetrick.  All right reserved.

*/

#ifndef i2c_h
#define i2c_h

//I2C DEFINES HERE -------------------------------------------------------------------------------
#define TRUE 1
#define FALSE 0

#define TWI_CLOCK_FREQ 100000L  //FCU should be > 16 x slave clock frequency

#define TWI_READY 			0
#define TWI_MRX   			1
#define TWI_MTX   			2
#define TWI_SRX   			3
#define TWI_STX   			4

#define TW_START  		 0x08
#define TW_REP_START  	 0x10
#define TW_MT_ARB_LOST	 0x38
#define TW_MR_ARB_LOST   0x38
#define TW_MT_SLA_ACK    0x18
#define TW_MT_SLA_NACK   0x20
#define  TW_MR_SLA_ACK   0x40
#define  TW_MR_SLA_NACK  0x48
#define TW_MT_DATA_ACK	 0x28
#define TW_MR_DATA_ACK	 0x50
#define TW_MR_DATA_NACK	 0x58
#define TW_WRITE	 			0
#define TW_READ		 		1

#define MAX_TRIES 	    	50
#define I2C_START 			0
#define I2C_DATA  			1
#define I2C_STOP  			2
#define I2C_ACK				3
#define I2C_NACK				4

#define READ_MAX_BYTES		14


//DEFINE VARIABLES HERE-----------------------------------------------------------------------
//uint8_t dataBuffer[READ_MAX_BYTES];
uint8_t dataBuffer[READ_MAX_BYTES];

//FUNCTION DECLARATIONS HERE -----------------------------------------------------------------
void i2c_init(void);
unsigned char i2c_transmit(unsigned char type);
int i2c_writebyte(unsigned int dev_addr,unsigned int reg_addr, char data);
int i2c_readbyte(unsigned int dev_addr, unsigned int reg_addr, unsigned int numbytes);
//--------------------------------------------------------------------------------------------

#endif

