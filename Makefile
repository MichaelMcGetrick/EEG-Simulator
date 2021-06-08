MCU=atmega328p
F_CPU=16000000
CC=avr-gcc
OBJCOPY=avr-objcopy
AVRSIZE=avr-size
FORMAT=avr
CFLAGS = -Wall -g -Os -mmcu=$(MCU) -DF_CPU=$(F_CPU) 
TARGET=EEG-Simulator
SRC=main.c
#Avrdude settings
PARTNUM=m328p
BAUD=115200
#NB: b=9600 does not work for Arduino/RPI combination
PROG_TYPE=arduino
MEM_OP_FLASH=flash:w:$(TARGET).hex:a
SERIAL_PORT=/dev/ttyACM0



#objcopy (create hex file from elf file):
#flags:
#-j : only keep this section
all:
	$(CC) $(CFLAGS) -c -o UARTComms.o UARTComms.c
	$(CC) $(CFLAGS) -c -o Timer0.o Timer0.c
	$(CC) $(CFLAGS) -c -o Timer1.o Timer1.c
	$(CC) $(CFLAGS) -c -o spi.o spi.c
	$(CC) $(CFLAGS) -c -o i2c.o i2c.c
	$(CC) $(CFLAGS) -c -o main.o main.c
	$(CC) -Wl,-Map,$(TARGET).map -mmcu=$(MCU) -o $(TARGET).elf  UARTComms.o Timer0.o Timer1.o spi.o i2c.o main.o -lm
	$(OBJCOPY) -j .text -j .data -O ihex $(TARGET).elf $(TARGET).hex
	$(AVRSIZE) --format=$(FORMAT) --mcu=$(MCU) $(TARGET).elf
	
flash:
	avrdude -p$(PARTNUM) -c$(PROG_TYPE) -b$(BAUD) -U$(MEM_OP_FLASH) -F -P$(SERIAL_PORT)

clean:
	rm -rf *.o *.map *.elf *.hex
		
			
