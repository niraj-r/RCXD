#include "wiringPi.h"
#include "wiringSerial.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>			//Used for UART
#include <fcntl.h>			//Used for UART
#include <termios.h>		//Used for UART

#define DEVICE "/dev/ttyUSB0"	// Device ID
#define BAUD 115200				// Baud Rate
#define START_SCAN "\xA5\x20" 	// Begins scanning
#define FORCE_SCAN "\xA5\x21" 	// Overrides anything preventing a scan
#define HEALTH "\xA5\x52" 		// Returns the state of the Lidar
#define STOP_SCAN "\xA5\x25" 	// Stops the scan
#define RESET "\xA5\x40" 		// Resets the device

void startScan2(int ser);

int main( int argc, char** argv ) {
	//-------------------------
	//----- SETUP USART 0 -----
	//-------------------------
	//At bootup, pins 8 and 10 are already set to UART0_TXD, UART0_RXD (ie the alt0 function) respectively
	int fd = -1;
	
	//OPEN THE UART
	//The flags (defined in fcntl.h):
	//	Access modes (use 1 of these):
	//		O_RDONLY - Open for reading only.
	//		O_RDWR - Open for reading and writing.
	//		O_WRONLY - Open for writing only.
	//
	//	O_NDELAY / O_NONBLOCK (same function) - Enables nonblocking mode. When set read requests on the file can return immediately with a failure status
	//											if there is no input immediately available (instead of blocking). Likewise, write requests can also return
	//											immediately with a failure status if the output can't be written immediately.
	//
	//	O_NOCTTY - When set and path identifies a terminal device, open() shall not cause the terminal device to become the controlling terminal for the process.
	fd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NDELAY);		//Open in non blocking read/write mode
	if (fd == -1)
	{
		//ERROR - CAN'T OPEN SERIAL PORT
		printf("Error - Unable to open UART.  Ensure it is not in use by another application\n");
	}
	
	//CONFIGURE THE UART
	//The flags (defined in /usr/include/termios.h - see http://pubs.opengroup.org/onlinepubs/007908799/xsh/termios.h.html):
	//	Baud rate:- B1200, B2400, B4800, B9600, B19200, B38400, B57600, B115200, B230400, B460800, B500000, B576000, B921600, B1000000, B1152000, B1500000, B2000000, B2500000, B3000000, B3500000, B4000000
	//	CSIZE:- CS5, CS6, CS7, CS8
	//	CLOCAL - Ignore modem status lines
	//	CREAD - Enable receiver
	//	IGNPAR = Ignore characters with parity errors
	//	ICRNL - Map CR to NL on input (Use for ASCII comms where you want to auto correct end of line characters - don't use for bianry comms!)
	//	PARENB - Parity enable
	//	PARODD - Odd parity (else even)
	struct termios options;
	tcgetattr(fd, &options);
	options.c_cflag = B115200 | CS8 | CLOCAL | CREAD;		//<Set baud rate
	options.c_iflag = IGNPAR;
	options.c_oflag = 0;
	options.c_lflag = 0;
	tcflush(fd, TCIFLUSH);
	tcsetattr(fd, TCSANOW, &options);
	
	//----- TX BYTES -----
	unsigned char tx_buffer[20];
	unsigned char *p_tx_buffer;
	
	p_tx_buffer = &tx_buffer[0];
	*p_tx_buffer++ = 'A';
	*p_tx_buffer++ = '5';
	*p_tx_buffer++ = '2';
	*p_tx_buffer++ = '0';
	
	if (fd != -1)
	{
		int count = write(fd, "\xA5\x20", 2);		//Filestream, bytes to write, number of bytes to write
		if (count < 0)
		{
			printf("UART TX error\n");
		}
	}
	
	//----- CHECK FOR ANY RX BYTES -----
	while(1){
	if (fd != -1)
	{
		// Read up to 255 characters from the port if they are there
		unsigned char rx_buffer[256];
		int rx_length = read(fd, (void*)rx_buffer, 1);		//Filestream, buffer to store in, number of bytes to read (max)
		if (rx_length < 0)
		{
			//printf("Error: no bytes waiting\n"); //(will occur if there are no bytes)
		}
		else if (rx_length == 0)
		{
			//No data waiting
		}
		else
		{
			//Bytes received
			rx_buffer[rx_length] = '\0';
			printf("%i bytes read : %d\n", rx_length, (int)rx_buffer);
		}
	}
	}
	
/*	int ser = serialOpen(DEVICE, BAUD);
	if (ser == -1) {
		printf("Error: Could not open serial");
	}else printf("Connected to: %d\n", ser);
	startScan2(ser);
*/
}

void startScan2(int ser){
	char line[100];
	int start = 0xA582;
	printf("%s\n", START_SCAN);
	memset(line, '\0', sizeof(line));
	serialPuts(ser, "\xA5\x20");
	while(1){	
		if(serialDataAvail(ser)){
			printf("%d", serialGetchar(ser));
		} else printf("%d\n", serialDataAvail(ser));
		delay(1000);
	}
}
