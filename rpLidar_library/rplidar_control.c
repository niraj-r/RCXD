#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/select.h>
#include <errno.h>
#include <string.h>
#include <inttypes.h>
#include <time.h>
#include <sys/time.h>
#include <math.h>

#include "rplidar_control.h"


#ifdef DEBUG
	#define DEBUG_PRINT(...) 									\
		do {											\
			printf("\nIn %s - function %s at line %d: ", __FILE__, __func__, __LINE__);	\
			printf(__VA_ARGS__);								\
		} while(0)
#else
	#define DEBUG_PRINT(...) (void)0
#endif


struct rplidar {
    int serial_fd;  /* port file descriptor */
};


//TODO: 

/* open a new rplidar connection */
rplidar_t* rplidar_create() {
    rplidar_t* rplidar = malloc( sizeof( rplidar_t ) );
    if ( !rplidar ) {
        printf( "Malloc error \n" );
        return NULL;
    }

    char port[20] = PORT; /* port to connect to */
    speed_t baud = BAUDRATE; /* baud rate */

    rplidar->serial_fd = open( port, O_RDWR | O_NOCTTY | O_NDELAY ); /* connect to port */
    if ( rplidar->serial_fd == -1 ) {
        printf("Error connecting \n");
        return 0;
    }

    /* set UART settings (always for RPlidar: 115200 8N1) */
    struct termios settings;
    tcgetattr( rplidar->serial_fd, &settings );

    cfsetispeed( &settings, baud ); /* set baud rate */
    cfsetospeed( &settings, baud ); /* set baud rate */
    settings.c_cflag |= ( CLOCAL | CREAD ); /* enable RX & TX */ //CLOCAL: ignore modem status lines (?)
    settings.c_cflag &= ~PARENB; /* no parity */
    settings.c_cflag &= ~CSTOPB; /* 1 stop bit */
    settings.c_cflag &= ~CSIZE; /* character size mask. Do this before setting size with CS8 */
    settings.c_cflag |= CS8;  /* 8 data bits */

    //settings.c_cflag &= ~CNEW_RTSCTS; // no hw flow control (not supported?)
    settings.c_iflag &= ~(IXON | IXOFF | IXANY); // no sw flow control
    settings.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);    // raw input mode
    settings.c_oflag &= ~OPOST;    // raw output mode

    tcflush( rplidar->serial_fd, TCIFLUSH );	//Flush input buffer

    if ( fcntl( rplidar->serial_fd, F_SETFL, FNDELAY ) ) {
        printf("Error: Line number %d in file %s \n", __LINE__, __FILE__);
    }

    settings.c_cc[VTIME] = 0;	/* inter-character timer unused */
    settings.c_cc[VMIN] = 1;	/* blocking read until 1 chars received */
    
    if ( tcsetattr( rplidar->serial_fd, TCSANOW, &settings ) ) {
        printf("Error: line number %d in file %s \n", __LINE__, __FILE__);
    }

    tcflush( rplidar->serial_fd, TCOFLUSH );	//Flush output buffer

    return rplidar;
}

/* close the connection with the rplidar */
void rplidar_destroy( rplidar_t** rplidar ) {
    uint8_t i = close( (*rplidar)->serial_fd );
    if ( i == -1 ) printf( "error closing serial port" );
    free( *rplidar );
    *rplidar = NULL;
}

/* send a request packet to the rplidar */
void rplidar_send_request( rplidar_t* rplidar, unsigned char command, unsigned char* payload, int payload_size ) {

    uint8_t size;
    if ( !payload ) {
        size = 2;
    } else {
        size = payload_size;
    }

    unsigned char msg[ size ];
    msg[0] = DES_REQUEST;
    msg[1] = command;

    uint8_t ans;
    unsigned char tx_len = 0;

    do {    /* write bytes and check if they are all written. Alternative: make blocking.  */
        ans = write( rplidar->serial_fd, msg + tx_len, size - tx_len );
        if (ans == -1) printf( "write error \n" );
        tx_len += ans;
    } while ( tx_len < size );

    printf("sent: %x, %x \n ", msg[0], msg[1] );
}

/* read data sent by rplidar to system */
void rplidar_read_data( rplidar_t* rplidar, unsigned char* data, uint8_t size ) {
    uint8_t ans, rx_len = 0;
    do {    /* write bytes and check if they are all written, if last bytes are not written, resend them. Alternative: make blocking. */
        ans = read( rplidar->serial_fd, &data[ rx_len ], size - rx_len );
        if ( ans == -1 ) {
            printf( "RD: Oh dear, something went wrong with read()! %s\n", strerror(errno) );
            return;
        }
        rx_len += ans;
        if ( rx_len < size ) printf( "Read: not enough bytes ( read = %d , size = %d), retrying \n", rx_len, size );
    } while ( rx_len < size );
}


/* check if the response matches the general and specific response descriptors. This function does not check for correct data */
void rplidar_check_response( rplidar_t* rplidar, int length, char send_mode, char data_type ) {

    unsigned char res_descriptor[ SIZE_HEADER ];
    rplidar_read_data( rplidar, &res_descriptor[0], SIZE_HEADER );

    uint8_t i;
    printf("header: \t");
    for ( i = 0; i < SIZE_HEADER; i++ ) {
        printf("%X ", res_descriptor[i] );
    }
    printf("\n");

    if ( ( *res_descriptor != DES_REQUEST ) || ( *(res_descriptor+1) != DES_RESPONSE ) ) {
        printf( "response error: wrong descriptor \n" );
    }

    if ( ( ( *(res_descriptor+5) & 0xC0 ) >> 6  ) != send_mode ) {
        printf( "response error: wrong send_mode (%d) \n", ( *(res_descriptor+5)  ) );
    }

    uint32_t res_length = (unsigned int) ( *(res_descriptor+2) & 0xCFFF );
    if ( res_length != length ) {
        printf( "%X \n ", *(res_descriptor+2) );
        printf( "response error: wrong length: %d \n", res_length );
    }

    if ( *(res_descriptor+6) != data_type ) {
        printf( "response error: wrong data type \n" );
    }
}


void print_raw( unsigned char data[], uint8_t size ) {
    printf( "raw byte data: " );
    int i;
    for ( i = 0; i < size; i++ ) printf( "%x ", data[i] );
    printf( "\n" );
}


/* FUNCTIONS :
 *  rplidar_stop:       exit the current state and enter the idle state (e.g. stop scanning)
 *  rplidar_reset:      reset/reboot the rplidar
 *  rplidar_info:       get device info
 *  rplidar_health      get device health info
 *  rplidar_scan:       start scanning, continuous response of scan packets
 *  rplidar_force_scan: same as rplidar_scan, but force output without checking rotation speed
 */

void rplidar_reset( rplidar_t* rplidar ) {
    rplidar_send_request( rplidar, CMD_RESET, NULL, 0 );
}


void rplidar_stop( rplidar_t* rplidar ) {
    rplidar_send_request( rplidar, CMD_STOP, NULL, 0 );
}


void rplidar_info( rplidar_t* rplidar ) {

    rplidar_send_request( rplidar, CMD_INFO, NULL, 0);

    rplidar_check_response( rplidar, SIZE_INFO, SINGLE_RES, DATA_INFO );

    unsigned char buff_data[ SIZE_INFO ];
    rplidar_read_data( rplidar, buff_data, SIZE_INFO );

    // print out the device serial number, model number, firmware and hardware version number, respectively
    int pos;
    printf("RPLIDAR S/N: ");
    for ( pos = 4; pos <= 19; pos++ ) {
        printf("%02X", buff_data[pos]);
    }
    printf( "\n"
            "Model ID: %d \n"
            "Firmware Ver: %d.%02d\n"
            "Hardware Rev: %d\n"
            , buff_data[0]
            , buff_data[1]
            , buff_data[2]
            , (int) buff_data[3] );
}


void rplidar_health( rplidar_t* rplidar ) {

    rplidar_send_request( rplidar, CMD_HEALTH, NULL, 0);

    rplidar_check_response( rplidar, SIZE_HEALTH, SINGLE_RES, DATA_HEALTH );

    unsigned char buff_data[ SIZE_HEALTH ];
    rplidar_read_data( rplidar, buff_data, SIZE_HEALTH );

    unsigned char status = buff_data[0];
    if ( status == 0 ) {
        printf( "I'm OK! \n" );
        return;
    }
    if ( status == 1 ) {
        printf( "Warning: %X%X \n", buff_data[2], buff_data[1] );
        return;
    }
    if ( status == 2 ) {
        printf( "Error: %X%X \n", buff_data[2], buff_data[1] );
        return;
    }

    printf( "I'm in an unspecified state (%X) :( \n", status );
}


void rplidar_scan( rplidar_t* rplidar ) {
    unsigned char buff_data[ SIZE_SCAN ];
    unsigned char quality, start_flag, start_flag_inv, check_bit;
    uint16_t angle, distance;
    
    float RPS; //store calculated rotation speed (in Hz = rotations per second)
    struct timespec time_raw; //necessary for storing time data from clock_gettime, in ns
    uint32_t old_time = 0; //store time of last scan (in ms) 
    uint32_t current_time = 0;       //store time of new scan

    rplidar_send_request( rplidar, CMD_SCAN, NULL, 0 );

    rplidar_check_response( rplidar, SIZE_SCAN, MULTIPLE_RES, DATA_SCAN );

    uint16_t i;
    for ( i = 0; i < 8000; i++ ) {
        rplidar_read_data( rplidar, buff_data, SIZE_SCAN );

        printf( "scan data (%d):\t", i );
        print_raw( buff_data, SIZE_SCAN );

        quality = ( buff_data[0] & 0xFC );
        start_flag = ( buff_data[0] & 0x01 );
        start_flag_inv = ( buff_data[0] & 0x02 );
        angle = ( (uint16_t) ( ( buff_data[1] >> 1 ) | ( buff_data[2] << 7 ) ) ) / 64;
        distance = (uint16_t) ( buff_data[3] | ( buff_data[4] << 8 ) ) / 4;
        check_bit = buff_data[1] & 0x01 ;

        if ( start_flag == start_flag_inv ) printf( "error: start flags wrong (%x, %x) \n", start_flag, start_flag_inv );
        if ( check_bit != 1 ) printf( "error: check bit (%x) \n", check_bit );

        //printf( "Measurement %d \n", i );
        printf( "\tQuality = %d \t", quality );
        printf( "\tAngle = %d \t", angle);
        printf( "\tDistance = %d \n", distance );
        
        if ( start_flag == 1 ) {
            printf( "NEW 360 SCAN \n" );
            clock_gettime( CLOCK_MONOTONIC, &time_raw );
            current_time = time_raw.tv_nsec / 1000000;  //(time in miliseconds)
            if ( old_time != 0 ) { 
                RPS = ( 1000.0 / ( current_time - old_time ) );
                printf( "old_time = %d \n", old_time );
                printf( "current_time = %d \n", current_time );
                printf( "Speed = %f RPM\n", RPS*60 );
            }
            old_time = current_time;
        }
    }
}


    //should wait at least 1ms between two requests, 2ms after reboot. In SDK, default timeout = 2ms
int main(){

    rplidar_t* rplidar = rplidar_create();
    if ( !rplidar ) {
        printf( "Error creating rplidar \n" );
        return 0;
    }
    
    usleep( 1000000 );

    rplidar_health( rplidar );

    usleep( 10000 );

    rplidar_stop( rplidar );

    usleep( 10000 );

    rplidar_info( rplidar );

    usleep( 10000 );

    rplidar_stop( rplidar );

    usleep( 10000 );

    rplidar_scan( rplidar );

    usleep( 100000 );

    rplidar_stop( rplidar );

    usleep( 10000 );

    rplidar_destroy( &rplidar );

    return 0;
}
