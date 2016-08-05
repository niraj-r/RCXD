#include "wiringPi.h"
#include "wiringSerial.h"
#include <string.h>

#define DEVICE "/dev/ttyUSB0"	// Device ID
#define BAUD 115200				// Baud Rate
#define START_SCAN "\xA5\x20" 	// Begins scanning
#define FORCE_SCAN "\xA5\x21" 	// Overrides anything preventing a scan
#define HEALTH "\xA5\x52" 		// Returns the state of the Lidar
#define STOP_SCAN "\xA5\x25" 	// Stops the scan
#define RESET "\xA5\x40" 		// Resets the device

int main( int argc, char** argv ) {
	int ser = serialOpen(DEVICE, BAUD)
	if (ser == -1) {
		printf("Error: Could not open serial");
	}
	startScan2(ser)
}

void startScan(int ser){
	int size = 1;
	int lock = 0;
	char read = "";
	char *line = calloc(size + 1, sizeof(char));
	
	printf("Connecting")
	while( lock == 0 ){
		printf("...");
		serialPutchar(ser, RESET);
		delay(2000);		// 2 sec delay
		serialPutchar(ser, START_SCAN);

		for(int i = 0; i <= 250; i++){
			if(serialDataAvail(ser) != -1){
				read = (char)serialGetchar(ser);
			}
			line = realloc(++size + 1, sizeof(char));
			line[size] = read;
			line[size + 1] = '\0';
			
			if(strstr(line, "\xa5\x5a") != NULL){
				lock = 1;
				break;
			}else if {
				strcpy(line, "");
				size = 0;
			}
		}
	}
	free(&line);
	return lock;
}

void startScan2(int ser){
	int size = 1;
	int lock = 0;
	char read = "";
	char line[100];
	memset(line, '\0', sizeof(line));
	serialPutchar(ser, START_SCAN);
	while(1){
		printf("%d\n", serialDataAvail(ser))
		delay(1000);
	}
}

void getPoints(int ser){
	int dist = 0, angle = 0;
	int size = 1;
	char read = "";
	char *line = calloc(size + 1, sizeof(char));
	while(true){
		if(serialDataAvail(ser) != -1){
			read = (char)serialGetchar(ser);
		}
		line = realloc(++size + 1, sizeof(char));
		line[size] = read;
		line[size + 1] = '\0';
		
		if(strlen(line) == 6){
			if(polar == true) pointPolar(line, dist, angle);
			else pointRect(line, dist, angle);
		}
	}
}

void pointPolar(char *line, int dist, int angle){
	dist = strtol(line[4], NULL, 6) + strtol(line[3], NULL, 6)
	dist = dist/4;
}

void pointRect(char *line, int dist, int angle){
}
