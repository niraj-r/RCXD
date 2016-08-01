// Property of Niraj Raniga
// July 9, 2016

// client.c: contains main for controlling rcxd

// Front Left: min(1275), max(1675)
// Front Right: min(825), max(1225)
// Back Left: min(950), max(1350)
// Back Right: min(1100), max(1500)

#include <stdio.h>
#include <stdlib.h>
#include <pigpio.h>
#include "pid.h"
#include "gyro.h"

#define SERVO 17
#define GYRO_ADR 0x68
#define I2C_BUS 1

int main(int argc, char *argv[]){
	
	int gyro = 0;
	double x_rotation = 0;
	double y_rotation = 0;
	
	gpioInitialise();
	gpioSetMode(SERVO, PI_OUTPUT);
	gyro = gyroInit(I2C_BUS, GYRO_ADR);
	
	
	while(1){
		//gpioServo(SERVO, 1100);
		//time_sleep(1);
		//gpioServo(SERVO, 1500);
		//time_sleep(1);
		//gyroUpdate(gyro, x_rotation, y_rotation, 1);
	}
	
	gyroClose(gyro);
	gpioTerminate();
	return 0;
}
