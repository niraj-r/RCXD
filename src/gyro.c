// Property of Niraj Raniga
// July 9, 2016

// gyro.c: Interface for MPU6050 gyro and accelerometer

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include <pigpio.h>
#include "gyro.h"

#ifndef M_PI
	#define M_PI 3.14159265359
#endif

// Function Declarations
int readWord(int gyro, int adr);
double readWord2C(int gyro, int adr);
double getDist(double x, double y);
double getRotationX(double x, double y, double z);
double getRotationY(double x, double y, double z);
void i2cOpenCheck(int status);

/******Private Helper Functions******/

// readWord()
// def: Reads a word from the i2c device
// par: gyro(unique handle), adr(i2c address from i2cdetect)
// out: val(the word read from the device)
int readWord(int gyro, int adr){
    int high = i2cReadByteData(gyro, adr);
    int low = i2cReadByteData(gyro, adr+1);
    int val = (high << 8) + low;
    return val;
}

// readWord2C()
// def: Adjusts the word read by readWord()
// par: gyro(unique handle), adr(i2c address from i2cdetect)
// out: val(adjusted word)
double readWord2C(int gyro, int adr) {
	double val = (double)readWord(gyro, adr);
	if (val >= 0x8000) return -((65535 - val) + 1);
	else return val;
}

// getDist()
// def: Pythagorean theoream to calculate distance
// par: x(distance value), y(distance value)
// out: val(distance inbetween x and y)
double getDist(double x, double y){
	return sqrt((x*x) + (y*y));
}

// getRotationX()
// def: Gets the rotation around the x axis
// par: x(distance value), y(distance value), z(distance value)
// out: val(angle of rotation around x)
double getRotationX(double x, double y, double z){
	double radian = atan2(y, getDist(x,z));
	return -(radian * (180.0 / M_PI));
}

// getRotationY()
// def: Gets the rotation around the y axis
// par: x(distance value), y(distance value), z(distance value)
// out: val(angle of rotation around y)
double getRotationY(double x, double y, double z){
	double radian = atan2(x, getDist(y,z));
	return -(radian * (180.0 / M_PI));
}

// i2cOpenCheck()
// def: Checks and prints errors when opening i2c device
// par: status(value returns from i2cOpen())
// out: void
void i2cOpenCheck(int status){
	if(status == PI_BAD_I2C_BUS) printf("Bad I2C Bus");
	else if(status == PI_BAD_I2C_ADDR) printf("Bad I2C Addr");
	else if(status == PI_BAD_FLAGS) printf("Bad Flags");
	else if(status == PI_NO_HANDLE) printf("No Handle");
	else if(status == PI_I2C_OPEN_FAILED) printf("I2C Open Failed");
}

/******Header Functions******/

// gyroInit()
// def: Initializes the i2c bus and wakes the gyro
// par: bus(i2c bus, usually 1), adr(i2c address from i2cdetect)
// out: gyro(unique handle for the device)
int gyroInit(int bus, int adr){
	
	// Initialize
	int gyro = i2cOpen(bus, adr, 0);
	i2cOpenCheck(gyro);
		
	// Wakes the gyro up
	i2cWriteWordData(gyro, PWR_MGMT_1, 0);
	
	return gyro;
}

// gyroUpdate()
// def: Gets the current x and y rotation
// par: gyrp(unique handle), x_rot(y rotation output), 
//		y_rot(x rotation output), print(for debugging)
// out: void
void gyroUpdate(int gyro, double x_rot, double y_rot, int print) {
	
	// Instantaneous angle change
	double gyro_xout = readWord2C(gyro, 0x43);
	double gyro_yout = readWord2C(gyro, 0x45);
	double gyro_zout = readWord2C(gyro, 0x47);
		
	double gyro_xout_scaled = gyro_xout / 131;
	double gyro_yout_scaled = gyro_yout / 131;
	double gyro_zout_scaled = gyro_zout / 131;
	
	// Instantaneous acceleration change
	double accel_xout = readWord2C(gyro, 0x3b);
	double accel_yout = readWord2C(gyro, 0x3d);
	double accel_zout = readWord2C(gyro, 0x3f);

	double accel_xout_scaled = accel_xout / 16384.0;
	double accel_yout_scaled = accel_yout / 16384.0;
	double accel_zout_scaled = accel_zout / 16384.0;

	x_rot = getRotationX(accel_xout_scaled, accel_yout_scaled, accel_zout_scaled);
	y_rot = getRotationY(accel_xout_scaled, accel_yout_scaled, accel_zout_scaled);
	
	if(print){
		printf("\n----------GYRO DATA\n");
	
		printf("gyro_xout: %f, scaled: %f\n", gyro_xout, gyro_xout_scaled);
		printf("gyro_yout: %f, scaled: %f\n", gyro_yout, gyro_yout_scaled);
		printf("gyro_zout: %f, scaled: %f\n", gyro_zout, gyro_zout_scaled);

		printf("\n----------ACCELEROMETER DATA\n");

		printf("accel_xout: %f, scaled: %f\n", accel_xout, accel_xout_scaled);
		printf("accel_yout: %f, scaled: %f\n", accel_yout, accel_yout_scaled);
		printf("accel_zout: %f, scaled: %f\n\n", accel_zout, accel_zout_scaled);

		printf("x rotation: %f\n", x_rot);
		printf("y rotation: %f\n", y_rot);
	}
}

// gyroCLose()
// def: Closes the i2c connection
// par: gyrp(unique handle)
// out: void
void gyroClose(int gyro){
	i2cClose(gyro);
}
