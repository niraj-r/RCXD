// Property of Niraj Raniga
// July 9, 2016

// gyro.h: Interface for MPU6050 gyro and accelerometer

#ifndef GYRO_H_
#define GYRO_H_

#define PWR_MGMT_1 0x6b

// gyroInit()
// def: Initializes the i2c bus and wakes the gyro
// par: bus(i2c bus, usually 1), adr(i2c address from i2cdetect)
// out: gyro(unique handle for the device)
int gyroInit(int bus, int adr);

// gyroUpdate()
// def: Gets the current x and y rotation
// par: gyrp(unique handle), x_rot(y rotation output), 
//		y_rot(x rotation output), print(for debugging)
// out: void
void gyroUpdate(int gyro, double x_rot, double y_rot, int print);

// gyroCLose()
// def: Closes the i2c connection
// par: gyrp(unique handle)
// out: void
void gyroClose(int gyro);

#endif
