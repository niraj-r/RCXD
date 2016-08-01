// Property of Niraj Raniga
// July 9, 2016

// pid.h: Proportional Integral (PID) control header

#ifndef PID_H_
#define PID_H_

typedef struct{
	float kp;			// proportion gain constant
	float ki;			// integral gain constant
	float kd;			// derivative gain constant

	float sp;			// theoretical value 
	float integral;		// integral sum
	float prev_err;		// derivative sum
} PID_t;

// pid_init()
// def: initializes the PID_t struct with the given gain constants
// par: *pid(pointer to struct), kp(proportional gain)
//		ki(integral gain), kd(derivative gain)
// out: void
void pid_init(PID_t *pid, float kp, float ki, float kd);

// pid_set()
// def: sets the set point (theoretical value) and zeros the integral,
//		derivative sums
// par: *pid(pointer to struct), sp(set point)
// out: void
void pid_set(PID_t *pid, float sp);

// pid_update()
// def: applies the PID control loop and calculates a new value for the
//		input needed to achieve the sp (theoretical value)
// par: *pid(pointer to struct), val(measured value), dt(time interval)
// out: total(new input value)
float pid_update(PID_t *pid, float val, float dt);

#endif
