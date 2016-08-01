// Property of Niraj Raniga
// July 9, 2016

// pid.c: Proportional Integral (PID) control library

#include <stdio.h>
#include "pid.h"

// pid_init()
// def: initializes the PID_t struct with the given gain constants
// par: *pid(pointer to struct), kp(proportional gain)
//		ki(integral gain), kd(derivative gain)
// out: void
void pid_init(PID_t *pid, float kp, float ki, float kd) {
	pid->kp = kp;
	pid->ki = ki;
	pid->kd = kd;

	pid->sp = 0;
	pid->prev_err = 0;
	pid->integral = 0;
}

// pid_set()
// def: sets the set point (theoretical value) and zeros the integral,
//		derivative sums
// par: *pid(pointer to struct), sp(set point)
// out: void
void pid_set(PID_t *pid, float sp) {
	pid->sp = sp;
	pid->prev_err = 0;
	pid->integral = 0;
}

// pid_update()
// def: applies the PID control loop and calculates a new value for the
//		input needed to achieve the sp (theoretical value)
// par: *pid(pointer to struct), val(measured value), dt(time interval)
// out: total(new input value)
float pid_update(PID_t *pid, float val, float dt) {
	float deriv, error, total;

	error = pid->sp - val;
	pid->integral = pid->integral + (error * dt);
	deriv = (error - pid->prev_err) / dt;

	total = (error * pid->kp) + (pid->integral * pid->ki) + (deriv * pid->kd);

	pid->prev_err = error;

	#ifdef MIN_OUT
	if (total < MIN_OUT) ) return MIN_OUT;
	#elif MAX_OUT
	if (total > MAX_OUT) return MAX_OUT;
	#endif
	
	return total;
}
