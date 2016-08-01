from RPIO import PWM
import time

servo = PWM.Servo()

#Set servo on GPIO17 to 1200us (1.2ms)
servo.set_servo(17,1200)
time.sleep(2)

#Set servo on GPIO17 to 2000us (2.0ms)
servo.set_servo(17, 2000)
time.sleep(2)

#Clear servo on GPIO17
servo.stop_servo(17)
