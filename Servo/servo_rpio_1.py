from RPIO import PWM
import time

servo= PWM.Servo()

Servo.set_servo(17,1200)
time.sleep(5)

servo.set_servo(17,2000)
time.sleep(5)

servo.stop_servo(17)
