import Adafruit_BBIO.PWM as PWM
import time

##############################################################################################
#   author: Alexander Sanchez
#   Code designed for the Engineering Design 2 Project of Florida Atlantic University 
#   term: Winter 2015
#   Date: 3-27-2015
#############################################################################################
class PWMMotor:
    motor1 = "P9_14"
    motor2 = "P9_22"
    motor3 = "P8_45"
    motor4 = "P9_42"
    def __init__(self):
        #initilize all PWM Beaglebone Outputs
        duty_min = 4.0
        duty_max = 10.0
        #initilize ESC and Motors
        PWM.start(self.motor1, duty_max, 55.0, 1)
        PWM.start(self.motor2, duty_max, 55.0, 1)
        PWM.start(self.motor3, duty_max, 55.0, 1)
        PWM.start(self.motor4, duty_max, 55.0, 1)
        time.sleep(.5)
        PWM.set_duty_cycle(self.motor1, duty_min)
        PWM.set_duty_cycle(self.motor2, duty_min)
        PWM.set_duty_cycle(self.motor3, duty_min)
        PWM.set_duty_cycle(self.motor4, duty_min)
    
    def setP(self,motor,percentage):
        
        #check what motor to work with
        if motor == "1":
            motor = self.motor1
        elif motor == "2":
            motor = self.motor2
        elif motor == "3":
            motor = self.motor3
        elif motor == "4":
            motor = self.motor4
        
        #convert from percentage to duty clycle between(4-10)
        setPWMCycle = (percentage * 0.06) + 4.0
        
        #set duty cycle
        PWM.set_duty_cycle(motor,setPWMCycle)
	
	return setPWMCycle
