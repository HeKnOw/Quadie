#!/usr/bin/python
##############################################################################################
#   author: Alexander Sanchez
#   Code designed for the Engineering Design 2 Project of Florida Atlantic University 
#   term: Winter 2015
#   Date: 3-27-2015
#############################################################################################
import Adafruit_BBIO.GPIO as GPIO
import time


class PingSensor: 
    def __init__(self):
        #initialize TRIG and ECHO
        self.TRIG = "P8_7"
        self.ECHO = "P8_9"
        GPIO.cleanup()
        GPIO.setup(self.TRIG, GPIO.OUT)
        GPIO.setup(self.ECHO, GPIO.IN)
        GPIO.output(self.TRIG, GPIO.LOW)
        self.ready = False
        self.pulse_start = 0.0

    def GetAlt(self):
        GPIO.output(self.TRIG, GPIO.LOW)
        GPIO.output(self.TRIG, GPIO.HIGH)
        time.sleep(0.00001)
        GPIO.output(self.TRIG, GPIO.LOW)
        while GPIO.input(self.ECHO)== 0:
            pulse_start = time.time()
        while GPIO.input(self.ECHO)== 1:
            pulse_end = time.time()
        pulse_duration = pulse_end - pulse_start
        distance= (pulse_duration/2) /29.1;
        distance = pulse_duration * 17150
        distance = distance*0.39370
    
        return distance
   
        

