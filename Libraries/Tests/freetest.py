#!/usr/bin/python

import numpy
from FreeIMU import FreeIMU as FreeIMU
from time import sleep
from calibrate import Calibrate as calibrate
from AccMagnCalib import AMCalib as AMC

#sensor = FreeIMU()
#calibrate = calibrate()
#calibrate.instructions()
#calibrate.loadSensor()
#try:
#    calibrate.openFiles()
#    while(True):
#	print "In loop"
#        calibrate.writeValues()
#except KeyboardInterrupt:
#    calibrate.closeFiles()

#acm = AMC()

#while(True):
#	ax,ay,az,gx,gy,gz,mx,my,mz = sensor.getValues()
#	print "----------Accelerometer------------------"
#	print " X = %f       Y = %f       Z = %f" %(ax,ay,az)
#	print "-----------Gyroscope---------------------"
#	print " X = %f	     Y = %f       Z = %f" %(gx,gy,gz)
#	print "-------------Magnetometer----------------"
#	print " X = %f	     Y = %f       Z = %f" %(mx,my,mz)
#	sleep(0.5)
	#yaw,pitch,roll = sensor.getYawPitchRoll()
	#print "--------------------------------------------------------"
	#print " Yaw = %f         Pitch = %f             Roll = %f  "%(yaw,pitch,roll)
	#sleep(0.5i)
	#baro = sensor.getBaroAlt();
	#print "Altitude: %f" %baro
