#!/usr/bin/python

from MPU6050 import MPU6050 as MPU6050
from time import sleep

mpu = MPU6050()


while(True):

	ax,ay,az,gx,gy,gz = mpu.getMotion6()
	print "---------------------Accelerometer-------------------------------"
	print " X = %f      		 Y = %f     			 Z = %f" %(ax,ay,az)
	print "----------------------Gyroscope----------------------------------"
	print " X = %f			 Y = %f				 Z = %f" %(gx,gy,gz)
	sleep(0.5)
