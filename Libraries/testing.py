#!/usr/bin/python

from MPU6050 import MPU6050 as MPU6050
from HMC5883 import HMC5883 as HMC5883
from BMP180 import BMP180 as BMP180
from sys import stdout
from time import sleep
import math

baro = BMP180()
baro.zeroCal(101712,0)
magn = HMC5883()
mpu = MPU6050()
mpu.setI2CMasterModeEnabled(False)
mpu.setI2CBypassEnabled(True)
magn.calibrate(1,32)
magn.setMode(0)
values = [0 for i in range(6)]
v = [0 for i in range(6)]
while(True):


#	v[0],v[1],v[2],v[3],v[4],v[5] = mpu.getMotion6()
#	print "AX = %f m/s^2  AY = %f m/s^2  AZ = %f m/s^2" %(v[0],v[1],v[2])
#	print"-------------------------------------------"
#	print "GX = %f deg/sec  GY = %f deg/sec   GZ = %f deg/sec" %(v[3],v[4],v[5])
#	print"-------------------------------------------"
#	sleep(0.5)
#	values[0],values[1],values[2],values[3],values[4],values[5] = mpu.getMotion6original()
#	print "original AX = %f m/s^2  original AY = %f m/s^2 original AZ = %f m/s^2" %(values[0],values[1],values[2])
#	print"-------------------------------------------"
#	print "original GX = %f deg/sec original GY = %f deg/sec  original GZ = %f deg/sec" %(values[3],values[4],values[5])
#	print"-------------------------------------------"
	sleep(0.5)
	#magn.init(0)	
	#values[0],values[1],values[2] = magn.getRaw()
	#print "X = %f; Y = %f; Z = %f"%(values[0],values[1],values[2])
	#print"-------------------------------------------------------"
	#sleep(0.05)
	#heading = math.atan2(values[1],values[0])
	#if(heading < 0):
	#	heading += 2 * math.pi
	#heading = heading * (180/math.pi)
	#print "Heading : %f" %heading
	#cm = baro.Cm_offset
	#pao = baro.Pa_offset
	pa = baro.getPressure()
	tmp = baro.getTemperature()
	alt = baro.getAltitude()
	print "Pressure = %d Altitude = %d Temperature = %d " %(pa,alt,tmp)
	print "------------------------------------------------------------------------------"
