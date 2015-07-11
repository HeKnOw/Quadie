#!/usr/bin/python

from FreeIMU import FreeIMU as FreeIMU
from time import sleep

sensor = FreeIMU()


while(True):
    ax,ay,az,gx,gy,gz,mx,my,mz = sensor.getValues()
    #print "-----------------Acceleration------------------------"
    #print "AX = %f m/s^2  AY = %f m/s^2  AZ = %f m/s^2" %(ax,ay,az)
    print"--------------------Gyroscope--------------------------"
    print "GX = %f deg/sec  GY = %f deg/sec   GZ = %f deg/sec" %(gx,gy,gz)
    #print"------------------Magnetometer-------------------------"
    #print "MX = %f deg/sec  MY = %f deg/sec    MZ = %f deg/sec" %(mx,my,mz)
    #alt = sensor.baro.getAltitude()

    #print "-----------------------Altitude------------------------"
    #print "                       meters = %f                     "%alt
    #x,y,z = sensor.getEuler()

    #print"---------------------Get Eulers--------------------------"
    #print"              x = %f o   y = %f o   z = %f o              "%(x,y,z)
    #sleep(0.1)
    
    #x,y,z = sensor.getYawPitchRoll()
    #mx,my,mz = sensor.magn.getValues()
    #print "--------------------Magnetic fields------------------"
    #print "  x = %f          y = %f         z =%f   "%(mx,my,mz)
    #print "------------------Get Yaw Pitch Roll-----------------"
    #print "   yaw = %d o     pitch = %d o    roll = %d o "%(x,y,z)
