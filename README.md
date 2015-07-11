# Quadie
Python based API for the use of a quadcopter developed on a Ubuntu embedded system ( Beaglebone Black).

## Libraries
Converted to python an open source project from the late Fabio Varesano, The project is an API
written in C++ for the arduino to control many different types of IMU's.
FreeIMU.cpp - A libre and easy to use orientation sensing library for Arduino
Copyright (C) 2011-2012 Fabio Varesano <fabio at varesano dot net>

This is the original repository [Fabio Varesano - FreeIMU](http://bazaar.launchpad.net/~fabio-varesano/freeimu/trunk/files "FreeIMU").

The libraries are targeted towards the GY-87 chip that contains a :


1. Barometer (BMP180)
2. Magnetometer(HMC58853)
3. Accelerometer/Gyroscope(MPU6050)

Thank you Fabio.

## PID
A library to create a PID controller 

##PING
A library to implement a PING sensor to the bottom of the quadcopter

Not fully implemented.

##PWM
A library to control the PWM's of our BB, thru an ESC(Qbrain)

##Main.py
Main program.
