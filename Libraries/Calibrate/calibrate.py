#!/usr/bin/python

from FreeIMU import FreeIMU

"""
calibrate.py - Guides user through FreeIMU accelerometer and magnetometer calibration

Copyright (C) 2012 Fabio Varesano <fvaresano@yahoo.it>

Development of this code has been supported by the Department of Computer Science,
Universita' degli Studi di Torino, Italy within the Piemonte Project
http://www.piemonte.di.unito.it/


This program is free software: you can redistribute it and/or modify
it under the terms of the version 3 GNU General Public License as
published by the Free Software Foundation.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

"""

class Calibrate:
    """
    Class to calibrate the sensors
	
    """

    def __init__(self):
        """
	Constructor of calibration
	"""
	self.count = 30
	self.freeIMU = FreeIMU()
	self.buff = [0.0 for i in range(11)]
	self.acc_file = 'acc.txt'
	self.magn_file = 'magn.txt'
	self.tot_readings = 0
	self.filesopened = False
	self.a_f = open(self.acc_file,'r+') 
	self.m_f = open(self.magn_file,'r+')
	    
    def instructions(self):
	"""
	Display instructions for the calibration
	"""
	print "\n\nWelcome to the library calibration routine!"

	print "loading the sensor"
	print "..."

    def loadSensor(self):
	self.accgyro = self.freeIMU.accgyro
	self.magn = self.freeIMU.magn
	print "sensors loaded"

    def openFiles(self):
	"""
	Sampling data 
	"""
        self.a_f = open(self.acc_file,'r+') 
	self.m_f = open(self.magn_file,'r+')
	print "files open\n"
	self.filesopened = True

    def writeValues(self):
        """
	Write values into files previously open
	"""
	if self.filesopened:
            for j in range(self.count):
                self.buff[0],self.buff[1],self.buff[2],self.buff[3],self.buff[4],self.buff[5],self.buff[6],self.buff[7],self.buff[8],self.buff[9],self.buff[10] = self.freeIMU.getRawValues()
		    
		#log accelerometer values
		readings_line = "%f %f %f\n" % (self.buff[0], self.buff[1], self.buff[2])
		self.a_f.write(readings_line)

		#log magnetometer values
		readings_line = "%f %f %f\n" % (self.buff[6], self.buff[7], self.buff[8])
		self.m_f.write(readings_line)

		self.tot_readings = self.tot_readings + 1
		if(self.tot_readings % 200 == 0):
		    print "%d readings logged. Hit CTRL + C to interrupt." % (self.tot_readings)
	else:
	    print "file hasn't been opened\n"

    def closeFiles(self):
        self.a_f.close()
	self.m_f.close()
	print "\n%d values logged to %s and %s" % (self.tot_readings, self.acc_file, self.magn_file)
	self.filesopened = False




                







