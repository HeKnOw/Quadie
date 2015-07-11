#!/usr/bin/python

from Adafruit_I2C import Adafruit_I2C
from time import sleep

class MS5611:
    """
    Barometer
    """


    MS561101BA_ADDR_CSB_LOW =  0x77   #CBR=0 0x77 I2C address when CSB is connected to LOW(GND)

    #define MS561101BA_D1 0x40
    #define MS561101BA_D2 0x50
    MS561101BA_RESET = 0x1E

    #define MS561101BA_D1D2_SIZE 3

    #define MS561101BA_OSR_256 0x00
    #define MS561101BA_OSR_512 0x02
    #define MS561101BA_OSR_1024 0x04
    #define MS561101BA_OSR_2048 0x06
    #define MS561101BA_OSR_4096 0x08

    #define MS561101BA_PROM_BASE_ADDR 0xA2 // by adding ints from 0 to 6 we can read all the prom configuration values. 
    
    #define MS561101BA_PROM_REG_COUNT 6 // number of registers in the PROM
    #define MS561101BA_PROM_REG_SIZE 2 // size in bytes of a prom registry.

    ####################################################################################
    #										       #
    #				       Constructor				       #
    #										       #
    ####################################################################################
    def __init__(self):
        """
	Constructor for barometer
	"""
	self.ms5611 = Adafruit_I2C(self.MS561101BA_ADDR_CSB_LOW,1)
	self.reset()
	sleep(0.010)
	readPROM()

    def rawTemperature(self,OSR):
	"""
	Get raw temperature
	"""
	
    def reset(self):
	"""
	reset the readings 
	"""
	
	self.ms5611.writeList(MS561101BA_RESET,[])
	sleep(2/001.0)

    def readPROM(self):
	"""
	Read the PROM
	"""

	# ms5611 prom
	t=ms5611.readList(0xa2,2)
	c1=t[0]*256+t[1]
	t=ms5611.readList(0xa4,2)
	c2=t[0]*256+t[1]
	t=ms5611.readList(0xa6,2)
	c3=t[0]*256+t[1]
	t=ms5611.readList(0xa8,2)
	c4=t[0]*256+t[1]
	t=ms5611.readList(0xaa,2)
	c5=t[0]*256+t[1]
	t=ms5611.readList(0xac,2)
	c6=t[0]*256+t[1]

