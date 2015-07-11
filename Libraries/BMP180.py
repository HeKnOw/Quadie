#!/usr/bin/python

##############################################################################################
#   author: Christian Hinostroza
#   Code designed for the Engineering Design 2 Project of Florida Atlantic University 
#   term: Winter 2015
#   Date: 3-27-2015
#############################################################################################
from Adafruit_I2C import Adafruit_I2C
from time import sleep
import  math
import ctypes 


class BMP180:
    """
    Barometer for gy-87 
    """

    BMP085_ADDR = 0x77     #0x77 default I2C address
    #define BUFFER_SIZE                 3

    #define AUTO_UPDATE_TEMPERATURE     true    //default is true
	    #when true, temperature is measured everytime pressure is measured (Auto).
	    #when false, user chooses when to measure temperature (just call calcTrueTemperature()).
	    #used for dynamic measurement to increase sample rate (see BMP085 modes below).
	   
    #/* ---- Registers ---- */
    CAL_AC1    =       0xAA # // R   Calibration data (16 bits)
    CAL_AC2    =      0xAC  #// R   Calibration data (16 bits)
    CAL_AC3    =       0xAE # // R   Calibration data (16 bits)    
    CAL_AC4    =       0xB0 # // R   Calibration data (16 bits)
    CAL_AC5    =       0xB2 # // R   Calibration data (16 bits)
    CAL_AC6    =       0xB4 # // R   Calibration data (16 bits)
    CAL_B1     =       0xB6 # // R   Calibration data (16 bits)
    CAL_B2     =       0xB8 # // R   Calibration data (16 bits)
    CAL_MB     =       0xBA # // R   Calibration data (16 bits)
    CAL_MC     =       0xBC # // R   Calibration data (16 bits)
    CAL_MD     =       0xBE # // R   Calibration data (16 bits)
    CONTROL    =       0xF4 # // W   Control register 
    CONTROL_OUTPUT  =  0xF6 # // R   Output registers 0xF6=MSB, 0xF7=LSB, 0xF8=XLSB

    # unused registers
    #define SOFTRESET         0xE0
    #define VERSION           0xD1  // ML_VERSION  pos=0 len=4 msk=0F  AL_VERSION pos=4 len=4 msk=f0
    #define CHIPID            0xD0  // pos=0 mask=FF len=8
				    #BMP085_CHIP_ID=0x55

    #/************************************/
    #/*    REGISTERS PARAMETERS          */
    #/************************************/
    # BMP085 Modes
    #define MODE_ULTRA_LOW_POWER    0 //oversampling=0, internalsamples=1, maxconvtimepressure=4.5ms, avgcurrent=3uA, RMSnoise_hPA=0.06, RMSnoise_m=0.5
    MODE_STANDARD = 1 #oversampling=1, internalsamples=2, maxconvtimepressure=7.5ms, avgcurrent=5uA, RMSnoise_hPA=0.05, RMSnoise_m=0.4
    MODE_HIGHRES     =       2 #//oversampling=2, internalsamples=4, maxconvtimepressure=13.5ms, avgcurrent=7uA, RMSnoise_hPA=0.04, RMSnoise_m=0.3
    MODE_ULTRA_HIGHRES   =   3 #//oversampling=3, internalsamples=8, maxconvtimepressure=25.5ms, avgcurrent=12uA, RMSnoise_hPA=0.03, RMSnoise_m=0.25
    #"Sampling rate can be increased to 128 samples per second (standard mode) for
    #dynamic measurement.In this case it is sufficient to measure temperature only 
    #once per second and to use this value for all pressure measurements during period."
    #from BMP085 datasheet Rev1.2 page 10).
    #To use dynamic measurement set AUTO_UPDATE_TEMPERATURE to false and
    #call calcTrueTemperature() from your code. 
    #Control register
    READ_TEMPERATURE   =     0x2E 
    READ_PRESSURE      =     0x34 
    #Other
    #define MSLP                    101325          // Mean Sea Level Pressure = 1013.25 hPA (1hPa = 100Pa = 1mbar) 


 
    ######################################################################################
    #											 #
    #					Constructor					 #
    #											 #
    ######################################################################################
	

    def __init__(self):
	"""
	Constructor for BMP180
	"""
	self.bmp180 = Adafruit_I2C(self.BMP085_ADDR,1)
	self.pressure_waittime =[0 for i in range(4)]
	self.pressure_waittime[0] = 0.005 #These are maximum convertion times
	self.pressure_waittime[1] = 0.008 #It is possible to use pin EOC (End of Conversion)
	self.pressure_waittime[2] = 0.014 #to check if conversion is finished ( logic 1)
	self.pressure_waittime[3] = 0.026 #or running (logic 0) instead of waiting for conversion times.
	self.Cm_offset = 0.0
	self.Pa_offset = 0.0		#1hPa = 100Pa = 1mbar
	#self.Temp_offset = -1124.5
	self.getCalData()
	self.calcTrueTemperature()
	self.setMode(self.MODE_ULTRA_HIGHRES)
	#self.setLocalAbsAlt(0) #this if you wanna use the current altitude as reference
	self.setLocalPressure(101710)  #this if you wanna use the known atmospheric pressure

    def getCalData(self):
	"""
	Initialize cal data
	"""
	buff = [0 for i in range(2)]
	buff = self.bmp180.readList(self.CAL_AC1,2)
	self.ac1 = buff[0] << 8 | buff[1]
	buff = self.bmp180.readList(self.CAL_AC2,2)
	self.ac2 = buff[0] << 8 | buff[1]
	buff = self.bmp180.readList(self.CAL_AC3,2)
	self.ac3 = buff[0] << 8 | buff[1]
	buff = self.bmp180.readList(self.CAL_AC4,2)
	self.ac4 = ((ctypes.c_uint(buff[0]).value << 8) | ctypes.c_uint(buff[1]).value)
	#if(self.ac4 < 0):
	#    self.ac4 = self.ac4 * -1
	buff = self.bmp180.readList(self.CAL_AC5,2)
	self.ac5 =((ctypes.c_uint(buff[0]).value << 8) | ctypes.c_uint(buff[1]).value)
	#if(self.ac5 < 0):
	#    self.ac5 = self.ac5 * -1
	buff = self.bmp180.readList(self.CAL_AC6,2)
	self.ac6 = ((ctypes.c_uint(buff[0]).value << 8) | ctypes.c_uint(buff[1]).value)
	#if(self.ac6 < 0):
	#    self.ac6 = self.ac6 * -1
	#print "ac4 is %d, ac5 is %d and ac6 is %d" %(self.ac4,self.ac5,self.ac6)
	buff = self.bmp180.readList(self.CAL_B1,2)
	self.b1 = buff[0] << 8 | buff[1]
	buff = self.bmp180.readList(self.CAL_B2,2)
	self.b2 = buff[0] << 8 | buff[1]
	buff = self.bmp180.readList(self.CAL_MB,2)
	self.mb = buff[0] << 8 | buff[1]
	buff = self.bmp180.readList(self.CAL_MC,2)
	self.mc = buff[0] << 8 | buff[1]
	buff = self.bmp180.readList(self.CAL_MD,2)
	self.md = buff[0] << 8 | buff[1]

#	print "ac1:%d ac2:%d ac3:%d ac4:%d ac5:%d ac6:%d b1:%d b2:%d mb:%d mc:%d md:%d"%(self.ac1,self.ac2,self.ac3,self.ac4,self.ac5,self.ac6,self.b1,self.b2,self.mb,self.mc,self.md)

    def setMode(self,mode):
	"""
	Set Mode
	"""
	self.oss = mode

    def setLocalAbsAlt(self,cm):
	"""
	set Known Altitude as reference
	"""
	self.param_cm = cm
	tmp_Pa = self.getPressure() #calc pressure based on current altitude
	self.param_datum = tmp_Pa

    def getAltitude(self):
	"""
	Get the altitude
	In Centimeters + offset
	"""
	TruePressure = self.calcTruePressure()
	cm = 44330 * (1 - math.pow((TruePressure / self.param_datum), 0.1903)) + self.Cm_offset
	return cm


    def getPressure(self):
	"""
	Get pressure
	In Pa + offset
	"""
	truePressure = self.calcTruePressure()
	Pa = truePressure / math.pow((1 - self.param_cm / 44330), 5.255) + self.Pa_offset
	#note that BMP085 abs accuracy from 700... 1100hPa and 0..+65 C is +-100Pa
	return Pa

    def getTemperature(self):
	"""
	get Temperature
	In Centigrades
	"""
	self.calcTrueTemperature() #force b5 update
	temperature = ((self.b5 + 8) >> 4)
	return temperature # + self.Temp_offset

    def calcTrueTemperature(self):
	"""
	updates self.b5
	Calculate True Temperature
	calc temperature data b5 ( only needed if AUTO_UPDATE_TEMPERATURE is false)
	"""
	#read raw temperature
	self.bmp180.write8(self.CONTROL, self.READ_TEMPERATURE)
	sleep(0.0045)
	buff = [0 for i in range(2)]
	buff = self.bmp180.readList(self.CONTROL_OUTPUT,2)
	ut = (buff[0] << 8) | buff[1] #uncompensated temperature value

	#print "++++++++++++++++++++++++++++++++++++++++++++++++"
	#print "UT: %d"%ut
	#print "++++++++++++++++++++++++++++++++++++++++++++++++"
	#calculate temperature
	x1 = (ut - self.ac6) * (self.ac5 >> 15)
	x2 = (self.mc << 11) / (x1 + self.md)
	self.b5 = x1 + x2

    def calcTruePressure(self):
	"""
	Calculate true pressure
	Pressure in Pa
	returns pressure
	"""
	#if we want to automatic calculate temperature when we measure pressure
	#uncomment the following line
	#self.calcTrueTemperature()       #update b5

	self.bmp180.write8(self.CONTROL,self.READ_PRESSURE + (self.oss << 6))
	sleep(self.pressure_waittime[self.oss])
	buff = self.bmp180.readList(self.CONTROL_OUTPUT,3)

	#uncompensated pressure value
	up = (((buff[0] << 16) | (buff[1] << 8) | (buff[2])) >> (8 - self.oss))

	#calculate true pressure
	b6 = self.b5 - 4000
	x1 = (self.b2* (b6 * b6 >> 12)) >> 11
	x2 = self.ac2 * b6 >> 11
	x3 = x1 + x2
	tmp = self.ac1
	tmp = (tmp * 4 + x3) << self.oss
	b3 = (tmp + 2) >> 2
	x1 = self.ac3 * b6 >> 13
	x2 = (self.b1 * (b6 * b6 >> 12)) >> 16
	x3 = ((x1 + x2) + 2) >> 2
	b4 = (self.ac4 * ctypes.c_uint32((x3 + 32768)).value) >> 15 
	b7 = ctypes.c_uint32((up - b3)).value * (50000 >> self.oss)
	#print "b4 is = %d and b7 is = %d"%(b4,b7)
	p = (b7 << 1) / b4 if b7 < 0x80000000 else (b7 / b4) << 1
	#print p
	x1 = (p >> 8) * (p >> 8)
	x1 = (x1 * 3038) >> 16
	x2 = (-7357 * p) >> 16
	truePressure = p + ((x1 + x2 + 3791) >> 4)
	#print "true pressure = %d " %truePressure
	return truePressure
	
    def setLocalPressure(self,Pa):
	"""
	set local pressure
	set known barometric pressure as reference 
	"""
	self.param_datum = Pa
	tmp_alt = self.getAltitude()
	self.param_cm = tmp_alt

    def setAltOffset(self,cm):
	"""
	set altitude offset
	"""
	self.Cm_offset = cm

    def setPaOffset(self,Pa):
	"""
	set pressure offset
	"""
	self.Pa_offset = Pa

    def zeroCal(self,Pa,cm):
	"""
	Zero Calibrate output to a specific Pa/altitude
	"""
	self.setAltOffset(cm - self.param_cm)
	self.setPaOffset(Pa - self.param_datum)


    def _lshift(self,other):
	if hasattr(other,'value'):
	    other = other.value
	return c_ulong(self.value << other)

