#!/usr/bin/python

from Adafruit_I2C import Adafruit_I2C
from time import sleep
import math

##############################################################################################
#   author: Christian Hinostroza
#   Code designed for the Engineering Design 2 Project of Florida Atlantic University 
#   term: Winter 2015
#   Date: 3-27-2015
#############################################################################################



###########################################################################
#									  #
#		 	        Constructor				  #
#									  #
###########################################################################


class HMC5883:
    """
    The HMC5883 is a magnetometer
    """

    HMC58X3_ADDR = 0x1E
    HMC_POS_BIAS = 1
    HMC_NEG_BIAS = 2

    HMC58X3_R_CONFA = 0
    HMC58X3_R_CONFB = 1
    HMC58X3_R_MODE = 2
    HMC58X3_R_XM = 3
    HMC58X3_R_XL = 4

    HMC58X3_R_YM = 7  #!< Register address for YM.
    HMC58X3_R_YL = 8  #!< Register address for YL.
    HMC58X3_R_ZM = 5  #!< Register address for ZM.
    HMC58X3_R_ZL = 6  #!< Register address for ZL.

    HMC58X3_X_SELF_TEST_GAUSS =  +1.16  #//!< X axis level when bias current is applied.
    HMC58X3_Y_SELF_TEST_GAUSS = HMC58X3_X_SELF_TEST_GAUSS #//!< Y axis level when bias current is applied.
    HMC58X3_Z_SELF_TEST_GAUSS = +1.08 #//!< Y axis level when bias current is applied.

    SELF_TEST_LOW_LIMIT =  (243.0/390.0)   #//!< Low limit when gain is 5.
    SELF_TEST_HIGH_LIMIT = (575.0/390.0)   #//!< High limit when gain is 5.

    HMC58X3_R_STATUS = 9
    HMC58X3_R_IDA = 10
    HMC58X3_R_IDB = 11
    HMC58X3_R_IDC = 12


    counts_per_milligauss = [0 for i in range(8)]
    counts_per_milligauss[0] = 1370
    counts_per_milligauss[1] = 1090
    counts_per_milligauss[2] = 820
    counts_per_milligauss[3] = 660
    counts_per_milligauss[4] = 440
    counts_per_milligauss[5] = 390
    counts_per_milligauss[6] = 330
    counts_per_milligauss[7] = 230
    
    
    def __init__(self):
        """
        Constructor for Magentometer
	"""
	self.hmc5883 = Adafruit_I2C(self.HMC58X3_ADDR,1)
	self.x_scale = 1.0
	self.y_scale = 1.0
	self.z_scale = 1.0

    def init(self,setMode):
	"""
	Initialize function 
	"""
        #You need to wait at least 5ms after power on to initialize
	sleep(0.005)
	if(setMode):
	    self.setMode(0)


	self.writeReg(self.HMC58X3_R_CONFA, 0x70)
	self.writeReg(self.HMC58X3_R_CONFB, 0xA0)
	self.writeReg(self.HMC58X3_R_MODE, 0x00)


    def setMode(self,mode):
        """
	set mode of magnetometer
	"""
	if(mode > 2):
	    return 
    	
    	self.writeReg(self.HMC58X3_R_MODE,mode)
	sleep(0.1)

    def calibrate(self,gain,n_samples):
	"""
	Calibrate using the self test operation 
	Average the values using bias mode to obtain the scale factors.
	return: returns false if any of the following occurs:
		invalid input parameters. (gain >7 or n_samples = 0)
		Id registers are wrong for the compiled device. Unfortunately, we cant distinguish between HMC5883l and 5843
		calibration saturates during the positive or neagative bias on any of the readings
		Readings are outside of the expected range for bias current
	"""
	bret = True
	xyz_total = [0 for i in range(3)]
	if((7> gain and 0 < n_samples)):
	    self.writeReg(self.HMC58X3_R_CONFA, 0x010 + self.HMC_POS_BIAS)
	    #Note that the  very first measurement after a gain change maintains the same gain as the previous setting.The new gain setting is effective from the second measurement and on.
	    self.setGain(gain)
	    self.setMode(1)
	    xyz = [0 for i in range(3)] #change to single measurement mode
	    xyz[0],xyz[1],xyz[2] = self.getRaw() #get the raw values and ignore since this reading may use previous gain

	    for i in range(n_samples):
		self.setMode(1)
		xyz[0],xyz[1],xyz[2] = self.getRaw()
		#Since the measurements are noisy they should be averaged rather than taking the max

		xyz_total[0] += xyz[0]
		xyz_total[1] += xyz[1]
		xyz_total[2] += xyz[2]
		#detect saturation

		if(-(1 << 12) >= min(xyz[0],min(xyz[1],xyz[2]))):
		    print "HMC58x3 Self test saturated. Increase range."
		    bret = False
		    break
	    #apply the negative bias (same gain)

	    self.writeReg(self.HMC58X3_R_CONFA, 0x010 + self.HMC_NEG_BIAS)
	    for i in range(n_samples):
		self.setMode(1)
		xyz[0],xyz[1],xyz[2] = self.getRaw()


		#Since the measurements are noisy they should be averaged rather than taking the max

		xyz_total[0] -= xyz[0]
		xyz_total[1] -= xyz[1]
		xyz_total[2] -= xyz[2]
		#detect saturation

		if(-(1 << 12) >= min(xyz[0],min(xyz[1],xyz[2]))):
		    print "HMC58x3 Self test saturated. Increase range."
		    bret = False
		    break
		#compare the values against the expected self test bias gauss
		#notice same limits are applied to all axis

		low_limit = self.SELF_TEST_LOW_LIMIT * self.counts_per_milligauss[gain] * 2* n_samples
		high_limit = self.SELF_TEST_HIGH_LIMIT * self.counts_per_milligauss[gain] * 2 * n_samples
		#print "running self test of magnetometer..."
		#for debug purposes
		#print "Low limit is : %f"%low_limit
		#print "High limit is : %f"%high_limit
		#print "bret is = %s" %bret
		#print "xyz limits should be between both low-limit and high limit"
		#print "xyz[0] = %f  xyz[1]  = %f   xyz[2]  = %f"%(xyz_total[0],xyz_total[1],xyz_total[2])
	    if((bret == True) and
		(low_limit <= xyz_total[0]) and (high_limit >= xyz_total[0]) and
		(low_limit <= xyz_total[1]) and (high_limit >= xyz_total[1]) and 
		(low_limit <= xyz_total[2]) and (high_limit >= xyz_total[2])):
		#Succesful calibration
		#Normalize the scale factors so all axis return the same range of values for the bias field
		#Factor of 2 is from summation of total of n_samples from both positive and negative bias
		self.x_scale = (self.counts_per_milligauss[gain] * (self.HMC58X3_X_SELF_TEST_GAUSS * 2)) / (xyz_total[0]/ n_samples)
		self.y_scale = (self.counts_per_milligauss[gain] * (self.HMC58X3_Y_SELF_TEST_GAUSS * 2)) / (xyz_total[1] / n_samples)
		self.z_scale = (self.counts_per_milligauss[gain] * (self.HMC58X3_Z_SELF_TEST_GAUSS * 2)) / (xyz_total[2] / n_samples)
	        print "self test ran succesfully, storing scale values"
	    else:
		print"HMC5883 self test out of range"
		bret = False

	    self.writeReg(self.HMC58X3_R_CONFA, 0x010)
        else:
	    print "HMC5883 bad parameters"
	    bret = False
        return bret





    def calibrateSimple(self,gain):
	"""
	Calibrate which has a few weaknesses
	1. Uses wrong gain for first reading
	2. Uses max instead ofo max of average when normalizing the axis to one another
	3.Doesn't use neg bias. (possible improvement in measurement)
	"""
	self.writeReg(self.HMC58X3_R_CONFA, 0x010 + self.HMC_POS_BIAS) # Reg A DOR=0x010 + MS1, MS0 set to pos bias
	self.setGain(gain)
	mx = 0
	my = 0
	mz = 0
	t = 10

	for i in range(t):
	    self.setMode(1)
	    x,y,z = self.getValues()
	    if(x > mx):
	        mx = x
	    if(y > my):
		my = y
	    if(z > mz):
		mz = z


        max = 0
	if(mx > max):
            max = mx
	if(my > max):
	    max = my
	if(mz > max):
	    max = mz

	self.x_scale = max / mx
	self.y_scale = max / my
	self.z_scale = max / mz

	self.writeReg(self.HMC58X3_R_CONFA, 0x010) #set regA/DOR back to default
	
    def setDOR(self,DOR):
        """
        set data output rate
        0-6 , 4 defualt, normal operation assumed
        """
        if(DOR>6):
	    return
        self.writeReg(self.HMC58X3_R_CONFA,DOR<<2)

    def setGain(self,gain):
	"""
	0-7 , 1 default
	"""
	if(gain > 7):
	    return 
        self.writeReg(self.HMC58X3_R_CONFB,gain<<5)

    def writeReg(self,reg,val):
	"""
	write register
	"""
	self.hmc5883.write8(reg,val)	

    def getValues(self):
	"""
	Get values from the magnetometer
	"""
	xr,yr,zr = self.getRaw()
	x = xr / self.x_scale
	y = yr / self.y_scale
	z = zr / self.z_scale

	x = x + 0.5
	y = y + 0.5
	z = z + 0.5

	return x,y,z

    def getRaw(self):
        """
 	Get Raw Values from the magnetoemeter
	"""
	#wait hmc5883 to be ready
	#while not self.hmc5883.readU8(0x09)&0x01 == 1: pass
	#read hmc5883 magnetometer in gauss
	magx = (self.hmc5883.readS8(self.HMC58X3_R_XM) << 8) | self.hmc5883.readU8(self.HMC58X3_R_XL)
	magz = (self.hmc5883.readS8(self.HMC58X3_R_ZM) << 8) | self.hmc5883.readU8(self.HMC58X3_R_ZL)
	magy = (self.hmc5883.readS8(self.HMC58X3_R_YM) << 8) | self.hmc5883.readU8(self.HMC58X3_R_YL)

	return magx,magy,magz


    def getID(self):
	"""
	Retrieve the value of the three ID registers 
	NOTE: both HMC5843 and HMC5883L have the same 'H43' identification register values.
	returns the three id register values
	"""
	id = [0 for i in range(3)]
	id =self.hmc5883.readList(self.HMC58X3_R_IDA,3)
	return id[0],id[1],id[2]





