#!/usr/bin/python

##############################################################################################
#   author: Christian Hinostroza
#   Code designed for the Engineering Design 2 Project of Florida Atlantic University 
#   term: Winter 2015
#   Date: 3-27-2015
#############################################################################################
import math
from time import sleep
from time import time
from MPU6050 import MPU6050 as MPU6050
from HMC5883 import HMC5883 as HMC5883
from BMP180 import BMP180 as BMP180


class FreeIMU:

    #get these values from a calibration program
    acc_off_x = 50.8605
    acc_off_y = -24.2159
    acc_off_z = -647.1851
    acc_scale_x = 15910
    acc_scale_y = 16377
    acc_scale_z = 17219
    magn_off_x = 18.9752
    magn_off_y = 68.3174
    magn_off_z =  -121.4185
    magn_scale_x = 546.9255
    magn_scale_y = 503.866
    magn_scale_z = 435.3899
    sea_press = 1013.25
    altitude = 0
    twoKp = 2 * 0.5 #2 times proportional gain
    twoKi = 2 * 0.1 #2 times integral gain  
    integralFBx = 0.0
    integralFBy = 0.0
    integralFBz = 0.0
    sampleFreq = 0.0
    lastUpdate = 0.0
    now = 0.0

    ######################################################################
    #           							 #
    #                           CONSTRUCTOR				 #
    #									 #
    ######################################################################
    def __init__(self):
        
        self.startTime = time() * 1000000.0
        self.lastUpdate = 0.0
        self.sampleFreq = 1.0
        #define MPU6050
        self.accgyro = MPU6050()
        #define HMC5883L
        self.magn = HMC5883()
        #define MS5611
        self.baro = BMP180()

        #initialize quarternion

        self.q0 = 1.0
        self.q1 = 0.0
        self.q2 = 0.0
        self.q3 = 0.0

        #initialize accgyro
        self.accgyro.setI2CMasterModeEnabled(0)
        self.accgyro.setI2CBypassEnabled(1)
        self.accgyro.setFullScaleGyroRange(0x03)
        sleep(0.005)

        self.magn.init(False) #initialize magn
        #calibrate it thru self test, not recommended to change gain after calibration
        self.magn.calibrate(1,64) #use gain 1=default
        self.magn.setMode(0)
        sleep(0.010)
        self.magn.setDOR(4)

        #initialize baro to a specific pressure and altitude 
        self.baro.zeroCal(self.sea_press,self.altitude)

        #zeroGyro
        self.zeroGyro()

        #load calibration
        #callLoad()
         
    #######################################################################################
    #										          #
    #                                Get Raw Values					  #
    #											  #
    #######################################################################################
    def getRawValues(self):
        raw_values = [0 for i in range(11)]
        raw_values[0],raw_values[1],raw_values[2],raw_values[3],raw_values[4],raw_values[5] =  self.accgyro.getMotion6()
        raw_values[6],raw_values[7],raw_values[8] = self.magn.getValues()
        raw_values[9] = self.baro.getTemperature()
        raw_values[10] = self.baro.getPressure()

        return raw_values[0],raw_values[1],raw_values[2],raw_values[3],raw_values[4],raw_values[5],raw_values[6],raw_values[7],raw_values[8],raw_values[9],raw_values[10]

    ######################################################################################
    #											 #
    #                               Get Values 						 #
    #											 #
    ######################################################################################
    def getValues(self):

        accgyroval = [0 for i in range(6)]
        accgyroval[0],accgyroval[1],accgyroval[2],accgyroval[3],accgyroval[4],accgyroval[5] = self.accgyro.getMotion6()

        #remove offsets from the gyroscope
        accgyroval[3] = accgyroval[3] - self.gyro_off_x
        accgyroval[4] = accgyroval[4] - self.gyro_off_y
        accgyroval[5] = accgyroval[5] - self.gyro_off_z

        values = [0 for i in range(9)]
        for i in range(6):
            if i < 3:
                values[i] = accgyroval[i]*1.0
            else:
                values[i] = accgyroval[i] / 16.4


        #remove offsets and scale accelerometer (calibration)
        values[0] = (values[0] - self.acc_off_x) / self.acc_scale_x
        values[1] = (values[1] - self.acc_off_y) / self.acc_scale_y
        values[2] = (values[2] - self.acc_off_z) / self.acc_scale_z
         
        #HMC883L
        values[6],values[7],values[8] = self.magn.getValues()

        #calibration
        values[6] = (values[6] - self.magn_off_x) / self.magn_scale_x
        values[7] = (values[7] - self.magn_off_y) / self.magn_scale_y
        values[8] = (values[8] - self.magn_off_z) / self.magn_scale_z
        
        return values[0],values[1],values[2],values[3],values[4],values[5],values[6],values[7],values[8]

    ########################################################################################
    #											   #
    #                                  Zero Gyro					   #
    #											   #
    ########################################################################################
    def zeroGyro(self):
    	"computes gyros offsets"
        totSamples = 3
        tmpOffsets = [0 for i in range(3)]
        raw = [0 for i in range(11)]

        for i in range(totSamples):
            raw[0],raw[1],raw[2],raw[3],raw[4],raw[5],raw[6],raw[7],raw[8],raw[9],raw[10] = self.getRawValues()
            tmpOffsets[0] += raw[3]
            tmpOffsets[1] += raw[4]
            tmpOffsets[2] += raw[5]


        self.gyro_off_x = tmpOffsets[0] / totSamples
        self.gyro_off_y = tmpOffsets[1] / totSamples
        self.gyro_off_z = tmpOffsets[2] / totSamples
	
    #######################################################################################
    #											  #
    #                                     AHRS upate                                      #
    #											  #
    #######################################################################################
    def AHRSupdate(self,gx,gy,gz,ax,ay,az,mx,my,mz):
        """
        Quarternion implementation of the 'DCM filter' [mayhony] 
        Incorporates the magnetic distortion compensation algorithms from Madgwick's filter
        which eliminates the need for a reference direction of flux (bx bz)
        to be predefined and limits the effect of magnetic distortions to yaw axis only

        """
        q0q0 = self.q0*self.q0    
        q0q1 = self.q0*self.q1
        q0q2 = self.q0*self.q2
        q0q3 = self.q0*self.q3
        q1q1 = self.q1*self.q1
        q1q2 = self.q1*self.q2
        q1q3 = self.q1*self.q3
        q2q2 = self.q2*self.q2
        q2q3 = self.q2*self.q3
        q3q3 = self.q3*self.q3
        halfex = 0.0
        halfey = 0.0 
        halfez = 0.0
        gx = gx * math.pi / 180
        gy = gy * math.pi / 180
        gz = gz * math.pi / 180



        #Use magnetometer measurement only when valid (avoids NaN in magnetometer normalization)
        if mx != 0.0000 and my != 0.0000 and mz != 0.0000:
            #Normalize magnetometer measurement
            recipNorm = (mx * mx + my * my + mz * mz)**-0.5
            mx *= recipNorm
            my *= recipNorm
            mz *= recipNorm

            #Reference direction of Earth's magnetic field
            hx = 2.0 * (mx * (0.5 - q2q2 - q3q3) + my* (q1q2 - q0q3) + mz * (q1q3 + q0q2))
            hy = 2.0 * (mx * (q1q2 + q0q3) + my * (0.5 - q1q1 - q3q3) + mz * (q2q3 - q0q1))
            bx = math.sqrt(hx * hx + hy * hy)
            bz = 2.0 * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5 - q1q1 - q2q2))

            #Estimated direction of magnetic field
            halfwx = bx * (0.5 - q2q2 - q3q3) + bz * (q1q3 - q0q2)
            halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3)
            halfwz = bx * (q0q2 + q1q3) + bz * (0.5 - q1q1 - q2q2)


            #Error is sum of cross product between estimated direction and measured direction of field vectors
            halfex = (my * halfwz - mz * halfwy)
            halfey = (mz * halfwx - mx * halfwz)
            halfez = (mx * halfwy - my * halfwx)

            #Compute feedback only if accelerometer measurements valid (avoids NaN in accelerometer normalization)		
        if ax != 0.0 and ay != 0.0 and az != 0.0:
            #Normalise accelerometer measurement
            recipNorm = (ax * ax + ay * ay + az * az)**-0.5
            ax *= recipNorm
            ay *= recipNorm
            az *= recipNorm

            #Estimated direction of gravity
            halfvx = q1q3 - q0q2
            halfvy = q0q1 + q2q3
            halfvz = q0q0 - 0.5 + q3q3

            #Error is sum cross product between estimated direction and measured direction of field vectors
            halfex += (ay * halfvz - az * halfvy)
            halfey += (az * halfvx - ax * halfvz)
            halfez += (ax * halfvy - ay * halfvx)

            #apply feedback only when valid data has been gathered from the accelerometer or magnetometer
        if halfex != 0.0 and halfey != 0.0 and halfez != 0.0:
            #compute and apply integral feedback if enabled
            #in a post on DIY Drones
            #(http://diydrones.com/forum/topics/freeimu-firmware-on-arduimu?commentId=705844%3AComment%3A1010680)
            #by Seb Madgwick in 2011 he suggested the following for tuning Kp and Ki:
            #Leave Ki as 0 and start with a Kp value of 5.
            #You will want to reduce your Kp value from this by 10 or even 100 times when tuning.
            #The lowest value of Kp you can use is dependent on:
            #gyroscope bias calibration errors, and gyroscope sensitivity calibration errors 
            #and expected angular dynamics of application (coupled characteristics).
            if self.twoKi > 0.0:
                self.integralFBx += self.twoKi * halfex * (1.0 / self.sampleFreq)
                self.integralFBy += self.twoKi * halfey * (1.0 / self.sampleFreq)
                self.integralFBz += self.twoKi * halfez * (1.0 / self.sampleFreq)
                gx += self.integralFBx #apply integral feedback
                gy += self.integralFBy
                gz += self.integralFBz

            else:
                self.integralFBx = 0.0 #prevent integral windup
                self.integralFBy = 0.0
                self.integralFBz = 0.0
            #apply proportional feedback
            gx += self.twoKp * halfex
            gy += self.twoKp * halfey
            gz += self.twoKp * halfez

        #integrate rate of change of quarternion
        gx *= (0.5 * (1.0 / self.sampleFreq)) #pre-multiply common factors
        gy *= (0.5 * (1.0 / self.sampleFreq))
        gz *= (0.5 * (1.0 / self.sampleFreq))
        qa = self.q0
        qb = self.q1
        qc = self.q2
        self.q0 += (-qb * gx - qc * gy - self.q3 * gz)
        self.q1 += (qa * gx + qc * gz - self.q3 * gy)
        self.q2 += (qa * gy - qb * gz + self.q3 * gx)
        self.q3 += (qa * gz + qb * gy - qc * gx)

        #Normalise quarternion
        recipNorm = (self.q0 * self.q0 + self.q1 * self.q1 + self.q2 * self.q2 + self.q3 * self.q3)**-0.5
        self.q0 *= recipNorm
        self.q1 *= recipNorm
        self.q2 *= recipNorm
        self.q3 *= recipNorm

    ###########################################################################################
    #											      #
    #						get Q					      #
    #											      #
    ###########################################################################################
    def getQ(self):
        """
        populates array q with a quarternion representing the IMU orientation 
        with respect to the Earth
        """
        val = [0 for i in range(9)]

        val[0],val[1],val[2],val[3],val[4],val[5],val[6],val[7],val[8] = self.getValues()
        
        self.now = time() * 1000000.0
        self.now = self.now - self.startTime
        self.sampleFreq = 1.0 / ((self.now - self.lastUpdate) / 1000000.0)
        self.lastUpdate = self.now
        
        #gyro values are expressed in deg/sec, the * math.pi/180 will convert it to radians/sec
        self.AHRSupdate(val[3] * math.pi/180, val[4] * math.pi/180, val[5] * math.pi/180, val[0], val[1], val[2], val[6], val[7], val[8])

        q = [0 for i in range(4)]

        q[0] = self.q0
        q[1] = self.q1
        q[2] = self.q2
        q[3] = self.q3

        return q[0],q[1],q[2],q[3]


    ###########################################################################################
    #											      #
    #					get Barometer Altitude 				      #
    #											      #
    ###########################################################################################
    def getBaroAlti(self,sea_press):
        """
        returns an altitude estimate from barometer readings
        only using sea_press as current sea level pressure
        """
        temp = self.baro.getTemperature()
        press = self.baro.getPressure()
        return ((pow((sea_press / press), 1/5.257) - 1.0) * (temp + 273.15)) / 0.0065
    
    #########################################################################################
    #											    #
    #				get Barometer altitude					    #
    #											    #
    #########################################################################################
    def getBaroAlt(self):
        """
        Returns an alitude estimate from barometer reading only 
        using a default sea level pressure
        """
        return self.getBaroAlti(self.sea_press);

    ##########################################################################################
    #											     #
    #				gravity Compensate accel				     #
    #											     #
    ##########################################################################################
    def gravityCompensateAcc(self,acc,q):
        """
        Compensate the accelerometer readings in the 3d vector 
        acc expressed in the sensor frame for gravity,
        acc(accelerometer)---- readings to compensate for gravity
        q---- the quarternion orientation of the sensor board with respect to the world
        """
        g = [0 for i in range(3)]

        g[0] = 2 * (q[1] * q[3] - q[0] * q[2])
        g[1] = 2 *  (q[0] * q[1] + q[2] * q[3])
        g[2] = q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]
            
        #compensate accelerometer readings with the expected direction of gravity
        acc[0] = acc[0] - g[0]
        acc[1] = acc[1] - g[1]
        acc[2] = acc[2] - g[2]

        return acc[0],acc[1],acc[2]

    ##########################################################################################
    #											     #
    #				Get Euler In Radians					     #
    #											     #
    ##########################################################################################
    def getEulerRad(self):
        """
        Returns the Euler angles in radians defined in the Aerospace sequence
            Three floats array which will be populated by the Euler angles in radians
        """
        q = [0 for i in range(4)]
        angles = [0 for i in range(3)]
        q[0],q[1],q[2],q[3] = self.getQ()
        angles[0] = math.atan2(2 * q[1] * q[2] - 2 * q[0] * q[3], 2 * q[0] * q[0] + 2 * q[1] * q[1] - 1)
        #For debug purposes
        #print "------------------------------------------------"
        #print " q[0] =%f      q[1] =%f       q[2] = %f      q[3] = %f "%(q[0],q[1],q[2],q[3])
        angles[1] = -math.asin(2 * q[1] * q[3] + 2 * q[0] * q[2])
        angles[2] = math.atan2(2 * q[2] * q[3] - 2 * q[0] * q[1], 2 * q[0] * q[0] + 2 * q[3] * q[3] - 1)

        return angles[0],angles[1],angles[2]

    ########################################################################################
    #											   #
    #				    Get Euler						   #
    #											   #
    ########################################################################################
    def getEuler(self):
        """
        Returns the Euler angles in degrees defined with the Aerospace sequence
        Three floats array which will be populated by the Euler angles in degrees
        """
        angles = [0 for i in range(3)]
        angles[0],angles[1],angles[2] = self.getEulerRad()
        angles[0],angles[1],angles[2] = self.arr3_rad_to_deg(angles)

        #for debug purposes
        #print "---------------------Euler Angles-----------------------------------"
        #print "        Psi = %f          Theta = %f            Phi = %f         "%(angles[0],angles[1],angles[2])
        return angles[0],angles[1],angles[2]

    ########################################################################################
    #											   #
    #				   Get Yaw Pitch Roll in Radians			   #
    #											   #
    ########################################################################################
    def getYawPitchRollRad(self):
        """
        Returns the yaw pitch and roll angles, respectively defined as 
        the angles in radians between the earth north and the IMU X axis (yaw)
        , the Earth ground plane and the iMU x axis (pitch)
            and the Earth ground plane and the IMU Y axis this is not an Euler representation:
        the rotations aren't consecuitve rotations but only angles from Earth
        and the IMU. For Euler representation Yaw,Pitch and Roll see getEuler
        """
        q = [0 for i in range(4)]
        q[0],q[1],q[2],q[3] = self.getQ()
        gx = 2 * (q[1] * q[3] - q[0] * q[2])
        gy = 2 * (q[0] * q[1] + q[2] * q[3])
        gz = q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3] 

        ypr = [0 for i in range(3)]
        ypr[0] = math.atan2(2 * q[1] * q[2] - 2 * q[0] * q[3], 2 * q[0] * q[0] + 2 * q[1] * q[1] - 1)
        ypr[1] = math.atan(gx / math.sqrt(gy * gy + gz * gz))
        ypr[2] = math.atan(gy / math.sqrt(gx * gx + gz * gz))

        return ypr[0],ypr[1],ypr[2]
	
    ########################################################################################
    #										  	   #
    #				   get Yaw Pitch Roll in Degrees			   #
    #											   #
    ########################################################################################
    def getYawPitchRoll(self):
        """
        Returns the yaw,pitch and roll angles, respectively defined as
        the angles in degress between the Earth North and the IMU X axis (yaw)
        , the Earth ground plane and the IMU X axis(pitch)
        and the Earth ground plane and the iMU Y axis
        """
        ypr = [0 for i in range(3)]
        ypr[0],ypr[1],ypr[2] = self.getYawPitchRollRad()
        ypr[0],ypr[1],ypr[2] = self.arr3_rad_to_deg(ypr)

        return ypr[0],ypr[1],ypr[2]

    #######################################################################################
    #											  #
    #                            Array of radians to degrees				  #
    #											  #
    #######################################################################################
    def arr3_rad_to_deg(self,arr):
        """
        Converts a 3 elements array arr of angels expressed in radians into degrees
        """
        arr[0] *= 180/math.pi
        arr[1] *= 180/math.pi
        arr[2] *= 180/math.pi

        return arr[0],arr[1],arr[2]

