#!/usr/bin/python

import numpy as np
import math

class AMCalib:
    """
    Object that will do the transfer of calibration offsets and scales
    """

    def __init__(self):
	"""
	Constructor for AMClib
	"""
	print "Initializing calibrate routine"
    	acc_data = np.loadtxt("acc.txt")
    	self.ax,self.ay,self.az = acc_data[:,0],acc_data[:,1],acc_data[:,2]
    	magn_data = np.loadtxt("magn.txt")
    	self.mx,self.my,self.mz = magn_data[:,0],magn_data[:,1],magn_data[:,2]
	A_OSx,A_OSy,A_OSz,A_SCx,A_SCy,A_SCz = self.Ellipsoid_to_Sphere(self.ax,self.ay,self.az)
	M_OSx,M_OSy,M_OSz,M_SCx,M_SCy,M_SCz = self.Ellipsoid_to_Sphere(self.mx,self.my,self.mz)
	print "values ready to be extracted\nUse .ExtractValues"

    def Ellipsoid_to_Sphere(self,x,y,z):
	"""
	Convert the plot from the calibration routine 
	the files inside acc.txt and magn.txt
	from ellipsoids into spheres giving us a better representation 
	of the sensor readings in space
	"""
	negYsqr = np.square(y)
	negYsqr = np.multiply(negYsqr,-1)
	Xsqr = np.square(x)
	negZsqr = np.square(z)
	negZsqr = np.multiply(negZsqr,-1)
	H = np.array([ x, y, z, negYsqr, negZsqr, 1])
	X = np.linalg.lstsq(H,Xsqr)
	OSx = np.divide(X[:,0],2)
	temp = np.multiply(2,X[:,3])
	OSy = np.divide(X[:,1],temp)
	temp1 = np.multiply(2,X[:,4])
	OSz = np.divide(X[:,2],temp1)
	A = np.divide(X[:,0],2)
	B = np.divide(A,X[:,3])
	C = np.divide(A,X[:,4])
	SCx = np.sqrt(A)
	SCy = np.sqrt(B)
	SCz = np.sqrt(C)
	

	return OSx,OSy,OSz,SCx,SCy,SCz

    def ExtractValues(self):
        """
	return the offsets and scales for accelerometer
	acc_off x,y,z then acc_scales x,y,z
	return the offset and scales for magnetometer
	magn_off x,y,z then magn_scales x,y,z
	"""
	return A_OSx,A_OSy,A_OSz,A_SCx,A_SCy,A_SCz,M_OSx,M_OSy,M_OSz,M_SCx,M_SCy,M_SCz


