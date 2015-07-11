#!/usr/bin/python

##############################################################################################
#   author: Christian Hinostroza
#   Code designed for the Engineering Design 2 Project of Florida Atlantic University 
#   term: Winter 2015
#   Date: 3-27-2015
#############################################################################################

class PID:
    """
    Pid Object
    """
    #########################################################################################
    #                                                                                       #
    #                                   CONSTRUCTOR                                         #
    #                                                                                       #
    #########################################################################################

    def __init__(self):
        """
        Constructor
        """
	self.Kd = 0.0
	self.Ki = 0.0
	self.Kp = 0.0
	self.err= 0.0
	self.sum_err = 0.0
	self.ddt_err = 0.0
	self.lastInput = 0.0
	self.outmax = 0.0
	self.outmin = 0.0

    def definePID(self,kp,ki,kd):
        """
        Define each PID 
        proportional, integral and derivative
        """
        self.Kp = kp
        self.Ki = ki
        self.Kd = kd
        self.err = 0.0
        self.sum_err = 0.0
        self.ddt_err = 0.0
        self.lastInput = 0.0
        self.outmax = 200.0
        self.outmin = -200.0

    def update_pid(self,setpoint,_input,dt):
        """
        Update pid's with right values
        """
        #compute error
        self.err = setpoint - _input

        #integrating errors
        self.sum_err += self.err * self.Ki * dt	

        #calculating error derivative
        #input derivaive is used to avoid derivative kick
        self.ddt_err = -self.Kd / dt * (_input - self.lastInput)
	
        #Calculation of the output
        #winds up boundaries
        output = self.Kp * self.err + self.sum_err + self.ddt_err
        if(output > self.outmax):
            #winds up boundaries
            self.sum_err = 0.0
            output = self.outmax
        elif(output < self.outmin):
            #winds up boundaries
            self.sum_err = 0.0
            output = self.outmin
		#Debug purposes
	    #print "output = %f selfsum_err = %f   Kp = %f Ki = %f self err = %f "%(output,self.sum_err,self.Kp,self.Ki,self.err)
        self.lastInput = _input

        return output

    def reset(self):
        """
        reset values 
        """
        self.sum_err = 0.0
        self.ddt_err = 0.0
        self.lastInput = 0.0

    def setKpid(self,kp,ki,kd):
        """
        Set Proportional constants
        """
        self.Kp = kp
        self.Ki = ki
        self.Kd = kd

    def set_windup_bounds(self,Min,Max):
        """
        Set windup bounds
        """

        self.outmax = Max
        self.outmin = Min

