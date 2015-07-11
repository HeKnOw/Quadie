#!/usr/bin/python

import math

class Controller:
    """
    Controller class
    """
    def __init__(self):
        """
        Constructor for controller object
        """
        self.angle = 0.0
        self.power = 0.0

    #################################################################################
    #                                                                               #
    #                          Get Pitch And Roll                                   #
    #                            From Controller                                    #
    #                               In Deg/sec                                      #
    #################################################################################
    def getPitchRoll(self,command):
        """
        Function to convert the command from tablet into 
        pitch and roll commands in deg/sec
        """
        self.translate(command)
        if self.angle >= 0 and self.angle <= 90 :
            rollPowerDist = self.angle / 0.9
            pitchPowerDist = 100 - rollPowerDist
        elif self.angle > 90 :
            pitchPowerDist = (self.angle - 90) / 0.9
            rollPowerDist = 100 - pitchPowerDist
            pitchPowerDist = (-1) * pitchPowerDist
        elif self.angle < 0 and self.angle >= -90:
            rollPowerDist = self.angle / 0.9
            pitchPowerDist = 100 + rollPowerDist
        elif self.angle < -90 :
            pitchPowerDist = (self.angle + 90) / 0.9
            rollPowerDist = 100 + pitchPowerDist
            rollPowerDist = (-1) * rollPowerDist
        
        pitch = self.power * (pitchPowerDist * 0.01)
        roll = self.power * (rollPowerDist * 0.01)
        
        return pitch,roll

    def translate(self,command):
        """
        use split to get commands from controller
        """
        angle,power = command.split(",",1)
        self.angle = float(angle)
        self.power = float(power)
