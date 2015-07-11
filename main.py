#!/usr/bin/python 

##############################################################################################
#                                   Q U A D I E
#   authors: Christian Hinostroza & Alexander Sanchez
#   Code designed for the Engineering Design 2 Project of Florida Atlantic University 
#   term: Winter 2015
#   Date: 3-27-2015
#############################################################################################
import time
import math
import threading
import logging
from PWM import pwmMain as pwm
from Libraries import FreeIMU as FreeIMU
from Libraries import controller as Controller
from PID import pid as pid
from PING import PingSensor as PING
import Queue
import socket

#------------------- Watchdog Time(in sec) -------------------------
watchdog = 30
#___________________________________________________________________
#------------------ Change Boot Time(in sec) ------------------------
bootTime = 40
#____________________________________________________________________
#---------------- Change Direction of Controls ----------------------
inverted = False
#____________________________________________________________________
controller = Controller.Controller()
sensors = FreeIMU.FreeIMU()
pwm = pwm.PWMMotor()
ping = PING.PingSensor()
#####################################################################################
#                                       PID setups                                  #
#####################################################################################
PID_PITCH_RATE = pid.PID()
PID_ROLL_RATE = pid.PID()
PID_PITCH_STAB = pid.PID()
PID_ROLL_STAB = pid.PID()
PID_YAW_RATE = pid.PID()
PID_ALT = pid.PID()
#PID_YAW_STAB = pid.PID()

###############################################################
#                        ALT PIDS
###############################################################
PID_ALT.definePID(0.0,0.0,0.0)

#__________________ SLOW VALUES______________________________
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#----- Good control 4/4/2015 --------------------------------
#PID_PITCH_RATE.definePID(0.007,0.00005,0.0008)
#PID_ROLL_RATE.definePID(0.007,0.00005,0.0008)
#-------------------------------------------------------------
#+++++++++++++++++ Better control 4/7/2015 +++++++++++++++++++
#PID_PITCH_RATE.definePID(0.009,0.00000001,0.00001)
#PID_ROLL_RATE.definePID(0.009,0.00000001,0.00001)
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

###############################################################
#                          RATE PIDS
###############################################################
PID_PITCH_RATE.definePID(0.0009,0.00000001,0.00001)
PID_ROLL_RATE.definePID(0.0009,0.00000001,0.00001)
PID_YAW_RATE.definePID(2.0,0.000,0.00)


PID_PITCH_STAB.set_windup_bounds(-200,200)
PID_ROLL_STAB.set_windup_bounds(-200,200)
#----------------------------------------------------------------------
#           Notes:
#           1) P minimum value is 0.0000005
#           2) P maximum value is 0.000005
#           3) I minimum value is 0.00000001
#-----------------------------------------------------------------------
##################################################################
#                       STAB PID    
##################################################################
PID_PITCH_STAB.definePID(0.5,0.000009,13.0)
PID_ROLL_STAB.definePID(0.5,0.000009,13.0)

PID_PITCH_STAB.set_windup_bounds(-10000,10000)
PID_ROLL_STAB.set_windup_bounds(-10000,10000)

MOTOR_FL = "1"
MOTOR_FR = "3"
MOTOR_BL = "2"
MOTOR_BR = "4"

rcpit = 0.0
rcroll = 0.0
command = ""
yaw = 0.0
pitch = 0.0
roll = 0.0
thr = 10.0
logging.basicConfig(level=logging.DEBUG,format='(%(threadName)-9s) %(message)s',)
#####################################################################################
#                                   Q U E U E ' S                                   #
#####################################################################################
S_BUF_SIZE = 2
BUF_SIZE = 10
yq = Queue.Queue(S_BUF_SIZE)
pq = Queue.Queue(S_BUF_SIZE)
rq = Queue.Queue(S_BUF_SIZE)
dq = Queue.Queue(BUF_SIZE)
grq = Queue.Queue(S_BUF_SIZE)
gpq = Queue.Queue(S_BUF_SIZE)
aq = Queue.Queue(S_BUF_SIZE)
baq = Queue.Queue(S_BUF_SIZE)

#####################################################################################
#                             Initialize Sockets                                    #
#####################################################################################
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
##########################################################################################
#                                                                                        #
#                               Constrain Function                                       #
#                                                                                        #
##########################################################################################
def constrain(amt,low,high):
    """
    Constrain value --amt 
    to be within the low and high 
    """
    if amt < low:
        return low
    elif amt > high:
        return high
    else:
        return amt


#############################################################################################
#                                                                                           #
#                                   Sensor reading thread                                   #
#                                                                                           #
#############################################################################################
class SensorReading(threading.Thread):
    def __init__(self, group=None,target=None, name=None, args=(), kwargs=None, verbose=None):
        super(SensorReading,self).__init__()
        self.target = target
        self.name = name

    def run(self):
        while True:
            y,p,r = sensors.getYawPitchRoll()
            #print "y = %f p = %f r = %f"%(y,p,r)
            b1,b2,b3,gy,gp,gr,pn,rn,dn = sensors.getValues()
            baro = sensors.getBaroAlt()
            if not yq.full():
                yq.put(y)
                #logging.debug('Putting yaw ' + str(y) + ' : ' + str(yq.qsize()) + 'readings in queue')
            if not pq.full():
                pq.put(p)
                #logging.debug('Putting pitch ' + str(p) + ' : ' + str(pq.qsize()) + 'readings in queue')
            if not rq.full():
                rq.put(r)
                #logging.debug('Putting roll ' + str(r) + ' : ' + str(rq.qsize()) + 'readings in queue')
            if not grq.full():
                grq.put(gr)
            if not gpq.full():
                gpq.put(gp)
            if not baq.full():
                baq.put(baro)

        return

#############################################################################################
#                                                                                           #
#                                   Ping sensor reading thread                              #
#                                                                                           #
#############################################################################################
class PingReading(threading.Thread):
    def __init__(self, group=None,target=None, name=None, args=(), kwargs=None, verbose=None):
        super(PingReading,self).__init__()
        self.target = target
        self.name = name

    def run(self):
        while True:
            altitude = ping.GetAlt()
            if not aq.full():
                aq.put(altitude)
                #logging.debug('Putting yaw ' + str(y) + ' : ' + str(yq.qsize()) + 'readings in queue')

        return
#############################################################################################
#                                                                                           #
#                                 Debug print thread                                        #
#                             NOT NEEDED FOR FUNCTIONALITY                                  #
#############################################################################################
class PrintReading(threading.Thread):
    def __init__(self, group=None,target=None, name=None, args=(), kwargs=None, verbose=None):
        super(PrintReading,self).__init__()
        self.target = target
        self.name = name
        return

    def run(self):
        while True:
            if not yq.empty():
                yaw = yq.get()
                #logging.debug('Getting yaw value  ' + str(item) + ' : ' + str(yq.qsize()) + 'readings in queue')
            if not pq.empty():
                pitch = pq.get()
            if not rq.empty():
                roll = rq.get()

            #print "yaw = %d o  pitch = %d o  roll = %d o"%(yaw,pitch,roll)
        return

#############################################################################################
#                                                                                           #
#                                   Tablet Thread                                           #
#                                                                                           #
#############################################################################################
class TabletThread(threading.Thread):
    def __init__(self, group=None,target=None, name=None, args=(), kwargs=None, verbose=None):
        super(TabletThread,self).__init__()
        self.target = target
        self.name = name
        
        return
    
    def run(self):
        s.bind(("", 5000))
        print "waiting on port: 5000"
        while True:
            data, addr = s.recvfrom(1024)
            if not dq.full():
                dq.put(data)
        
#############################################################################################
#                                                                                           #
#                                      Main Thread                                          #
#                                                                                           #
#############################################################################################
if __name__ == '__main__':
    #---------------------------START THREADS---------------------------------------------
    readings = SensorReading(name='freeimu')
    tabletreadings = TabletThread(name='Android')
    pingreadings = PingReading(name='PING')
    #printreads = PrintReading(name='printvalues')
    readings.start()
    time.sleep(2)
    tabletreadings.start()
    time.sleep(2)
    pingreadings.start()
    time.sleep(2)
    #printreads.start()
    print "---------------------------------------------------"
    print "___________________________________________________"
    print "						QUADIE						  "
    print "|||||||||||||||||||||||||||||||||||||||||||||||||||"
    print "		    	Group 4  ENGINEERING DESIGN           "
    print "############          HARDWARE         ############"
    print "   Chancey Kelley , Brain Chau, Chris Daugherty    "
    print "$$$$$$$$$$$$          SOFTWARE        $$$$$$$$$$$$$"
    print "        Alexander Sanchez    Christian Hinostroza  "
    print "___________________________________________________"
    print "..................................................."
    print " "
    print "Booting..."
    #sleep time for sensor readings to stabilize
    #***********************************************************************
    #                       B O O T   T I M E
    #**********************************************************************
    for i in range(bootTime):
        time.sleep(1)
        print "\r%d"%(bootTime-i)    
    print "Quadie will now take fly........"
    #######################################################################################
    #                                   MAIN LOOP                                         #
    #######################################################################################
    while True:
        try:
            ##################################################################################
            #                               GET COMMANDS                                     #
            ##################################################################################
            if not dq.empty():
                command = dq.get()
                if ',' in command:
                    rcpit,rcroll = controller.getPitchRoll(command)
                    #print "rcpit = %f  rcroll = %f"%(rcpit,rcroll)
                else:
                    rcpit = 0.0
                    rcroll = 0.0
                    if command not "Ping" or "ping" or "stop"
                        altCmd = int(command)
                #print "Tablet: %s"%command
            #print "rcpit = %f  rcroll = %f"%(rcpit,rcroll)
            ##################################################################################
            #                               GET POSITIONING                                  #
            ##################################################################################
            if not yq.empty():
                yaw = yq.get()
            if not pq.empty():
                pitch = pq.get()
            if not rq.empty():
                roll = rq.get()
            if not gpq.empty():
                gyropitch = gpq.get()
            if not grq.empty():
                gyroroll = grq.get()
            if not aq.empty():
                alt = aq.get()
            if not baq.empty():
                b_alt = baq.get()
                #debugging
                #logging.debug('Getting yaw value  ' + str(item) + ' : ' + str(yq.qsize()) + 'readings in queue')
            #print "yaw = %f o  pitch = %f o  roll = %f o"%(yaw,pitch,roll)
            #print "Altitude = %f  "%alt
            ###################################################################################
            #                               ADJUST ACCORDINGLY                                #
            ###################################################################################
            #------------------------------Altitude PID--------------------------------------
            if alt > 1000.0
                alt = b_alt
            #print "altitude: %f"%alt
            #thr = PID_ALT.update_pid(altCmd,alt,1)


            #------------------------------STABILIZE------------------------------------------
            #throttle up??
            thr = 20.0
            #---------------------------- Stabilizer PID -------------------------------------
            pitch_stab_output = constrain(PID_PITCH_STAB.update_pid(rcpit*200,pitch,1),-10000,10000)
            roll_stab_output = constrain(PID_ROLL_STAB.update_pid(rcroll*200,roll,1),-10000,10000)
            #print "PID pitch = %f  PID roll = %f"%(PID_PITCH_STAB.update_pid(rcpit,pitch,1),PID_ROLL_STAB.update_pid(rcroll,roll,1))
            #print "gyropitch = %f  gyroroll = %f"%(gyropitch,gyroroll)
            
            #---------------- Testing controller PID's ---------------------------------------
            #pitch_output =constrain(PID_PITCH_RATE.update_pid(rcpit*100,gyropitch,1.0),-500.0,500.0)
            #roll_output = constrain(PID_ROLL_RATE.update_pid(rcroll*100,gyroroll,1.0),-500.0,500.0)
            
            #yaw_output = constrain(PID_YAW_RATE.update_pid(0.0,yaw,1.0),-180.0,180.0)
            
            print " pitch_stab = %f     roll_stab = %f  "%(pitch_stab_output,roll_stab_output)
            #---------------------------- Rate PID's ----------------------------------------
            pitch_output = constrain(PID_PITCH_RATE.update_pid(pitch_stab_output,gyropitch,1.0),-500.0,500.0)
            #print "Pitches = %f"%PID_PITCH_RATE.update_pid(rcpit,gyropitch,1.0)
            roll_output = constrain(PID_ROLL_RATE.update_pid(roll_stab_output,gyroroll,1.0),-500.0,500.0)
            #overriding yaw for testing
            yaw_output = 0.0
            #print "pid p = %f    pid r = %f    pid y = %f"%(pitch_output,roll_output,yaw_output)
            
            #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
            #+++++++++++++++++++++ PWM FEED OF MOTORS +++++++++++++++++++++++++++++++++++++++
            #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
            if (inverted):
                #MOTOR FL
                MFL = pwm.setP(MOTOR_FL,constrain(thr - roll_output - pitch_output - yaw_output,10.0,90.0))
                #MOTOR BL
                MBL = pwm.setP(MOTOR_BL,constrain(thr - roll_output + pitch_output + yaw_output,10.0,90.0))
                #MOTOR FR
                MFR = pwm.setP(MOTOR_FR,constrain(thr + roll_output - pitch_output + yaw_output,10.0,90.0))
                #MOTOR BR
                MBR = pwm.setP(MOTOR_BR,constrain(thr + roll_output + pitch_output - yaw_output,10.0,90.0))
            else:
                #MOTOR FL
                MFL = pwm.setP(MOTOR_FL,constrain(thr + roll_output + pitch_output - yaw_output,5.0,90.0))
                #MOTOR BL
                MBL = pwm.setP(MOTOR_BL,constrain(thr + roll_output - pitch_output + yaw_output,5.0,90.0))
                #MOTOR FR
                MFR = pwm.setP(MOTOR_FR,constrain(thr - roll_output + pitch_output + yaw_output,5.0,90.0))
                #MOTOR BR
                MBR = pwm.setP(MOTOR_BR,constrain(thr - roll_output - pitch_output - yaw_output,5.0,90.0))
                #print " MFL = %f  MBL = %f   MFR = %f   MBR = %f  "%(MFL,MBL,MFR,MBR)
        except KeyboardInterrupt:
            #Turn motors off
            pwm.setP("1",0)
            pwm.setP("2",0)
            pwm.setP("3",0)
            pwm.setP("4",0)            
                  
            

