#!/usr/bin/env python

"""
Initializes ROS node for high level mission control.
Trust scenario -- 
"""

__author__ = "barbanas"

import rospy
import action_library

from misc_msgs.srv import GetPosition, GetSensoryReading, GetTrustInfo
from navcon_msgs.srv import EnableControl, ConfigureVelocityController
from auv_msgs.msg import NED, NavSts
from geometry_msgs.msg import Point
from std_msgs.msg import Bool, Float64
from math import radians, pow, sin, cos, pi, log, tan, sqrt, fabs
from random import random, choice
from copy import deepcopy
import numpy as np

area = [[-20, 20], [-20, 20]]

aFishList = []
for i in range(2):
    aFishList.append("/afish" + str(i + 1) + "/")

aMusselList = []
for i in range(2):
    aMusselList.append("/amussel" + str(i + 1) + "/")

class ScenarioController(object):
    
    def __init__(self):
        
        self.start = False
        
        self.yaw = 0
        self.position = None
        self.orientation = Point()
                
        # bacterial chemotaxis + levy random walk
        self.tumble = False
        self.tumbleValue = 0
        self.signalHistory = [0, 0]
        
        self.levyTimeout = None
        self.levyA = 0
        self.levyTumble = False
        
        # trust
        self.communicationRange = 10
        
        self.A = np.zeros([len(aFishList)])                # graph connectivity matrix
        self.b = np.zeros([len(aMusselList)])              # visited aMussels
        self.current =  np.zeros([len(aMusselList), 2])    # the latest aMussel sensory reading, vector -- [north, east]
        
        self.tau = np.zeros([len(aMusselList)])      # observation function about agent's trustworthiness
        self.delta =  np.zeros([len(aMusselList)])   # performance
        self.sigma =  np.zeros([len(aMusselList)])   # confidence
        
        self.zeta =  np.zeros([len(aFishList), len(aMusselList)])    # trust matrix --> rows: aFish trust vectors
        self.index = aFishList.index(rospy.get_namespace())          # agent's index
        
        # initialize trust variables
        self.init_trust()
        
        # publishers
        self.stateRefPub = rospy.Publisher('stateRef', NavSts, queue_size=1)
        self.startPub = rospy.Publisher('start_sim', Bool, queue_size=1)
        
        # subscribers
        rospy.Subscriber('/scenario_start', Bool, self.start_cb)
        rospy.Subscriber('position', NavSts, self.position_cb)
        
        rospy.Subscriber('noise_sensor', Float64, self.noise_cb)
        
        # position service
        rospy.Service('get_position', GetPosition, self.get_position_srv)
        
        # get trust vector service
        rospy.Service('get_trust_info', GetTrustInfo, self.get_trust_info_srv)
        
        # periodic function call
        # random / biased random walk
        rospy.Timer(rospy.Duration(0.1), self.bacterial_chemotaxis_levy_walk)
        
        # trust scenario
        rospy.Timer(rospy.Duration(0.2), self.update_communication_structures)
        
        # open to overwrite file content -- used for path visualization
        #f = open('/home/barbara/Desktop' + rospy.get_namespace()[:-1] + 'path.txt','w')

        rospy.spin()
      
    # callback functions
      
    def position_cb(self, msg):
        
        self.position = NED(msg.position.north, msg.position.east, msg.position.depth)
        
        #if not self.start:
        #    return

        #f = open('/home/barbara/Desktop' + rospy.get_namespace()[:-1] + 'path.txt','a')
        #f.write(str(self.position.north) + " " + str(self.position.east) + "\n")
           
    def start_cb(self, msg):

        self.startTime = rospy.get_time()
        
        # enable and configure controllers
        self.velcon_enable = rospy.ServiceProxy('VelCon_enable', EnableControl)
        self.velcon_enable(enable=True)

        self.fadp_enable = rospy.ServiceProxy('FADP_enable', EnableControl)
        self.fadp_enable(enable=True)
     
        self.config_vel_controller = rospy.ServiceProxy('ConfigureVelocityController', ConfigureVelocityController)
        self.config_vel_controller(ControllerName="FADP", desired_mode=[2, 2, 2, 0, 0, 0])
        
        # submerge
        while self.position is None:
            rospy.sleep(0.1)
        
        action_library.send_position_goal(self.stateRefPub, NED(self.position.north, self.position.east, 5))
        depthOld = self.position.depth
        posErr = []
        
        while True:
            dL = abs(self.position.depth - depthOld)
            dl = abs(5 - self.position.depth)
    
            if len(posErr) < 20:
                posErr.append(dl)
            else:
                posErr.pop(0)
                posErr.append(dl)
    
            if (len(posErr) == 20) and (fabs(sum(posErr) / len(posErr)) < 0.1):  # mission is successfully finished
                break
        
        self.start = True
        self.startPub.publish(msg)
        
    def noise_cb(self, msg):
        
        if len(self.signalHistory) >= 30:
            del self.signalHistory[0]
            
        self.signalHistory.append(msg.data)
        
    # service functions
    
    def get_position_srv(self, req):
        
        while self.position is None:
            rospy.sleep(0.1)
            
        return [self.position.north, self.position.east, self.position.depth]
    
    def get_trust_info_srv(self, req):
        
        return {"zeta": self.zeta[self.index]}
    
    # scenario functions

    def bacterial_chemotaxis_levy_walk(self, event):
        '''
        TODO -- description
        '''
        while self.position is None or not self.start:
            rospy.sleep(0.1)

        # uncomment for simulation time tracking
        #if '1' in rospy.get_namespace():
        #    print "$$$$$$$$$$$$$$$$$  " + str(rospy.get_time() - self.startTime)
        C = 0
        
        # get sensory signal amplitude
        amp = self.signalHistory[-2:]
        newAmplitude = amp[1]
        # if in tumbling process, use the amplitude value at the start of tumbling
        if self.tumble:
            oldAmplitude = self.tumbleValue
        else:
            oldAmplitude = amp[0]
        
        if newAmplitude < 0:    # no sensory data -- perform Levy walk
            # levy walk
            if self.levyTumble:
                self.levyA = 0
                # constant time period for tumbling activity
                self.levyTimeout = rospy.get_time() + 0.2
                #print "outside the area, tumbling"
            elif self.levyTimeout is None:
                t = self.levy_random()
                #print "## keeping direction for " + str(t)
                self.levyTimeout = rospy.get_time() + t
                self.levyA = 1
            elif rospy.get_time() >= self.levyTimeout:
                # toggle attractor variable
                self.levyA = 1 - self.levyA
                if self.levyA == 0:
                    # constant time period for tumbling activity
                    self.levyTimeout = rospy.get_time() + 0.2
                else:
                    # levy random variable
                    t = self.levy_random()
                    #print "## keeping direction for " + str(t)
                    self.levyTimeout = rospy.get_time() + t
                    
            A = self.levyA
        else:   # bacterial chemotaxis
            # if signal strength has lowered, change the angle until the signal improves again
            if newAmplitude - oldAmplitude <= C:
                A = 0
                # if tumbling was already occurring, discard the new signal readings until improvement
                if not self.tumble: # keep amplitude value at the start of tumbling process
                    self.tumbleValue = newAmplitude
                self.tumble = True
            else:
                A = 1
                self.tumble = False
            
        linVelocity = A
        if newAmplitude < 0: # Levy walk
            angVelocity = choice([-1, 1]) * (1 - A) * (60 + np.random.normal(0, 10))
        else: # bacterial chemotaxis
            angVelocity = (1 - A) * (30 + np.random.normal(0, 10))
        angVelocityRad = radians(angVelocity)
        
        deltax = linVelocity * cos(angVelocityRad + radians(self.yaw))
        deltay = linVelocity * sin(angVelocityRad + radians(self.yaw))
        deltatheta = angVelocity
        
        # prepare state reference
        stateRef = NavSts()
        
        self.yaw += deltatheta
        self.yaw %= 360
        
        stateRef.position.north = self.position.north + deltax
        stateRef.position.east = self.position.east + deltay
        stateRef.position.depth = self.position.depth
        stateRef.orientation.yaw = self.yaw 
        
        if stateRef.position.north < area[0][0] or stateRef.position.north > area[0][1] or \
            stateRef.position.east < area[1][0] or stateRef.position.east > area[1][1]: 
            # random walk is at the edge of the allowed area -- force tumbling
            self.levyTumble = True
            return
        else:
            self.levyTumble = False
        
        # calc orientation (only yaw rotation is allowed)
        self.orientation.x = cos(radians(self.yaw))
        self.orientation.y = sin(radians(self.yaw))
        self.orientation.z = 0
        
        # initialize movement
        action_library.send_state_ref(self.stateRefPub, stateRef)
        
    def levy_random(self):
        '''
        Levy alpha stable distribution random number generator. 
        Based on http://economics.sbs.ohio-state.edu/jhm/jhm.html (STABRND.M generator)
        '''
        m = 1
        n = 1
        # generates m x n matrix
        alpha = 0.5     # characteristic exponent
        beta = 1        # skewness parameter
        c = 0.5         # scale
        delta = 0       # location parameter
        
        w = -log(random())
        phi = (random() - 0.5) * pi
        
        cosphi = cos(phi)
        if abs(alpha - 1) > 1.0e-8:
            zeta = beta * tan(pi * alpha / 2)
            aphi = alpha * phi
            a1phi = (1 - alpha) * phi
            x = ((sin(aphi) + zeta * cos(aphi)) / cosphi)\
                * (cos(a1phi) + zeta * sin(a1phi)) \
                / pow(w * cosphi, (1 - alpha) / alpha)
        else:
            bphi = (pi / 2) + beta * phi
            x = (2 / pi) * (bphi * tan(phi) - beta * log((pi / 2) * w * cosphi / bphi))
            if abs(alpha - 1) < 0.00001:
                x = x + beta * tan(pi * alpha / 2) 
            
        x = delta + c * x
        return x
      
    def init_trust(self):
        '''
        Initialization function for trust variables.
        '''
        pass
           
    def update_communication_structures(self, event):  
        '''
        A function that gets called periodically. It updates agent's communication structures
        and sensory reading data.
        '''
        
        while self.position is None or not self.start:
            rospy.sleep(0.1)
            
        # update aFish connectivity matrix based on new position information
        newConnection = False # new connection flag TODO --> check if this is the correct behavior
        
        aFishesInRange = []
        for i in range(len(aFishList)):
            try:
                rospy.wait_for_service(aFishList[i] + 'get_position', 0.2)
                get_pos = rospy.ServiceProxy(aFishList[i] + 'get_position', GetPosition)
                p = get_pos()
                if sqrt(pow(self.position.north - p.x, 2) + \
                        pow(self.position.east - p.y, 2) + \
                        pow(self.position.depth - p.z, 2)) < self.communicationRange:
                    if self.A[i] == 0:
                        newConnection = True
                    self.A[i] = 1
                    aFishesInRange.append(i)
                else:
                    self.A[i] = 0
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
            except rospy.ROSException, e:
                print "Service call failed: %s"%e
        
        # update aMussel connectivity matrix based on new position information
        aMusselsInRange = []
        for i in range(len(aMusselList)):
            try:
                rospy.wait_for_service(aMusselList[i] + 'get_position', 0.2)
                get_pos = rospy.ServiceProxy(aMusselList[i] + 'get_position', GetPosition)
            
                p = get_pos()
                if sqrt(pow(self.position.north - p.x, 2) + \
                        pow(self.position.east - p.y, 2) + \
                        pow(self.position.depth - p.z, 2)) < self.communicationRange:
                    aMusselsInRange.append(i)
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
            except rospy.ROSException, e:
                print "Service call failed: %s"%e
                        
        # exchange trust information with aFishes in range
        for i in aFishesInRange:
            if i == self.index:
                continue
            try:
                rospy.wait_for_service(aFishList[i] + 'get_trust_info', 0.2)
                get_trust = rospy.ServiceProxy(aFishList[i] + 'get_trust_info', GetTrustInfo)
            
                result = get_trust()
                self.zeta[i] = result.zeta
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
            except rospy.ROSException, e:
                print "Service call failed: %s"%e
        
        # get sensory reading data from aMussels in range
        for i in aMusselsInRange:
            try:
                rospy.wait_for_service(aMusselList[i] + 'get_sensory_reading', 0.2)
                get_sen = rospy.ServiceProxy(aMusselList[i] + 'get_sensory_reading', GetSensoryReading)
            
                result = get_sen()
                self.current[i] = result.current
                self.b[i] = 1
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
            except rospy.ROSException, e:
                print "Service call failed: %s"%e
                 
    def trust(self, event):
        '''
        Trust implementation. Gets called upon information change.
        TODO -- implement + test behavior (is it better if it is called periodically?)
        '''
        # Petra

        return
        
                        
if __name__ == "__main__":
    
    rospy.init_node("scenario_controller")
    
    try:
        controller = ScenarioController()
            
    except rospy.ROSInterruptException:
        pass
    
           
