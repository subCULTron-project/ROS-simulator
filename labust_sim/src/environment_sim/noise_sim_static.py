#!/usr/bin/python

"""
Implementation of static noise source simulation.
---
"""

from noise_sim import NoiseSim
from geometry_msgs.msg import Point
from std_msgs.msg import Float64
from auv_msgs.msg import NavSts, NED
from math import sin, cos, acos, sqrt, pow, exp, pi, radians, degrees
import rospy

__author__ = "barbanas"

class NoiseSimStatic(NoiseSim):
    """
    NoiseSimStatic inherits NoiseSim class. It has to implement
    get_noise and ok functions. For other noise simulation functions,
    read the noise_sim.py file.
    """
    def __init__(self, sourcePos):
        super(NoiseSimStatic, self).__init__()

        self.sourcePosition = NED()
        self.sourcePosition.north = sourcePos.north
        self.sourcePosition.east = sourcePos.east
        self.sourcePosition.depth = sourcePos.depth
        
        self.orientation = Point(0, 0, 0)
        
        rospy.Subscriber("stateRef", NavSts, self.state_ref_cb)

    def get_noise(self, x, y, z):
        """
        Gets the noise value on position (x, y, z).
        Temperature data type is std_msgs/Float64.
        """
        if not self.ok():
            return None

        if self.intensity < 0:
            return -1
        
        vectorToSource = Point(self.sourcePosition.north - self.position.north,\
                                self.sourcePosition.east - self.position.east,\
                                self.sourcePosition.depth - self.position.depth)
        
        vectProduct = vectorToSource.x * self.orientation.x + vectorToSource.y * self.orientation.y + \
                      vectorToSource.z * self.orientation.z
                      
        a1 = sqrt(pow(vectorToSource.x, 2) + pow(vectorToSource.y, 2) + pow(vectorToSource.z, 2))
        a2 = sqrt(pow(self.orientation.x, 2) + pow(self.orientation.y, 2) + pow(self.orientation.z, 2))
        
        if a2 == 0:
            a2 = 1 # if there is no rotation (unit vector, angle is 0)
        theta = acos(float(vectProduct) / (a1 * a2))
        
        if abs(theta) <= pi / 4:
            '''
            print "-------"
            print self.position
            print self.orientation
            print vectorToSource
            print "got signal, angle " + str(theta) + "  " + str(degrees(theta)) + "    " + str(float(100) * exp(-2 * pow(theta, 2)) / a1)
            ''' 
            return float(100) * exp(-2 * pow(theta, 2)) / a1
        else:
            return 0
        
    def ok(self):
        return self.start and (self.orientation is not None)
    
    # callback functions
    
    def state_ref_cb(self, msg):
        
        self.orientation = Point()
        
        self.orientation.x = cos(radians(msg.orientation.yaw)) * cos(radians(msg.orientation.pitch))
        self.orientation.y = sin(radians(msg.orientation.yaw)) * cos(radians(msg.orientation.pitch))
        self.orientation.z = sin(radians(msg.orientation.pitch))
        
             