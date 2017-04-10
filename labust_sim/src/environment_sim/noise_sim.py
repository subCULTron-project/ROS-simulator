#!/usr/bin/python

"""
Interface for noise simulation class.
"""

__author__ = "barbanas"

import rospy
from std_msgs.msg import Bool, Float64
from auv_msgs.msg import NavSts, NED
from math import sin, cos

class NoiseSim(object):
    """
    Abstract class for noise simulation.
    It provides the template on which all actual noise simulation classes have to be implemented.
    """
    def __init__(self):

        self.start = False
        self.position = NED()
        self.orientation = None #Point() the orientation vector
        self.oldNoise = None
        self.intensity = rospy.get_param('~noise_intensity')

        if rospy.has_param('~noise_publish_rate'):
            self.noisePubRate = rospy.get_param('~noise_publish_rate')
        else:
            self.noisePubRate = 0.1
        
        self.noisePubTimeout = rospy.get_time() + self.noisePubRate

        rospy.Subscriber("start_sim", Bool, self.start_cb)
        rospy.Subscriber("position", NavSts, self.position_cb)
        
        rospy.Subscriber("/noise_intensity", Float64, self.noise_intensity_cb)

        self.noiseSensorPub = rospy.Publisher("noise_sensor", Float64, queue_size=1)

    def get_noise(self, x, y, z):
        """
        Gets the noise value on position (x, y, z).
        Noise data type is std_msgs/Float64.
        """
        raise NotImplementedError("Should have implemented this")

    def ok(self):
        """
        Returns True/False, depending if the noise simulator is ready to work.
        This has to be implemented for every noise sim class.
        """
        raise NotImplementedError("Should have implemented this")

    # callback functions

    def start_cb(self, msg):
        self.start = True
        
    def noise_intensity_cb(self, msg):
        self.intensity = msg.data

    def position_cb(self, msg):
        if not self.ok():
            return

        self.position.north = msg.position.north
        self.position.east = msg.position.east
        self.position.depth = msg.position.depth
        
        newNoise = self.get_noise(self.position.north, self.position.east, self.position.depth)

        # signal noise source shutdown
        if newNoise < 0:
            if self.oldNoise >= 0 or self.oldNoise is None:
                self.oldNoise = newNoise
                self.noiseSensorPub.publish(newNoise)
            return
            
        if newNoise is None:
            newNoise = 0
            self.oldNoise = newNoise
            self.noiseSensorPub.publish(newNoise)
            return

        publish = False
        if self.oldNoise is None:
            # if noise was never published
            publish = True
        else:
            # if noise value changed
            publish = abs(self.oldNoise - newNoise) > 0.001

        # publish only when temperature changes or temperature was not published in some time
        if publish or (self.noisePubTimeout < rospy.get_time()):
            # save temperature value
            self.oldNoise = newNoise
            # set temperature sensor info
            self.noiseSensorPub.publish(newNoise)
            # move timeout to a different time
            self.noisePubTimeout = rospy.get_time() + self.noisePubRate
            
        