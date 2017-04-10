#!/usr/bin/python

"""
Implementation of periodic current mode simulation.
Current heading changes in time on parametric circle.
"""

from current_sim import CurrentSim
from geometry_msgs.msg import TwistStamped
import rospy
from math import pi, cos, sin

__author__ = "barbanas"

# current change parameters
angleChangeRate = 5     # in seconds  
angleChangeValue = 10   # in degrees

class CurrentSimPeriodic(CurrentSim):
    """
    CurrentSimPeriodic inherits CurrentSim class. It has to implement
    get_current and ok functions. For other current simulation functions,
    read the current_sim.py file.
    """
    def __init__(self):
        super(CurrentSimPeriodic, self).__init__() 

        self.currStartTime = rospy.get_rostime().secs

    def get_current(self, x, y):
        """
        Gets the current value on position (x, y).
        Current data type is geometry_msgs/TwistStamped.
        """
        timeSec = rospy.get_rostime().secs
        n = (timeSec - self.currStartTime) / angleChangeRate
        angleChange = n * angleChangeValue
        
        angle = float((angleChange % 360) * pi / 180)
        curr = TwistStamped()
        curr.twist.linear.x = cos(angle)
        curr.twist.linear.y = sin(angle)
        return curr

    def ok(self):
        return self.start