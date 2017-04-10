#!/usr/bin/env python

"""
Clock API class.
"""

__author__ = "barbanas"


import rospy

class Clock(object):
    
    def __init__(self):
        pass
    
    def get_time(self):
        return float(rospy.get_rostime().to_nsec()) / 1000