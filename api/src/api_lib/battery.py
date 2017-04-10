#!/usr/bin/env python

"""
Battery API class.
"""

__author__ = "barbanas"


from misc_msgs.srv import GetChargeInfo
from std_msgs.msg import Bool
import rospy

class Battery(object):
    
    def __init__(self):
        # publishers
        self.sleepModePub = rospy.Publisher('power_sleep_mode', Bool, queue_size=1)
    
    def get_level(self):
        '''
        Get battery level.
        Args:
           none
        Returns:
           batteryLevel - float
        '''
        batteryLevel = -1
        
        charge_info = rospy.ServiceProxy('get_battery_level', GetChargeInfo)
        try:
            c = charge_info()
            batteryLevel = c.battery_level
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))
        
        return batteryLevel
    
    def toggle_sleep_mode(self, activate):
        '''
        Toggle sleep mode.
        Args:
           activate: bool (activate/deactivate sleep mode)
        '''
        
        self.sleepModePub.publish(activate)
        
            