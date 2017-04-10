#!/usr/bin/env python

"""
Initializes ROS node for high level mission control.
"""

__author__ = "barbanas"

import rospy
from auv_msgs.msg import NED, NavSts

class ScenarioController(object):
    
    def __init__(self):
                
        self.position = NED(0,0,0)
        rospy.Subscriber('position', NavSts, self.position_cb)
        rospy.spin()    
        
    def position_cb(self, msg):
        
        self.position = NED(msg.position.north, msg.position.east, msg.position.depth)
        
if __name__ == "__main__":
    
    rospy.init_node("scenario_controller")
    
    try:
        controller = ScenarioController()
            
    except rospy.ROSInterruptException:
        pass
    
           
