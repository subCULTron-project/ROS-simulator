#!/usr/bin/env python

import rospy
from auv_msgs.msg import NED, NavSts
from std_msgs.msg import Bool

"""
...
"""

__author__ = ""


class ScenarioController(object):
    
    def __init__(self):

        # position
        self.position = None
        rospy.Subscriber('position', NavSts, self.position_cb)

        # experiment start
        self.startPub = rospy.Publisher('start_sim', Bool, queue_size=1)
        rospy.Subscriber('/test_start', Bool, self.start_cb)

        rospy.spin()

    def start_cb(self, msg):
        
        while self.position is None:
            print rospy.get_namespace() + "waiting for position.. "
            rospy.sleep(0.1)
        
        # start device simulation
        self.startPub.publish(msg)

        # TODO your controller code here!
        
    def position_cb(self, msg):
        # read current position
        self.position = NED(msg.position.north, msg.position.east, msg.position.depth)
        
if __name__ == "__main__":
    
    rospy.init_node("scenario_controller")
    
    try:
        controller = ScenarioController()
            
    except rospy.ROSInterruptException:
        pass
