#!/usr/bin/env python

"""
Initializes ROS node for high level mission control.
"""

__author__ = "barbanas"

import rospy
from auv_msgs.msg import NED, NavSts
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Bool
from math import atan2, radians, degrees, sqrt, pow, fabs
import action_library

class ScenarioController(object):
    
    def __init__(self):
        
        self.start = False
        
        self.current = NED(0, 0, 0)
        self.position = None
        
        rospy.Subscriber('/scenario_start', Bool, self.start_cb)
        self.startPub = rospy.Publisher('start_sim', Bool, queue_size=1)

        self.stateRefPub = rospy.Publisher('stateRef', NavSts, queue_size=1)
        
        rospy.Subscriber('position', NavSts, self.position_cb)
        rospy.Subscriber("ping_sensor", NED, self.ping_sensor_cb)
        rospy.Subscriber('goto_surface', Bool, self.surface_cb)

        self.dockingPub = rospy.Publisher('/apad1/docked', NavSts, queue_size=1)
        
        rospy.spin()

   
    def start_cb(self, msg): 
        
        self.send_depth_goal(10)
        self.startPub.publish(msg)
        self.start = True

        #d = NavSts()
        
        #rate = rospy.Rate(10)
        
        #while not rospy.is_shutdown():
        
			#d.position.north = self.position.north
			#d.position.east = self.position.east
			#d.header.frame_id = rospy.get_namespace().replace('/', '')
			#self.dockingPub.publish(d)
        
			#rate.sleep()

    def surface_cb(self, msg): 
    
        self.send_depth_goal(0.5)
        d = NavSts()
        d.position.north = self.position.north
        d.position.east = self.position.east
        d.header.frame_id = rospy.get_namespace().replace('/', '')
        self.dockingPub.publish(d)
        
    def position_cb(self, msg):
        
        self.position = NED(msg.position.north, msg.position.east, msg.position.depth)
        
        if self.position is None or self.start is False:
			return
        
        #if (msg.position.depth<0.5):
			#d = NavSts()
			#d.position.north = self.position.north
			#d.position.east = self.position.east
			#d.header.frame_id = rospy.get_namespace().replace('/', '')
			#self.dockingPub.publish(d)
        
    def ping_sensor_cb(self, msg):
        return
        if self.position is None or self.start is None:
            return

    def send_depth_goal(self, depth):

        pos_old = NED(self.position.north, self.position.east, self.position.depth)
        goal = NED(pos_old.north, pos_old.east, depth)
        if action_library.send_depth_goal(self.stateRefPub, goal) == -1:
            return

        pos_err = []
        while not rospy.is_shutdown():
            dL = abs(goal.depth - pos_old.depth)
            dl = abs(goal.depth - self.position.depth)
            if len(pos_err) < 10:
                pos_err.append(dl)
            else:
                pos_err.pop(0)
                pos_err.append(dl)

            if (len(pos_err) == 10) and (fabs(sum(pos_err) / len(pos_err)) < 0.05):  # mission is successfully finished
                return
            else:  #mission is still ongoing
                rospy.sleep(0.1)
        
if __name__ == "__main__":
    
    rospy.init_node("scenario_controller")
    
    try:
        controller = ScenarioController()
            
    except rospy.ROSInterruptException:
        pass
    
           
