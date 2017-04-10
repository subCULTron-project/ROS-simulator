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
        
        self.reset_ping_structures()
        
        self.current = NED(0, 0, 0)
        self.position = None
        
        rospy.Subscriber('/scenario_start', Bool, self.start_cb)
        self.startPub = rospy.Publisher('start_sim', Bool, queue_size=1)

        self.stateRefPub = rospy.Publisher('stateRef', NavSts, queue_size=1)
        
        rospy.Subscriber('position', NavSts, self.position_cb)
        rospy.Subscriber("ping_sensor", NED, self.ping_sensor_cb)
        rospy.Subscriber("current_sensor", TwistStamped, self.current_sensor_cb)
        
        rospy.spin()
        
        
    def check_current_suitability(self): 
        
        if self.pingCount < 50:
            return  
        
        currentAngle = atan2(self.current.north, self.current.east) 
        swarmCenterAngle = atan2(self.pingAvgHeading.north, self.pingAvgHeading.east)
        
        if sqrt(pow(self.pingAvgHeading.north, 2) + pow(self.pingAvgHeading.east, 2)) < 3:
            return
        
        if sqrt(pow(self.current.north, 2) + pow(self.current.east, 2)) < 0.5:
            return
        
        if abs(currentAngle - swarmCenterAngle) < radians(30):
            self.reset_ping_structures()
            self.start_drifting(1.0)
            
    def start_drifting(self, duration):
        
        self.send_depth_goal(0.5)
        rospy.sleep(rospy.Duration(duration, 0))
        self.send_depth_goal(10)
        
    def reset_ping_structures(self):
        
        self.pingCount = 0
        self.pingAvgHeading = NED(0, 0, 0) 
        self.pingSum = NED(0, 0, 0)
         
    def start_cb(self, msg):
        
        self.send_depth_goal(10)
        self.startPub.publish(msg)
        
    def position_cb(self, msg):
        
        self.position = NED(msg.position.north, msg.position.east, msg.position.depth)
        
    def ping_sensor_cb(self, msg):
        
        if self.position is None or self.start is None:
            return
        
        if self.pingCount == 300:
            self.reset_ping_structures()
            
        self.pingCount += 1
        self.pingSum.north += (msg.north - self.position.north)
        self.pingSum.east += (msg.east - self.position.east)
        self.pingAvgHeading.north = self.pingSum.north / self.pingCount
        self.pingAvgHeading.east = self.pingSum.east / self.pingCount
        
        self.check_current_suitability()
    
    def current_sensor_cb(self, msg):
        
        if self.start is None:
            return
        
        self.current.north = msg.twist.linear.x
        self.current.east = msg.twist.linear.y
        
        self.check_current_suitability()
    
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
    
           
