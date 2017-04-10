#!/usr/bin/env python

"""
Initializes ROS node for high level mission control.
"""

__author__ = "barbanas"

import rospy
from auv_msgs.msg import NED, NavSts
from geometry_msgs.msg import TwistStamped, Point
from std_msgs.msg import Bool, Float64, String
from math import atan2, radians, degrees, sqrt, pow, fabs
from misc_msgs.srv import GetChargeInfo
import action_library

class ScenarioController(object):
    
    def __init__(self):
        
        self.start = False
        
        self.current = NED(0, 0, 0)
        self.position = None
        
        self.neighbors = {}
        
        rospy.Subscriber('/scenario_start', Bool, self.start_cb)
        self.startPub = rospy.Publisher('start_sim', Bool, queue_size=1)

        self.stateRefPub = rospy.Publisher('stateRef', NavSts, queue_size=1)
        
        rospy.Subscriber('position', NavSts, self.position_cb)

        self.batteryLevel = None
        rospy.Subscriber('battery_level', Float64, self.battery_level_cb)
        self.batteryAlertPub = rospy.Publisher('/battery_alert', NavSts, queue_size=100)
        self.sleepModePub = rospy.Publisher('power_sleep_mode', Bool, queue_size=1)
        self.alertPublished = False
        rospy.Service('charge_info', GetChargeInfo, self.charge_info_srv)
        self.chargeType = None
        
        rospy.spin()

    def start_cb(self, msg):
        
        while self.position is None:
            rospy.sleep(0.1)
        self.send_depth_goal(10)
        self.startPub.publish(msg)
        self.start= True
        
    def position_cb(self, msg):
        
        self.position = NED(msg.position.north, msg.position.east, msg.position.depth)
        
    def battery_level_cb(self, msg):
        
        self.batteryLevel = msg.data
        if self.batteryLevel < 45 and not self.alertPublished:
            alert = NavSts()
            alert.position.north = self.position.north
            alert.position.east = self.position.east
            alert.header.frame_id = rospy.get_namespace()
            self.chargeType = 'consumer'
            self.alertPublished = True
            self.batteryAlertPub.publish(alert)
            b = Bool()
            b.data = True
            self.sleepModePub.publish(b)
            self.send_depth_goal(0.1)

    def charging_procedure(self, msg):
        
        # gather data
        return
        
    def charge_info_srv(self, req):
        
        if self.chargeType is None:
            return
        
        return {'battery_level': self.batteryLevel, 'x': self.position.north, 'y': self.position.east, 'z': 0, 'type': self.chargeType}
    
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
    
           
