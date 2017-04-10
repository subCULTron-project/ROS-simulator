#!/usr/bin/env python

"""
Initializes ROS node for high level mission control.
"""

__author__ = ""

import rospy
from auv_msgs.msg import NED, NavSts
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Bool
from math import atan2, radians, degrees, sqrt, pow, fabs
import action_library

class ScenarioController(object):
    
    def __init__(self):
        
        # - - -
        # class variables 
        # start flag, all callback functions will wait until the controller gets the scenario_start message
        self.start = False
        
        # ping structures
        self.pingCount = 0
        self.pingAvgHeading = NED(0, 0, 0) 
        self.pingSum = NED(0, 0, 0)
        
        # current heading
        self.current = NED(0, 0, 0)
        
        # position
        self.position = None
        
        # battery call
        self.batteryCall = None
        
        # - - -
        # topic subscribers 
        rospy.Subscriber('/scenario_start', Bool, self.start_cb)
        rospy.Subscriber('/battery_alert', NED, self.battery_cb)
        rospy.Subscriber('position', NavSts, self.position_cb)
        rospy.Subscriber('ping_sensor', NED, self.ping_sensor_cb)
        rospy.Subscriber('current_sensor', TwistStamped, self.current_sensor_cb)

        # - - -
        # topic publishers 
        self.startPub = rospy.Publisher('start_sim', Bool, queue_size=1)
        self.stateRefPub = rospy.Publisher('stateRef', NavSts, queue_size=1)

        # - - -
        # timers
        self.timerOne = rospy.Timer(rospy.Duration(1), self.timer_one_cb)
        
        # - - -
        # hand over control to ROS 
        rospy.spin()
       
    #--------------------# 
    # scenario functions #
    #--------------------#
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

    #--------------------#
    # callback functions #
    #--------------------#

    def start_cb(self, msg):
        
        self.send_depth_goal(10)
        # publish start to all sensors
        self.startPub.publish(msg)
        
    def position_cb(self, msg):
        
        self.position = NED(msg.position.north, msg.position.east, msg.position.depth)
        
    def battery_cb(self, msg):
        
        self.batteryCall = NED(msg.north, msg.east, msg.depth)
        #self.send_position_goal = self.batteryCall
        
        pos_old = NED(self.position.north, self.position.east, self.position.depth)
        goal = NED(msg.north, msg.east, pos_old.depth)
        action_library.send_position_goal(self.stateRefPub, goal)
        
    def ping_sensor_cb(self, msg):
        
        # check for start
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
        
        # check for start
        if self.start is None:
            return
        
        self.current.north = msg.twist.linear.x
        self.current.east = msg.twist.linear.y
        
        self.check_current_suitability()
    
    def timer_one_cb(self, event):

        # check for start
        if self.start is None:
            return
        
        rospy.loginfo("aMussel %s: Current time in seconds is %s ", rospy.get_namespace(), rospy.get_time())
    
    #------------------#
    # action functions #
    #------------------#

    def send_depth_goal(self, depth):

        pos_old = NED(self.position.north, self.position.east, self.position.depth)
        goal = NED(pos_old.north, pos_old.east, depth)
        
        # send the goal to depth controller
        if action_library.send_depth_goal(self.stateRefPub, goal) == -1:
            return False

        # init error structure
        pos_err = []
        # wait until goal is reached
        while not rospy.is_shutdown():
            # distance from goal depth
            dl = abs(goal.depth - self.position.depth)
            # remembers the last 10 errors --> to change this, just replace 10 with desired number 
            # (bigger err structure equals more precision, but longer time to acheive the goal)
            if len(pos_err) < 10:
                pos_err.append(dl)
            else:
                pos_err.pop(0)
                pos_err.append(dl)

            # goal precision in m --> for better precision replace the number with the smaller one, e.g. 0.001 for 1 mm precision
            # for worse precision, insert greater one, e.g. 1 for 1 m precision
            if (len(pos_err) == 10) and (fabs(sum(pos_err) / len(pos_err)) < 0.05):  # mission is successfully finished
                return True
            else:  # mission is still ongoing, sleep for 0.1 s
                rospy.sleep(rospy.Duration(0.1))
                
	#def send_position_goal(self, depth):

        #pos_old = NED(self.position.north, self.position.east, self.position.depth)
        #goal = NED(pos_old.north, pos_old.east, depth)
        
        ## send the goal to depth controller
        #if action_library.send_depth_goal(self.stateRefPub, goal) == -1:
            #return False

        ## init error structure
        #pos_err = []
        ## wait until goal is reached
        #while not rospy.is_shutdown():
            ## distance from goal depth
            #dl = abs(goal.depth - self.position.depth)
            ## remembers the last 10 errors --> to change this, just replace 10 with desired number 
            ## (bigger err structure equals more precision, but longer time to acheive the goal)
            #if len(pos_err) < 10:
                #pos_err.append(dl)
            #else:
                #pos_err.pop(0)
                #pos_err.append(dl)

            ## goal precision in m --> for better precision replace the number with the smaller one, e.g. 0.001 for 1 mm precision
            ## for worse precision, insert greater one, e.g. 1 for 1 m precision
            #if (len(pos_err) == 10) and (fabs(sum(pos_err) / len(pos_err)) < 0.05):  # mission is successfully finished
                #return True
            #else:  # mission is still ongoing, sleep for 0.1 s
                #rospy.sleep(rospy.Duration(0.1))

if __name__ == "__main__":
    
    rospy.init_node("scenario_controller")
    
    try:
        # creates new ScenarioController object (calls function __init__)
        controller = ScenarioController()
            
    except rospy.ROSInterruptException:
        pass
    
           
