#!/usr/bin/env python

"""
Initializes ROS node for high level mission control.
"""

__author__ = "barbanas"

import rospy
from auv_msgs.msg import NED, NavSts
from std_msgs.msg import Bool, Float64, String
from misc_msgs.srv import GetChargeInfo, GetDockingInfo

class ScenarioController(object):
    
    def __init__(self):
        
        self.position = None
        self.startPub = rospy.Publisher('start_sim', Bool, queue_size=1)
        rospy.Subscriber('/scenario_start', Bool, self.start_cb)
        rospy.Subscriber('position', NavSts, self.position_cb)

        # battery
        self.sleepModePub = rospy.Publisher('power_sleep_mode', Bool, queue_size=1)
        self.chargePub = rospy.Publisher('charging', Float64, queue_size=1)
        self.drainPub = rospy.Publisher('draining', Float64, queue_size=1)

        rospy.spin()

    def start_cb(self, msg):
        
        while self.position is None:
            rospy.sleep(0.1)
        
        # start device simulation
        
        self.startPub.publish(msg)
        
        self.battery_test()
        
        # motion
        stateRefPub = rospy.Publisher('stateRef', NavSts, queue_size=1)
        
    def battery_test(self):
        
        ####################
        ### battery test ###
        ####################
        
        print "\n%%%%% battery test %%%%"
        
        # get battery level
        batteryLevel = 0
        charge_info = rospy.ServiceProxy('get_battery_level', GetChargeInfo)
        try:
            c = charge_info()
            batteryLevel = c.battery_level
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))
        
        print "\nInitial battery level: " + str(batteryLevel)
        rospy.sleep(rospy.Duration(3))
        
        try:
            c = charge_info()
            batteryLevel = c.battery_level
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))
        
        print "Battery level after 3 seconds: " + str(batteryLevel)
        
        self.sleepModePub.publish(True)
        print "\nActivated sleep mode..."
        rospy.sleep(rospy.Duration(2))
        
        try:
            c = charge_info()
            batteryLevel = c.battery_level
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))
        
        print "Battery level after 2 seconds: " + str(batteryLevel)
        
        print "\nCharging with 0.1% for 5 seconds, charging rate: 0.5s... \nExpected battery increase: 5 / 0.5 * 0.1 = 1"
        r = rospy.Rate(2) # 2hz
        i = 0
        while i < 10:
            self.chargePub.publish(0.1)
            i += 1
            r.sleep()
        
        try:
            c = charge_info()
            batteryLevel = c.battery_level
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))
        
        print "Battery level after charging for 5 seconds: " + str(batteryLevel)
        
        print "\nDraining with 0.2% for 5 seconds, charging rate: 0.5s... \nExpected battery decrease: 5 / 0.5 * 0.2 = 2"
        r = rospy.Rate(2) # 2hz
        i = 0
        while i < 10:
            self.drainPub.publish(0.2)
            i += 1
            r.sleep()
        
        try:
            c = charge_info()
            batteryLevel = c.battery_level
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))
        
        print "Battery level after draining for 5 seconds: " + str(batteryLevel)
        
        self.sleepModePub.publish(False)
        print "\nDeactivated sleep mode..."
        rospy.sleep(rospy.Duration(2))
        
        try:
            c = charge_info()
            batteryLevel = c.battery_level
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))
        
        print "Battery level after 2 seconds: " + str(batteryLevel)
        
    def docking_test(self):
        
        docking_avail = rospy.ServiceProxy('check_docking_availability', GetDockingInfo)
        try:
            d = docking_avail()
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))
        
        print "\nInitial docking status: " + d.slots + "   " + d.available_slots
        rospy.sleep(rospy.Duration(3))
        
        
    def position_cb(self, msg):
        
        self.position = NED(msg.position.north, msg.position.east, msg.position.depth)
        
if __name__ == "__main__":
    
    rospy.init_node("scenario_controller")
    
    try:
        controller = ScenarioController()
            
    except rospy.ROSInterruptException:
        pass
    
           
