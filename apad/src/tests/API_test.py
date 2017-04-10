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
from api_lib import docking, battery, clock, induction, wifi
#import action_library

class ScenarioController(object):
    
    def __init__(self):
        
        self.position = None
        self.startPub = rospy.Publisher('start_sim', Bool, queue_size=1)
        rospy.Subscriber('/scenario_start', Bool, self.start_cb)
        rospy.Subscriber('position', NavSts, self.position_cb)

        # battery
        self.chargePub = rospy.Publisher('charging', Float64, queue_size=1)
        self.drainPub = rospy.Publisher('draining', Float64, queue_size=1)

        self.i = induction.Induction(4)
        self.w = wifi.WiFi()
        
        rospy.spin()

    def start_cb(self, msg):
        
        while self.position is None:
            rospy.sleep(0.1)
        
        # start device simulation
        self.startPub.publish(msg)
        
        # API test
        self.clock_test()
        
        #self.battery_test()
        
        #raw_input("\n...press any key for docking test...")
        #self.docking_test()
        
        #raw_input("\n...press any key for induction test...")
        #self.induction_test()
        
        raw_input("\n...press any key for WiFi test...")
        self.wifi_test()
        
        # motion
        stateRefPub = rospy.Publisher('stateRef', NavSts, queue_size=1)
        
    def clock_test(self):
        
        print "\n%%%%% Clock test %%%%%"
        
        c = clock.Clock()
        
        print "Time in msec is: " + str(c.get_time())
        
    def battery_test(self):
        
        print "\n%%%%% Battery test %%%%%"
        b = battery.Battery()
         
        # get battery level
        print "\nInitial battery level: " + str(b.get_level())
        rospy.sleep(rospy.Duration(3))
        
        print "Battery level after 3 seconds: " + str(b.get_level())
        
        b.toggle_sleep_mode(True)
        print "\nActivated sleep mode..."
        rospy.sleep(rospy.Duration(2))
        
        print "Battery level after 2 seconds: " + str(b.get_level())
        
        print "\nCharging with 0.1% for 5 seconds, charging rate: 0.5s... \nExpected battery increase: 5 / 0.5 * 0.1 = 1"
        r = rospy.Rate(2) # 2hz
        i = 0
        while i < 10:
            self.chargePub.publish(0.1)
            i += 1
            r.sleep()
        
        print "Battery level after charging for 5 seconds: " + str(b.get_level())
        
        print "\nDraining with 0.2% for 5 seconds, charging rate: 0.5s... \nExpected battery decrease: 5 / 0.5 * 0.2 = 2"
        r = rospy.Rate(2) # 2Hz
        i = 0
        while i < 10:
            self.drainPub.publish(0.2)
            i += 1
            r.sleep()
        
        print "Battery level after draining for 5 seconds: " + str(b.get_level())
        
        b.toggle_sleep_mode(False)
        print "\nDeactivated sleep mode..."
        rospy.sleep(rospy.Duration(2))
        
        print "Battery level after 2 seconds: " + str(b.get_level())
        
        print "\n---"
        
    def docking_test(self):
        
        print "\n%%%%% Docking test %%%%%"
        
        d = docking.Docking()
        
        print "\nInitial docking status: " + str(d.check_availability())
        print "Slots: " + str(d.check_docking_slots()) + ", available slots: " + str(d.check_candidates())
   
        print "\nRequesting docking on slot 1 for agent amussel1 and slot 3 for amussel2..."
        d.lock_robot(1, "amussel1")
        d.lock_robot(3, "amussel2")
        
        res = d.get_docking_status()
        print "Slots: " + str(res[1]) + ", available slots: " + str(res[2])
        
        print "\nRequesting release for slot 1..."
        d.release_robot(1)
        
        res = d.get_docking_status()
        print "Slots: " + str(res[1]) + ", available slots: " + str(res[2])
        
        print "\n---"
        
    def induction_test(self):
        
        b = battery.Battery()
        
        print "\n%%%%% Induction test %%%%%"
        from misc_msgs.srv import GetChargeInfo
        
        print "Receiver: amussel1, sender: apad1"
        
        charge_info = rospy.ServiceProxy('/amussel1/get_battery_level', GetChargeInfo)
        try:
            c = charge_info()
            batteryLevel = c.battery_level
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))
        
        print "Battery level, amussel1: " + str(batteryLevel)
        print "Battery level, apad1: " + str(b.get_level())  
        
        print "\nSending energy to amussel1 for 5 seconds..."
        
        self.i.request_charging("/amussel1/", 1)
        r = rospy.Rate(2)
        i = 0
        while i < 20:
            self.i.send_energy("/amussel1/")
            r.sleep()
            i += 1
            
        self.i.stop_sending("/amussel1/")
        
        try:
            c = charge_info()
            batteryLevel = c.battery_level
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))
        
        print "Battery level, amussel1: " + str(batteryLevel)
        print "Battery level, apad1: " + str(b.get_level())  
        
    def wifi_test(self):
        
        msg = "Message from aPad to aMussel"
        print rospy.get_namespace() + " sending message: \"" + msg + "\"" 
        self.w.send("/amussel1/", msg)
        
        
    def position_cb(self, msg):
        
        self.position = NED(msg.position.north, msg.position.east, msg.position.depth)
        
if __name__ == "__main__":
    
    rospy.init_node("scenario_controller")
    
    try:
        controller = ScenarioController()
            
    except rospy.ROSInterruptException:
        pass
    
           
