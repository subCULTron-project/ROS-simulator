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
from api_lib import docking, battery, clock, induction, wifi, anchoring
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

        self.i = induction.Induction(1)
        self.w = wifi.WiFi()
        
        rospy.spin()

    def start_cb(self, msg):
        
        while self.position is None:
            rospy.sleep(0.1)
        
        # start device simulation
        self.startPub.publish(msg)
        
        
        # API test
        #self.clock_test()
        #self.battery_test()
        
        # WiFi test
        #self.wifi_test()
        
        # anchoring test
        self.anchor_test()
        
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
        
    def wifi_test(self):
        
        message = None
        while message is None:
            message = self.w.receive()
            rospy.sleep(0.5)
        print rospy.get_namespace() + " got message: \"" + message + "\""
        
    def anchor_test(self):
        
        print "\n%%%%% Anchor test %%%%%\n"
        a = anchoring.Anchor()
        raw_input("Press any key to hold position")
        print "Holding position...\n"
        a.hold()
    
        raw_input("Press any key to release position")
        print "Releasing position..."
        a.release()
        
    def position_cb(self, msg):
        
        self.position = NED(msg.position.north, msg.position.east, msg.position.depth)
        
if __name__ == "__main__":
    
    rospy.init_node("scenario_controller")
    
    try:
        controller = ScenarioController()
            
    except rospy.ROSInterruptException:
        pass
    
           
