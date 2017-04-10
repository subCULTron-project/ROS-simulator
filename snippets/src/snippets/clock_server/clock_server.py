#!/usr/bin/env python
'''
Created on Mar 16, 2015
 \todo Add license information here
@author: dnad
'''
from __future__ import print_function

import rospy
from std_msgs.msg import Float64

from functools import partial
           
class ClockServer:
    def __init__(self):
        # Get the clock topics which should be tracked
        # Populate the subscribers
        topics = rospy.get_param("~track_clocks", "")
        self.create_subs(topics)
                        
        # Start the clock publisher
        self.ctime = rospy.Publisher("clock",Float64,queue_size=1)         
                
    def create_subs(self, topics):
        self.diff_pubs = {}
        for topic in topics:
            self.diff_pubs[topic] = rospy.Publisher("clock_diff" + topic, Float64, queue_size=1)
            rospy.Subscriber(topic, Float64, partial(self.on_time, topic))
    
    def on_time(self, name, data):
        dt = Float64()
        dt.data = data.data - rospy.Time.now().to_sec()
        self.diff_pubs[name].publish(dt)
        
    def start(self):
        rate = rospy.Rate(1.0)
        msg = Float64()
        while not rospy.is_shutdown():    
            msg.data = rospy.Time.now().to_sec()
            self.ctime.publish(msg)
            rate.sleep()


if __name__ == '__main__':
    rospy.init_node("clock_server")
    srv = ClockServer()
    srv.start()
   