#!/usr/bin/env python

"""
WiFi API class.
"""
from std_msgs.msg import String
import rospy

class WiFi(object):
    
    def __init__(self):
        
        self.wifiSub = rospy.Subscriber("wifi", String, self.message_cb)
        self.wifiSubBroad = rospy.Subscriber("/wifi", String, self.message_cb)
        self.wifiPub = None
        self.buffer = []
    
    def send(self, destination, data):
        '''
        Send a message over WiFi.
        Args:
           destination: string, destination id
           data: string, message
        '''
        if destination == 0:
            self.wifiPub = rospy.Publisher("/wifi", String, queue_size = 1)
            self.wifiPub.publish(data)
        else:
            self.wifiPub = rospy.Publisher(destination + "wifi", String, queue_size=1, latch=True)
            self.wifiPub.publish(data)
    
    def receive(self):
        '''
        Receive a message over WiFi.
        '''
        if len(self.buffer) == 0:
            return None
        else:
            return self.buffer.pop()
    
    def message_cb(self, msg):    
        self.buffer.append(msg.data)