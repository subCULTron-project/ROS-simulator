#!/usr/bin/env python
'''
Created on Feb 04, 2013

@author: dnad
'''
import rospy;
import math;
import numpy;
from auv_msgs.msg import NavSts;
      
class MessageTransformer:
    def __init__(self):
        self.stateHat = rospy.Subscriber("stateHat", NavSts, self.onState)
        self.trackState = rospy.Subscriber("trackState", NavSts, self.onTrack)
        self.orientationSub = rospy.Subscriber("orientation", NavSts, self.onOrientation)
        self.gotoState = rospy.Subscriber("gotoState", NavSts, self.onGoTo)
        self.hasGoto = False
        self.pub = rospy.Publisher("stateRef", NavSts)
        
        self.track = NavSts();
        self.orientation = NavSts();
        self.goto = NavSts();
        
        self.radius = 4;
        self.maxAngle = 15.0/180.0*numpy.pi;
        self.Kr = 10;
        
    def onGoTo(self,data):
        self.goto = data;
        self.hasGoto = True
           
    def onTrack(self, data):
        self.track = data;
    
    def onOrientation(self, data):
        self.orientation = data;
    
    def onState(self, data):
        ref = NavSts()
        dy = self.track.position.east - data.position.east;
        dx = self.track.position.north - data.position.north;
        ref.orientation.yaw = math.atan2(dy,dx);
        
        #Diver to goto point yaw
        dy2 = self.goto.position.east - self.track.position.east;
        dx2 = self.goto.position.north - self.track.position.north;
        dgyaw = 0;
        if math.sqrt(dy2*dy2 + dx2*dx2) > 1:
            dgyaw = math.atan2(dy2,dx2);
        
        dyaw = 0
        if self.hasGoto:
            delta = (-self.track.orientation.yaw + dgyaw + numpy.pi)%(2*numpy.pi) - numpy.pi; 
            dyaw = self.maxAngle*math.tanh(self.Kr*delta);
       
        #print("Desired yaw diver: %f, desired yaw target: %f,  Dyaw: %f, delta: %f" 
        #      % (self.track.orientation.yaw, dgyaw, dyaw, delta))
       
        ref.position.north = self.track.position.north + self.radius*math.cos(self.orientation.orientation.yaw + dyaw);
        ref.position.east = self.track.position.east + self.radius*math.sin(self.orientation.orientation.yaw + dyaw);
        ref.body_velocity.x = -self.track.body_velocity.x
        ref.body_velocity.y = -self.track.body_velocity.y;

        pb = numpy.array([ref.position.north, ref.position.east]);
        pg = numpy.array([self.goto.position.north, self.goto.position.east]);
        if numpy.linalg.norm(pb-pg,2) < self.radius/2:
            ref.position.north = self.goto.position.north;
            ref.position.east = self.goto.position.east;
        
        ref.position.depth = self.track.position.depth;
        ref.header.stamp = rospy.Time.now();
                
        self.pub.publish(ref)
                    
if __name__ == "__main__":
    rospy.init_node("diver_track");
    MessageTransformer();
    rospy.spin();
        
        