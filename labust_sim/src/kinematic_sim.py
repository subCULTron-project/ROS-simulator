#!/usr/bin/env python
'''
Created on Feb 04, 2013

@author: dnad
'''
import rospy;
from sensor_msgs.msg import Joy;
from auv_msgs.msg import NavSts;

import numpy
import math
import tf
      
class DiverSim:
    def __init__(self):
        '''Setup subscribers'''
        rospy.Subscriber("joy", Joy, self.onJoy);     
        '''Setup publishers'''
        self.out = rospy.Publisher("TrackPoint", NavSts);
        self.point = NavSts();
        self.u_max = 0.5;
        self.angle_max = 0.25;
        self.Ts = 0.1;   
        
        '''Temporary addition for lat-lon outputs'''
        self.point.global_position.latitude = 44;
        self.point.global_position.longitude = 15.305;
        self.point.position_variance.north = 167.123;
        self.point.position_variance.east = 155.623;
        
        self.broadcaster = tf.TransformBroadcaster();
        self.base_frame = rospy.get_param("~base_frame","base_link");
        self.underactuated = rospy.get_param("~underactuated",False);
    
    def onJoy(self,joy):
        u = joy.axes[1]*self.u_max;
        v = 0;
        w = joy.axes[3]*self.u_max;
        p = 0;
        q = 0;
        r = -joy.axes[0]*self.angle_max
       
        psi = self.point.orientation.yaw;

        if not self.underactuated: v = -joy.axes[0]*self.u_max;
        
        #kinematic step 
        xdot = u*math.cos(psi) - v*math.sin(psi);
        ydot = u*math.sin(psi) + v*math.cos(psi);
        self.point.position.north += self.Ts * xdot;
        self.point.position.east += self.Ts * ydot;
        self.point.position.depth += self.Ts * w;
        self.point.orientation.yaw += self.Ts * r; 

        '''Temporary addition for lat-lon outputs'''
        self.point.global_position.latitude += 0.0001*xdot*self.Ts;
        self.point.global_position.longitude += 0.0001*ydot*self.Ts;
              
        self.point.body_velocity.x = u;
        self.point.body_velocity.y = v;
        self.point.body_velocity.z = w;
        self.point.orientation_rate.roll = p;
        self.point.orientation_rate.pitch = q;
        self.point.orientation_rate.yaw = r;
        self.out.publish(self.point);
        
        #self.broadcaster.sendTransform((0,0,0), q.conjugate(), rospy.Time.now(), self.tf_prefix + "/uwsim_frame", self.tf_prefix + "/local");
        q = tf.transformations.quaternion_from_euler(0, 0, self.point.orientation.yaw)
        self.broadcaster.sendTransform((self.point.position.north,
                                        self.point.position.east,
                                        self.point.position.depth), q, rospy.Time.now(), 
                                        self.base_frame, 
                                       "local");
        
if __name__ == "__main__":
    rospy.init_node("diver_sim");
    dp = DiverSim();    
    rospy.spin();
        
