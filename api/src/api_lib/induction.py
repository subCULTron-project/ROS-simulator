#!/usr/bin/env python

"""
Induction API class.
"""

__author__ = "barbanas"


from misc_msgs.srv import GetChargeInfo, SendEnergy
from std_msgs.msg import Bool, Float64
import rospy

class Induction(object):
    
    def __init__(self, nCoils):
        self.enabledCoils = []
        self.disabledCoils = range(nCoils)
        rospy.Service('send_energy', SendEnergy, self.send_energy_srv)
    
        self.pubCharge = {}
        self.pubDrain = rospy.Publisher("draining", Float64, queue_size = 1)
        
    def enable_coil(self, coil):
        '''
        Enable coil.
        Returns:
            success: bool
        '''
        if coil not in self.enabledCoils and coil in self.disabledCoils:
            self.enabledCoils.append(coil)
            self.disabledCoils.remove(coil)
            return True
        else:
            return False 
    
    def disable_coil(self, coil):
        '''
        Disable coil.
        Returns:
            success: bool
        '''
        if coil not in self.disabledCoils and coil in self.enabledCoils:
            self.disabledCoils.append(coil)
            self.enabledCoils.remove(coil)
            return True
        else:
            return True
    
    def request_charging(self, receiver, coil):
        
        try:
            rospy.wait_for_service(receiver + 'send_energy', 0.3)
            send_energy = rospy.ServiceProxy(receiver + 'send_energy', SendEnergy)
            if not send_energy():
                return False
            
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        except rospy.ROSException, e:
            print "Service call failed: %s"%e
        
        # send energy to receiver
        self.pubCharge[receiver] = rospy.Publisher(receiver + "charging", Float64, queue_size=1)
        
        
    def send_energy(self, receiver):
        '''
        Send energy through coil.
        Args:
            receiver: string, namespace of receiver
            coil: int, index of coil on receiver side??
        Returns:
            success: bool
        '''
        self.pubCharge[receiver].publish(0.1)
        self.pubDrain.publish(0.1)
        
    def stop_sending(self, receiver):
        
        self.pubCharge[receiver] = None
        
    
    def get_charging_power(self, coil):
        '''
        Get charging power on coil.
        Returns:
            power: float, charging power
        '''
        pass     
    
    def send_energy_srv(self, req):
        '''
        '''
        return self.enable_coil(req.coil)
    
    