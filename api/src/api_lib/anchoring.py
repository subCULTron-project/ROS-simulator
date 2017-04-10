#!/usr/bin/env python

"""
Anchoring API class.
"""

from std_srvs.srv import Trigger
import rospy

class Anchor(object):
    
    def __init__(self):
        self.status = -1
    
    def hold(self):
        '''
        Activate anchor.
        Returns:
            success: bool
        '''
        if self.status == 1:
            return False
        else:
            self.status = 1
            hold = rospy.ServiceProxy('anchor', Trigger)
            try:
                hold()
                return True
            except rospy.ServiceException as exc:
                print("Service did not process request: " + str(exc))
    
        return False
    
    def release(self):
        '''
        Deactivate anchor.
        Returns:
            success: bool
        '''
        if self.status == 0:
            return False
        else:
            self.status = 0
            hold = rospy.ServiceProxy('anchor', Trigger)
            try:
                hold()
                return True
            except rospy.ServiceException as exc:
                print("Service did not process request: " + str(exc))
    
        return False
    
    def check_status(self):
        '''
        Check anhor status.
        Returns:
            status: int, predefined status id
        '''
        
        return self.status