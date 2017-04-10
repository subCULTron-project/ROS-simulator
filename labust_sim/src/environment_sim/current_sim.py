#!/usr/bin/python

"""
Interface for current simulation class.
"""

__author__ = "barbanas"

import rospy
from std_msgs.msg import Bool
from auv_msgs.msg import NavSts, NED
from geometry_msgs.msg import TwistStamped

class CurrentSim(object):
    """
    Abstract class for current simulation.
    It provides the template on which all actual current classes have to be implemented.
    """
    def __init__(self):

        self.start = False
        self.position = NED()
        self.oldCurrent = None

        if rospy.has_param('~current_publish_rate'):
            self.currPubRate = rospy.get_param('~current_publish_rate')
        else:
            self.currPubRate = 0.2
        
        self.currPubTimeout = rospy.get_time() + self.currPubRate

        rospy.Subscriber("start_sim", Bool, self.start_cb)
        rospy.Subscriber("position", NavSts, self.position_cb)

        self.currSensorPub = rospy.Publisher("current_sensor", TwistStamped, queue_size=1)

    def get_current(self, x, y):
        """
        Gets the current value on position (x, y).
        Current data type is geometry_msgs/TwistStamped.
        """
        raise NotImplementedError("Should have implemented this")

    def ok(self):
        """
        Returns True/False, depending if the current simulator is ready to work.
        This has to be implemented for every current sim class.
        """
        raise NotImplementedError("Should have implemented this")

    # callback functions

    def start_cb(self, msg):
        self.start = True

    def position_cb(self, msg):
        if not self.ok():
            return

        self.position.north = msg.position.north
        self.position.east = msg.position.east
        self.position.depth = msg.position.depth
 
        newCurrent = self.get_current(self.position.north, self.position.east)

        if newCurrent is None:
            newCurrent = TwistStamped()
            newCurrent.twist.linear.x = 0
            newCurrent.twist.linear.y = 0
            newCurrent.twist.linear.z = 0
            self.oldCurrent = newCurrent
            self.currSensorPub.publish(newCurrent)
            return

        publish = False
        if self.oldCurrent is None:
            # if current was never published
            publish = True
        else:
            # if current value changed
            publish = (self.oldCurrent.twist.linear.x != newCurrent.twist.linear.x) or (self.oldCurrent.twist.linear.y != newCurrent.twist.linear.y) \
                            or (self.oldCurrent.twist.linear.z != newCurrent.twist.linear.z)

        # publish only when current changes or current was not published in some time
        if publish or (self.currPubTimeout < rospy.get_time()):
            # save current value
            self.oldCurrent = newCurrent
            # set current sensor info
            self.currSensorPub.publish(newCurrent)
            # move timeout to a different time
            self.currPubTimeout = rospy.get_time() + self.currPubRate
