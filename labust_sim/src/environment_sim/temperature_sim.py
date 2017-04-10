#!/usr/bin/python

"""
Interface for temperature simulation class.
"""

__author__ = "barbanas"

import rospy
from std_msgs.msg import Bool, Float64
from auv_msgs.msg import NavSts, NED

class TemperatureSim(object):
    """
    Abstract class for temperature simulation.
    It provides the template on which all actual temperature simulation classes have to be implemented.
    """
    def __init__(self):

        self.start = False
        self.position = NED()
        self.oldTemp = None

        if rospy.has_param('~temp_publish_rate'):
            self.tempPubRate = rospy.get_param('~temp_publish_rate')
        else:
            self.tempPubRate = 0.2
        
        self.tempPubTimeout = rospy.get_time() + self.tempPubRate

        rospy.Subscriber("start_sim", Bool, self.start_cb)
        rospy.Subscriber("position", NavSts, self.position_cb)

        self.tempSensorPub = rospy.Publisher("temperature_sensor", Float64, queue_size=1)

    def get_temperature(self, x, y):
        """
        Gets the temperature value on position (x, y).
        Temperature data type is std_msgs/Float64.
        """
        raise NotImplementedError("Should have implemented this")

    def ok(self):
        """
        Returns True/False, depending if the temperature simulator is ready to work.
        This has to be implemented for every temperature sim class.
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
 
        newTemp = self.get_temperature(self.position.north, self.position.east)

        if newTemp is None:
            newTemp = 0
            self.oldTemp = newTemp
            self.tempSensorPub.publish(newTemp)
            return

        publish = False
        if self.oldTemp is None:
            # if temperature was never published
            publish = True
        else:
            # if temperature value changed
            publish = self.oldTemp != newTemp

        # publish only when temperature changes or temperature was not published in some time
        if publish or (self.tempPubTimeout < rospy.get_time()):
            # save temperature value
            self.oldTemp = newTemp
            # set temperature sensor info
            self.tempSensorPub.publish(newTemp)
            # move timeout to a different time
            self.tempPubTimeout = rospy.get_time() + self.tempPubRate