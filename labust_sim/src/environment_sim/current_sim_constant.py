#!/usr/bin/python

"""
Implementation of constant current simulation.
World is divided into areas. For every area a constant current is defined.
---
Current specification file is located in amussel/data/simulation/current/ directory.
Default filename is currentInfo.txt. For loading different temperature values, you
can create a new file and set param "current_spec_filename" of "environment_sim" node 
to desired filename (simulation config file is amussel/dat/simulation/simulation_standard.xml).
"""

from current_sim import CurrentSim
from geometry_msgs.msg import Point, TwistStamped
import rospy
import rospkg
rospack = rospkg.RosPack()

__author__ = "barbanas"

class CurrentSimConstant(CurrentSim):
    """
    CurrentSimPeriodic inherits CurrentSim class. It has to implement
    get_current and ok functions. For other current simulation functions,
    read the current_sim.py file.
    """
    def __init__(self, filename):
        super(CurrentSimConstant, self).__init__()

        self.currentInfoLoaded = False
        self.currentRegions = []
        self.load_current_info(rospack.get_path("amussel") + "/data/simulation/current/" + filename)

    def get_current(self, x, y):
        """
        Gets the current value on position (x, y).
        Current data type is geometry_msgs/TwistStamped.
        """
        if not self.ok():
            return None

        for region in self.currentRegions:
            if region.point_in_region(Point(x, y, 0)):
                return region.get_current()
        
        return None

    def ok(self):
        return self.start and self.currentInfoLoaded

    def load_current_info(self, filename):
        try:
            f = open(filename)
        except IOError as e:
            rospy.logerr("I/O error({0}): {1}".format(e.errno, e.strerror))
            return
        
        lines = f.readlines()         
        f.close()

        i = 0
        # remove comments and empty lines
        while i < len(lines):
            lines[i] = lines[i].replace('\n', '')
            if len(lines[i]) == 0:
                del lines[i]
                continue
            if lines[i][0] == '#':
                del lines[i]
                continue
            i += 1

        err = 0

        p1 = Point()
        p2 = Point()
        curr = TwistStamped()
        
        i = 0
        while i < len(lines): 
            # upper left point
            data = lines[i].split(' ')
            rospy.logerr( data)
            i += 1
            if len(data) != 2:
                err = -1
                break
            p1.x = float(data[0]) # north
            p1.y = float(data[1]) # east
            
            # lower right point
            data = lines[i].split(' ')
            rospy.logerr( data)
            i += 1
            if len(data) != 2:
                err = -1
                break
            p2.x = float(data[0]) # north
            p2.y = float(data[1]) # east

            # current value
            data = lines[i].split(' ')
            i += 1
            rospy.logerr( data)
            if len(data) != 3:
                err = -1
                break
            curr.twist.linear.x = float(data[0]) # north
            curr.twist.linear.y = float(data[1]) # east
            curr.twist.linear.z = float(data[2]) # depth

            self.currentRegions.append(CurrentRegion(p1, p2, curr))
        # err (0: no error, -1: file format error)
        if err == -1:
            rospy.logerr("ERROR current file format: %s", filename)
        else:
            self.currentInfoLoaded = True


class CurrentRegion:

    def __init__(self, p1, p2, c):    
        self.topLeft = Point(p1.x, p1.y, 0)
        self.bottomRight = Point(p2.x, p2.y, 0)
        self.current = TwistStamped()
        self.current.twist.linear.x = c.twist.linear.x
        self.current.twist.linear.y = c.twist.linear.y
        self.current.twist.linear.z = c.twist.linear.z

    def point_in_region (self, point):
        # x --> north, y --> east
        if (point.y > self.topLeft.y) and (point.y <= self.bottomRight.y) and (point.x <= self.topLeft.x) and (point.x > self.bottomRight.x):
            return True
        else:
            return False

    def get_current(self):
        return self.current
        