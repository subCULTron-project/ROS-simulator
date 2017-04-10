#!/usr/bin/python

"""
Implementation of constant temperature simulation.
World is divided into areas. For every area a constant temperature is defined.
---
Temperature specification file is located in amussel/data/simulation/temperature/ directory.
Default filename is temperatureInfo.txt. For loading different temperature values, you
can create a new file and set param "temperature_spec_filename" of "environment_sim" node 
to desired filename (simulation config file is amussel/dat/simulation/simulation_standard.xml).
"""

from temperature_sim import TemperatureSim
from geometry_msgs.msg import Point
from std_msgs.msg import Float64
import rospy
import rospkg
rospack = rospkg.RosPack()

__author__ = "barbanas"

class TemperatureSimConstant(TemperatureSim):
    """
    TemperatureSimPeriodic inherits TemperatureSim class. It has to implement
    get_temperature and ok functions. For other temperature simulation functions,
    read the temperature_sim.py file.
    """
    def __init__(self, filename):
        super(TemperatureSimConstant, self).__init__()

        self.tempInfoLoaded = False
        self.tempRegions = []
        self.load_temperature_info(rospack.get_path("amussel") + "/data/simulation/temperature/" + filename)

    def get_temperature(self, x, y):
        """
        Gets the temperature value on position (x, y).
        Temperature data type is std_msgs/Float64.
        """
        if not self.ok():
            return None

        for region in self.tempRegions:
            if region.point_in_region(Point(x, y, 0)):
                return region.get_temperature()
        
        return None

    def ok(self):
        return self.start and self.tempInfoLoaded

    def load_temperature_info(self, filename):
        try:
            f = open(filename)
        except IOError as e:
            rospy.logerr("I/O error({0}): {1}".format(e.errno, e.strerror))
        
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
        temp = None
        
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

            # temperature value
            data = lines[i].split(' ')
            i += 1
            rospy.logerr( data)
            if len(data) != 1:
                err = -1
                break
            temp = float(data[0]) # temp
        
            self.tempRegions.append(TemperatureRegion(p1, p2, temp))
        # err (0: no error, -1: file format error)
        if err == -1:
            rospy.logerr("ERROR temperature file format: %s", filename)
        else:
            self.tempInfoLoaded = True


class TemperatureRegion:

    def __init__(self, p1, p2, t):    
        self.topLeft = Point(p1.x, p1.y, 0)
        self.bottomRight = Point(p2.x, p2.y, 0)
        self.temp = t

    def point_in_region (self, point):
        # x --> north, y --> east
        if (point.y > self.topLeft.y) and (point.y <= self.bottomRight.y) and (point.x <= self.topLeft.x) and (point.x > self.bottomRight.x):
            return True
        else:
            return False

    def get_temperature(self):
        return self.temp
        