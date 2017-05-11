#!/usr/bin/python

from auv_msgs.msg import NED
import rospy

"""
Environment simulation class. Initializes the ROS node for complete environment simulation.
It includes current simulation, temperature simulation, noise source simulation.
"""

__author__ = "barbanas"


class EnvironmentSim(object):

    def __init__(self):
        self.start = False

        # current
        if rospy.has_param('~current_mode'):
            self.currentSim = None
            self.init_current_sim()

        # temperature
        if rospy.has_param('~temperature_mode'):
            self.tempSim = None
            self.init_temp_sim()

        # noise
        if rospy.has_param('~noise_mode'):
            self.noiseSim = None
            self.init_noise_sim()
        
        rospy.spin()
    
    def init_current_sim(self):
        """
        Initializes current simulation object. Depending on current_mode parameter,
        it generates an instance of respective class. If the current_mode is set to 
        "none", the object will not be created.
        """
        current_mode = rospy.get_param('~current_mode')
        if current_mode == "constant":
            try:
                from current_sim_constant import CurrentSimConstant
            except ImportError:
                rospy.logerr("ERROR: Can't import module CurrentSimConstant")
                return -1
            filename = None
            if rospy.has_param('~current_spec_filename'):
                filename = rospy.get_param('~current_spec_filename')
            if filename is None:
                filename = "currentInfo.txt"
            self.currentSim = CurrentSimConstant(filename)

        elif current_mode == "periodic":
            try:
                from current_sim_periodic import CurrentSimPeriodic
            except ImportError:
                rospy.logerr("ERROR: Can't import module CurrentSimPeriodic")
                return -1
            self.currentSim = CurrentSimPeriodic()

            '''
            You can insert new current modes here (just unindent for one tab to be in the
                same level as other elif keywords).
            ***
            Template:
            elif current_mode == "mode":
                try:
                    from current_sim_mode import CurrentSimMode
                except ImportError:
                    rospy.logerr("ERROR: Can't import module CurrentSimMode")
                    return -1
                self.currentSim = CurrentSimMode()
            ***
            IMPORTANT: implement the desired behavior in current_sim_mode.py (for reference, 
                    see already implemented modes)
            '''

        elif current_mode == "none":
            return 0

        else:
            rospy.logerr("Current mode {0} is not implemented!".format(current_mode))

    def init_temp_sim(self):
        """
        Initializes temperature simulation object. Depending on temp_mode parameter,
        it generates an instance of respective class. If the temp_mode is set to 
        "none", the object will not be created.
        """
        temp_mode = rospy.get_param('~temp_mode')
        if temp_mode == "constant":
            try:
                from temperature_sim_constant import TemperatureSimConstant
            except ImportError:
                rospy.logerr("ERROR: Can't import module TemperatureSimConstant")
                return -1
            filename = None
            if rospy.has_param('~temp_spec_filename'):
                filename = rospy.get_param('~temp_spec_filename')
            if filename is None:
                filename = "temperatureInfo.txt"
            self.tempSim = TemperatureSimConstant(filename)

            '''
            You can insert new temperature modes here (just unindent for one tab to be in the
                    same level as other elif keywords).
            ***
            Template:
            elif temp_mode == "mode":
                try:
                    from temperature_sim_mode import TemperatureSimMode
                except ImportError:
                    rospy.logerr("ERROR: Can't import module TemperatureSimMode")
                    return -1
                self.tempSim = TemperatureSimMode()
            ***
            IMPORTANT: implement the desired behavior in temperature_sim_mode.py (for reference, 
                    see already implemented modes)
            '''

        elif temp_mode == "none":
            return 0

        else:
            rospy.logerr("Temperature mode {0} is not implemented!".format(temp_mode))
            
    def init_noise_sim(self):
        """
        Initializes noise simulation object. Depending on noise_mode parameter,
        it generates an instance of respective class. If the noise_mode is set to 
        "none", the object will not be created.
        """
        noise_mode = rospy.get_param('~noise_mode')
        if noise_mode == "static":
            try:
                from noise_sim_static import NoiseSimStatic
            except ImportError:
                rospy.logerr("ERROR: Can't import module NoiseSimstatic")
                return -1
            
            source_position = NED()
            source_position.north = rospy.get_param('~noise_source_north')
            source_position.east = rospy.get_param('~noise_source_east')
            source_position.depth = rospy.get_param('~noise_source_depth')
            
            self.noiseSim = NoiseSimStatic(source_position)

            '''
            You can insert new noise modes here (just unindent for one tab to be in the
                    same level as other elif keywords).
            ***
            Template:
            elif noise_mode == "mode":
                try:
                    from noise_sim_mode import NoiseSimMode
                except ImportError:
                    rospy.logerr("ERROR: Can't import module NoiseSimMode")
                    return -1
                self.noiseSim = NoiseSimMode()
            ***
            IMPORTANT: implement the desired behavior in noise_sim_mode.py (for reference, 
                    see already implemented modes)
            '''

        elif noise_mode == "none":
            return 0

        else:
            rospy.logerr("Noise mode {0} is not implemented!".format(noise_mode))


if __name__ == '__main__':
    
    rospy.init_node('environment_sim', anonymous=True)
    try:
        EnvironmentSim()
    except rospy.ROSInterruptException:
        pass
