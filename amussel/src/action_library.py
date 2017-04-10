        
import rospy
from navcon_msgs.srv import EnableControl, ConfigureVelocityController
from auv_msgs.msg import NavSts

def get_temperature(x,y):
	return (x+10)*(x+10)+(y+10)*y

def send_depth_goal(stateRefPub, position):

    try:
        depth_enable = rospy.ServiceProxy('DEPTH_enable', EnableControl)
        depth_enable(enable=True)

        velcon_enable = rospy.ServiceProxy('VelCon_enable', EnableControl)
        velcon_enable(enable=True)
        
        config_vel_controller = rospy.ServiceProxy('ConfigureVelocityController', ConfigureVelocityController)
        config_vel_controller(ControllerName="DEPTH", desired_mode=[0, 0, 2, 0, 0, 0])

        stateRef = NavSts()
        stateRef.header.seq = 0
        stateRef.header.stamp.secs = 0
        stateRef.header.stamp.nsecs = 0
        stateRef.global_position.latitude = 0.0
        stateRef.global_position.longitude = 0.0
        stateRef.origin.latitude = 0.0
        stateRef.origin.longitude = 0.0
        stateRef.position.north = position.north
        stateRef.position.east = position.east
        stateRef.position.depth = position.depth
        stateRef.altitude = 0.0
        stateRef.body_velocity.x = 0
        stateRef.body_velocity.y = 0
        stateRef.body_velocity.z = 0
        stateRef.gbody_velocity.x = 0
        stateRef.gbody_velocity.y = 0
        stateRef.gbody_velocity.z = 0
        stateRef.orientation.roll = 0
        stateRef.orientation.pitch = 0
        stateRef.orientation.yaw = 0
        stateRef.orientation_rate.roll = 0
        stateRef.orientation_rate.pitch = 0
        stateRef.orientation_rate.yaw = 0
        stateRef.position_variance.north = 0
        stateRef.position_variance.east = 0
        stateRef.position_variance.depth = 0
        stateRef.orientation_variance.roll = 0
        stateRef.orientation_variance.pitch = 0
        stateRef.orientation_variance.yaw = 0
        stateRef.status = 0
        stateRefPub.publish(stateRef)
        return 0

    except rospy.ServiceException, e:
        rospy.logerr("Failed to call change depth action" % e)
        return -1
    
class Anchor(object):
    
    def __init__(self):
        self.anchored = False
        
        
    def hold(self):
        '''
        Args:
            - none
        Returns:
            bool -- task success
        '''
        self.anchored = True
        return True
    
    def release(self):
        '''
        Args:
            - none
        Returns:
            bool -- task success
        '''
        self.anchored = False
        return True
    
    def check_status(self):
        '''
        Args:
            - none
        Returns:
            bool -- anchoring status
        '''
        return self.anchored
