        
import rospy
from navcon_msgs.srv import EnableControl, ConfigureVelocityController
from auv_msgs.msg import NavSts
from misc_msgs.srv import GetChargeInfo
from std_msgs.msg import Float64

def send_position_goal(stateRefPub, position):

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
    
    send_state_ref(stateRefPub, stateRef)

def send_state_ref(stateRefPub, stateRef):
    
    try:
        '''
        fadp_enable = rospy.ServiceProxy('FADP_enable', EnableControl)
        fadp_enable(enable=True)
	   
        depth_enable = rospy.ServiceProxy('DEPTH_enable', EnableControl)
        depth_enable(enable=True)
		
        velcon_enable = rospy.ServiceProxy('VelCon_enable', EnableControl)
        velcon_enable(enable=True)
        
        config_vel_controller = rospy.ServiceProxy('ConfigureVelocityController', ConfigureVelocityController)
        config_vel_controller(ControllerName="DEPTH", desired_mode=[2, 2, 2, 0, 0, 2])
        '''
        stateRefPub.publish(stateRef)
        return 0

    except rospy.ServiceException, e:
        rospy.logerr("Failed to call change depth action" % e)
        return -1

class GPS(object):
    
    def has_fix(self):
        '''
        Args:
            - none
        Returns:
            True/False
        '''
        
        pass
    
    def get_data(self):
        '''
        Args:
            - none
        Returns:
            vector<float> (latitude, longitude, altitude)
        '''
        
        pass
    
    def get_accuracy(self):
        '''
        Args:
            - none
        Returns:
            float (hdop)
        '''
        pass
    
class AcousticModem(object):
    
    def send(self, destination, data):
        '''
        Args:
            destination: receiver id #TODO --> define how it is represented (int/string?)
            data: string
        Returns:
            - none
        '''
        pass
    
    def receive(self):
        '''
        Args:
            - none
        Returns:
            data: source + received data (string)
        '''
        pass

class WiFi(object):
    
    def send(self, destination, data):
        '''
        Args:
            destination: receiver id #TODO --> define how it is represented (int/string?)
            data: string
        Returns:
            - none
        '''
        pass
    
    def receive(self):
        '''
        Args:
            - none
        Returns:
            data: source + received data (string)
        '''
        pass

class Clock(object):
    
    def get_time(self):
        '''
        Args:
            - none
        Returns:
            time: long (float)
        '''
        
        return rospy.get_time()

class Battery(object):
    '''
    Battery module.
    '''
    
    def __init__(self):
        pass
    
    def get_level(self):
        rospy.wait_for_service('get_battery_level', 0.2)
        battery_level = rospy.ServiceProxy('get_battery_level', GetChargeInfo)
        level = battery_level().battery_level
        return level
    
class Induction(object):
    '''
    Inductive charging module.
    '''
    
    def __init__(self):
        self.coils = ["", "", "", ""]
        self.enabledCoils = set()
        self.drainPub = rospy.Publisher("draining", Float64, queue_size=1) 
        
    def enable_coil(self, i):
        '''
        For receiver.
        
        Args:
            i: int -- coil index 
        Returns:
            bool -- action success
        '''
        if i in self.enabledCoils:
            raise AttributeError("Trying to enable already enabled coil!")
            return False
            
        self.enabledCoils.add(i)
        return True

    def disable_coil(self, i):
        '''
        For receiver.
        
        Args:
            i: int -- coil index 
        Returns:
            bool -- action success
        '''
        if i not in self.enabledCoils:
            raise AttributeError("Trying to disable already disabled coil!")
            return False
            
        self.enabledCoils.remove(i)
        return True
    
    def send_energy(self, receiver, i):
        '''
        Args:
            i: int -- coil index 
        Returns:
            bool -- task success
        '''
        step = 0.1
        
        chargingPub = rospy.Publisher(receiver + "charging", Float64, queue_size=1)
        self.drainPub.publish(step)
        chargingPub.publish(step)
    
    def get_charging_power(self, i):
        '''
        Args:
            i: int -- coil index 
        Returns:
            int --
        '''
        pass
    
class Docking(object):
    '''
    Docking module.
    '''
    
    def __init__(self):
        
        self.slots = ["", "", "", ""]
        self.slotLocationOffset = [[-5, 0], [0, -5], [5, 0], [0, 5]]
        self.occupiedSlots = set()
        
    def lock_robot(self, target, i):
        '''
        Args:
            target: string -- target robot id
            i: int -- docking slot id
        Returns:
            bool -- task success
        '''
        if i in self.occupiedSlots:
            raise AttributeError("Trying to lock robot onto already occupied slot!")
            return False
        
        if i <= len(self.slots):
            raise AttributeError("Trying to lock robot onto non-existent slot!")
            return False
        
        self.occupiedSlots.add(i)
        self.slots[i] = target
        return True

    def release_robot(self):
        '''
        Args:
            target: string -- target robot id
            i: int -- docking slot id
        Returns:
            bool -- task success
        '''
        if i not in self.occupiedSlots:
            raise AttributeError("Trying to release robot from unoccupied slot!")
            return False
        
        if i <= len(self.slots):
            raise AttributeError("Trying to release robot from non-existent slot!")
            return False
        
        self.occupiedSlots.remove(i)
        self.slots[i] = ""
        return True
    
    def check_availability(self):
        '''
        Args:
            - none
        Returns:
            int[] -- available slots
        '''
        
        return (set(range(len(self.slots))) - self.occupiedSlots).toList()
    