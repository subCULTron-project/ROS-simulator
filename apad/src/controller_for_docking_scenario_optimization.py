#!/usr/bin/env python

"""
Initializes ROS node for high level mission control.
"""

__author__ = "barbanas"

import rospy
from auv_msgs.msg import NED, NavSts
from geometry_msgs.msg import TwistStamped, Point
from std_msgs.msg import Bool, Int32, String
from math import atan2, radians, degrees, sqrt, pow, fabs
from action_client import actionClient
from cbm_lib import cbm_algorithm, mdvrp
from misc_msgs.srv import GetChargeInfo
from cbm.msg import BestSolution, WeightMatrix

class ScenarioController(object):
    
    def __init__(self):
        
        self.start = False
        
        self.current = NED(0, 0, 0)
        self.position = None
        
        self.neighbors = {}
        rospy.Subscriber('/hello', String, self.hello_cb)
        self.helloPub = rospy.Publisher('/hello', String, queue_size=100)
        
        rospy.Subscriber('/scenario_start', Bool, self.start_cb)
        self.startPub = rospy.Publisher('start_sim', Bool, queue_size=1)

        self.stateRefPub = rospy.Publisher('stateRef', NavSts, queue_size=1)
        
        rospy.Subscriber('position', NavSts, self.position_cb)

        self.batteryLevel = None
        #rospy.Subscriber('battery_level', Int32, self.battery_level_cb)
        self.batteryLevel = 80
        rospy.Service('charge_info', GetChargeInfo, self.charge_info_srv)
        self.chargeType = 'charger'
        
        self.chargeRequests = []
        rospy.Subscriber('/battery_alert', NavSts, self.charge_cb)
        
        self.cl = actionClient(rospy.get_namespace())
        self.mdvrp = mdvrp.mdvrp()
        self.cbmAlgorithm = cbm_algorithm.cbm_algorithm("cbm_parameters.xml")
        
        self.bestSolutionPub = rospy.Publisher("/bestSolution", BestSolution, queue_size=20)
        self.weightMatrixPub = rospy.Publisher("/weightMatrix", WeightMatrix, queue_size=20)
        rospy.Subscriber('/optimize', Bool, self.charging_procedure)
        
        rospy.spin()

    def start_cb(self, msg):
        
        self.startPub.publish(msg)
        self.start = True
        
    def position_cb(self, msg):
        
        self.position = NED(msg.position.north, msg.position.east, msg.position.depth)
        
    def battery_level_cb(self, msg):
        
        self.batteryLevel = msg.data
        
    def hello_cb(self, msg):
        
        self.neighbors[msg.data] = rospy.get_time()
        print [rospy.get_namespace(), self.neighbors.keys()]
            
    def charge_cb(self, msg):
        
        if self.start is False:
            return
        
        id = msg.header.frame_id
        if id not in self.chargeRequests:
            self.chargeRequests.append(id)

        #if len(self.chargeRequests) >= 10:
        #    self.charging_procedure()
        
    def docking(self,msg):
		if self.start is False:
			return
		#go to amussel
		print("Picking up amussel with ID " + msg.header.frame_id)
		heading = [msg.position.north - self.position.north, msg.position.east - self.position.east]
		dist = sqrt(pow(heading[0], 2) + pow(heading[1], 2))
		p = Point()
		p.x = self.position.north + (dist - 0.05) * heading[0] / dist
		p.y = self.position.east + (dist - 0.05) * heading[1] / dist
		self.cl.send_position_goal(p)
		self.cl.send_perch_goal(msg.header.frame_id)
		
		if (dist<-10):

			# perch
			self.cl.send_perch_goal(msg.header.frame_id)

			# go to position (with amussel docked)
			p = Point(0,0,0)
			self.cl.send_position_goal(p)
        
			# release amussel
			self.cl.send_release_goal()

			# go to another position
			p = Point(-10,-10,0)
			self.cl.send_position_goal(p)    

    def charging_procedure(self, msg):
        
        chargers = {}
        consumers = {}
        
        # gather data 
        # 1. get all potential chargers
        self.helloPub.publish(String(rospy.get_namespace()))
        
        rospy.sleep(1)
        for charger in self.neighbors.keys():
            if charger == rospy.get_namespace():
                chargers[charger] = [self.position.north, self.position.east, self.batteryLevel]
                continue
            charge_info = rospy.ServiceProxy(charger + 'charge_info', GetChargeInfo)
            try:
                c = charge_info()
                chargers[charger] = [c.x, c.y, c.battery_level]
            except rospy.ServiceException as exc:
                print("Service did not process request: " + str(exc))
                
        tmp = chargers.keys()
        tmp.sort()
        
        for d in tmp:
            self.mdvrp.add_node('depot', d, chargers[d][0], chargers[d][1], chargers[d][2])
            
        print self.mdvrp.depotLabels
        
        for consumer in self.chargeRequests:
            charge_info = rospy.ServiceProxy(consumer + 'charge_info', GetChargeInfo)
            try:
                c = charge_info()
                consumers[consumer] = [c.x, c.y, c.battery_level]
            except rospy.ServiceException as exc:
                print("Service did not process request: " + str(exc))
                
        tmp = consumers.keys()
        tmp.sort()
        
        for c in tmp:
            self.mdvrp.add_node('consumer', c, consumers[c][0], consumers[c][1], (100 - consumers[c][2]) / 4)
            
        print self.mdvrp.consumerLabels
        
        self.cbmAlgorithm.optimize(self.bestSolutionPub, self.weightMatrixPub, self.mdvrp)    
        
        rospy.sleep(1)
        
        solution = self.cbmAlgorithm.bestSolutionCoalition
        indices = solution.routes[self.mdvrp.depotLabels.index(rospy.get_namespace())]
        
        toVisit = []
        for i in indices[0]:
            label = self.mdvrp.nodeLabels[i]
            toVisit.append([label, consumers[label][0], consumers[label][1]])
            
        print [rospy.get_namespace(), toVisit]
        
        nextTarget = NavSts()
        for tgt in toVisit:
			t=tgt[0].strip("/amussel")
			nextTarget.header.frame_id=t
			nextTarget.position.north=tgt[1]
			nextTarget.position.east=tgt[2]
			#print nextTarget
			self.docking(nextTarget)
         
        
        
        
        
    def charge_info_srv(self, req):
        
        return [self.batteryLevel, self.position.north, self.position.east, 0, self.chargeType]
        
if __name__ == "__main__":
    
    rospy.init_node("scenario_controller")
    
    try:
        controller = ScenarioController()
            
    except rospy.ROSInterruptException:
        pass
    
           
