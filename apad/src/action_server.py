#!/usr/bin/python

"""
Initializes ROS node for executing aPad actions.

Summary:

    Publications: 
        * /namespace/action_server/result [apad/aPadActionResult]
        * /namespace/action_server/status [actionlib_msgs/GoalStatusArray]
        * /namespace/action_server/feedback [apad/aPadActionFeedback]
        * /namespace/stateRef [auv_msgs/NavSts]

    Subscriptions: 
         * /namespace/position [auv_msgs/NavSts]
         * /namespace/action_server/goal [apad/aPadActionGoal]
         * /namespace/action_server/cancel [actionlib_msgs/GoalID]
         * /clock [rosgraph_msgs/Clock]

    Services: 

    Arguments:
"""

__author__ = 'barbanas'

import rospy
import actionlib

from apad.msg import aPadAction, aPadGoal, aPadResult, aPadFeedback

from geometry_msgs.msg import Point, Twist, Vector3, Pose
from auv_msgs.msg import NavSts, NED
from navcon_msgs.srv import EnableControl, ConfigureVelocityController

from math import pow, sqrt, fabs

"""
Actions:

  id  |     action type
--------------------------------
   0  |     go to position
   1  |     perch
   2  |     release

"""


class aPadActionServer:

    def __init__(self):
        
        # position
        self.position = Point(0, 0, 0)
        rospy.Subscriber('position', NavSts, self.position_cb)

        # state reference publisher
        self.stateRefPub = rospy.Publisher('stateRef', NavSts, queue_size=1)
        self.staticPosPub = [None, None, None, None] # position publisher for perched objects
        self.perchedObjects = ['','','','']
        self.positionOffset = [[-0.1,-0.1], [-0.1,0.1], [0.1,-0.1], [0.1,0.1]]
        self.perched_flag = 0  # flag for dummy action perching onto aMussel/aFish/other objects action
        self.perched_count = 0 # number of currently docked objects (default max 4)
        
        self.action_rec_flag = 0  # 0 - waiting action, 1 - received action, 2 - executing action
        
        self.as_goal = aPadGoal()
        self.as_res = aPadResult()
        self.as_feed = aPadFeedback()

        # TODO create 4 docking stations -- 4 offset positions for every station + before docking, align the object to station
        self.position_offset = Point(-0.5, -0.5, 0)  # for docking -- object offset 

        self.pos_old = Point(self.position.x, self.position.y, self.position.z)
        
        self.pos_err = []
        
        # initialize actions server structures
        self.action_server = actionlib.SimpleActionServer("action_server", aPadAction, auto_start=False)
        self.action_server.register_goal_callback(self.action_execute_cb)
        self.action_server.register_preempt_callback(self.action_preempt_cb)
        self.action_server.start()

        while not rospy.is_shutdown():
            
            if self.perched_flag == 1:
                for i in range(self.perched_count):
                    if self.perchedObjects[i] == '':
                        continue
                    # goal position for aMussel/aFish/object docked onto aPad (if perched!)
                    #TODO multiple docking stations
                    goalPosition = NED()
                    goalPosition.north = self.position.x + self.positionOffset[i][0]
                    goalPosition.east = self.position.y + self.positionOffset[i][1]
                    #goalPosition.z = self.position.z + self.position_offset.z
                    self.set_model_state(i, goalPosition)

            if self.action_server.is_active():

                ###################
                # received action #
                ###################

                if self.action_rec_flag == 1:
                
                    if self.as_goal.id == 0:
                        print 'Go to position action' # 2d movement
                        self.pos_old = Point(self.position.x, self.position.y, 0)
                        start = Vector3(self.position.x, self.position.y, 0)
                        end = Vector3(self.as_goal.pose.position.x, self.as_goal.pose.position.y, 0)

                        dl = sqrt(pow(self.as_goal.pose.position.x - self.position.x, 2) +
                                        pow(self.as_goal.pose.position.y - self.position.y, 2))

                        # if within 10cm of goal
                        if dl < 0.1:
                            self.action_rec_flag = 0  # waiting for new action
                            self.as_res.status = 0
                            self.pos_err = []
                            print 'finished executing go_to_position action'
                            print "###"
                            print self.position
                            print "###"
                            self.action_server.set_succeeded(self.as_res)
                        else:
                            try:
                                # send goal
                                fadp_enable = rospy.ServiceProxy('FADP_enable', EnableControl)
                                fadp_enable(enable=True)

                                config_vel_controller = rospy.ServiceProxy('ConfigureVelocityController', ConfigureVelocityController)
                                config_vel_controller(ControllerName="FADP", desired_mode=[2, 2, 0, 0, 0, 0])

                                velcon_enable = rospy.ServiceProxy('VelCon_enable', EnableControl)
                                velcon_enable(enable=True)
        
                                stateRef = NavSts()
                                stateRef.header.seq = 0
                                stateRef.header.stamp.secs = 0
                                stateRef.header.stamp.nsecs = 0
                                stateRef.global_position.latitude = 0.0
                                stateRef.global_position.longitude = 0.0
                                stateRef.origin.latitude = 0.0
                                stateRef.origin.longitude = 0.0
                                stateRef.position.north = self.as_goal.pose.position.x
                                stateRef.position.east = self.as_goal.pose.position.y
                                stateRef.position.depth = 0
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
                                self.stateRefPub.publish(stateRef)
                                
                                self.action_rec_flag = 2  #executing trajectory

                            except rospy.ServiceException, e:
                                print "Service for activating controller failed: %s" % e
                                self.as_res.status = -1
                                self.action_server.set_aborted(self.as_res)
                                self.action_rec_flag = 0  # waiting for new actions

                    elif self.as_goal.id == 1:
                        print 'perch action'
                        self.action_rec_flag = 2  # start executing action

                    elif self.as_goal.id == 2:
                        print 'release action'
                        self.action_rec_flag = 2  # start executing action

                ####################
                # executing action #
                ####################

                elif self.action_rec_flag == 2:

                    self.as_res.id = self.as_goal.id
                    self.as_feed.id = self.as_goal.id

                    if self.as_goal.id == 0:
                        # Go to position action
                        dL = sqrt(pow(self.as_goal.pose.position.x - self.pos_old.x, 2) +
                                       pow(self.as_goal.pose.position.y - self.pos_old.y, 2))
                        dl = sqrt(pow(self.as_goal.pose.position.x - self.position.x, 2) +
                                       pow(self.as_goal.pose.position.y - self.position.y, 2))

                        if len(self.pos_err) < 20:
                            self.pos_err.append(dl)
                        else:
                            self.pos_err.pop(0)
                            self.pos_err.append(dl)

                        if (len(self.pos_err) == 20) and (fabs(sum(self.pos_err) / len(self.pos_err)) < 0.5):  # mission is successfully finished
                            self.action_rec_flag = 0  # waiting for new action
                            self.as_res.status = 0
                            self.pos_err = []
                            print 'finished executing go_to_position action'
                            self.action_server.set_succeeded(self.as_res)
                        else:  # mission is still ongoing
                            self.as_feed.status = (1 - dl / dL) * 100  # mission completeness
                            self.as_feed.pose.position = self.position
                            #print [fabs(sum(self.pos_err) / len(self.pos_err)), str(self.position), str(self.as_goal.pose.position)]
                            self.action_server.publish_feedback(self.as_feed)

                    elif self.as_goal.id == 1:
                        print 'executing perch action'
                        if self.perched_count < 4:
                            self.perched_flag = 1
                            self.perched_count += 1
                            # static position publisher
                            #TODO make 4 static pose publishers, for 4 docking stations
                            index = self.staticPosPub.index(None)
                            self.staticPosPub[index] = rospy.Publisher('/' + self.as_goal.object + '/position_static', NavSts, queue_size=1)
                            self.perchedObjects[index] = self.as_goal.object
                            print 'finished executing perch action ' + self.as_goal.object
                        self.action_rec_flag = 0  # waiting for new action
                        self.as_res.status = 0
                        #self.perched_count+=1
                        self.action_server.set_succeeded(self.as_res)

                    elif self.as_goal.id == 2:
                        print 'executing release action'
                        index = self.perchedObjects.index(self.as_goal.object)
                        self.perchedObjects[index] = ''
                        self.staticPosPub[index] = None
                        self.perched_count -= 1
                        if self.perched_count == 0:
                            self.perched_flag = 0
                        msg = NavSts()
                        msg.position = NED(self.position.x + self.position_offset.x, self.position.y + self.position_offset.y, 0)
                        msg.status = 1 # status 1 -- the last static position, give control back to uvsim
                        #TODO make 4 static pose publishers, for 4 docking stations
                        self.staticPosPub.publish(msg) # signal the end of static position to attached object
                        print 'finished executing release action'
                        self.action_rec_flag = 0  # waiting for new action
                        self.as_res.status = 0
                        #self.perched_count-=1
                        self.action_server.set_succeeded(self.as_res)
            else:
                pass

            rospy.sleep(rospy.Duration(0.05))

    def set_model_state(self, i, position):

        msg = NavSts()
        msg.position = position
        self.staticPosPub[i].publish(msg)

    def position_cb(self, msg):    
        self.position.x = msg.position.north
        self.position.y = msg.position.east
        self.position.z = msg.position.depth

    def action_execute_cb(self):
        print 'Received action = '
        self.action_rec_flag = 1
        self.as_goal = self.action_server.accept_new_goal()

    def action_preempt_cb(self):
        print 'Cancelling action'
        #TODO -- send stop message to controllers

        self.action_rec_flag = 0  # wait for new action
        self.action_server.set_preempted()


if __name__ == '__main__':
    # Initialize action server node
    rospy.init_node('action_server', anonymous=True)
    try:
        aPadActionServer()
    except rospy.ROSInterruptException:
        pass
