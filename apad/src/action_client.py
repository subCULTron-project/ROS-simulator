
__author__ = "barbanas"      

import actionlib
from apad.msg import aPadAction, aPadGoal
from geometry_msgs.msg import Pose

"""
Actions:

  id  |     action type
--------------------------------
   0  |     go to position
   1  |     perch
   2  |     release

"""

class actionClient(object):
    '''
    Implements action client used for forwarding task execution requests to action server.
    This is the interface through which high level control communicates with low level controllers.
    '''
    def __init__(self, namespace):

        # initialize action client
        self.client = actionlib.SimpleActionClient(namespace + 'action_server', aPadAction)
        self.feedback = 0

    def send_position_goal(self, position):

        self.client.wait_for_server()
        print 'Connected to server'

        # Creates a goal to send to the action server.
        goal_pose = Pose()
        goal_pose.position = position
        goal = aPadGoal(id=0, pose=goal_pose)
        
        # Sends the goal to the action server.
        self.client.send_goal(goal, None, None, self.feedback_cb)

        # Waits for the server to finish performing the action.
        self.client.wait_for_result()
        
        # Prints out the result of executing the action
        print self.client.get_result()
        return self.client.get_result()

    def send_perch_goal(self, obj):

        self.client.wait_for_server()
        print 'Connected to server'

        # Creates a goal to send to the action server.
        goal = aPadGoal(id=1, object=obj)
        
        # Sends the goal to the action server.
        self.client.send_goal(goal)

        # Waits for the server to finish performing the action.
        self.client.wait_for_result()
        
        # Prints out the result of executing the action
        print self.client.get_result()
        return self.client.get_result()

    def send_release_goal(self):

        self.client.wait_for_server()
        print 'Connected to server'

        # Creates a goal to send to the action server.
        goal = aPadGoal(id=2)

        # Sends the goal to the action server.
        self.client.send_goal(goal)

        # Waits for the server to finish performing the action.
        self.client.wait_for_result()

        # Prints out the result of executing the action
        print self.client.get_result()
        return self.client.get_result()

    def feedback_cb(self, feedback):
        '''
        Get feedback from action server == task completion percentage.
        '''
        if feedback.status - self.feedback >= 5:
            #print " " + str(feedback.status) + "%"
            self.feedback = feedback.status
