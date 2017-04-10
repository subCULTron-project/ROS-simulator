#!/usr/bin/env python

"""
"""

__author__ = "barbanas"


import rospy
from misc_msgs.srv import GetChargeInfo, GetDockingInfo, ChangeDockingStatus

class Docking(object):
	
	def __init__(self):
		self.slots = [""] * len(self.check_candidates())
		
	def get_docking_status(self):
		
		docking_avail = rospy.ServiceProxy('check_docking_availability', GetDockingInfo)
		try:
			d = docking_avail()
			if len(d.available_slots) == 0:
				return [False, self.slots, d.available_slots]
			else:
				return [True, self.slots, d.available_slots]
		except rospy.ServiceException as exc:
			print("Service did not process request: " + str(exc))


	def check_availability(self):
		
		docking_avail = rospy.ServiceProxy('check_docking_availability', GetDockingInfo)
		try:
			d = docking_avail()
			if len(d.available_slots) == 0:
				return False
			else:
				return True
		except rospy.ServiceException as exc:
			print("Service did not process request: " + str(exc))

	def check_docking_slots(self):
		
		return self.slots
			
	def check_candidates(self):
		
		docking_avail = rospy.ServiceProxy('check_docking_availability', GetDockingInfo)
		try:
			d = docking_avail()
			return d.available_slots
		except rospy.ServiceException as exc:
			print("Service did not process request: " + str(exc))
		
	def lock_robot(self, slot, robot = None):
		
		change_status = rospy.ServiceProxy('change_docking_status', ChangeDockingStatus)
		try:
			resp = change_status(slot, True)
			if robot is None:
				self.slots[slot] = ".."
			else:
				self.slots[slot] = robot
				
		except rospy.ServiceException as exc:
			print("Service did not process request: " + str(exc))
	
	def release_robot(self, slot):
			
		change_status = rospy.ServiceProxy('change_docking_status', ChangeDockingStatus)
		try:
			resp = change_status(slot, False)
			self.slots[slot] = ""
		except rospy.ServiceException as exc:
			print("Service did not process request: " + str(exc))
			
