#!/usr/bin/env python

"""This ROS node allows the user to publish moving/stationary/pop-up obstacles, using <ObjectDetectArray> message type

"""

import rospy
import numpy as np
from tamu_sa.msg import ObjectDetectArray, ObjectDetect
import pdb

class FakeObstacles:
	"""Helper class for creating ObstacleDetectArray elements

		Contact Tianqi for updates to the ROS MESSAGE

		Variables:
			objectArrayDict (dict): A collection of 'ObjectDetect', which will be returned!

	"""
	objectArrayDict = {}
	obstacleCache = {}

	# def __init__(self, easting, northing, id, bb_height, bb_width, obj_confidence, type_="ST"):
	# 	# cur_det = ObjectDetect()
	# 	# cur_det.easting = 100
	# 	# cur_det.northing = 300
	# 	# cur_det.id = 134
	# 	# cur_det.bb_height = 25
	# 	# cur_det.bb_width = 25
	# 	# cur_det.obj_confidence = 1

	# 	self.newObject = ObjectDetect()
	# 	self.newObject.easting = easting
	# 	self.newObject.northing = northing
	# 	self.newObject.id = id
	# 	self.newObject.bb_height = bb_height
	# 	self.newObject.bb_width = bb_width
	# 	self.newObject.obj_confidence = obj_confidence

	# 	self.init_easting = easting
	# 	self.init_northing = northing
	# 	self.type_ = type_

	# 	#objectArrayMsg.detections.append(self.newObject)
	# 	#objectArrayDict[id] = self.newObject
	# 		# Keep track of obstacles
	# 		cls._add_obstacles(id, self.newObject, type_)

	@classmethod
	def add_obstacles(cls,	easting,	northing,	id,	bb_height,	bb_width,	obj_confidence,	type_, **kwargs):
		"""Class method to add obstacles

		Parameters:
			easting (float, int): Center coordinate of the bounding box
			northing (float, int): Center coordinate of the bounding box
			id (int): Detected object's id number
			bb_height (float, int): Heigh of bounding box
			bb_width (float, int): Width of bounding box
			obj_confidence(float): Confidence detection is valid, interval in [0-1]
			type_ (str): "MLR", "MUD", "ST", "POP" (not part of the messages!)
			**kwargs: For arbitrary additional keyworded inputs

		"""
		newObject = ObjectDetect()
		newObject.easting = easting
		newObject.northing = northing
		newObject.id = id
		newObject.bb_height = bb_height
		newObject.bb_width = bb_width
		newObject.obj_confidence = obj_confidence

		init_easting = easting
		init_northing = northing
		type_ = type_

		cls._add_obstacles(id, newObject, type_, init_easting, init_northing, kwargs)

	@classmethod
	def _add_obstacles(cls, id, object, type_, init_e, init_n, kwargs):
		cls.objectArrayDict[id] = {'obj':{'msg': object, 'init_e': init_e, 'init_n': init_n }, 'return': True, 'type': type_, 'args': kwargs}

	@classmethod
	def update_objects(cls, time):
		"""Apply motion model to our obstacles if relevant

		Parameter:
		time (float): Current clock (elapsed since start)

		"""
		for k in cls.objectArrayDict.values():
			if k['type'] is 'MLR':
				cls.use_motion_model_x(k['obj'], time, **k['args'])
			elif k['type'] is 'MUD':
				cls.use_motion_model_y(k['obj'], time, **k['args'])
			elif k['type'] is 'ST':
				pass
			elif k['type'] is 'POP':
				cls.pop_up(k, time, k['args']['tAppear'])

		msg = cls.return_object_array()
		return msg

	###############################################

	@staticmethod
	def use_motion_model_x(obstacle, t, period=None, amplitude=None):
		"""Function to move obstacle left-right"""
		# move left/right 25 meters
		new_x = amplitude*np.sin(2*np.pi/period*t)
		obstacle['msg'].easting = new_x + obstacle['init_e']

	@staticmethod
	def use_motion_model_y(obstacle, t, period=None, amplitude=None):
		"""Function to move obstacle up-down"""
		# move up and down 25 meters
		new_y = amplitude*np.sin(2*np.pi/period*t)
		obstacle['msg'].northing = new_y + obstacle['init_n']

	################################################

	@classmethod
	def pop_up(cls, f_obstacle, t, tAppear=None):
		"""Obstacle will pop up after some time """
		# After tAppear seconds, add the obstacles to objectArrayDict = {}
		if t > tAppear:
			# if f_obstacle['obj']['msg'].id in cls.obstacleCache:
			# 	cls.add_object(f_obstacle['msg'].id, f_obstacle)
			# 	del cls.obstacleCache[f_obstacle.newObject.id]
			f_obstacle['return'] = True
		else:
			# Before tAppear, make sure obstacle hasn't appeared in objectArrayDict
			# if f_obstacle['obj']['msg'].id in cls.objectArrayDict:
			# 	cls.obstacleCache[f_obstacle['obj']['msg'].id] = f_obstacle
			# 	cls.delete_object(f_obstacle['obj']['msg'].id)
			# 	print("wip")
			f_obstacle['return'] = False


	# @classmethod
	# def delete_object(cls,id):
	# 	del cls.objectArrayDict[id]

	# @classmethod
	# def add_object(cls,id, f_obstacles):
	# 	cls.objectArrayDict[id] = f_obstacles

	################################################

	@classmethod
	def return_object_array(cls):
		""" Construct message for publishing """
		msg = ObjectDetectArray()
		# msg.detections = list(cls.objectArrayDict.values())
		for k in cls.objectArrayDict.values():
			if k['return'] == True:
				msg.detections.append(k['obj']['msg'])
		return msg

class FakeObstacleNode:
	"""ROS Node which allows the user to set obstacles

	Attributes:
	objPub (rospy.Publisher): A ROS object for publishing message of type 'ObjectDetectArray'

	Parameters:
	fakeOstacles (FakeObstacles): Expects 'FakeObstacles' class!
	topic_name (str): A string denoting the published topic's name

	"""
	def __init__(self, fakeObstacles, topic_name="/test/object_detection"):

		# create publisher
		self.objPub = rospy.Publisher(topic_name, ObjectDetectArray, queue_size=1)

		# set loop rate
		rate = rospy.Rate(10)

		# Not ru the while loop
		initTime = rospy.get_time()
		while not rospy.is_shutdown():
			timeNow = rospy.get_time() - initTime
			# self.use_motion_model_x(obstacle1, timeNow, 10, 25)
			# self.use_motion_model_y(obstacle2, timeNow, 10, 25)

			# self.pop_up(obstacle2, timeNow, 15)
			# self.pop_up(obstacle3, timeNow, 10)

			# self.objPub.publish(return_object_array())
			self.objPub.publish(fakeObstacles.update_objects(timeNow))
			rate.sleep()

if __name__=="__main__":
	rospy.init_node("fake_obstacle_detect")

	# Create some fake obstacles to be stored in ObjectDetect Array
	# type_ (str): "MLR", "MUD", "ST", "POP" (not part of the messages!)
	# arguments: easting, northing, id, bb_height, bb_width, obj_confidence
	obstacle1 = FakeObstacles.add_obstacles(100, -80, 134, 25, 25, 1, "MLR", period=10, amplitude=25)     # Moving left/right
	obstacle2 = FakeObstacles.add_obstacles(480, 119, 2, 10, 10, 1, "MUD", period=10, amplitude=25)        # Moving up/down
	obstacle3 = FakeObstacles.add_obstacles(110, -80, 10, 20, 20, 1, "ST")      # Stationary / POP-UP
	obstacle4 = FakeObstacles.add_obstacles(80, -80, 11, 20, 20, 1, "POP", tAppear=5)      # Stationary / POP-UP

	# Create Node instance, pass in Class of FakeObstacles
	fod = FakeObstacleNode(FakeObstacles)
	rospy.spin()
