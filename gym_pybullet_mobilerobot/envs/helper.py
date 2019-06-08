import pybullet as p
import time
import numpy as np
import os
import math
import sys
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))

#---Directory Path---#
dirPath = os.path.dirname(os.path.realpath(__file__))

class helper():
	def __init__(self):

		self.wall_urdf = dirPath+'/data/wall.urdf'
		self.target_urdf = dirPath +'/data/cylinder.urdf'
		self.min_x, self.max_x = 0, 4
		self.min_y, self.max_y = 0, 4
		self.max_STEPS = 1500
		self.hokuyo_joint = 14

	def load_robot(self):
		# Init the robot randomly
		# x_start = self.max_x / 2 + np.random.uniform(- self.max_x / 3, self.max_x / 3)
		# y_start = self.max_y / 2 + np.random.uniform(- self.max_y / 3, self.max_y / 3)
		self.robot_uid = p.loadURDF((dirPath +'/data/turtlebot.urdf'),  np.array([self.max_x/2, self.max_x/2, 0]) )
		p.changeVisualShape(self.robot_uid, -1, rgbaColor=[0.0, 0.0, 0.0, 1.0])
		return np.array([self.max_x/2, self.max_x/2])

	def load_walls(self):
		# print('1234:',self.wall_urdf)
		plane = p.loadURDF(dirPath +'/data/plane.urdf')
		red, green, blue = [0.8, 0, 0, 1], [0, 0.8, 0, 1], [0, 0, 0.8, 1]

		wall_left = p.loadURDF(self.wall_urdf, [self.max_x / 2, 0, 0], useFixedBase=True)
		# Change color
		p.changeVisualShape(wall_left, -1, rgbaColor=red)

		# getQuaternionFromEuler -> define orientation
		wall_bottom = p.loadURDF(self.wall_urdf, [self.max_x, self.max_y / 2, 0],
								 p.getQuaternionFromEuler([0, 0, np.pi / 2]), useFixedBase=True)

		wall_right = p.loadURDF(self.wall_urdf, [self.max_x / 2, self.max_y, 0], useFixedBase=True)
		p.changeVisualShape(wall_right, -1, rgbaColor=green)

		wall_top = p.loadURDF(self.wall_urdf, [self.min_x, self.max_y / 2, 0],
							  p.getQuaternionFromEuler([0, 0, np.pi / 2]), useFixedBase=True)
		p.changeVisualShape(wall_top, -1, rgbaColor=blue)
		# walls = [wall_left, wall_bottom, wall_right, wall_top]

	def load_target(self):
		trans,rot = p.getBasePositionAndOrientation(self.robot_uid)
		x_pos = 0.9 * self.max_x
		y_pos = self.max_y * 3 / 4
		margin = 0.1 * self.max_x
		x_pos = np.random.uniform(self.min_x + margin, self.max_x - margin)
		y_pos = np.random.uniform(self.min_y + margin, self.max_y - margin)

		while np.linalg.norm(np.array([trans[0],trans[1]]) - np.array([x_pos ,y_pos])) < 0.2:
			print("Target too close to robot -> Resetting")
			x_pos = 0.9 * self.max_x
			y_pos = self.max_y * 3 / 4
			margin = 0.1 * self.max_x
			x_pos = np.random.uniform(self.min_x + margin, self.max_x - margin)
			y_pos = np.random.uniform(self.min_y + margin, self.max_y - margin)

		self.target_cylinder = p.loadURDF(self.target_urdf, [x_pos, y_pos, 0], useFixedBase=True)
		return np.array([x_pos,y_pos])

	def reset_target(self):
		p.removeBody(self.target_cylinder)
		self.target = self.load_target()
		print("deleted old target")

	def differential_drive(self,action ,L=0.22,R=0.076,speed=10):
		# L=distance between wheels, R=wheel radius
		# action[0]=linear vel , action[1] = angular vel
		# print('yes')
		rightWheelVelocity = 0.
		leftWheelVelocity = 0.
		# rightWheelVelocity += (2*action[0] + action[1]*L) / 2*R
		# leftWheelVelocity += (2*action[0] - action[1]*L) / 2*R
		rightWheelVelocity+= (action[0]+action[1])*speed
		leftWheelVelocity += (action[0]-action[1])*speed
		return np.array([rightWheelVelocity,leftWheelVelocity])

	def init_laserscanner(self):
		numRays=10
		replaceLines=True
		rayFrom=[]
		rayTo=[]
		rayIds=[]
		rayHitColor = [1,0,0]
		rayMissColor = [0,1,0]
		rayLen = 8
		rayStartLen=0.25

		for i in range (numRays):
			#rayFrom.append([0,0,0])
			rayFrom.append([rayStartLen*math.sin(-0.5*0.25*2.*math.pi+0.75*2.*math.pi*float(i)/numRays), rayStartLen*math.cos(-0.5*0.25*2.*math.pi+0.75*2.*math.pi*float(i)/numRays),0])
			rayTo.append([rayLen*math.sin(-0.5*0.25*2.*math.pi+0.75*2.*math.pi*float(i)/numRays), rayLen*math.cos(-0.5*0.25*2.*math.pi+0.75*2.*math.pi*float(i)/numRays),0])
			if (replaceLines):
				rayIds.append(p.addUserDebugLine(rayFrom[i], rayTo[i], rayMissColor,parentObjectUniqueId=self.robot_uid , parentLinkIndex=self.hokuyo_joint ))
			else:
				rayIds.append(-1)

		return rayFrom,rayTo

	def read_laser_scan(self):
		numRays=10

		# rayFrom,rayTo = self.init_laserscanner()
		numThreads=0
		results = p.rayTestBatch(self.rayFrom,self.rayTo,numThreads, parentObjectUniqueId=self.robot_uid, parentLinkIndex=self.hokuyo_joint)
		result = []
		trans,rot = p.getBasePositionAndOrientation(self.robot_uid)
		for i in range (numRays):

			laser = np.linalg.norm(np.array([trans[0],trans[1]]) - np.array([results[i][3][0],results[i][3][1]]))
			result.append(laser)
		return np.array([result])

	def IL_get_action(self):
		keys = p.getKeyboardEvents()
		forward = 0
		turn = 0

		# while forward!=0 and turn!=0
		# while keys == {}:
		# 	keys = p.getKeyboardEvents()
		# 	print('Empty')

		for k,v in keys.items():

	                if (k == p.B3G_RIGHT_ARROW and (v&p.KEY_WAS_TRIGGERED)):
	                        turn = -0.5
	                if (k == p.B3G_RIGHT_ARROW and (v&p.KEY_WAS_RELEASED)):
	                        turn = 0
	                if (k == p.B3G_LEFT_ARROW and (v&p.KEY_WAS_TRIGGERED)):
	                        turn = 0.5
	                if (k == p.B3G_LEFT_ARROW and (v&p.KEY_WAS_RELEASED)):
	                        turn = 0

	                if (k == p.B3G_UP_ARROW and (v&p.KEY_WAS_TRIGGERED)):
	                        forward=1
	                if (k == p.B3G_UP_ARROW and (v&p.KEY_WAS_RELEASED)):
	                        forward=0
	                if (k == p.B3G_DOWN_ARROW and (v&p.KEY_WAS_TRIGGERED)):
	                        forward=-1
	                if (k == p.B3G_DOWN_ARROW and (v&p.KEY_WAS_RELEASED)):
	                        forward=0

		return np.array([forward,turn])













#
# p.setRealTimeSimulation(1)
