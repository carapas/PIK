"""
This file contains all the joint data and functions to parse the configuration file
"""
import numpy as np
from json import loads
import math

class Joint:
	"""
	Contains all the information related to joints
	"""
	def __init__(self, config, parent=None):
		"""
		Initialises the objects with the following data:

		origin - The xyz coordinates of the joint (Numpy array)
		angles - The rpy angles of the joint (Numpy array)
		type - The joint type, currently supporting "revolute" and "end"
		parent - The parent Joint object
		child - The child Joint object
		name - The name of the joint (Preferably unique)
		axis - The rotation or translation axis of the joint
		childName - The string name of the child
		"""

		self.origin = np.array(config["xyz"])
		self.angles = np.array(config["rpy"])
		self.type = config["type"]
		self.parent = parent
		self.name = config["name"]
		self.child = None
		self.axis = config["axis"]
		self.childName = None
		if "child" in config:
			self.childName = config["child"]

	def __str__(self):
		"""
		Converts the object into a nicely formatted string for output purposes
		"""

		toOutput = "---------- %s ----------\n" % self.name
		toOutput += str(self.getTransformationMatrix(0))
		if self.type != "end":
			toOutput += "\n######## %s's Child #########\n" % self.name
			toOutput += str(self.child)
		return toOutput

	def forwardKinematic(self, transMat=np.eye(4), angles=[], jointsPos=[]):
		"""
		Calculates the forward kinematics from a list angles

		transMat - The current transformation matrix. Defaults to the identity matrix.
		angles - The joint angles from ordered from the first one to the last one
		"""

		angle = 0
		if len(angles):
			angle, angles = angles[0], angles[1:]

		curMat = np.dot(transMat, self.getTransformationMatrix(angle))
		jointsPos.append(curMat[:3, -1])
		if self.child:
			return self.child.forwardKinematic(curMat, angles, jointsPos)
		else:
			return curMat, jointsPos

	def getTransformationMatrix(self, jointAngle):
		"""
		Calculates the relative transformation matrix relative to the parent

		jointAngle - The angle of the joint
		"""

		parentOrigin = np.array([0, 0, 0])
		parentAngles = np.array([0, 0, 0])
		if self.parent:
			parentOrigin = self.parent.origin
			parentAngles = self.parent.angles

		jointAngles = np.array([0, 0, 0], dtype=float)
		jointAngles[self.axis] = jointAngle
		transMat = np.append(rpyAnglesToRot(self.angles - parentAngles + jointAngles), np.transpose([self.origin - parentOrigin]), axis=1)
		transMat = np.append(transMat, [[0,0,0,1]], axis=0)
		return transMat

def createChildJoints(configsByName, parent):
	"""
	Creates the joints recursively and assign the childs to their parent

	configsByName - Dictionary of the joint configurations by name
	parent - The Joint object of the parent
	"""

	childJoint = Joint(configsByName[parent.childName], parent)
	parent.child = childJoint
	if "child" not in configsByName[childJoint.name]:
		return

	createChildJoints(configsByName, childJoint)

def parseJointsConfigFile(filePath):
	"""
	Parses the configuration file and outputs the root joint with the number of joints.

	filePath - The patht to the configuration file
	"""
	configsByName = {}
	rootConfig = {}
	numJoints = 0
	#Read the robot's configuration from the conf.json file
	with file(filePath) as f:
		robotDefinition = loads(f.read())
		numJoints = len(robotDefinition) - 1 #Must substract the end effector since it is not a joint
		for joint in robotDefinition:
			configsByName[joint["name"]] = joint
			if "isRoot" in joint and joint["isRoot"]:
				rootConfig = joint

	#Create the joints object
	rootJoint = Joint(rootConfig)
	createChildJoints(configsByName, rootJoint)
	return rootJoint, numJoints

def rpyAnglesToRot(rpy):
	"""
	Converts the euler angles to a rotation matrix using Numpy

	rpy - An array containing the roll, pitch, yaw angles
	"""

	R_x = np.array([[1, 0, 0],
	                [0, math.cos(rpy[0]), -math.sin(rpy[0])],
	                [0, math.sin(rpy[0]), math.cos(rpy[0])]])

	R_y = np.array([[math.cos(rpy[1]), 0, math.sin(rpy[1])],
	                [0, 1, 0],
	                [-math.sin(rpy[1]), 0, math.cos(rpy[1])]])

	R_z = np.array([[math.cos(rpy[2]), -math.sin(rpy[2]), 0],
	                [math.sin(rpy[2]), math.cos(rpy[2]), 0],
	                [0, 0, 1]])

	R = np.dot(R_z, np.dot( R_y, R_x ))
	return R