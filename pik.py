from json import loads
import numpy as np
import math

class Joint:
	"""
	Contains all the information related to joints
	"""
	def __init__(self, config, parent=None):
		"""
		Initialises the objects with the following data:
		origin - The xyz coordinates of the joint (Numpy array)
		rotation - The matrix rotation of the join (Numpy array)
		type - The joint type, currently supporting "revolute" and "end"
		parent - The parent Joint object
		child - The child Joint object
		name - The name of the joint (Preferably unique)
		"""
		self.origin = np.array(config["xyz"])
		self.angles = np.array(config["rpy"])
		self.type = config["type"]
		self.parent = parent
		self.name = config["name"]
		self.child = None
		self.axis = config["axis"]

	def __str__(self):
		"""
		Converts the object into a nicely formatted string for output purposes
		"""
		toOutput = "---------- %s ----------\n" % self.name
		toOutput += str(self.transMat)
		if self.type != "end":
			toOutput += "\n######## %s's Child #########\n" % self.name
			toOutput += str(self.child)
		return toOutput

	def forwardKinematic(self, transMat, angles=[]):
		angle = 0
		if angles:
			angle = angles.pop(0)

		curMat = np.dot(transMat, self.getTransformationMatrix(angle))
		if self.child:
			print curMat
			return self.child.forwardKinematic(curMat, angles)
		else:
			return curMat

	def getTransformationMatrix(self, jointAngle):
		parentOrigin = np.array([0, 0, 0])
		parentAngles = np.array([0, 0, 0])
		if self.parent:
			parentOrigin = self.parent.origin
			parentAngles = self.parent.angles

		jointAngles = np.array([0, 0, 0], dtype=float)
		jointAngles[self.axis] = jointAngle
		transMat = np.append(eulerAnglesToRot(self.angles - parentAngles + jointAngles), np.transpose([self.origin - parentOrigin]), axis=1)
		transMat = np.append(transMat, [[0,0,0,1]], axis=0)
		return transMat

def eulerAnglesToRot(rpy):
	"""
	Converts the euler angles to a rotation matrix using Numpy
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

def createChildJoints(configByParent, parent):
	"""
	Creates the joints recursively and assign the childs to their parent
	"""
	if parent.name not in configByParent:
		return

	childJoint = Joint(configByParent[parent.name], parent)
	parent.child = childJoint
	createChildJoints(configByParent, childJoint)

def main():
	"""
	Controls the main flow of the program
	"""
	configByParent = {}
	#Read the robot's configuration from the conf.json file
	with file('conf.json') as f:
		robotDefinition = loads(f.read())
		for joint in robotDefinition:
			configByParent[joint["parent"]] = joint

	#Create the joints object
	rootJoint = Joint(configByParent["root"])
	createChildJoints(configByParent, rootJoint)
	print rootJoint.forwardKinematic(np.eye(4), [1,1,0])

if __name__ == '__main__':
	main()