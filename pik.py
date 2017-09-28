import numpy as np
import math
from random import random
import scipy.optimize
from joint import parseJointsConfigFile
import sys
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def inverseKinematics(target, rootJoint, numJoints):
	"""
	Solve the inverse Kinematics problem using the BFGS optimisation algorithm as described in:
	Inverse Kinematics The state of the art (http://image.diku.dk/projects/media/morten.engell_noerregaard.07.pdf)

	target - The target coordinates
	rootJoint - The robot rootJoint
	numJoints - The number of joints
	"""

	def translationError(q):
		"""
		Returns the translation error between the end effector's position with q angles and the target position.
		This function is used by the scipy minize function to find a correct solution to the inverse kinematic problem.

		q - The angles of the robot sorted from first to last.
		"""
		return np.linalg.norm(rootJoint.forwardKinematic(angles=q)[0][:3, -1] - target)

	# Unrestricted angles for now, we should add a way to restrict them in the future
	bounds = [[0, math.pi*2] for x in xrange(numJoints)]
	# Initialise random angles within bound otherwise the optimisation function won't converge
	initAngles = [random() * (bounds[x][1] - bounds[x][0]) + bounds[x][0] for x in xrange(numJoints)]
	return scipy.optimize.minimize(translationError, initAngles, method='L-BFGS-B', bounds=bounds)

def main():
	"""
	Controls the main flow of the program
	"""

	if len(sys.argv) < 2:
		print "pik.py usage:"
		print "python pik.py \"<target position array>\" -v"
		print "<target position array>: An array representing the position of the target to reach"
		print "-v: Outputs a visualisation graph of the robot"
		print "Example: python pik.py \"[0, 0, 2]\""
		exit(1)

	target = eval(sys.argv[1])
	if type(target) is not list or len(target) != 3:
		print "target position array must be an array of 3 values"
		exit(1)

	isVisualise = False
	for option in sys.argv[2:]:
		if option == "-v":
			isVisualise = True

	target = np.array(target)
	rootJoint, numJoints = parseJointsConfigFile('conf.json')
	result = inverseKinematics(target, rootJoint, numJoints)
	print result
	if isVisualise:
		_, jointsPos = rootJoint.forwardKinematic(angles=result.x, jointsPos=[])
		xs = []
		ys = []
		zs = []
		for position in jointsPos:
			xs.append(position[0])
			ys.append(position[1])
			zs.append(position[2])

		fig = plt.figure()
		ax = fig.add_subplot(111, projection='3d')
		ax.scatter([target[0]], [target[1]], [target[2]], c='r')
		ax.plot(xs, ys, zs)
		plt.show()

if __name__ == '__main__':
	main()