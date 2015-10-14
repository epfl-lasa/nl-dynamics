# Starter code for plotting a Kuka trajectory from a rosbag file.

import rosbag
import numpy as np

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Data structur
class PositionData:
	position = []
	velocity = []

def load(name):
	# Open the given file and extract the CartState topics.
	#bag = rosbag.Bag('2015.10.09-kuka.with.correction.01.bag')
	bag = rosbag.Bag(name)
	gen = bag.read_messages(topics=['/KUKA/CartState'])
	stuff = list(gen)

	# The rosbag tells us the topic, message data (what we want), and time.
	[topics, msgs, times] = zip(*stuff)

	print("Have a trajectory with " + str(len(msgs)) + " data points")

	# create output structure
	trajectory_data = PositionData()
	original_data   = PositionData()

	# Extract x/y/z coordinates of the position and the linear velocity between each point
	trajectory_data.position  = np.transpose  ([[msg.pose.position.x for msg in msgs],
												[msg.pose.position.y for msg in msgs],
												[msg.pose.position.z for msg in msgs]])
	trajectory_data.velocity  = np.transpose  ([[msg.twist.linear.x for msg in msgs],
												[msg.twist.linear.y for msg in msgs],
												[msg.twist.linear.z for msg in msgs]])

	# Extract x/y/z coordinates of the original Dynamic velocity
	gen = bag.read_messages(topics=['/KUKA/DesiredState'])
	stuff = list(gen)
	[topics, msgs, times] = zip(*stuff)
	original_data.velocity  = np.transpose ([[msg.twist.linear.x for msg in msgs],
											 [msg.twist.linear.y for msg in msgs],
											 [msg.twist.linear.z for msg in msgs]])

	# should I take care about the case that len(original_data.velocity) != len(trajectory_data.velocity) ??

	return trajectory_data, original_data
	
def plotPoints(data):
	# create new figure in 3D
	fig = plt.figure()
	ax = fig.gca(projection='3d')

	# plot points and add label
	ax.plot(data.position[:,0], data.position[:,1], data.position[:,2], label='2015.10.09-kuka-no.correction.bag')

	# label axis
	ax.set_xlabel('x')
	ax.set_ylabel('y')
	ax.set_zlabel('z')
	ax.legend()

	# show result
	plt.ion()
	plt.show()

	return ax

def drawStartStop(start, stop, ax):

	# plot star and triangle shape on desired points
	ax.scatter(start[0,:], start[1,:], start[2,:], s=100, c='r', marker='*')
	ax.scatter(stop[0,:],  stop[1,:],  stop[2,:],  s=100, c='b', marker='^')

	# show result
	plt.ion()
	plt.show()

def analyzeData (trajectory_data, original_data, precisionFactor):
	# Precision factor is the more precise when = 0

	# normalization of the vectors in the matrix
	for (vect_t,vect_o) in zip(trajectory_data.velocity, original_data.velocity) :
		vect_t = vect_t / np.linalg.norm(vect_t)
		vect_o = vect_o / np.linalg.norm(vect_o)

	# calculation of the dot product
	dotprod=[];
	for (trajectory, original) in zip(trajectory_data.velocity, original_data.velocity) :
		dotprod.append(np.dot(trajectory, original))

	# verifying all the starts-and-stops
	isIn=0
	count=0    # count variable is quite useless in fact..
	start=[]
	stop=[]
	for (pos, dot) in zip(trajectory_data.position, dotprod) :
		if not(isIn) and dot<(1-precisionFactor) :
			start.append(pos)
			isIn = 1;
			count = count+1;

		elif isIn and dot>=(1-precisionFactor) :
			stop.append(pos)
			isIn = 0;

	# in case of there weren't the last stop
	if isIn :
		stop.append(trajectory_data.position[-1])

	return np.transpose(start), np.transpose(stop)


def run():
	tab=['2015.10.09-kuka.with.correction.01.bag',
		 '2015.10.09-kuka.with.correction.02.bag',
		 '2015.10.09-kuka.two.corrections.bag',
		 '2015.10.09-kuka-no.correction.bag']

	for name in tab :
		# load data
		(trajectory_data, original_data) = load(name)

		# prepare figure and plot points
		ax = plotPoints(trajectory_data)

		# analyze data
		(start, stop) = analyzeData(trajectory_data, original_data, 0.5)

		# drawing start-stops points (red stars for the starts and blue triangle for the stops)
		drawStartStop(start, stop, ax)

	# pause
	raw_input('press enter')








if __name__ == '__main__':
	run()

__author__ = 'felixd'
