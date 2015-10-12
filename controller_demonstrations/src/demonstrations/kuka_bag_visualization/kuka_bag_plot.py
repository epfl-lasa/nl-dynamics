# Starter code for plotting a Kuka trajectory from a rosbag file.

import rosbag

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Data structur
class Vector3D:
    x=[]
    y=[]
    z=[]

class PositionData:
    position = Vector3D()
    velocity = Vector3D()

def load(name):
    # Open the given file and extract the CartState topics.
    #bag = rosbag.Bag('2015.10.09-kuka.with.correction.01.bag')
    bag = rosbag.Bag(name)
    gen = bag.read_messages(topics=['/KUKA/CartState'])
    stuff = list(gen)

    # The rosbag tells us the topic, message data (what we want), and time.
    [topics, msgs, times] = zip(*stuff)

    print("Have a trajectory with %d data points" % len(msgs))

    # create output structure
    trajectory_data = PositionData()
    original_data   = PositionData()

    # Extract x/y/z coordinates of the position and the linear velocity between each point
    trajectory_data.position.x = [msg.pose.position.x for msg in msgs]
    trajectory_data.position.y = [msg.pose.position.y for msg in msgs]
    trajectory_data.position.z = [msg.pose.position.z for msg in msgs]
    trajectory_data.velocity.x = [msg.twist.linear.x for msg in msgs]
    trajectory_data.velocity.y = [msg.twist.linear.y for msg in msgs]
    trajectory_data.velocity.z = [msg.twist.linear.z for msg in msgs]

    # Extract x/y/z coordinates of the original Dynamic velocity
    gen = bag.read_messages(topics=['/KUKA/DesiredState'])
    stuff = list(gen)
    [topics, msgs, times] = zip(*stuff)
    original_data.velocity.x = [msg.twist.linear.x for msg in msgs]
    original_data.velocity.y = [msg.twist.linear.y for msg in msgs]
    original_data.velocity.z = [msg.twist.linear.z for msg in msgs]

    # should I take care about the case that len(original_data.velocity) != len(trajectory_data.velocity) ??

    return trajectory_data, original_data
    
def plotPoints(data):
	fig = plt.figure()
	ax = fig.gca(projection='3d')

	ax.plot(data.position.x, data.position.y, data.position.z, label='2015.10.09-kuka-no.correction.bag')
	ax.set_xlabel('x')
	ax.set_ylabel('y')
	ax.set_zlabel('z')

	ax.scatter(data.position.x[0:1], data.position.y[0:1], data.position.z[0:1], label='Start')

	ax.legend()
	plt.ion()
	plt.show()


def run():
    (trajectory_data, original_data) = load('2015.10.09-kuka-no.correction.bag')
    plotPoints(trajectory_data)

    # trace('2015.10.09-kuka.with.correction.01.bag')
    # trace('2015.10.09-kuka.with.correction.02.bag')
    # trace('2015.10.09-kuka.two.corrections.bag')
    # trace('2015.10.09-kuka-no.correction.bag')
	
    raw_input('press enter')


if __name__ == '__main__':
    run()

__author__ = 'felixd'
