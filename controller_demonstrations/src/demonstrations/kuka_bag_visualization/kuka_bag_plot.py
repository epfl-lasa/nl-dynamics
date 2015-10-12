# Starter code for plotting a Kuka trajectory from a rosbag file.

import rosbag

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


def load(name):
    # Open the given file and extract the CartState topics.
    #bag = rosbag.Bag('2015.10.09-kuka.with.correction.01.bag')
    bag = rosbag.Bag(name)
    gen = bag.read_messages(topics=['/KUKA/CartState'])
    #gen = bag.read_messages(topics=['/KUKA/DesiredState'])
    stuff = list(gen)

    # The rosbag tells us the topic, message data (what we want), and time.
    [topics, msgs, times] = zip(*stuff)

    print("Have a trajectory with %d data points" % len(msgs))

    # Extract x/y/z coordinates
    x = [msg.pose.position.x for msg in msgs]
    y = [msg.pose.position.y for msg in msgs]
    z = [msg.pose.position.z for msg in msgs]
    #x = [msg.twist.linear.x for msg in msgs]
    #y = [msg.twist.linear.y for msg in msgs]
    #z = [msg.twist.linear.z for msg in msgs]

    return x, y, z
    
def trace(name):
	(x, y, z) = load(name)

	fig = plt.figure()
	ax = fig.gca(projection='3d')

	ax.plot(x, y, z, label=name)
	ax.set_xlabel('x')
	ax.set_ylabel('y')
	ax.set_zlabel('z')

	ax.scatter(x[0:1], y[0:1], z[0:1], label='Start')

	ax.legend()
	plt.ion()
	plt.show()


def run():
    trace('2015.10.09-kuka.with.correction.01.bag')
    trace('2015.10.09-kuka.with.correction.02.bag')
    trace('2015.10.09-kuka.two.corrections.bag')
    trace('2015.10.09-kuka-no.correction.bag')
	
    raw_input('press enter')


if __name__ == '__main__':
    run()

__author__ = 'felixd'
