# Starter code for plotting a Kuka trajectory from a rosbag file.

import rosbag

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


def load():
    # Open the given file and extract the CartState topics.
    bag = rosbag.Bag('2015.10.09-kuka.with.correction.02.bag')
    gen = bag.read_messages(topics=['/KUKA/CartState'])
    stuff = list(gen)

    # The rosbag tells us the topic, message data (what we want), and time.
    [topics, msgs, times] = zip(*stuff)

    print("Have a trajectory with %d data points" % len(msgs))

    # Extract x/y/z coordinates
    x = [msg.pose.position.x for msg in msgs]
    y = [msg.pose.position.y for msg in msgs]
    z = [msg.pose.position.z for msg in msgs]

    return x, y, z


def run():
    (x, y, z) = load()

    fig = plt.figure()
    ax = fig.gca(projection='3d')

    ax.plot(x, y, z, label='Kuka trajectory')
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')

    ax.scatter(x[0:1], y[0:1], z[0:1], label='Start')

    ax.legend()
    plt.show()


if __name__ == '__main__':
    run()

__author__ = 'felixd'
