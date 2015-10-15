# Starter code for plotting a Kuka trajectory from a rosbag file.

import rosbag
import numpy as np
from spline import *

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Data structur
class PositionData:
    position = []
    velocity = []

def getSmoothDataIndex(nb_points, alpha):
# return a vector of the index of the new point 
    if nb_points*alpha < 2 :
        return [1,nb_points]
    else :
        t=np.linspace(0,nb_points-1, nb_points*alpha)
        return [int(i) for i in t]

def smoothData(tab, alpha):
    t=getSmoothDataIndex(len(tab), alpha)
    return tab[t]

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

    # checking length
    lenPos =len(trajectory_data.position)
    lenVel =len(trajectory_data.velocity)
    lenOri =len(original_data.velocity)

    if lenPos != lenVel :
        print("WARNING, trajectory position and velocity don't have the same length")
        lenPos=min(lenPos,lenVel)
        lenVel=lenPos
        trajectory_data.position=trajectory_data.position[range(lenPos)]
        trajectory_data.velocity=trajectory_data.velocity[range(lenVel)]

    if lenVel != lenOri :
        print("WARNING, trajectory position/velocity and original velocity don't have the same length")
        lenVel=min(lenVel,lenOri)
        lenOri=lenVel
        lenPos=lenVel
        trajectory_data.position=trajectory_data.position[range(lenPos)]
        trajectory_data.velocity=trajectory_data.velocity[range(lenVel)]
        original_data.velocity  =original_data.velocity[range(lenOri)]

    # should I take care about the case that len(original_data.velocity) != len(trajectory_data.velocity) ??

    return trajectory_data, original_data
    
def plotPoints(data, name):
    # create new figure in 3D
    fig = plt.figure()
    ax = fig.gca(projection='3d')

    # plot points and add label
    ax.plot(data.position[:,0], data.position[:,1], data.position[:,2], c='b', label=name)

    # label axis
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')
    ax.legend()

    # show result
    plt.ion()
    plt.show()

    return ax

def plotSpline3D(data,ax):
    #calculate splines data
    splineData3D = spline3D(data.position, data.velocity)

    #calculate new points
    size = max(len(data.position), len(data.velocity)) * 1
    t=np.linspace(0,1,size)

    new_pos=getPointsSpline3D(splineData3D, t)

    #plot result
    ax.plot(new_pos[0], new_pos[1], new_pos[2], c='r', label='spline3D')

def drawStartStop(start, stop, ax):

    # plot star and triangle shape on desired points
    ax.scatter(start[0,:], start[1,:], start[2,:], s=100, c='r', marker='*')
    ax.scatter(stop[0,:],  stop[1,:],  stop[2,:],  s=100, c='b', marker='^')

    # show result
    plt.ion()
    plt.show()

def analyzeData (trajectory_data, original_data, precision_factor):
    # trajectory_data represent the actual data of the robot, the position and the velocity of the robot for all the points
    # original_data only represent a vector from the robot to the attractor point (following the OriginalDynamics) for all the points
    # Precision factor how precise the algorithm need to be to detect some jump in the trajectory : 0 is absolute precision

    # normalization of the vectors in the matrix
    for (vect_t,vect_o) in zip(trajectory_data.velocity, original_data.velocity) :
        vect_t = vect_t / np.linalg.norm(vect_t)
        vect_o = vect_o / np.linalg.norm(vect_o)

    # calculation of the dot product
    dot_prod=[np.dot(trajectory, original) for (trajectory, original) in zip(trajectory_data.velocity, original_data.velocity)]

    # verifying all the starts-and-stops
    is_in=0
    start=[]
    stop=[]
    for (pos, dot) in zip(trajectory_data.position, dot_prod) :
        if not(is_in) and dot<(1-precision_factor) :
            start.append(pos)
            is_in = 1;

        elif is_in and dot>=(1-precision_factor) :
            stop.append(pos)
            is_in = 0;

    # in case of there weren't the last stop
    if is_in :
        stop.append(trajectory_data.position[-1])

    return np.transpose(start), np.transpose(stop)


def run():
    tab=['2015.10.09-kuka.with.correction.01.bag',
         '2015.10.09-kuka.with.correction.02.bag',
         '2015.10.09-kuka.two.corrections.bag',
         '2015.10.09-kuka-no.correction.bag']

    tab2=['2015.10.09-kuka.with.correction.01.bag']

    #set some parameter
    alpha = 1./100
    precision_factor = 0.5

    for name in tab2 :
        # load data
        (trajectory_data, original_data) = load(name)

        #smooth data
        (trajectory_data.position, trajectory_data.velocity, original_data.velocity) = (smoothData(trajectory_data.position, alpha),
                                                                                        smoothData(trajectory_data.velocity, alpha),
                                                                                        smoothData(  original_data.velocity, alpha))
        # prepare figure and plot points
        ax = plotPoints(trajectory_data, name + ", with alpha=" + str(alpha))

        # analyze data
        (start, stop) = analyzeData(trajectory_data, original_data, precision_factor)

        # drawing start-stops points (red stars for the starts and blue triangle for the stops)
        drawStartStop(start, stop, ax)

        #drawing splineparts
        #plotSpline3D(trajectory_data,ax)

    # pause
    raw_input('press enter')








if __name__ == '__main__':
    run()

__author__ = 'felixd'
