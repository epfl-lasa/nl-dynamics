# Starter code for plotting a Kuka trajectory from a rosbag file.

import rosbag
import numpy as np
from spline import spline3D, getPointsSpline3D

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


def getSmoothDataIndex(nb_points, alpha):
# return a vector of the index of the new point
    if nb_points*alpha < 2:
        return [1, nb_points]
    else:
        t = np.linspace(0, nb_points-1, nb_points*alpha)
        return [int(i) for i in t]


def smoothData(tab, alpha):
    t = getSmoothDataIndex(len(tab), alpha)
    return [tab[i] for i in t]


def load(name):
    # Open the given file and extract the CartState topics.
    bag = rosbag.Bag(name)
    gen = bag.read_messages(topics=['/KUKA/CartState'])
    stuff = list(gen)

    # The rosbag tells us the topic, message data (what we want), and time.
    [topics, msgs, times] = zip(*stuff)
    trajectory_data = msgs

    print("Have a trajectory with " + str(len(msgs)) + " data points")

    # Extract x/y/z coordinates of the original Dynamic velocity
    gen = bag.read_messages(topics=['/KUKA/DesiredState'])
    stuff = list(gen)
    [topics, msgs, times] = zip(*stuff)
    original_data = msgs

    # checking length
    lenPos = len(trajectory_data)
    lenOri = len(original_data)

    if lenPos != lenOri:
        print("WARNING, trajectory and Desired_Velocity don't have the same length")
        lenPos = min(lenPos, lenOri)
        lenOri = lenPos
        trajectory_data = trajectory_data[:lenPos]
        original_data = original_data[:lenPos]

    return trajectory_data, original_data


def plotPoints(data, name):
    # create new figure in 3D
    fig = plt.figure()
    ax = fig.gca(projection='3d')

    # plot points and add label
    ax.plot([d.pose.position.x for d in data[:]],
            [d.pose.position.y for d in data[:]],
            [d.pose.position.z for d in data[:]], c='b', label=name)

    # label axis
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')
    ax.legend()

    # show result
    plt.ion()
    plt.show()

    return ax


def plotSpline3D(data, ax, factor):
    #the factor parameter represent how much you want MORE points that are being in data

    #calculate splines data
    splineData3D = spline3D(data)

    #calculate new points
    size = len(data) * factor
    t = np.linspace(0, 1, size)

    new_pos = getPointsSpline3D(splineData3D, t)

    #plot result
    ax.plot(new_pos[0], new_pos[1], new_pos[2], c='r', label='spline3D')

    # show result
    plt.ion()
    plt.show()


def drawStartStop(start, stop, ax):
    if (len(start) > 0):
        # plot star and triangle shape on desired points
        ax.scatter(start[0, :], start[1, :], start[2, :], s=100, c='r', marker='*')
        ax.scatter(stop[0, :],  stop[1, :],  stop[2, :],  s=100, c='b', marker='^')

        # show result
        plt.ion()
        plt.show()


def analyzeData(trajectory_data, original_data, precision_factor=0.4, return_type='point'):
    # trajectory_data represent the actual data of the robot, the position and the velocity of the robot for all the points
    # original_data only represent a vector from the robot to the attractor point (following the OriginalDynamics) for all the points
    # Precision factor how precise the algorithm need to be to detect some jump in the trajectory : 0 is absolute precision
    # return_type is the type of return, can be 'point' for an array of start and stop coord, can also be 'index', for an array of index

    #create some usefull variables
    is_in = 0
    start = []
    stop = []
    index_start = []
    index_stop = []

    for i, (tra, ori) in enumerate(zip(trajectory_data, original_data)):
    #calculate dot product
        pos = [tra.pose.position.x, tra.pose.position.y, tra.pose.position.z]
        dis_vel = [tra.twist.linear.x, tra.twist.linear.y, tra.twist.linear.z]
        vel = [ori.twist.linear.x, ori.twist.linear.y, ori.twist.linear.z]

        dot = np.dot(dis_vel/np.linalg.norm(dis_vel), vel/np.linalg.norm(vel))

    #analyse data and eventualy store new start-stop point
        if not(is_in) and dot < (1-precision_factor):
            start.append(pos)
            index_start.append(i)
            is_in = 1

        elif is_in and dot >= (1-precision_factor):
            stop.append(pos)
            index_stop.append(i)
            is_in = 0

    # in case of there weren't the last stop
    if is_in:
        pos = trajectory_data[-1].pose.position
        stop.append([pos.x, pos.y, pos.z])
        index_stop.append(i)

    if return_type == 'point':
        return np.transpose(start), np.transpose(stop)
    elif return_type == 'index':
        return index_start, index_stop

    else:
        rospy.logerr("error in function analyzeData, return_type is wrong")
        return 0, 0


#reflexion here : maybe the algorithm would more precise with a filter : if 2 points (start then stop) are too closed, we remove then.
#or better : if 2 points are too closed and the dot product is not so low, keep them, but if they are closed with a high dot product remove them.
# maybe find a formula ?
#this can be done with another correction-factor, maybe a factor that tell us up to how many points the start and stop is validate ?
#after reflexion it's quite like a lowpass filter for big amplitude wave.

#another thing :  when we found a jump on the curve that is supposed to be a jump, the start and stops are in the middle of the jump, should correct it

#another thing, frequency analysis ??


def forcePlot(data):
    force_try = [np.sqrt(f.wrench.force.x**2 + f.wrench.force.y**2 + f.wrench.force.z**2) for f in data]

    fig = plt.figure()
    ax = fig.gca()

    # plot points and add label
    ax.plot(range(len(force_try)), force_try, c='b', label="force_try")

    # label axis
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.legend()

    # show result
    plt.ion()
    plt.show()


def dotProdPlot(vel1, vel2, name):
    dot_prod = [np.dot(v1/np.linalg.norm(v1), v2/np.linalg.norm(v2)) for (v1, v2) in zip(vel1, vel2)]

    # create new figure in 2D
    fig = plt.figure()
    ax = fig.gca()

    # plot points and add label
    ax.plot(range(len(dot_prod)), dot_prod, c='b', label=name + " - dot product")

    # label axis
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.legend()

    # show result
    plt.ion()
    plt.show()


def run():

    tab = ['2015.10.09-kuka.with.correction.01.bag',
           '2015.10.09-kuka.with.correction.02.bag',
           '2015.10.09-kuka.two.corrections.bag',
           '2015.10.09-kuka-no.correction.bag']

    tab1 = ['2015.10.09-kuka.with.correction.01.bag']
    tab2 = ['2015.10.09-kuka.with.correction.02.bag']
    tab3 = ['2015.10.09-kuka.two.corrections.bag']
    tab4 = ['2015.10.09-kuka-no.correction.bag']

    #set some parameter
    alpha = 1./50           # alpha si the coefficient of smoothness, the most it goes to zero, the less points there will remain
    precision_factor = 0.4  # this factor is for analysis, the most it goes to 0, the sharper the analysis will be
    factor = 30             # this factor represent how much more point than the robot gave are drawn by the spline3D

    for name in tab3:
        # load data
        (trajectory_data, original_data) = load(name)

        (trajectory_data, original_data) = (smoothData(trajectory_data, alpha),
                                            smoothData(original_data, alpha))

        # analyze data
        (start, stop) = analyzeData(trajectory_data, original_data, precision_factor)

        #drawpoints (to be deleated)
        ax = plotPoints(trajectory_data, name + ", with alpha=" + str(alpha))

        # drawing start-stops points (red stars for the starts and blue triangle for the stops)
        drawStartStop(start, stop, ax)

        #drawing splineparts
        plotSpline3D(trajectory_data, ax, factor)

        #plot dotprod
        #dotProdPlot(trajectory_data.velocity, original_data.velocity, name)

    # pause
    raw_input('press enter')
    plt.close("all")


if __name__ == '__main__':
    run()

__author__ = 'felixd'
