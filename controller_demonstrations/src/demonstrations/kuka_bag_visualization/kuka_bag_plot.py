# Starter code for plotting a Kuka trajectory from a rosbag file.

import rosbag
import numpy as np
from spline import spline3D, getPointsSpline3D

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


class PositionData:
    def __init__(self):
        self.position = []
        self.velocity = []


def getSmoothDataIndex(nb_points, alpha):
# return a vector of the index of the new point
    if nb_points*alpha < 2:
        return [1, nb_points]
    else:
        t = np.linspace(0, nb_points-1, nb_points*alpha)
        return [int(i) for i in t]


def smoothData(tab, alpha):
    t = getSmoothDataIndex(len(tab), alpha)
    return tab[t]


def load(name):
    # Open the given file and extract the CartState topics.
    bag = rosbag.Bag(name)
    gen = bag.read_messages(topics=['/KUKA/CartState'])
    stuff = list(gen)

    # The rosbag tells us the topic, message data (what we want), and time.
    [topics, msgs, times] = zip(*stuff)

    print("Have a trajectory with " + str(len(msgs)) + " data points")

    # create output structure
    trajectory_data = PositionData()
    original_data = PositionData()

    # Extract x/y/z coordinates of the position and the linear velocity
    # between each point
    trajectory_data.position = np.transpose([[msg.pose.position.x for msg in msgs],
                                             [msg.pose.position.y for msg in msgs],
                                             [msg.pose.position.z for msg in msgs]])
    trajectory_data.velocity = np.transpose([[msg.twist.linear.x for msg in msgs],
                                             [msg.twist.linear.y for msg in msgs],
                                             [msg.twist.linear.z for msg in msgs]])

    # Extract x/y/z coordinates of the original Dynamic velocity
    gen = bag.read_messages(topics=['/KUKA/DesiredState'])
    stuff = list(gen)
    [topics, msgs, times] = zip(*stuff)
    original_data.velocity = np.transpose([[msg.twist.linear.x for msg in msgs],
                                           [msg.twist.linear.y for msg in msgs],
                                           [msg.twist.linear.z for msg in msgs]])

    # checking length
    lenPos = len(trajectory_data.position)
    lenVel = len(trajectory_data.velocity)
    lenOri = len(original_data.velocity)

    if lenPos != lenVel:
        print("WARNING, trajectory position and velocity don't have the same length")
        lenPos = min(lenPos, lenVel)
        lenVel = lenPos
        trajectory_data.position = trajectory_data.position[range(lenPos)]
        trajectory_data.velocity = trajectory_data.velocity[range(lenVel)]

    if lenVel != lenOri:
        print("WARNING, trajectory position/velocity and original velocity don't have the same length")
        lenVel = min(lenVel, lenOri)
        lenOri = lenVel
        lenPos = lenVel
        trajectory_data.position = trajectory_data.position[range(lenPos)]
        trajectory_data.velocity = trajectory_data.velocity[range(lenVel)]
        original_data.velocity = original_data.velocity[range(lenOri)]

    return trajectory_data, original_data


def plotPoints(data, name, plot=1):
    # create new figure in 3D
    fig = plt.figure()
    ax = fig.gca(projection='3d')

    # plot points and add label
    ax.plot(data.position[:, 0], data.position[:, 1], data.position[:, 2], c='b', label=name)

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
    #the factor parameter represent how much you want MORE points that were sent by the robot

    #calculate splines data
    splineData3D = spline3D(data.position, data.velocity)

    #calculate new points
    size = len(data.position) * factor
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

    for i,(pos, dis_vel, vel) in enumerate(zip(trajectory_data.position, trajectory_data.velocity, original_data.velocity)):
    #calculate dot product
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
        stop.append(trajectory_data.position[-1])
        index_stop.append(i)

    if return_type == 'point':
        return np.transpose(start), np.transpose(stop)
    elif return_type == 'index':
        return index_start, index_stop

    else:
        rospy.logerr("error in function analyzeData, return_type is wrong")
        return 0,0


#reflexion here : maybe the algorithm would more precise with a filter : if 2 points (start then stop) are too closed, we remove then.
#or better : if 2 points are too closed and the dot product is not so low, keep them, but if they are closed with a high dot product remove them.
# maybe find a formula ?
#this can be done with another correction-factor, maybe a factor that tell us up to how many points the start and stop is validate ?
#after reflexion it's quite like a lowpass filter for big amplitude wave.

#another thing :  when we found a jump on the curve that is supposed to be a jump, the start and stops are in the middle of the jump, should correct it

#another thing, frequency analysis ??


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

    # first fft
    fourrier = np.fft.fft(dot_prod)

    fourrier1 = []
    fourrier2 = []
    fourrier3 = []
    fourrier4 = []

    #creation of a perfect low pass filter of 20%
    percentl = 30
    filtre = [1 for i in range(percentl * len(fourrier) / 100)]
    filtre.extend([0 for i in range(len(fourrier) - len(filtre))])
    fourrier1 = np.array(fourrier) * np.array(filtre)

    #creation of a highpass filter of 20%
    percenth = 30
    filtre = [0 for i in range((100-percenth) * len(fourrier) / 100)]
    filtre.extend([1 for i in range(len(fourrier) - len(filtre))])
    fourrier2 = np.array(fourrier) * np.array(filtre)

    #creation of a pass anti bande filter of 10% - 10%
    percentb1 = 3
    percentb2 = 30
    filtre = [1 for i in range(percentb1 * len(fourrier) / 100)]
    filtre.extend([0 for i in range((100-percentb1-percentb2) * len(fourrier) / 100)])
    filtre.extend([1 for i in range(len(fourrier) - len(filtre))])
    fourrier3 = np.array(fourrier) * np.array(filtre)

    #creation of a pass bande filter of 10% - 10%
    percentb3 = 10
    percentb4 = 90
    filtre = [0 for i in range(percentb3 * len(fourrier) / 100)]
    filtre.extend([1 for i in range((100-percentb3-percentb4) * len(fourrier) / 100)])
    filtre.extend([0 for i in range(len(fourrier) - len(filtre))])
    fourrier4 = np.array(fourrier) * np.array(filtre)

    #fourrier = np.imag(fourrier)
    fourrier = np.fft.ifft(fourrier)
    fourrier1 = np.fft.ifft(fourrier1)
    fourrier2 = np.fft.ifft(fourrier2)
    fourrier3 = np.fft.ifft(fourrier3)
    fourrier4 = np.fft.ifft(fourrier4)

    # create new figure in 2D
    fig2 = plt.figure()
    ax2 = fig2.gca()

    # plot points and add label
    ax2.plot(range(len(dot_prod)), dot_prod, c='b', label=name + " - normal")
    ax2.plot(range(len(fourrier1)), fourrier1, c='r', label=name + " - lowpassfilter " + str(percentl)+"%")
    ax2.plot(range(len(fourrier2)), fourrier2, c='g', label=name + " - highpassfilter " + str(percenth)+"%")
    ax2.plot(range(len(fourrier3)), fourrier3, c='k', label=name + " - antibandepassfilter " + str(percentb1) + "% low and" + str(percentb2) + "% high")
    ax2.plot(range(len(fourrier4)), fourrier3, c='m', label=name + " - bandepassfilter " + str(percentb3) + "% low and" + str(percentb4) + "% high")

    # label axis
    ax2.set_xlabel('x')
    ax2.set_ylabel('y')
    ax2.legend()

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

        #smooth data
        (trajectory_data.position, trajectory_data.velocity, original_data.velocity) = (smoothData(trajectory_data.position, alpha),
                                                                                        smoothData(trajectory_data.velocity, alpha),
                                                                                        smoothData(original_data.velocity, alpha))

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
