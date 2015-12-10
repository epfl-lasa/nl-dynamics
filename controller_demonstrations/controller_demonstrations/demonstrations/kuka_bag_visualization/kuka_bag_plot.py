# Starter code for plotting a Kuka trajectory from a rosbag file.

import rosbag
import numpy as np
from spline import spline3D, getPointsSpline3D

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import animation


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


def plotPoints(data, name, a, b):
    # create new figure in 3D
    fig = plt.figure()
    ax = fig.gca(projection='3d')

    # plot points and add label
    ax.scatter([d.pose.position.x for d in data[:a:]],
               [d.pose.position.y for d in data[:a:]],
               [d.pose.position.z for d in data[:a:]], c='b', label=name, s=30)

    ax.scatter([d.pose.position.x for d in data[b::]],
               [d.pose.position.y for d in data[b::]],
               [d.pose.position.z for d in data[b::]], c='b', label=name, s=30)

    #plot a   at the start and at the end
    ax.scatter(data[0].pose.position.x,  data[0].pose.position.y,  data[0].pose.position.z,  s=100, c='g', marker='o')
    ax.scatter(data[-1].pose.position.x,  data[-1].pose.position.y,  data[-1].pose.position.z,  s=200, c='g', marker='x')

    # label axis
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')
    ax.legend()

    # show result
    plt.ion()
    plt.show()

    return ax


def plotSpline3D_correction(data, ax, factor, start_stop):
    (start, stop) = start_stop
    for (s1, s2) in zip(start, stop):
        plotSpline3D(data[s1:s2+1:], ax, factor)


def plotSpline3D(data, ax, factor):
    #the factor parameter represent how much you want MORE points that are being in data

    #calculate splines data
    splineData3D = spline3D(data)

    #calculate new points
    size = len(data) * factor
    t = np.linspace(0, 1, size)

    new_pos = getPointsSpline3D(splineData3D, t)

    #plot result
    ax.plot(new_pos[0], new_pos[1], new_pos[2], c='r', label='spline3D', linewidth=3)

    # show result
    plt.ion()
    plt.show()


def drawStartStop(start, stop, ax):
    if (len(start) > 0):
        # plot star and triangle shape on desired points
        ax.scatter(start[0, :], start[1, :], start[2, :], s=200, c='r', marker='*')
        ax.scatter(stop[0, :],  stop[1, :],  stop[2, :],  s=200, c='b', marker='^')

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


def analyzeData_online(actual_point, Desired_Velocity, is_in, precision_factor=0.4):
    # function that return a string based on the same algorithm as analyzeData(..)
    # - 'start' if this is a start point
    # - 'stop'  if this is a stop point
    # - 'none' if this a nothing

    # calculate dot product
    dis_vel = [Desired_Velocity.twist.linear.x, Desired_Velocity.twist.linear.y, Desired_Velocity.twist.linear.z]
    vel = [actual_point.twist.linear.x, actual_point.twist.linear.y, actual_point.twist.linear.z]
    dot = np.dot(dis_vel/np.linalg.norm(dis_vel), vel/np.linalg.norm(vel))

    #analyze data
    if not(is_in) and dot < (1-precision_factor):
        return 'start'

    elif is_in and dot >= (1-precision_factor):
        return 'stop'

    else:
        return 'none'


def analyzeData_force(trajectory_data, precision_factor=0.4, return_type='point'):
    # trajectory_data represent the actual data of the robot, the position and the velocity of the robot for all the points
    # original_data only represent a vector from the robot to the attractor point (following the OriginalDynamics) for all the points
    # Precision factor how precise the algorithm need to be to detect some jump in the trajectory : 0 is absolute precision
    # return_type is the type of return, can be 'point' for an array of start and stop coord, can also be 'index', for an array of index

    #create some usefull variables
    threshold = 10
    is_in = 0
    start = []
    stop = []
    index_start = []
    index_stop = []

    for i, tra in enumerate(trajectory_data):
    #calculate norm
        norm_force = np.sqrt(tra.wrench.force.x**2 + tra.wrench.force.y**2 + tra.wrench.force.z**2)
        pos = [tra.pose.position.x, tra.pose.position.y, tra.pose.position.z]

    #analyse data and eventualy store new start-stop point
        if not(is_in) and norm_force > threshold - precision_factor:
            start.append(pos)
            index_start.append(i)
            is_in = 1

        elif is_in and norm_force < threshold + precision_factor:
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


def forcePlot(data):

    force_try = [np.sqrt(f.wrench.force.x**2 + f.wrench.force.y**2 + f.wrench.force.z**2) for f in data]

    fig = plt.figure()
    ax = fig.gca()

    # plot points and add label
    ax.plot(range(len(force_try)), force_try, c='b', label="force_try", linewidth=5)

    # label axis
    ax.set_xlabel('x')
    ax.set_ylabel('norm of the force applied on the robot')
    ax.legend()

    # show result
    plt.ion()
    plt.show()


def dotProdPlot(vel1, vel2, name):

    dot_prod = []

    for (tra, ori) in zip(vel1, vel2):
        dis_vel = [tra.twist.linear.x, tra.twist.linear.y, tra.twist.linear.z]
        vel = [ori.twist.linear.x, ori.twist.linear.y, ori.twist.linear.z]

        temp = np.dot(dis_vel/np.linalg.norm(dis_vel), vel/np.linalg.norm(vel)).item()
        dot_prod.append(temp)

    #dot_prod = [np.dot(v1.twist.linear/np.linalg.norm(v1), v2/np.linalg.norm(v2)) for (v1, v2) in zip(vel1, vel2)]

    # create new figure in 2D
    fig = plt.figure()
    ax = fig.gca()

    # plot points and add label
    ax.plot(range(len(dot_prod)), dot_prod, c='b', linewidth=5)

    # label axis
    ax.set_xlabel('x')
    ax.set_ylabel('dot product between the 2 vectors')
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

    for name in tab2:
        # load data
        (trajectory_data, original_data) = load(name)

        MYVAL = 1700
        trajectory_data = trajectory_data[:MYVAL]
        original_data = original_data[:MYVAL]

        (trajectory_data, original_data) = (smoothData(trajectory_data, alpha),
                                            smoothData(original_data, alpha))

        # analyze data
        (start, stop) = analyzeData(trajectory_data, original_data, precision_factor)
        #(start, stop) = analyzeData_force(trajectory_data, precision_factor)

        (istart, istop) = analyzeData(trajectory_data, original_data, precision_factor, 'index')

        #drawpoints (to be deleated)
        #ax = plotPoints(trajectory_data, name + ", with alpha=" + str(alpha))
        ax = plotPoints(trajectory_data, 'trajectory', istart[0], istop[0])

        # drawing start-stops points (red stars for the starts and blue triangle for the stops)
        drawStartStop(start, stop, ax)

        #drawing splineparts
        plotSpline3D_correction(trajectory_data, ax, factor, analyzeData(trajectory_data, original_data, precision_factor, 'index'))

        #plot dotprod
        #dotProdPlot(trajectory_data, original_data, name)
        #forcePlot(trajectory_data)

    # pause
    raw_input('press enter')
    plt.close("all")


if __name__ == '__main__':
    run()

__author__ = 'felixd'
