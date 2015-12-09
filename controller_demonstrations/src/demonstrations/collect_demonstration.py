#! /usr/bin/env python

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

import argparse
import cPickle as pickle
import copy
import sys

import rospy
import rosbag
import tf_conversions
import PyKDL
import numpy as np

from kuka_bag_visualization.kuka_bag_plot import analyzeData, analyzeData_force, forcePlot
from kuka_bag_visualization.spline import spline3D, getPointsSpline3D, Spline, Spline3D

from nl_msgs.msg import CartStateStamped
from nl_msgs.msg import AnchoredDemonstration
from nl_msgs.msg import AttractorDemonstration
from nl_msgs.msg import Correction
from nl_msgs.msg import SplineClass
from nl_msgs.srv import Demonstration

from geometry_msgs.msg import Point


class CollectDemonstration(object):
    """Collect and store kinesthetic demonstrations for a natural language.

    This listens for Kuka messages, and collects a trajectory. We then
    interpolate the points (keeping only --num <N> of them), transforms the
    trajectory points so it begins at the origin (with no orientation), and then
    saves a single 'AnchoredDemonstration' message as a bag file.

    Note we use rosbag as a storage because it's convenient, we won't actually
    play back the rosbag file.

    While collecting demonstrations, use <Enter> to end the demonstrations,
    which automatically moves to the next phase.

    Options:
      --num: Number of states to save.
      --output: rosbag filename
      words: Mandatory argument: command being demonstrated

    """

    channel = 'KUKA/CartState'
    channel2 = 'KUKA/DesiredState'

    def __init__(self, words, num_desired_points, bag_filename, discard_static_points, plot=False, format='points'):
        rospy.Subscriber(CollectDemonstration.channel, CartStateStamped,
                         self.callback_state)
        rospy.Subscriber(CollectDemonstration.channel2, CartStateStamped,
                         self.callback_desired)

        self._num_demo_points = 0
        self._demonstration_anchor = None
        self._demonstration_vector = []

        self._num_velocity_points = 0
        self._velocity_vector = []

        self._words = words
        self._num_desired_points = num_desired_points
        self._bag_filename = bag_filename

        self._discard_static_points = discard_static_points
        self._plot = plot
        self._format = format

        self.MOTION_DISTANCE_THRESHOLD = 1e-3

        self._collecting_data = True

        if len(words) > 0:
            rospy.loginfo('Collecting demonstration for words: {}'.format(words))

    def do(self):
        rospy.loginfo('Listening to messages on {} {} channels'.format(
            CollectDemonstration.channel, CollectDemonstration.channel2))

        if self._format != 'spline' and self._format != 'points':
            rospy.logwarn('wrong format, taking "points" one')
            self._format = 'points'

        # Spin but do not catch keyboard interrupt exception -- just move onto
        # processing & saving the demonstration.
        #rospy.spin()

        try:
            while self._collecting_data:
                rospy.sleep(0.01)
        except KeyboardInterrupt:
            print 'User stoped the waiting loop, stop getting msg, starting analyze'
            self._collecting_data = False

        # here the analysis begin, data are stored

        #size checking
        if not self._num_demo_points == self._num_velocity_points:
            rospy.logerr('size are not the same, taking the smallest one')
            self._num_demo_points = min(self._num_demo_points, self._num_velocity_points)
            self._num_velocity_points = self._num_demo_points
            self._demonstration_vector = self._demonstration_vector[:self._num_demo_points]
            self._velocity_vector = self._velocity_vector[:self._num_velocity_points]

        rospy.loginfo('Finished collecting demonstration, have {} points'.format(
            self._num_demo_points))

        # checking if the program catch some data
        if not self._demonstration_anchor or len(self._demonstration_vector) == 0:
            rospy.logerr('No training data collected, doing nothing.')
            return

        demonstration_data = {'anchor': self._demonstration_anchor,
                              'corrections': self._demonstration_vector,
                              'desired': self._velocity_vector}

        (processed_anchor, processed_data) = self.process_demonstration(demonstration_data)

        if self._format == 'points':
            msg = self.make_message_point(processed_anchor, processed_data)
        else:
            msg = self.make_message_spline(processed_anchor, processed_data)

        # Save message to a rosbag file.
        topic = 'demonstration'
        with rosbag.Bag(self._bag_filename, 'w') as bag:
            bag.write(topic, msg)
            bag.close()
            rospy.loginfo('Saved bag {} with format {}'.format(self._bag_filename, self._format))

        self._num_demo_points = 0
        self._demonstration_anchor = None
        self._demonstration_vector = []

        self._num_velocity_points = 0
        self._velocity_vector = []

        return msg

    def process_demonstration(self, demonstration_data):
        """Process demonstration data.

        Downsample the data to only keep a smaller number of them, and subtract
        the anchor pose from all points such that the first correction is
        centered at 'zero' pose. Note that only poses are subtracted, no other
        information is changed.

        Remove any non-moving poses if desired (before downsampling).

        Input: dict{'anchor'->CartStateStamped, 'corrections'->list[CartStateStamped]}

        Returns (anchor, corrections)

        """
        num_corrections = len(demonstration_data['corrections'])

        # Remove any points that are closer than 'MOTION_DISTANCE_THRESHOLD'
        # from the anchor point.
        if self._discard_static_points:
            # Start with the anchor as a datapoint to keep so that the first
            # point is (0, 0, 0).
            #data_keep = [demonstration_data['anchor']]
            data_keep = [demonstration_data['corrections'][0]]
            vel_keep = [demonstration_data['desired'][0]]
            for (data, ori_vel) in zip(demonstration_data['corrections'], demonstration_data['desired']):
                dist = self.dist_from_anchor(data)
                if dist > self.MOTION_DISTANCE_THRESHOLD:
                    data_keep.append(data)
                    vel_keep.append(ori_vel)
            rospy.loginfo('Discarded {} non-moving data points from dataset.'.
                          format(num_corrections - len(data_keep)))
            demonstration_data['corrections'] = data_keep
            demonstration_data['desired'] = vel_keep

        num_corrections = len(demonstration_data['corrections'])  # Update number.
        rospy.loginfo('Processing {} demonstrations. Downsampling to {}.'.
                      format(num_corrections,
                             min(self._num_desired_points, num_corrections)))

        downsampled = demonstration_data['corrections'][:]
        downsampled_vel = demonstration_data['desired'][:]

        # search for start and stop
        #(start, stop) = analyzeData(downsampled, downsampled_vel, 0.4, 'index')
        (start, stop) = analyzeData_force(downsampled, 0.4, 'index')

        # keep only data between start-stop points
        if len(start) == 0:
            rospy.loginfo('no correction found')
            newData = []
            listData = [[]]
            anchor = downsampled[0]
        else:
            #newData=[ downsampled[sta:sto] for (sta, sto) in zip(start, stop) ]   doesn't work, need a real for-loop :(
            newData = []        # list of correction points
                                #   e.g : [1,2,3,8,9,10,67,68,69,70,71]
            listData = []       # list of list of point for the different conrrection
                                #   e.g : [ [1,2,3], [8,9,10], [67,68,69,70,71] ]

            for (sta, sto) in zip(start, stop):
                #downsampling data
                temp = downsampling(downsampled[sta:sto+1], self._num_desired_points)

                #adding data in 2 differents way
                newData.extend(temp)
                listData.append(temp)  # here need to fill lisData with point or spline3D

            anchor = newData[0]        # anchor is no longer the first given point but the first start point

        # Subtract the pose of the anchor from all downsampled points.
        rospy.loginfo('Removing anchor ({:.2f} {:.2f} {:.2f}) from {} data points'.
                      format(anchor.pose.position.x,
                             anchor.pose.position.y,
                             anchor.pose.position.z,
                             len(downsampled)))

        # return of the function

        if self._format == 'points':
            rospy.loginfo('points')
            (anchor_new, corrections_new) = self.remove_anchor_pose(anchor, newData)
        else:
            corrections_new = []
            anchor_new = anchor
            for dat in listData:
                (to_the_bin, dat) = self.remove_anchor_pose(anchor, dat)
                corrections_new.append(spline3D(dat))

        if self._plot:
            self._plot = False   # a segfault appear when ploted twice, so we turn the plot off after the first plot.
            fig = plt.figure()
            ax = fig.gca(projection='3d')
            # plotting new position

            (anchor_new, downsampled) = self.remove_anchor_pose(anchor, downsampled)
            # downsampling data for a better visibility
            shifted_x = downsampling([f.pose.position.x for f in downsampled], self._num_desired_points)
            shifted_y = downsampling([f.pose.position.y for f in downsampled], self._num_desired_points)
            shifted_z = downsampling([f.pose.position.z for f in downsampled], self._num_desired_points)

            ax.scatter(shifted_x, shifted_y, shifted_z, c='b', zorder=2)

            # plotting new compute spline position
            if len(newData):
                t = np.linspace(0, 1, self._num_desired_points)
                for dat in listData:
                    (temp, dat) = self.remove_anchor_pose(anchor, dat)
                    [xx, yy, zz] = getPointsSpline3D(spline3D(dat), t)
                    ax.plot(xx, yy, zz, c='g', lw=3, zorder=4)

            # display a star at the beggining
            ax.scatter(shifted_x[0], shifted_y[0], shifted_z[0], s=150, c='r', marker='*', zorder=3)

            # plotting anchor new position in a black point
            ax.scatter(anchor_new.pose.position.x,
                       anchor_new.pose.position.y,
                       anchor_new.pose.position.z, c='k', zorder=2)
            ax.scatter(0, 0, 0, c='r', zorder=2)

            ax.set_xlabel('X axis')
            ax.set_ylabel('Y axis')
            ax.set_zlabel('Z axis')
            ax.axis('equal')

            plt.ion()
            plt.show()

            raw_input('press enter to continue')
            plt.close(fig)

        return (anchor_new, corrections_new)

    @classmethod
    def remove_anchor_pose(cls, anchor, data):

        # Convert anchor pose into a PyKDL.Frame: simplifies
        anchor_frame = tf_conversions.fromMsg(anchor.pose)
        anchor_frame.M = PyKDL.Rotation()  # NO ROTATION

        def subtract_pose(point, verbose=False):
            p = copy.deepcopy(point)

            # Find the difference in poses. NOTE we do not change anything other
            # than the pose (twist & wrench stay the same).
            point_frame = tf_conversions.fromMsg(point.pose)
            delta = anchor_frame.Inverse() * point_frame
            p.pose = tf_conversions.toMsg(delta)

            if verbose:
                print('{} {} {} -> {} {} {}'.format(
                    point.pose.position.x, point.pose.position.y,
                    point.pose.position.z, p.pose.position.x,
                    p.pose.position.y, p.pose.position.z))
            return p

        parsed = [subtract_pose(x) for x in data]

        # Create an identity translation/rotation for the new anchor pose.
        anchor_new = copy.deepcopy(anchor)
        identity = tf_conversions.Frame()
        anchor_new.pose = tf_conversions.toMsg(identity)

        return (anchor_new, parsed)

    def make_message_point(self, anchor, data):
        """ Create a AnchoredDemonstration message.

        Message contains three parts:
         - anchor
         - demonstration points
         - words
        """

        msg = AnchoredDemonstration()

        assert isinstance(anchor, CartStateStamped)
        msg.anchor = anchor

        msg.num_points = len(data)
        for d in data:
            assert isinstance(d, CartStateStamped), 'Incorrect type: {}'.format(d)
            msg.demonstration.append(d)

        msg.num_words = len(self._words)
        msg.words = self._words

        return msg

    def make_message_spline(self, anchor, data):
        """ Create a AnchoredDemonstration message.
            here the parameter 'data' must contain an array of spline3D

        Message contains three parts:
         - anchor
         - demonstration spline
         - words
        """

        msg = AttractorDemonstration()

        assert isinstance(anchor, CartStateStamped)
        msg.anchor = anchor

        #fill Correction[] trajectory
        for dat in data:
            temp = Correction()

            # filling x_spline
            spl = dat.get_spline('x')

            for (coef, point) in zip(spl.coef, spl.points):
                temp2 = SplineClass()
                temp2.coefficients = coef
                temp2.points.append(Point(point[0], point[1], 0))
                temp.x_splines.append(temp2)

            # filling y_spline
            spl = dat.get_spline('y')

            for (coef, point) in zip(spl.coef, spl.points):
                emp2 = SplineClass()
                temp2.coefficients = coef
                temp.y_splines.append(temp2)

            # filling z_spline
            spl = dat.get_spline('z')

            for (coef, point) in zip(spl.coef, spl.points):
                temp2 = SplineClass()
                temp2.coefficients = coef
                temp.z_splines.append(temp2)

            temp.number_points = spl.nbPoints

            # store the correction
            msg.trajectory.append(temp)

        msg.num_words = len(self._words)
        msg.words = self._words

        return msg

    def dist_from_anchor(self, point):

        anchor_frame = tf_conversions.fromMsg(self._demonstration_anchor.pose)
        point_frame = tf_conversions.fromMsg(point.pose)

        delta = anchor_frame.Inverse() * point_frame

        return delta.p.Norm()

    def callback_state(self, data):

        if not self._collecting_data:
            return

        # set the anchor position, the starting point
        if not self._demonstration_anchor:
            self._demonstration_anchor = data
            rospy.loginfo('Received demonstration anchor {} {} {}'.format(
                self._demonstration_anchor.pose.position.x,
                self._demonstration_anchor.pose.position.y,
                self._demonstration_anchor.pose.position.z))

        self._num_demo_points += 1
        self._demonstration_vector.append(data)

        if self._num_demo_points % 50 == 0:
            rospy.loginfo('Got demonstration {} \t d={:.2f}'.format(
                self._num_demo_points, self.dist_from_anchor(data)))

    def callback_desired(self, data):

        if not self._collecting_data:
            return

        self._num_velocity_points += 1
        self._velocity_vector.append(data)

        if self._num_velocity_points % 50 == 0:
            rospy.loginfo('Got desired velocity {}'.format(
                self._num_velocity_points))

        # automatic stop when collecting data
        if np.sqrt(data.twist.linear.x**2 + data.twist.linear.y**2 + data.twist.linear.z**2) < 0.18:
            self._collecting_data = False

    def handle_service_callback(self, req):
        self._collecting_data = True
        msg = self.do()
        self._collecting_data = False

        return True


def downsampling(list_, nb_element_to_keep):
    # downsample data by keeping only nb_element_to_keep linearly in the tab
    if nb_element_to_keep > len(list_):
        return list_

    t = np.linspace(0, len(list_)-1, nb_element_to_keep).astype(int)
    return np.array(list_)[t]


def run_service(arguments):

    parser = argparse.ArgumentParser(
        description=('Collect demonstrations from the robot. Optionally change the filename as well'))
    parser.add_argument('--output', default='out.bag', metavar='output_filename',
                        help='Filename of output bag file (default=out.bag).')
    parser.add_argument('--num', default=50, type=int,
                        help='Number of demonstration points to store for each corrections (default=50).')
    parser.add_argument('--discard_static', action='store_true',
                        dest='discard_static_points',
                        default=True,
                        help='Discard any points without motion from the start (default=True).')
    parser.add_argument('--no-discard_static',
                        dest='discard_static_points', action='store_false')
    parser.add_argument('--word', default='', metavar='WORD',
                        help='Format for result, can be "spline" or "points", default="points"')
    parser.add_argument('--plot', default=False, action='store_true',
                        help='Plot demonstration (default=False).')
    parser.add_argument('--format', default='points', metavar='FORMAT',
                        help='Format for result, can be "spline" or "points", default="points"')
    args = parser.parse_args(arguments)

    # checking if a word was enter
    if len(args.word) > 0:
        rospy.init_node('waiting_for_request_to_start_server')
        demonstrator = CollectDemonstration(args.word, args.num, args.output, args.discard_static_points, args.plot, args.format)
        demonstrator.do()

    else:
        rospy.init_node('waiting_for_request_to_start_server')

        if args.plot:
            rospy.logwarn('There is an issues with plot, the result will be plot only after the first demonstration')

        mydemonstration = CollectDemonstration(args.word, args.num, args.output, args.discard_static_points, args.plot, args.format)

        # initial the node and add service usability
        s = rospy.Service('Correction_Isolation', Demonstration, mydemonstration.handle_service_callback)
        print 'ready to receive request'

        # wait for request
        rospy.spin()

if __name__ == '__main__':
    arguments = sys.argv[1:]  # argv[0] is the program name.
    run_service(arguments)
