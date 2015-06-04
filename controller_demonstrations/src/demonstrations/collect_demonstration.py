#! /usr/bin/env python

import argparse
import cPickle as pickle
import copy
import numpy as np
import sys

import rospy

from nl_msgs.msg import CartStateStamped
from nl_msgs.msg import AnchoredDemonstration


class CollectDemonstration(object):

    channel = 'KUKA/CartState'

    def __init__(self, num_desired):
        rospy.init_node('collect_demonstration', anonymous=True)
        rospy.Subscriber(CollectDemonstration.channel, CartStateStamped, self.callback_state)

        self._num_demo_points = 0
        self._demonstration_anchor = None
        self._demonstration_vector = []

        self._num_desired_points = num_desired

    def do(self):
        rospy.loginfo('Listening to messages on {} channel'.format(CollectDemonstration.channel))
        rospy.spin()

        rospy.loginfo('Finished collecting demonstration, have {} points'.format(
            self._num_demo_points))

        demonstration_data = {'anchor': self._demonstration_anchor,
                   'corrections': self._demonstration_vector}

        try:
            filepath = 'demo_data-all.pck'
            with open(filepath, 'w') as f:
                pickle.dump(demonstration_data, f)
                print 'Object saved to [%s].' % filepath
        except IOError as e:
            print e
            pass

        (anchor, processed_data) = self.process_demonstration(demonstration_data)
        msg = self.make_message(processed_data)


    def process_demonstration(self, demonstration_data):
        """Process demonstration data

        Downsample the data to only keep a smaller number of them, and subtract
        the anchor pose from all points such that the first correction is
        centered at 'zero' pose. Note that only poses are subtracted, no other
        information is changed.

        Returns (anchor, corrections)

        """
        rospy.loginfo('Processing {} demonstrations. Downsampling to {}.'.
                      format(len(demonstration_data['corrections']),
                             self._num_desired_points))

        # First, downsample data. For now, just take every Nth piece of data,
        # and truncate anything extra.
        every_nth = len(demonstration_data['corrections']) / self._num_desired_points
        downsampled = demonstration_data['corrections'][::every_nth]
        downsampled = downsampled[:self._num_desired_points]

        # Subtract the pose of the anchor from all downsampled points.
        anchor = demonstration_data['anchor']
        rospy.loginfo('Removing anchor {} {} {} from {} data points'.
                      format(anchor.pose.position.x,
                             anchor.pose.position.y,
                             anchor.pose.position.z,
                             len(downsampled)))
        corrections = self.remove_anchor_pose(anchor, downsampled)

        return (anchor, corrections)

        #times = [x.header.stamp.to_time() for x in demonstration_data['corrections']]
        #return times

    def remove_anchor_pose(self, anchor, data):

        def subtract_pose(anchor, point, verbose=True):
            p = copy.deepcopy(point)
            p.pose.position.x -= anchor.pose.position.x
            p.pose.position.y -= anchor.pose.position.y
            p.pose.position.z -= anchor.pose.position.z

            # TODO: this is a terrible way to do this. Use ROS tf module instead.

            if verbose:
                print('{} {} {} -> {} {} {}'.format(
                    point.pose.position.x,
                    point.pose.position.y,
                    point.pose.position.z,
                    p.pose.position.x,
                    p.pose.position.y,
                    p.pose.position.z))

            return p

        parsed = [subtract_pose(anchor, x) for x in data]
        return (anchor, parsed)


    def make_message(self, anchor, data):
        pass


    def callback_state(self, data):

        if not self._demonstration_anchor:
            self._demonstration_anchor = data
            rospy.loginfo('Received demonstration anchor {} {} {}'.format(
                self._demonstration_anchor.pose.position.x,
                self._demonstration_anchor.pose.position.y,
                self._demonstration_anchor.pose.position.z))

        self._num_demo_points += 1
        self._demonstration_vector.append(data)

        rospy.loginfo('Got a demonstration {}'.format(self._num_demo_points))

def run(arguments):
    parser = argparse.ArgumentParser(
        description=('Collect demonstrations from the robot. Specify the words '
                     'on the command line. Optionally change the filename as well'))
    parser.add_argument('--output', default='out.bag', metavar='output_filename',
                        help='Filename of output bag file.')
    parser.add_argument('--num', default=50, type=int,
                        help='Number of demonstration points to store.')
    parser.add_argument('words', default='default', metavar='words',
                        nargs='+',
                        help='Demonstration word(s)')

    args = parser.parse_args(arguments)

    print args
    print args.output
    print args.words
    print args.num


    demonstrator = CollectDemonstration(args.num)
    #demonstrator.do()


    filepath = 'demo_data-all.pck'
    with open(filepath) as f:
        data = pickle.load(f)

        demonstrator.process_demonstration(data)


if __name__ == '__main__':
    arguments = sys.argv[1:]  # argv[0] is the program name.
    run(arguments)
