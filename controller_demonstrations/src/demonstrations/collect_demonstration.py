#! /usr/bin/env python

import argparse
import cPickle as pickle
import copy
import sys

import rospy
import rosbag
import tf_conversions

from nl_msgs.msg import CartStateStamped
from nl_msgs.msg import AnchoredDemonstration


class CollectDemonstration(object):
    """Collect and store kinesthetic demonstrations for a natural language.

    This listens for Kuka messages, and collects a trajectory. We then
    interpolate the points (keeping only --num <N> of them), transforms the
    trajectory points so it begins at the origin (with no orientation), and
    then saves a single 'AnchoredDemonstration' message as a bag file.

    While collecting demonstrations, use <Ctrl+C> to end the demonstrations,
    which automatically moves to the next phase.

    Options:
      --num: Number of states to save.
      --output: rosbag filename
      words: Mandatory argument: command being demonstrated

    """

    channel = 'KUKA/CartState'

    def __init__(self, words, num_desired_points, bag_filename):
        rospy.init_node('collect_demonstration', anonymous=True)
        rospy.Subscriber(CollectDemonstration.channel, CartStateStamped,
                         self.callback_state)

        self._num_demo_points = 0
        self._demonstration_anchor = None
        self._demonstration_vector = []

        self._words = words
        self._num_desired_points = num_desired_points
        self._bag_filename = bag_filename

        rospy.loginfo('Collecting demonstration for words: {}'.format(words))

    def do(self, plot=False):
        rospy.loginfo('Listening to messages on {} channel'.format(
            CollectDemonstration.channel))
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

        (processed_anchor, processed_data) = self.process_demonstration(
            demonstration_data, plot)
        msg = self.make_message(processed_anchor, processed_data)

        # Save message to a rosbag file.
        topic = 'demonstration'
        with rosbag.Bag(self._bag_filename, 'w') as bag:
            bag.write(topic, msg)
            bag.close()
            rospy.loginfo('Saved bag {}'.format(self._bag_filename))




    def process_demonstration(self, demonstration_data, plot=True):
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
        (anchor_new, corrections_new) = self.remove_anchor_pose(anchor, downsampled)

        if plot:
            import matplotlib.pyplot as plt
            from mpl_toolkits.mplot3d import Axes3D
            fig = plt.figure()
            ax = fig.add_subplot(111, projection='3d')
            original_x = [f.pose.position.x for f in downsampled]
            original_y = [f.pose.position.y for f in downsampled]
            original_z = [f.pose.position.z for f in downsampled]

            shifted_x = [f.pose.position.x for f in corrections_new]
            shifted_y = [f.pose.position.y for f in corrections_new]
            shifted_z = [f.pose.position.z for f in corrections_new]

            ax.scatter(original_x, original_y, original_z, c='r')
            ax.scatter(shifted_x, shifted_y, shifted_z, c='g')
            ax.scatter(anchor.pose.position.x, anchor.pose.position.y,
                       anchor.pose.position.z, c='k')
            ax.scatter(anchor_new.pose.position.x,
                       anchor_new.pose.position.y,
                       anchor_new.pose.position.z, c='k')

            ax.axis('equal')
            plt.show()
            pass

        # Note: if you need the header time:
        # times = [x.header.stamp.to_time() for x in demonstration_data['corrections']]


        return (anchor_new, corrections_new)


    @classmethod
    def remove_anchor_pose(cls, anchor, data):

        # Convert anchor pose into a PyKDL.Frame: simplifies
        anchor_frame = tf_conversions.fromMsg(anchor.pose)

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

    def make_message(self, anchor, data):
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
            assert isinstance(d, CartStateStamped)
            msg.demonstration.append(d)

        msg.num_words = len(self._words)
        msg.words = self._words

        return msg

    def dist_from_anchor(self, point):

        anchor_frame = tf_conversions.fromMsg(self._demonstration_anchor.pose)
        point_frame = tf_conversions.fromMsg(point.pose)

        delta = anchor_frame.Inverse() * point_frame

        return delta.p.Norm()


    def callback_state(self, data):

        if not self._demonstration_anchor:
            self._demonstration_anchor = data
            rospy.loginfo('Received demonstration anchor {} {} {}'.format(
                self._demonstration_anchor.pose.position.x,
                self._demonstration_anchor.pose.position.y,
                self._demonstration_anchor.pose.position.z))

        self._num_demo_points += 1
        self._demonstration_vector.append(data)


        rospy.loginfo('Got demonstration {} \t d={:.2f}'.format(
            self._num_demo_points, self.dist_from_anchor(data)))

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
    parser.add_argument('--plot', default=False, action='store_true',
                        help='Plot demonstration.')
    args = parser.parse_args(arguments)

    demonstrator = CollectDemonstration(args.words, args.num, args.output)
    demonstrator.do(args.plot)

    return

if __name__ == '__main__':
    arguments = sys.argv[1:]  # argv[0] is the program name.
    run(arguments)
