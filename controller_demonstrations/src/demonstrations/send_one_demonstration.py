#! /usr/bin/env python
import sys
import argparse
import time
import os

import rospy
import rosbag
from nl_msgs.msg import CartStateStamped
from nl_msgs.msg import AnchoredDemonstration

import file_util

class PublishCorrections(object):
    """
    ROS node that publishes 'anchored corrections' when it receives a NL command.

    This node listens for the nl command and generates an AnchoredDemonstration
    message when it receives any new command.

    """

    channel_corrections = 'nl_corrections'  # output
    channel_command = 'nl_command'  # input
    channel_kuka_state = 'KUKA/CartState'  # input

    def __init__(self, demonstration_dir):
        rospy.init_node('publish_corrections', anonymous=True)
        self.pub = rospy.Publisher(PublishCorrections.channel_corrections,
                                   AnchoredDemonstration, queue_size=10)
        rospy.Subscriber(PublishCorrections.channel_kuka_state,
                         CartStateStamped, self.kuka_callback)

        # Corrections are dict word->data
        self._corrections = self.load_all_demonstrations(demonstration_dir)

        # Kuka state: we store the current robot state every time it is
        # received, and when a command *begins* we copy the current state (at
        # that time) to 'anchor' the correction. This specifies where the
        # demonstration should begin from.
        self._robot_state = None
        self._robot_anchor = None

    def load_all_demonstrations(self, demonstration_dir):
        ret = {}

        # Find all files in the directory.
        files = file_util.get_files('', demonstration_dir)
        for f in files:
            filepath = file_util.get_fpath(demonstration_dir, f)
            anchored_demo = self.load_demonstration(filepath)

            words = anchored_demo.words
            for w in words:
                if w in ret:
                    rospy.logwarn('Word [{}] already in demonstration. '
                        ' Overwriting with data from: {}.'.format(w, f))
                ret[w] = anchored_demo.demonstration

        rospy.loginfo('Have {} total demonstrations: {}'.format(
            len(ret.keys()), ret.keys()))
        return ret

    def load_demonstration(self, filepath):
        # Load a single demonstration. Returns a single AnchoredDemonstration message.
        rospy.loginfo('Loading demonstration from {}.'.format(filepath))

        msg = None
        with rosbag.Bag(filepath) as bag:
            assert bag.get_message_count() == 1, 'Demonstration should have exactly one message'
            for (_, msg, _) in bag.read_messages():  # Get the message contents.
                pass

        assert msg is not None

        rospy.loginfo('  Loaded {} demonstration points, words: {}'.format(
            msg.num_points, msg.words))

        return msg

    def send_correction(self, nl_command, robot_anchor):
        if not robot_anchor:
            rospy.logerr('Cannot send correction with no anchor')
            return

        command_split = nl_command.split(' ')
        corrections = []
        for word in command_split:
            if word in self._corrections:
                rospy.loginfo('demo: {}'.format(word))

                # Create a message with the data & using the anchor.
                msg = self.create_correction(word, robot_anchor)
                corrections.append(msg)
            else:
                rospy.loginfo('skip: {}'.format(word))

        for c in corrections:
            self.pub.publish(c)
            rospy.loginfo('Sent demonstration at t={} -- {}  anchor_t={}'.format(
                rospy.get_time(), c.words, c.anchor.header.stamp.to_time()))
        pass

    def create_correction(self, word, robot_anchor):
        """ Create an AnchoredCorrection message using the given word & anchor.

        Get the correction points from the stored map, using a *single* word.
        """

        if word not in self._corrections:
            rospy.logerr('Word [{}] not in known corrections: {}'.format(
                word, self._corrections.keys()))
            return None

        assert isinstance(word, basestring)  # Get a string, not a list of strings.

        correction = AnchoredDemonstration()
        correction.header.stamp = rospy.Time.now()
        correction.words.append(word)
        correction.num_words = 1

        correction.demonstration = self._corrections[word]
        correction.num_points = len(self._corrections[word])

        correction.anchor = robot_anchor

        return correction

    def kuka_callback(self, data):
        self._robot_state = data
        assert isinstance(data, CartStateStamped)

    def process_command(self, nl_command, use_current_state_as_anchor=False):

        # Either use the anchor (a copy of the state when the command first
        # started) or the current state directly.
        anchor = self._robot_anchor
        if use_current_state_as_anchor:
            anchor = self._robot_state

        self.send_correction(nl_command, anchor)


def run_send_one_command(arguments):
    parser = argparse.ArgumentParser(
        description=('Load a directory of demonstration data (stored as bag '
                     'files).'))
    parser.add_argument('--demo_dir', metavar='directory', required=True)
    parser.add_argument('command', nargs='+')
    args = parser.parse_args(arguments)


    demo_publisher = PublishCorrections(args.demo_dir)

    nl_command = ' '.join(args.command)

    # Sleep just long enough to get the Kuka State.
    time.sleep(0.1)

    try:
        demo_publisher.process_command(nl_command, use_current_state_as_anchor=True)
    except rospy.ROSInterruptException as e:
        print e
        pass

    #rospy.spin()


if __name__ == '__main__':
    arguments = sys.argv[1:]  # argv[0] is the program name.
    run_send_one_command(arguments)
