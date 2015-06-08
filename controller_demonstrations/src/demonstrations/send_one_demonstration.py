#! /usr/bin/env python

import sys
import argparse
import numpy as np
import time
import os

import rospy
import rosbag
from nl_msgs.msg import CartStateStamped
from nl_msgs.msg import AnchoredDemonstration

import file_util

class PublishCorrections(object):
    channel_corrections = 'nl_corrections'
    channel_kuka_state = 'KUKA/CartState'

    def __init__(self, demonstration_dir):
        rospy.init_node('publish_corrections', anonymous=True)
        self.pub = rospy.Publisher(PublishCorrections.channel_corrections,
                                   AnchoredDemonstration, queue_size=10)
        rospy.Subscriber(PublishCorrections.channel_kuka_state,
                         CartStateStamped, self.kuka_callback)

        # Corrections are dict word->data
        self._corrections = {}

        # Load demonstrations
        self.load_all_demonstrations(demonstration_dir)


    def load_all_demonstrations(self, demonstration_dir):
        # Find all files in the directory.
        ret = {}

        #TODO: iterate over files in
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

    def send_correction(self, command):
        pass

    def kuka_callback(self, data):
        print 'Received kuka state'
        pass

    def do(self):
        data = [(0, 1, 2, 3), (1, 2, 3, 4)]
        v = self.make_demo_vec(data)
        self.pub.publish(v)

        log_str = "Sent demonstration at t={} -- {}".format(rospy.get_time(), v)
        rospy.loginfo(log_str)

        time.sleep(0.5)


def run(arguments):
    parser = argparse.ArgumentParser(
        description=('Load a directory of demonstration data (stored as bag '
                     'files).'))
    parser.add_argument('--demo_dir', metavar='directory', required=True)
    args = parser.parse_args(arguments)


    demo_publisher = PublishCorrections(args.demo_dir)

    return

    try:
        demo_publisher.run()
    except rospy.ROSInterruptException as e:
        print e
        pass


if __name__ == '__main__':
    arguments = sys.argv[1:]  # argv[0] is the program name.
    run(arguments)
