#! /usr/bin/env python

import argparse
import sys

import numpy as np

import rospy

from nl_msgs.msg import CartStateStamped


class CollectDemonstration(object):

    channel = 'KUKA_CartState'

    def __init__(self):
        rospy.init_node('collect_demonstration', anonymous=True)
        rospy.Subscriber(CollectDemonstration.channel, CartStateStamped, self.callback_state)

        self._num_demo_points = 0
        self._demonstration_offset = None
        self._demonstration_vector = []


    def do(self):
        print('Listening to messages on {} channel'.format(CollectDemonstration.channel))
        rospy.spin()

    def callback_state(self, data):
        self._num_demo_points += 1
        rospy.loginfo('Got a message {}'.format(self._num_demo_points))

def run(arguments):
    parser = argparse.ArgumentParser(
        description=('Collect demonstrations from the robot. Specify the words '
                     'on the command line. Optionally change the filename as well'))
    parser.add_argument('--output', default='out.bag', metavar='output_filename',
                        help='Filename of output bag file.')
    parser.add_argument('words', default='default', metavar='words',
                        nargs='+',
                        help='Demonstration word(s)')

    args = parser.parse_args(arguments)

    print args
    print args.output
    print args.words


    demonstrator = CollectDemonstration()
    #demonstrator.do()

if __name__ == '__main__':
    arguments = sys.argv[1:]  # argv[0] is the program name.
    run(arguments)
