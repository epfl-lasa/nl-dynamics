#! /usr/bin/env python

import numpy as np

import rospy

from nl_msgs.msg import CartStateStamped


class CollectDemonstration(object):

    channel = 'KUKA_CartState'

    def __init__(self):
        rospy.init_node('collect_demonstration', anonymous=True)
        rospy.Subscriber(CollectDemonstration.channel, CartStateStamped, self.callback_state)

        self._num_demo_points = 0

    def do(self):
        print('Listening to messages on {} channel'.format(CollectDemonstration.channel))
        rospy.spin()

    def callback_state(self, data):
        self._num_demo_points += 1
        rospy.loginfo('Got a message {}'.format(self._num_demo_points))

def run():
    demonstrator = CollectDemonstration()
    demonstrator.do()

if __name__ == '__main__':
    run()
