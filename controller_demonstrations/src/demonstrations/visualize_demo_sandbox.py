#! /usr/bin/env python

import sys
import argparse
import time

import rospy
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker

from correction_publisher import PublishCorrections
from nl_msgs.msg import AnchoredDemonstration

channel_viz = 'visualization_channel'


def run(arguments):
    parser = argparse.ArgumentParser(
        description=('Load a directory of demonstration data (stored as bag '
                     'files).'))
    parser.add_argument('--demo_dir', metavar='directory', required=True)
    args = parser.parse_args(arguments)

    do(args.demo_dir)

    rospy.loginfo('Spin...')
    rospy.spin()

    pass

def create_correction_viz_msg(anchored_correction):

    marker = Marker()

    # TODO(Felix) Fill in marker base stuff

    for cart_state in anchored_correction:
        point = cart_state_to_point(cart_state)
        marker.points.append(point)

    return marker

def cart_state_to_point(point):
    # Go from CartStateStamped -> geometry_msgs.Point
    p = Point(point.pose.position.x, point.pose.position.y, point.pose.position.z)
    return p


def do(demo_dir):

    # Use the correction publisher to load the demonstrations for a directory.


    rospy.init_node('visualization_node', anonymous=False)
    publisher = rospy.Publisher(channel_viz, Marker, queue_size=10)

    time.sleep(1.0)

    corrections = PublishCorrections.load_all_demonstrations(demo_dir)

    for (word, correction) in corrections.iteritems():
        # Correction is a list[CartStateStamped]
        print 'word: ', word

        marker = create_correction_viz_msg(correction)
        print '  Got a marker with {} points'.format(len(marker.points))

        # Publish each message
        publisher.publish(marker)
        pass

    publisher.unregister()



    pass


if __name__ == '__main__':
    arguments = sys.argv[1:]  # argv[0] is the program name.
    run(arguments)

__author__ = 'felixd'
