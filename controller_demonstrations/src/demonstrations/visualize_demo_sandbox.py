#! /usr/bin/env python

import sys
import argparse

from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker

from correction_publisher import PublishCorrections
from nl_msgs.msg import AnchoredDemonstration


def run(arguments):
    parser = argparse.ArgumentParser(
        description=('Load a directory of demonstration data (stored as bag '
                     'files).'))
    parser.add_argument('--demo_dir', metavar='directory', required=True)
    args = parser.parse_args(arguments)

    do(args.demo_dir)

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
    corr = PublishCorrections(demo_dir)

    for (word, correction) in corr.corrections:
        # Correction is a list[CartStateStamped]
        print 'word: ', word

        marker = create_correction_viz_msg(correction)
        print '  Got a marker with {} points'.format(len(marker.points))

    create_correction_viz_msg(correction)


    pass


if __name__ == '__main__':
    arguments = sys.argv[1:]  # argv[0] is the program name.
    run(arguments)

__author__ = 'felixd'
