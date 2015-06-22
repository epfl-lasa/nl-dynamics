#! /usr/bin/env python

import sys
import argparse
import time

import rospy
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

from correction_publisher import PublishCorrections
from nl_msgs.msg import AnchoredDemonstration

channel_viz = 'visualization_marker_array'


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

def create_correction_viz_msg(anchored_correction, id=0):

    marker = Marker()

    marker.header.frame_id = "/my_frame"
    marker.header.stamp = rospy.Time.now()
    marker.ns = "my_namespace"
    marker.id = id
    marker.type = Marker.LINE_STRIP
    marker.action = Marker.ADD
    marker.pose.position.x = 1
    marker.pose.position.y = 0
    marker.pose.position.z = 0
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0
    marker.scale.x = 0.1
    marker.scale.y = 0
    marker.scale.z = 0
    marker.color.a = 1.0  # Alpha must not be zero.
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 1.0

    for cart_state in anchored_correction:
        point = cart_state_to_point(cart_state)
        marker.points.append(point)

    return marker

def cart_state_to_point(point):
    # Go from CartStateStamped -> geometry_msgs.Point
    p = Point(point.pose.position.x, point.pose.position.y, point.pose.position.z)
    return p


def do(demo_dir):

    rospy.init_node('visualization_node', anonymous=False)
    publisher = rospy.Publisher(channel_viz, MarkerArray, queue_size=10)

    time.sleep(1.0)

    marker_array = MarkerArray()

    # Use the correction publisher to load the demonstrations for a directory.
    corrections = PublishCorrections.load_all_demonstrations(demo_dir)

    for (idx, (word, correction)) in enumerate(corrections.iteritems()):
        # Correction is a list[CartStateStamped]
        print 'word: ', word

        marker = create_correction_viz_msg(correction, idx)
        print '  Got a marker with {} points'.format(len(marker.points))

        marker_array.markers.append(marker)

        pass


    # Publish the entire marker array
    publisher.publish(marker_array)
    rospy.loginfo('Published markers: {}'.format(len(marker_array.markers)))

    publisher.unregister()



    pass


if __name__ == '__main__':
    arguments = sys.argv[1:]  # argv[0] is the program name.
    run(arguments)

__author__ = 'felixd'
