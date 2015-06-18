#! /usr/bin/env python
import sys

import rospy
import time
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
#from visualization_msgs import MarkerArray

def run(arguments):
    print 'hello'

    rospy.init_node('visualize_corrections', anonymous=True)

    marker = Marker()
    marker.header.frame_id = "/my_frame"
    marker.header.stamp = rospy.Time.now()
    marker.ns = "my_namespace"
    marker.id = 1
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

    marker.points.append(Point(1, 2, 3))
    marker.points.append(Point(2, 4, 0))
    marker.points.append(Point(-2, 4, 0))
    marker.points.append(Point(0, 0, 0))
    marker.points.append(Point(5, 5, 5))


    #//only if using a MESH_RESOURCE marker type:
    #marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae"


    topic = '/visualization_marker'
    time.sleep(1.0)
    pub = rospy.Publisher(topic, Marker, queue_size=10)
    time.sleep(1.0)
    pub.publish(marker)
    time.sleep(1.0)

    print 'Published: {}'.format(marker)

    #rospy.spin()

    pass


if __name__ == '__main__':
    arguments = sys.argv[1:]  # argv[0] is the program name.
    run(arguments)

__author__ = 'Felix Duvallet'
