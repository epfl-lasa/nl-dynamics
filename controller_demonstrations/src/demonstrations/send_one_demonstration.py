#! /usr/bin/env python

import numpy as np

import rospy
from std_msgs.msg import String
from controller_demonstrations.msg import Demonstration, Demonstration_vec

class DemonstrationPublisher(object):
    def __init__(self):
        self.pub = rospy.Publisher('nl_demonstrations', Demonstration_vec, queue_size=10)
        rospy.init_node('send_one_demonstration', anonymous=True)

    def make_demo_point(self, x, y, dx, dy):
        demo = Demonstration(x, y, dx, dy)
        return demo

    def make_demo_vec(self, data):
        vec = Demonstration_vec()
        for d in data:
            point = self.make_demo_point(*d)
            vec.points.append(point)
            vec.num_points += 1
        return vec


    def run(self):
        data = [(0, 1, 2, 3), (1, 2, 3, 4)]
        v = self.make_demo_vec(data)
        self.pub.publish(v)

        log_str = "Sent demonstration at t={} -- {}".format(rospy.get_time(), v)
        rospy.loginfo(log_str)




if __name__ == '__main__':
    try:
        demo_publisher = DemonstrationPublisher()
        demo_publisher.run()
    except rospy.ROSInterruptException as e:
        print e
        pass