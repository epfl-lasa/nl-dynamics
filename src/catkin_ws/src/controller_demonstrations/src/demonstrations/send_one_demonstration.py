#! /usr/bin/env python

import numpy as np

import rospy
from std_msgs.msg import String
from controller_demonstrations.msg import Demonstration

class DemonstrationPublisher(object):
    def __init__(self):
        self.pub = rospy.Publisher('nl_demonstrations', Demonstration, queue_size=10)
        rospy.init_node('send_one_demonstration', anonymous=True)


    def make_demo(self, x, y, dx, dy):
        demo = Demonstration(x, y, dx, dy)
        return demo

    def run(self):
        d = self.make_demo(0, -1, 2, 1)
        self.pub.publish(d)
        log_str = "Sent demonstration at t={} -- {}".format(rospy.get_time(), d)
        rospy.loginfo(log_str)

if __name__ == '__main__':
    try:
        demo_publisher = DemonstrationPublisher()
        demo_publisher.run()
    except rospy.ROSInterruptException:
        pass
