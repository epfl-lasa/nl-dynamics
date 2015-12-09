#! /usr/bin/env python


import rospy

from std_msgs.msg import String

if __name__ == '__main__':
    rospy.init_node('send_robot_continue', anonymous=True)
    pub = rospy.Publisher('/KUKA/PauseCommand', String, queue_size=10)
    msg = String("CONTINUE")
    pub.publish(msg)
    rospy.loginfo("Sent continue command: {}".format(msg.data))
