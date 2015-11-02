#! /usr/bin/env python

import rospy
from std_msgs

class Receiver(object):

    def __init__(self):
        # This code initializes the turtle commander: sets the initial
        # speed and has the publisher.
        rospy.Subscriber("/robot_control/desired_velocity", std_msgs.msg.Int8, self.callback_speed)
        rospy.Subscriber("/robot_control/desired_command", std_msgs.msg.String, self.callback_command)
        #self.pub = rospy.Publisher('turtle1/cmd_vel', Twist, queue_size=5) LATER
        rospy.sleep(1)

    def callback_speed(self, data):
        speed_change=data.data
        rospy.loginfo('I changed the speed to the speed : %d' % self.speed_change) 
        #Where should it be readable ? In the roslaunch window, no ?

    def callback_command(self, data):
        command_change=data.data
        rospy.loginfo('I changed the command to the command : %s' % self.command_change)

def listener():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'talker' node so that multiple talkers can
    # run simultaneously.
    rospy.init_node("robot_receiver", anonymous=False)
    receiver = Receiver()
    rospy.spin()


if __name__ == '__main__':
    listener()