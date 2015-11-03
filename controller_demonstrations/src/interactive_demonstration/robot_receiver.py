#! /usr/bin/env python

import rospy
import std_msgs
from geometry_msgs.msg import Twist

class Receiver(object):

    def __init__(self):
        # This code initializes the turtle commander: sets the initial
        # speed and has the publisher.
        self.pub = rospy.Publisher('turtle1/cmd_vel', Twist, queue_size=5)
        rospy.Subscriber("/robot_control/desired_velocity", std_msgs.msg.Int8, self.callback_speed)
        rospy.Subscriber("/robot_control/desired_command", std_msgs.msg.String, self.callback_command)
        #self.pub = rospy.Publisher('turtle1/cmd_vel', Twist, queue_size=5) LATER
        self.speed=2
        rospy.sleep(1)

    def callback_speed(self, data):
        speed_change=data.data
        rospy.loginfo('I am changing the current speed to the speed : %d'% speed_change)
        self.speed=speed_change

    def callback_command(self, data):
        twist=Twist()
        command_change=data.data
        rospy.loginfo('I am doing the command : {} at speed {}'.format(
            command_change, self.speed))
        if(command_change=='left'):
            twist.angular.z = self.speed
        if(command_change=='right'):
            twist.angular.z = -self.speed
        if(command_change=='up'):
            twist.linear.x = self.speed
        if(command_change=='down'):
            twist.linear.x = -self.speed
        pass

        self.pub.publish(twist)


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