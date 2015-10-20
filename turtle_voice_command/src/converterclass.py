#!/usr/bin/env python

from sound_play.libsoundplay import SoundClient
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class TurtleCommander(object):

        def __init__(self):
                # This code initializes the turtle commander: sets the initial
                # speed and has the publisher.
                self.speed = 2

                rospy.Subscriber("nl_command_parsed", String, self.callback)
                self.pub = rospy.Publisher('turtle1/cmd_vel', Twist, queue_size=5)
                self.soundhandle = SoundClient()
                rospy.sleep(1)
                rospy.loginfo('Started turtle controller')

        def callback(self, data):

                # To access the speed variable, you need to use self.speed.
                #rospy.loginfo('speed %d', self.speed)

                twist = Twist()
                rospy.loginfo("I heard %s", data.data)
		#if(data.data=="okay robot go faster"):
		if(str.find(data.data,"go")>0 and str.find(data.data,"faster")>0):
			#rospy.loginfo('Making the robot go faster.')
			self.speed+=1;
			self.speaking("New speed is %s" % self.speed)
		#if(data.data=="okay robot go up slower"):
		if(str.find(data.data,"go")>0 and str.find(data.data,"slower")>0):
			#rospy.loginfo('Making the robot go slower.')
			self.speed-=1;
			self.speaking("New speed is %s" % self.speed)
                if(data.data=="okay robot go right"):
		#if(str.find(data.data,"go")>0 and str.find(data.data,"right")>0):
                        twist.linear.x = 0;
                        twist.linear.y = 0; twist.linear.z = 0;
                        twist.angular.x = 0; twist.angular.y = 0;
                        twist.angular.z = -self.speed;
			self.speaking("go right")
                #if(data.data=="go left"):
		if(str.find(data.data,"go")>0 and str.find(data.data,"left")>0):
                        twist.linear.x = 0;
                        twist.linear.y = 0; twist.linear.z = 0;
                        twist.angular.x = 0; twist.angular.y = 0;
                        twist.angular.z = self.speed;
			self.speaking("go left")
                #if(data.data=="okay robot go up"):
		if(str.find(data.data,"go")>0 and str.find(data.data,"up")>0):
                        twist.linear.x = self.speed;
                        twist.linear.y = 0; twist.linear.z = 0;
                        twist.angular.x = 0; twist.angular.y = 0;
                        twist.angular.z = 0;
			self.speaking("go up")
                #if(data.data=="okay robot go back"):
		if(str.find(data.data,"go")>0 and str.find(data.data,"back")>0):
                        twist.linear.x = -self.speed;
                        twist.linear.y = 0; twist.linear.z = 0;
                        twist.angular.x = 0; twist.angular.y = 0;
                        twist.angular.z = 0;
			self.speaking("go back")
                        pass

		rospy.loginfo("going trhough")
                self.pub.publish(twist)


        def speaking(self, text):
                # The soundhandle has been defined in the class init.
		self.soundhandle.say("Acknowledge %s" % text)
		



def listener():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'talker' node so that multiple talkers can
    # run simultaneously.
    rospy.init_node("converterclass", anonymous=False)

    turtleCommander = TurtleCommander()


    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    listener()
