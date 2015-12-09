#! /usr/bin/env python

import sys
import rospy
import std_msgs
import time
from time import gmtime, strftime


def callback(data):
        f.write(data.data+'\n')
        rospy.loginfo(data.data	)

def listener():
        rospy.init_node('listener',anonymous=False)
        topic_sub = '/nl_command_parsed'
        rospy.Subscriber(topic_sub, std_msgs.msg.String, callback, queue_size=1)
        rospy.spin()


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print 'Provide filename'
        exit(0)

    fname = sys.argv[1]
    print('Writing to {}'.format(fname))

    f = open(fname, 'w')
    listener()
    f.write('\n')
    f.close()

