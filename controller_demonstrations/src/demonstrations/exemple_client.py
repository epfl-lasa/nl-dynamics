#! /usr/bin/env python

import rospy
from nl_msgs.srv import Demonstration


def run():

    rospy.wait_for_service('Correction_Isolation')

    try:
        isolation = rospy.ServiceProxy('Correction_Isolation', Demonstration)
        reponse = isolation('testDemonstration')

        if reponse:
            print 'sucess'
        else:
            print 'fail'

    except rospy.ServiceException, e:
            print('Service call failed: %s' % e)

if __name__ == "__main__":
    print "hello"
    run()
