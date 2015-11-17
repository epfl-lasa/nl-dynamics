#! /usr/bin/env python

import rospy
from nl_msgs.srv import Com

# -------- input format : 
# string word
# string output
# int64 num
# bool discard_static_points
# bool plot
# string format


def run():

	rospy.wait_for_service('Correction_Isolation')

	try:
		isolation = rospy.ServiceProxy('Correction_Isolation', Com)
		reponse = isolation('hi') #, 'out.bag', 50, True, True, 'points')

		if reponse:
			print 'sucess'
		else:
			print 'fail'

	except rospy.ServiceException, e:
    		print('Service call failed: %s' % e)


if __name__ == "__main__":
	print "hello"
	run()
