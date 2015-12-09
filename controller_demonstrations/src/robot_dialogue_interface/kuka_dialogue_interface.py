#!/usr/bin/env python

import rospy
from robot_dialogue_interface.robot_dialogue_interface import \
    RobotDialogueInterface
from nl_msgs.srv import Demonstration, DemonstrationRequest



class KukaDialogueInterface(RobotDialogueInterface):
    def __init__(self):
        super(KukaDialogueInterface, self).__init__()
        self.srv = None

    def connect(self):

        rospy.wait_for_service('Correction_Isolation')
        self.srv = rospy.ServiceProxy('Correction_Isolation', Demonstration)
        pass

    def _robot_record_command(self, command, *args, **kwargs):
        rospy.loginfo('Recording command: {}'.format(command))

        response = self.srv(command)

        if response.success:
            return response.demonstration

        # Make sure the result is not stored.
        return False

    def _robot_set_speed(self, *args, **kwargs):
        rospy.loginfo('Unable to set speed at this time.')


if __name__ == '__main__':
    rospy.init_node('kuka_dialogue_interface')
