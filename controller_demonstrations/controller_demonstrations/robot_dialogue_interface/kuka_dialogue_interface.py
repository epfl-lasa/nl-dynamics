#!/usr/bin/env python

import roslib
import rospy

from controller_demonstrations.demonstrations.correction_publisher import PublishCorrections
from controller_demonstrations.robot_dialogue_interface.robot_dialogue_interface import RobotDialogueInterface
from nl_msgs.srv import Demonstration


class KukaDialogueInterface(RobotDialogueInterface):
    def __init__(self):
        super(KukaDialogueInterface, self).__init__()
        self.srv = None

        # Create a correction publisher with no loaded corrections. The
        # correction publisher handles 'anchoring' the desired correction using
        # the current robot pose. Note that we link the _corrections variable
        # name with _known_commands since they are storing the exact same data:
        # changes to one will become changes to both.
        self.correction_publisher = PublishCorrections(demonstration_dir=None)
        # link names: python is awesome.
        self.correction_publisher._corrections = self._known_commands

    def connect(self):
        rospy.loginfo('Waiting for service...')
        rospy.wait_for_service('Correction_Isolation')
        self.srv = rospy.ServiceProxy('Correction_Isolation', Demonstration)
        pass

    def _robot_record_command(self, command, *args, **kwargs):
        rospy.loginfo('KUKA recording command: {}'.format(command))

        # Send the request and wait for a response.
        rospy.loginfo('Sending service request.')
        response = self.srv(command)
        rospy.loginfo('Received service response.')

        if response.success:
            rospy.loginfo('Received successful KUKA demonstration')
            # The demonstration publisher expects a list of CartStateStamped,
            # not an AnchoredDemonstration message -- extract the demonstration
            # to get the correct type stored inside the known commands.
            return response.demonstration.demonstration

        # Make sure the result is not stored.
        return False

    def execute_command(self, command):
        # Override the default behavior -- instead of calling do_command with
        # the stored value, use the key (the command string) itself, as this is
        # what the publisher expects.
        if command in self._known_commands:
            return self._robot_do_command(command)
        return False

    def _robot_do_command(self, command, *args, **kwargs):

        rospy.loginfo('Publishing command {} to Kuka.'.format(command))
        self.correction_publisher.process_command(
            nl_command=command, use_current_state_as_anchor=True)
        rospy.loginfo('Finished publishing command.')
        return True

    def _robot_set_speed(self, *args, **kwargs):
        rospy.loginfo('Unable to set speed at this time.')


if __name__ == '__main__':
    rospy.init_node('kuka_dialogue_interface', anonymous=False)
    kuka_interface = KukaDialogueInterface()
    kuka_interface.connect()
