#! /usr/bin/env python

import rospy
import smach
from sound_play.libsoundplay import SoundClient

class SayState(smach.State):
    # A state in the state machine can have multiple outcomes. The outcomes must
    # be unique names.

    outcome_success = 'success'
    outcomes = [outcome_success]

    def __init__(self, message):
        # The state initialization can store information.

        # Make sure to specify the outcomes here.
        super(SayState, self).__init__(outcomes=SayState.outcomes)
        # Init speaker

        self.soundhandle = SoundClient()
        self._message = message  # Store the message to say later.

    def speaking(self, text):
        self.soundhandle.say(text)

    def execute(self, user_data):
        # The execute function gets called when the node is active. In this
        # case, it just prints a message, sleeps.
        self.speaking(self._message)
        rospy.loginfo(self._message)
        rospy.sleep(4)
        # The execute function *must* return one of its defined outcomes. Here,
        # we only have one outcome (SayState.outcome_success) so return it.
        return SayState.outcome_success

if __name__ == '__main__':
    import smach_ros
    rospy.init_node('interactive_demo')
    say=SayState()
