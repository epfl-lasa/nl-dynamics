#! /usr/bin/env python

import rospy
import smach
from sound_play.libsoundplay import SoundClient

class SayState(smach.State):
    # A state in the state machine can have multiple outcomes. The outcomes must
    # be unique names.

    outcome_success = 'success'
    outcomes = [outcome_success]

    def __init__(self, message, blocking=False):
        # The state initialization can store information.

        # Make sure to specify the outcomes here.
        super(SayState, self).__init__(outcomes=SayState.outcomes,
                                       input_keys=['command_in'],
                                       output_keys=['command_out'])
        # Init speaker

        self.soundhandle = SoundClient(blocking=blocking)
        self._message = message  # Store the message to say later.

    def speaking(self, text):
        self.soundhandle.say(text)

    def execute(self, user_data):
        # The execute function gets called when the node is active. In this
        # case, it just prints a message, sleeps.

        print 'User data: ', user_data.keys()

        message_to_say = self._message
        if hasattr(user_data, 'command_in'):
            print 'Added user data:', user_data.command_in
            message_to_say += ' ' + user_data.command_in
            user_data.command_out=user_data.command_in


        self.speaking(message_to_say)
        rospy.loginfo(message_to_say)
        # The execute function *must* return one of its defined outcomes. Here,
        # we only have one outcome (SayState.outcome_success) so return it.
        return SayState.outcome_success

if __name__ == '__main__':
    import smach_ros
    rospy.init_node('interactive_demo')
    say=SayState()
