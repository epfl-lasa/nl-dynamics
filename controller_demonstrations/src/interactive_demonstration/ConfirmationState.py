import rospy
import smach
from sound_play.libsoundplay import SoundClient

class ConfirmationState(smach.State):
    # A state in the state machine can have multiple outcomes. The outcomes must
    # be unique names.

    outcome_success = 'success'
    outcome_failure = 'failure'
    outcome_redostate ='redostate'
    outcomes = [outcome_success, outcome_failure, outcome_redostate]

    def __init__(self, message):
        # The state initialization can store information.

        # Make sure to specify the outcomes here.
        super(ConfirmationState, self).__init__(outcomes=ConfirmationState.outcomes)
        # Init speaker
        topic_sub = '/nl_command_parsed'
        rospy.Subscriber(topic_sub, std_msgs.msg.String, self.callback, queue_size=1)
        self.soundhandle = SoundClient()
        self._message = message  # Store the message to say later.

    def speaking(self, text):
        self.soundhandle.say(text)

    def execute(self, user_data):
        # The execute function gets called when the node is active. In this
        # case, it just prints a message, sleeps.
        self.cmd=''
        self.speaking(self._message)
        rospy.loginfo(self._message)
        rospy.sleep(4)
        while(self.cmd!=''):
            if(self.cmd=='yes' or self.cmd=='right' or self.cmd=='okay' or self.cmd=='true'):
                return ConfirmationState.outcome_success #Branch to the normal following state
            elif(self.cmd=='no' or self.cmd=='wrong' or self.cmd=='false'):
                self.cmd=''
                self.speaking('Do you want to leave the branch or retry ?')
                while(self.cmd==''):
                    if(self.cmd=='leave' or self.cmd=='quit' or self.cmd=='stop' or self.cmd=='done'):
                        return ConfirmationState.outcome_failure #Branch to ReadyState so then no change is done
                    else:
                        return ConfirmationState.outcome_redostate #Branch to the state just above

    def callback(self, data):
        self.cmd=data.data


if __name__ == '__main__':
    import smach_ros
    rospy.init_node('interactive_demo')
    say=ConfirmationState()
