import rospy
import smach
import std_msgs
from sound_play.libsoundplay import SoundClient

class ConfirmationState(smach.State):
    # A state in the state machine can have multiple outcomes. The outcomes must
    # be unique names.

    outcome_success = 'success'
    outcome_reset = 'reset'
    outcome_redostate ='redostate'
    outcomes = [outcome_success, outcome_reset, outcome_redostate]

    def __init__(self, message, value): #Value is to use one or another option
        # The state initialization can store information.

        # Make sure to specify the outcomes here.
        super(ConfirmationState, self).__init__(outcomes=ConfirmationState.outcomes)
        # Init speaker
        topic_sub = '/nl_command_parsed'
        rospy.Subscriber(topic_sub, std_msgs.msg.String, self.callback, queue_size=1)
        self.soundhandle = SoundClient()
        self._message = message  # Store the message to say later.
        self._value=value

    def speaking(self, text):
        self.soundhandle.say(text)

    def execute(self, user_data):
        # The execute function gets called when the node is active. In this
        # case, it just prints a message, sleeps.
        self.cmd=''
        rospy.sleep(4)
        rospy.loginfo('THERE')
        while(1):
            rospy.sleep(0.5)
            rospy.loginfo('HERE')
            if(self.cmd!=''):
                rospy.loginfo('HERE')
                if(self._value==1):
                    self.speaking(self._message + self.cmd)
                    break
                else:
                    self.speaking(self._message)
                    break
        self.cmd=''
        while(1):
            if(self.cmd=='yes' or self.cmd=='right' or self.cmd=='okay' or self.cmd=='true'):
                return ConfirmationState.outcome_success #Branch to the normal following state
            elif(self.cmd=='no' or self.cmd=='wrong' or self.cmd=='false'):
                self.cmd=''
                self.speaking('Do you want to leave the branch or retry ?')
                while(1):
                    if(self.cmd=='leave' or self.cmd=='quit' or self.cmd=='stop' or self.cmd=='done'):
                        return ConfirmationState.outcome_reset #Branch to ReadyState so then no change is done
                    else:
                        return ConfirmationState.outcome_redo #Branch to the state just above

    def callback(self, data):
        self.cmd=data.data


if __name__ == '__main__':
    import smach_ros
    rospy.init_node('interactive_demo')
    say=ConfirmationState()
