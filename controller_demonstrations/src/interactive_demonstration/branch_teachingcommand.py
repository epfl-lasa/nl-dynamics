#! /usr/bin/env python

import rospy
import smach
import std_msgs
import time
from sound_play.libsoundplay import SoundClient

from say_state import SayState


class GetTeachCommand(smach.State):

    outcome_commandteached ='commandteached'
    outcome_misspelling='misspelling'
    outcomes = [outcome_commandteached, outcome_misspelling]

    def __init__(self):
        smach.State.__init__(self, outcomes=GetTeachCommand.outcomes)
        topic_sub = '/nl_command_parsed'
        rospy.Subscriber(topic_sub, std_msgs.msg.String, self.callback, queue_size=1)
        self.cmd=''
        
        self.soundhandle = SoundClient()
        self._message = 'Is the following the right command '  # Store the message to say later.

    def speaking(self, text):
        self.soundhandle.say(text)

    def execute(self, userdata):
        self.cmd=''
        while(self.cmd==''):
            rospy.sleep(0.5)
            if(self.cmd!=''):
                rospy.sleep(0.5)
                rospy.loginfo(self._message + self.cmd)
                confirmation=self.speaking(self._message + self.cmd)
                self.cmd=''
                while(self.cmd!='yes' or self.cmd!='no' or self.cmd!='right' or self.cmd!='wrong'):
                        if(self.cmd=='yes' or self.cmd=='right'):
                                rospy.loginfo('YEAAAAAAAAH')
                                return GetTeachCommand.outcome_commandteached
                        elif(self.cmd=='no' or self.cmd=='wrong'):
                                return GetTeachCommand.outcome_misspelling

    def callback(self, data):
        self.cmd = data.data


class Demonstration(smach.State):

    outcome_demonstration ='Demonstration'
    outcomes = [outcome_demonstration]

    def __init__(self):
        smach.State.__init__(self, outcomes=Demonstration.outcomes)
        topic_sub = '/nl_command_parsed'
        rospy.Subscriber(topic_sub, std_msgs.msg.String, self.callback, queue_size=1)
        self.cmd=''

    def execute(self, userdata):
        self.cmd=''
        while(self.cmd!='Start'):
            rospy.sleep(0.5)
            while(self.cmd!='Stop'):
                rospy.sleep(0.5)
                return Demonstration.outcome_demonstration

    def callback(self, data):
        self.cmd=data.data

class GetConfirmation(smach.State):
    outcome_success ='Success'
    outcome_failure ='Failure'
    outcome_othertry = 'Other Try'

    outcomes = [outcome_success, outcome_failure, outcome_othertry]

    def __init__(self):
        smach.State.__init__(self, outcomes=GetConfirmation.outcomes)
        topic_sub = '/nl_command_parsed'
        rospy.Subscriber(topic_sub, std_msgs.msg.String, self.callback, queue_size=1)
        self.cmd=''

    def execute(self, userdata):
        self.cmd=''
        begin=rospy.get_rostime()
        end=rospy.get_rostime()
        while (end - begin).to_sec() < 10:
            if(self.cmd=='Right'):
                return GetConfirmation.outcome_success
            elif(self.cmd=='Retry'):
                return GetConfirmation.outcome_othertry
            rospy.sleep(0.5)
            end=rospy.get_rostime()
        return GetConfirmation.outcome_success

    def callback(self, data):
        self.cmd=data.data


class TeachingCommandBranch(smach.StateMachine):
    outcome_success = 'success'
    outcome_failure = 'failure'
    outcomes = [outcome_success, outcome_failure]

    def __init__(self):

        super(TeachingCommandBranch, self).__init__(
            outcomes=TeachingCommandBranch.outcomes)

        askteachcommand_name='Which command ?'
        askteachcommand_state=SayState('Which command would you like to teach me ?')

        getteachcommand_name='Get Command'
        getteachcommand_state=GetTeachCommand()

        startdemonstration_name='Start Demonstration'
        startdemonstration_state=SayState('Ok, say Start to begin the recording and Stop to end it')

        demonstration_name="Demonstration"
        demonstration_state=Demonstration() #create class
        #SHould this class begin a timer to know when the demonstration is over ?

        explanation_name='Explanation'
        explanation_state=SayState('Say Yes or No please')

        confirmation_name='Confirmation'
        confirmation_state=SayState('Do you want me to save this new command ?')

        getconfirmation_name='Get Confirmation'
        getconfirmation_state=GetConfirmation()

        with self:
            self.add(askteachcommand_name, askteachcommand_state,
                     transitions={SayState.outcome_success: getteachcommand_name})

            self.add(getteachcommand_name, getteachcommand_state,
                     transitions={GetTeachCommand.outcome_commandteached: startdemonstration_name,
                                  GetTeachCommand.outcome_misspelling: askteachcommand_name})

         

            self.add(startdemonstration_name, startdemonstration_state,
                     transitions={SayState.outcome_success: demonstration_name})

            self.add(demonstration_name, demonstration_state,
                     transitions={Demonstration.outcome_demonstration: confirmation_name})

            self.add(confirmation_name, confirmation_state,
                     transitions={SayState.outcome_success: getconfirmation_name})

            self.add(getconfirmation_name, getconfirmation_state,
                     transitions={GetConfirmation.outcome_success: TeachingCommandBranch.outcome_success,
                                  GetConfirmation.outcome_failure: TeachingCommandBranch.outcome_failure,
                                  GetConfirmation.outcome_othertry: startdemonstration_name})



if __name__ == '__main__':

    import smach_ros
    rospy.init_node('interactive_demo')

    machine = TeachingCommandBranch()

    # Visualize the machine.
    machine_viz = smach_ros.IntrospectionServer(
        'smash_server', machine, '/SM_ROOT')
    machine_viz.start()

    outcome = machine.execute()

    rospy.loginfo('Outcome: {}'.format(outcome))

    machine_viz.stop()


