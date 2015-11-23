#! /usr/bin/env python

import rospy
import smach
import std_msgs
import time
from sound_play.libsoundplay import SoundClient

from say_state import SayState
from ConfirmationState import ConfirmationState

class Demonstration(smach.State):

    outcome_demonstration = 'Demonstration'
    outcome_reset = 'Reset'
    outcomes = [outcome_demonstration, outcome_reset]

    def __init__(self):
        smach.State.__init__(self, outcomes=Demonstration.outcomes)
        topic_sub = '/nl_command_parsed'
        rospy.Subscriber(topic_sub, std_msgs.msg.String, self.callback, queue_size=1)
        self.cmd=''

    def execute(self, userdata):
        self.cmd=''
        while(1):
            rospy.sleep(0.5)
            if(self.cmd=='start'):
                while(self.cmd!='stop'):
                    rospy.sleep(0.5)
                return Demonstration.outcome_demonstration
            elif(self.cmd=='reset'):
                return Demonstration.outcome_reset

    def callback(self, data):
        self.cmd=data.data


class TeachingCommandBranch(smach.StateMachine):
    outcome_success = 'success'
    outcome_reset = 'reset'
    outcomes = [outcome_success, outcome_reset]

    def __init__(self):

        super(TeachingCommandBranch, self).__init__(
            outcomes=TeachingCommandBranch.outcomes)

        askteachcommand_name='Which command ?'
        askteachcommand_state=SayState('Which command would you like to teach me ?')

        startdemonstration_name='Start Demonstration'
        startdemonstration_state=SayState('Ok, say Start to begin the recording and Stop to end it')

        getconfirmationname_name='Get Confirmation Command Name'
        getconfirmationname_state=ConfirmationState('Do you want to record the command name ?',1)

        explanation_name='Explanation'
        explanation_state=SayState('Say Yes or No please')

        demonstration_name='Demonstration'
        demonstration_state=Demonstration()

        getconfirmationtrajectory_name='Get Confirmation Command Trajectory'
        getconfirmationtrajectory_state=ConfirmationState('Do you want to record this trajectory ?',2)

        with self:
            self.add(askteachcommand_name, askteachcommand_state,
                     transitions={SayState.outcome_success: getconfirmationname_name})

            self.add(getconfirmationname_name, getconfirmationname_state,
                     transitions={ConfirmationState.outcome_success: startdemonstration_name,
                                  ConfirmationState.outcome_reset: TeachingCommandBranch.outcome_reset,
                                  ConfirmationState.outcome_redostate: askteachcommand_name})

            self.add(startdemonstration_name, startdemonstration_state,
                     transitions={SayState.outcome_success: demonstration_name})

            self.add(demonstration_name, demonstration_state,
                     transitions={Demonstration.outcome_demonstration: getconfirmationtrajectory_name,
                                  Demonstration.outcome_reset: TeachingCommandBranch.outcome_reset})

            self.add(getconfirmationtrajectory_name, getconfirmationtrajectory_state,
                     transitions={ConfirmationState.outcome_success: TeachingCommandBranch.outcome_success,
                                  ConfirmationState.outcome_reset: TeachingCommandBranch.outcome_reset,
                                  ConfirmationState.outcome_redostate: startdemonstration_name})



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


