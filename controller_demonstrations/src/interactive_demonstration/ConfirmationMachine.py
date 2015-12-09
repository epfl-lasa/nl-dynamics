#! /usr/bin/env python

import rospy
import smach
import std_msgs
from say_state import SayState


class ListenCommand(smach.State):

    outcome_listened = 'Listened'
    outcome_reset = 'reset'
    outcomes = [outcome_listened, outcome_reset]

    def __init__(self):

        super(ListenCommand, self).__init__(outcomes=ListenCommand.outcomes,
            output_keys=['listenedcommand_out'])
        topic_sub = '/nl_command_parsed'
        rospy.Subscriber(topic_sub, std_msgs.msg.String, self.callback, queue_size=1)

    def execute(self, user_data):
        self.cmd=''
        while(self.cmd==''):
            if(self.cmd=='reset'):
                return ListenCommand.outcome_reset

        user_data.listenedcommand_out=self.cmd
        return ListenCommand.outcome_listened


    def callback(self, data):
        self.cmd=data.data

class ListenConfirmation(smach.State):

    outcome_success = 'success'
    outcome_failure = 'failure'
    outcome_reset = 'reset'
    outcomes = [outcome_success, outcome_failure, outcome_reset]

    def __init__(self):

        super(ListenConfirmation, self).__init__(outcomes=ListenConfirmation.outcomes)
        topic_sub = '/nl_command_parsed'
        rospy.Subscriber(topic_sub, std_msgs.msg.String, self.callback, queue_size=1)

    def execute(self, user_data):
        self.cmd=''
        while(1):
            if(self.cmd=='reset'):
                return ListenConfirmation.outcome_reset
            elif(self.cmd=='right'):
                return ListenConfirmation.outcome_success
            elif(self.cmd=='wrong'):
                return ListenConfirmation.outcome_failure
            rospy.sleep(0.1)

    def callback(self, data):
        self.cmd=data.data


class ConfirmationMachine(smach.StateMachine):
    outcome_success = 'success'
    outcome_failure = 'failure'
    outcome_reset = 'reset'
    outcomes = [outcome_success, outcome_failure, outcome_reset]

    def __init__(self):

        super(ConfirmationMachine, self).__init__(outcomes=ConfirmationMachine.outcomes,
         output_keys=['UsersCommand_out'])

        self.userdata.command_userdata = 0

        listencommand_state = ListenCommand()
        listencommand_name = 'Listen Command'

        askconfirmation_state = SayState('Do you really want to use the command ')
        askconfirmation_name = 'Ask Confirmation'

        listenconfirmation_state = ListenConfirmation()
        listenconfirmation_name = 'Listen Confirmation'


        with self:
            self.add(listencommand_name, listencommand_state,
                     transitions={ListenCommand.outcome_listened: askconfirmation_name,
                                  ListenCommand.outcome_reset: ConfirmationMachine.outcome_reset},
                     remapping={'listenedcommand_out': 'command_userdata'})


            self.add(askconfirmation_name, askconfirmation_state,
                     transitions={SayState.outcome_success: listenconfirmation_name},
                      remapping={'command_in': 'command_userdata',
                                 'command_out':'UsersCommand_out'})

            self.add(listenconfirmation_name, listenconfirmation_state,
                     transitions={ListenConfirmation.outcome_success: ConfirmationMachine.outcome_success,
                                  ListenConfirmation.outcome_failure: ConfirmationMachine.outcome_failure,
                                  ListenConfirmation.outcome_reset: ConfirmationMachine.outcome_reset})


if __name__ == '__main__':
    import smach_ros
    rospy.init_node('interactive_demo')

    machine = ConfirmationMachine()

    # Visualize the machine.
    machine_viz = smach_ros.IntrospectionServer('smash_server', machine, '/SM_ROOT')
    machine_viz.start()

    outcome = machine.execute()

    rospy.loginfo('Outcome: {}'.format(outcome))

    machine_viz.stop()
