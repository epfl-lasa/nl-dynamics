#! /usr/bin/env python

import rospy
import smach
import std_msgs

from say_state import SayState


class GetTeachCommand(smach.State):
    #subscribe to position topic
    def __init__(self, my_list):

    def execute(self, userdata):

    def current_command(self):
        current_command=self.cmd
        return self.current_cmd

    def callback(self, data):



class RightOrWrong(smach.State):
    def __init__(self, my_list):
    	#two outcomes : succes (if right word) failure (if wrong word)
    	#this will be conclude by the input of the user
    def execute(self, userdata):

    def callback(self, data):



class Demonstration(smach.State):

    def __init__(self, my_list):

    def execute(self, userdata):

    def callback(self, data):




class TeachingCommandMachine(smach.StateMachine):
    outcome_success = 'success'
    outcome_failure = 'failure'
    outcomes = [outcome_success, outcome_failure]

    def __init__(self):

        super(GettingCommandBranch, self).__init__(
            outcomes=GettingCommandBranch.outcomes)

        askteachcommand_name='Which command ?'
        askteachcommand_state=SayState('Which command would you like to teach me ?')

        getteachcommand_name='Get Command'
        getteachcommand_state=GetTeachCommand()

        confirmteachcommand_name='Right command ?'
        confirmteachcommand_state=SayState('Are you going to teach ' + GetTeachCommand.current_command)

        rw_name='Right or Wrong'
        rw_state=RightOrWrong()

        demonstration_name="Demonstration"
        demonstration_state=Demonstration() #create class
        #SHould this class begin a timer to know when the demonstration is over ?

        confirmation_name='Confirmation'
        confirmation_state=SayState('Do you want me to save this new command ?')

        with self:
            self.add(askteachcommand_name, askteachcommand_state,
                     transitions={SayState.outcome_success: getteachcommand_name})

            self.add(getteachcommand_name, getteachcommand_state,
                     transitions={GetTeachCommand.outcome_success: confirmteachcommand_name})

            self.add(confirmteachcommand_name, confirmteachcommand_state,
                     transitions={SayState.outcome_success: demonstration_name
                                  SayState.outcome_failure: askteachcommand_name})

            self.add(demonstartion_name, demonstration_state,
                     transitions={SayState.outcome_success: confirmation_name})

            self.add(confirmation_name, confirmation_state,
                     transitions={SayState.outcome_success: TeachingCommandMachine.outcome_succes
                                  SayState.outcome_failure: demonstration_state})



if __name__ == '__main__':

    import smach_ros
    rospy.init_node('branch_teachingcommand')

    machine = TeachingCommandMachine()

    # Visualize the machine.
    machine_viz = smach_ros.IntrospectionServer(
        'smash_server', machine, '/SM_ROOT')
    machine_viz.start()

    outcome = machine.execute()

    rospy.loginfo('Outcome: {}'.format(outcome))

    machine_viz.stop()


