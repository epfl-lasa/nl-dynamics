#! /usr/bin/env python

import rospy
import smach
import smach_ros
import sys
import time
import std_msgs

from say_state import SayState
from branch_changingspeed import ChangingSpeedBranch
from branch_gettingcommand import GettingCommandBranch
from branch_teachingcommand import TeachingCommandBranch
from sound_play.libsoundplay import SoundClient
#from branch_changespeed import

from controller_demonstrations.robot_dialogue_interface.kuka_dialogue_interface import KukaDialogueInterface

class ReadyState(smach.State):
    # This state has two possible outcomes.
    outcome_finished = 'finished'
    outcome_success = 'success'
    outcome_askingspeed = 'askingspeed'
    outcome_askcommand = 'askcommand'
    outcome_teach = 'teach'
    outcomes = [outcome_finished, outcome_success,
                outcome_askingspeed, outcome_askcommand, outcome_teach]

    def __init__(self):
        # Again, specify the outcomes.
        smach.State.__init__(self, outcomes=ReadyState.outcomes)

        # Subscribe to a topic, defined using a callback.
        topic = '/nl_command_parsed'
        rospy.Subscriber(topic, std_msgs.msg.String, self.callback, queue_size=1)

        # Internal data.
        self.msg = ''

    def execute(self, userdata):
        self.msg=''
        rospy.loginfo('Executing ReadyState')

        while True :
        # There are two outcomes possible from this state; always return one of
        # them.
            msg_split=self.msg.split()
            length_msg = len(msg_split)
            for i in range(length_msg):
                if (msg_split[i] == 'quit' or msg_split[i] == 'stop' or msg_split[i] == 'done'):
                    rospy.loginfo('State machine is finished.')
                    return ReadyState.outcome_finished
                elif (msg_split[i] == "finish"):
                    rospy.loginfo('What a success')
                    return ReadyState.outcome_success
                elif (msg_split[i] == 'askingspeed'):
                    return ReadyState.outcome_askingspeed
                elif (msg_split[i] == 'command'):
                    return ReadyState.outcome_askcommand
                elif (msg_split[i] == 'teach'):
                    rospy.loginfo('Now we teach !')
                    return ReadyState.outcome_teach

    def callback(self, data):
        # This is the callback for the subscribed topic.
        self.msg = data.data
        rospy.loginfo('Got message: {}'.format(self.msg))





class UserInteraction(smach.StateMachine):
    # A state machine similarly has possible outcomes.
    outcome_success = 'success'
    outcome_failure = 'failure'
    outcomes = [outcome_success, outcome_failure]

    def __init__(self, robot_interface=None):
        super(UserInteraction, self).__init__(outcomes=UserInteraction.outcomes)

        self._robot_interface = robot_interface
        rospy.loginfo('Have robot interface: {}'.format(self._robot_interface))

        soundhandle = SoundClient(blocking=True)
        soundhandle.say('Hi there, I am a robot')

        # Create the states and give them names here. Each state (an instance of
        # the class) has an associated name (a string), used by the transitions.
        hw_state = SayState(message='What would you like me to do ? command, changing speed or teaching ?', blocking=False)
        hw_name = 'Introduction'

        ready_state = ReadyState()
        ready_name = 'Which Branch ?'

        branchspeed_name = 'Changing Speed'
        branchspeed_machine = ChangingSpeedBranch()

        branchcommand_name = 'Giving Command'
        branchcommand_machine = GettingCommandBranch(robot_interface=self._robot_interface)

        branchteach_name = 'Teaching Command'
        branchteach_machine = TeachingCommandBranch(robot_interface=robot_interface)

        finished_state = SayState("I am finished")
        finished_name = 'Finishing'


        # All states are now defined. Connect them.
        with self:
            # The first state added is the initial state.
            self.add(hw_name, hw_state,
                    transitions={SayState.outcome_success: ready_name})

            # state. In this example, the ready outcome from ReadyState goes to
            # the collect node (identified by collect_name), and the finshed
            # outcome goes to the SAY_FINISHED node (again, identified by its
            # name). It's important to remember that all transitions are defined
            # by *strings*, not the underlying nodes.
            self.add(ready_name, ready_state,
                    transitions={ ReadyState.outcome_finished: finished_name,
                                  ReadyState.outcome_success: hw_name,
                                  ReadyState.outcome_askingspeed: branchspeed_name,
                                  ReadyState.outcome_askcommand: branchcommand_name,
                                  ReadyState.outcome_teach: branchteach_name})

            # Here the connected state is actually a whole other
            # StateMachine. This is valid as long as its outcomes are properly
            # connected.
            self.add(branchspeed_name, branchspeed_machine,
                    transitions={ChangingSpeedBranch.outcome_success: hw_name,
                                 ChangingSpeedBranch.outcome_reset: hw_name})

            self.add(branchcommand_name, branchcommand_machine,
                    transitions={GettingCommandBranch.outcome_success: hw_name})

            self.add(branchteach_name, branchteach_machine,
                    transitions={TeachingCommandBranch.outcome_success: hw_name,
                                 TeachingCommandBranch.outcome_reset: hw_name})

            self.add(finished_name, finished_state,
                    transitions={SayState.outcome_success: UserInteraction.outcome_success})

        pass


def run(arguments):
    rospy.init_node('interactive_demo')

    robot_interface = KukaDialogueInterface()
    robot_interface.connect()

    # Define the state machine here.
    machine = UserInteraction(robot_interface)

    # Visualize the machine.
    machine_viz = smach_ros.IntrospectionServer(
        'smash_server', machine, '/SM_ROOT')
    machine_viz.start()

    # Run it.
    outcome = machine.execute()

    rospy.loginfo('Outcome: {}'.format(outcome))

    machine_viz.stop()


if __name__ == '__main__':
    arguments = sys.argv[1:]  # argv[0] is the program name.
    run(arguments)
