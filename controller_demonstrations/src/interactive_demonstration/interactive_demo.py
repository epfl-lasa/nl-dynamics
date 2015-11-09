#! /usr/bin/env python

import rospy
import smach
import smach_ros
import sys
import time
import std_msgs

from demo_collection import DemoCollectionMachine
from say_state import SayState
from branch_changingspeed import ChangingSpeedBranch
#from branch_changespeed import


class ReadyState(smach.State):
    # This state has two possible outcomes.
    outcome_ready = 'ready'
    outcome_finished = 'finished'
    outcome_success = 'success'
    outcome_askingspeed = 'askingspeed'
    outcome_askcommand = 'askcommand'
    outcomes = [outcome_ready, outcome_finished, outcome_success,
                outcome_askingspeed, outcome_askcommand]

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
                elif (msg_split[i] == 'collect'):
                    rospy.loginfo('The show will go on')
                    return ReadyState.outcome_ready

    def callback(self, data):
        # This is the callback for the subscribed topic.
        self.msg = data.data
        rospy.loginfo('Got message: {}'.format(self.msg))


class GetCommand(smach.State):
    outcome_getcommand = 'getcommand'
    outcome_unknowncommand = 'unknowncommand'
    outcome_list = 'list'
    outcomes = [outcome_getcommand, outcome_unknowncommand, outcome_list]

    def __init__(self, my_list):
        # specify the outcomes
        smach.State.__init__(self, outcomes=GetCommand.outcomes)
        # Subscribe to a Topic
        topic_sub = '/nl_command_parsed'
        rospy.Subscriber(topic_sub, std_msgs.msg.String, self.callback, queue_size=1)
        topic_pub = '/robot_control/desired_command'
        self.pub = rospy.Publisher(topic_pub, std_msgs.msg.String, queue_size=5)
        # internal data
        self.msg = ''
        self.command_list = my_list

    def command_list_str(self):
        my_command_list= ' '.join(self.command_list)
        return my_command_list

    def command_in_dictionnary(self, msg):
        b = -1
        cmd = ''
        msg_split = self.msg.split()  # Separe la string par mot (separateur *espace*
        length_msg = len(msg_split)  # Retourne le nombre de mots dans la string
        for i in range(length_msg):  # Parcoure chaque mot
            if (msg_split[i] in self.command_list):  # Si un des mots est dans la string msg
                b = i  # Alors il donne la place du mot dans la string

        if (b >= 0):  # empty string is checked here and if the number is in the string also
            cmd = msg_split[b]  # contient le mot qui est dans le dictionnaire
            self.pub.publish(cmd)
            return True
        else:
            return False

    def execute(self, userdata):
        # Assumption for now: wait until the user provides *one* of the available commands. Stay in this state until this is true.
        # We only return from this state once the user has provided a known command.
        # TODO: LOGINFO for the command type.
        # TODO later: after 10 seconds in this state, return outcome_unknowncommand
        self.msg=''
        begin=time.time()
        end=0
        while ((begin+10)>end):
            if (self.msg != ''):
                if (self.command_in_dictionnary(self.msg)):
                    rospy.loginfo('This command exist yet.')
                    #pub.publish(self.msg) !!!!!!!!!!!!!
                    self.msg=''
                    return GetCommand.outcome_getcommand
                elif(self.msg=='list'):
                    return GetCommand.outcome_list
            end=time.time()
        rospy.loginfo('This command does not exist yet.')
        return GetCommand.outcome_unknowncommand

                    # Publish the string to a node where all the commands are registered (Command_Node) which will publish in the Robot_Node to execute it
                    # Should I make the check before 'if the command exist' ? And so implement the dictionnary when I teach a new command ?

    def callback(self, data):
        self.msg = data.data


class UserInteraction(smach.StateMachine):
    # A state machine similarly has possible outcomes.
    outcome_success = 'success'
    outcome_failure = 'failure'
    outcomes = [outcome_success, outcome_failure]

    def __init__(self):
        super(UserInteraction, self).__init__(outcomes=UserInteraction.outcomes)

        # Create the states and give them names here. Each state (an instance of
        # the class) has an associated name (a string), used by the transitions.
        hw_state = SayState(message='Hello, world')
        hw_name = 'SAY_HW'

        ready_state = ReadyState()
        ready_name = 'READY'

        collect_name = 'COLLECT'
        collect_machine = DemoCollectionMachine()

        branchspeed_name = 'SPEED'
        branchspeed_machine = ChangingSpeedBranch()

        finished_state = SayState("I am finished")
        finished_name = 'SAY_FINISHED'

        askcommand_state = SayState('Which command would you like me to do ?')
        askcommand_name = 'ASK_COMMAND'

        getcommand_state = GetCommand(['left', 'right', 'up', 'down', 'dance'])
        getcommand_name = 'GET_COMMAND'


        # implement something like: getcommand_state.command_list() -> 'left, right, up, down'
        #  tip: loop up ', '.join() google string join
        # listing_state = SayState('The available commands are ' + command_list)

        commanddone_state = SayState('Okay I have done your command')
        commanddone_name = 'COMMAND_DONE'

        unknowncommand_state = SayState('Unknown Command')
        unknowncommand_name = 'UNKNOWN_COMMAND'

        listing_state = SayState('The list of command is :'+ getcommand_state.command_list_str())
        listing_name = 'LISTING'

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
                    transitions={ReadyState.outcome_ready: collect_name,
                                  ReadyState.outcome_finished: finished_name,
                                  ReadyState.outcome_success: hw_name,
                                  ReadyState.outcome_askingspeed: branchspeed_name,
                                  ReadyState.outcome_askcommand: askcommand_name})

            # Here the connected state is actually a whole other
            # StateMachine. This is valid as long as its outcomes are properly
            # connected.
            self.add(collect_name, collect_machine,
                    transitions={
                         DemoCollectionMachine.outcome_success: hw_name,
                         # Go back to hello world
                         DemoCollectionMachine.outcome_failure: UserInteraction.outcome_failure})

            self.add(branchspeed_name, branchspeed_machine,
                    transitions={ChangingSpeedBranch.outcome_success: hw_name})

            self.add(finished_name, finished_state,
                    transitions={SayState.outcome_success: UserInteraction.outcome_success})


            # Giving a command to do States
            self.add(askcommand_name, askcommand_state,
                     transitions={SayState.outcome_success: getcommand_name})

            self.add(getcommand_name, getcommand_state,
                     transitions={GetCommand.outcome_getcommand: commanddone_name,
                                GetCommand.outcome_unknowncommand: unknowncommand_name #})
                                ,GetCommand.outcome_list: listing_name})

            self.add(unknowncommand_name, unknowncommand_state,
                     transitions={SayState.outcome_success: hw_name})

            self.add(commanddone_name, commanddone_state,
                     transitions={SayState.outcome_success: hw_name})

            self.add(listing_name,listing_state,
                    transitions={SayState.outcome_success: askcommand_name})

        pass


def run(arguments):
    rospy.init_node('interactive_demo')

    # Define the state machine here.
    machine = UserInteraction()

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
