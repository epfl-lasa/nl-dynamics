#! /usr/bin/env python

import rospy
import smach
import std_msgs
import time
from sound_play.libsoundplay import SoundClient

#from controller_demonstrations.robot_dialogue_interface.turtle_dialogue_interface import TurtleDialogueInterface

from say_state import SayState

class GetCommand(smach.State):
    outcome_getcommand = 'getcommand'
    outcome_unknowncommand = 'unknowncommand'
    outcome_list = 'list'
    outcomes = [outcome_getcommand, outcome_unknowncommand, outcome_list]

    def __init__(self, command_list=None, robot_interface=None):
        # specify the outcomes
        smach.State.__init__(self, outcomes=GetCommand.outcomes)
        # Subscribe to a Topic
        topic_sub = '/nl_command_parsed'
        rospy.Subscriber(topic_sub, std_msgs.msg.String, self.callback, queue_size=1)
        topic_pub = '/robot_control/desired_command'
        self.pub = rospy.Publisher(topic_pub, std_msgs.msg.String, queue_size=5)
        # internal data
        self.msg = ''
        self.robot_interface = robot_interface
        self.command_list = []
        if command_list:
            self.command_list = command_list
        if self.robot_interface is not None:
            self.command_list = self.robot_interface.known_commands()
        self.soundhandle = SoundClient(blocking=False)

    def command_list_str(self):
        my_command_list= ' '.join(self.command_list)
        return my_command_list

    def command_in_dictionnary(self, message):
        for token in self.command_list:
            if token in message:
                return token
        return None

    def execute(self, userdata):
        # Assumption for now: wait until the user provides *one* of the available commands. Stay in this state until this is true.
        # We only return from this state once the user has provided a known command.
        # TODO: LOGINFO for the command type.
        # TODO later: after 10 seconds in this state, return outcome_unknowncommand
        self.msg=''
        begin=rospy.get_rostime()
        end=rospy.get_rostime()
        if self.robot_interface:
            self.command_list = self.robot_interface.known_commands()
            rospy.loginfo('Known commands: {}'.format(self.command_list))
        while (end - begin).to_sec() < 10:
            if (self.msg != ''):
                cmd = self.command_in_dictionnary(self.msg)
                if (cmd):
                    rospy.loginfo('Executing command: {}.'.format(cmd))
                    self.soundhandle.say('Executing command {}'.format(cmd), blocking=True)
                    self.robot_interface.execute_command(cmd)
                    self.msg=''
                    return GetCommand.outcome_getcommand
                elif(self.msg=='list'):
                    self.soundhandle.say('The list of commands is: ' + self.command_list_str(),
                                         blocking=True)
                    self.soundhandle.say('What would you like me to do?')
                    begin = rospy.get_rostime()  # Restart the timeout.
                    self.msg = ''  # Clear the input string.
                    continue

            end=rospy.get_rostime()
        rospy.loginfo('This command does not exist yet.')
        return GetCommand.outcome_unknowncommand

                    # Publish the string to a node where all the commands are registered (Command_Node) which will publish in the Robot_Node to execute it
                    # Should I make the check before 'if the command exist' ? And so implement the dictionnary when I teach a new command ?

    def callback(self, data):
        self.msg = data.data

class GettingCommandBranch(smach.StateMachine):
    outcome_success = 'success'
    outcome_failure = 'failure'
    outcomes = [outcome_success, outcome_failure]

    def __init__(self, robot_interface):

        super(GettingCommandBranch, self).__init__(
            outcomes=GettingCommandBranch.outcomes)

        askcommand_state = SayState('Which command would you like me to do ?')
        askcommand_name = 'Which Command ?'

        getcommand_state = GetCommand(robot_interface=robot_interface)
        getcommand_name = 'Receiving Command'

        commanddone_state = SayState('Okay I have done your command', blocking=True)
        commanddone_name = 'Aknowledge Command Done'

        unknowncommand_state = SayState('Unknown Command', blocking=True)
        unknowncommand_name = 'Unknow Command'

        listing_state = SayState('The list of command is :'+ getcommand_state.command_list_str(), blocking=True)
        listing_name = 'Listing Commands'

        with self:
            self.add(askcommand_name, askcommand_state,
                     transitions={SayState.outcome_success: getcommand_name})

            self.add(getcommand_name, getcommand_state,
                     transitions={GetCommand.outcome_getcommand: commanddone_name,
                                GetCommand.outcome_unknowncommand: unknowncommand_name,
                                GetCommand.outcome_list: listing_name})

            self.add(unknowncommand_name, unknowncommand_state,
                     transitions={SayState.outcome_success: GettingCommandBranch.outcome_success})

            self.add(commanddone_name, commanddone_state,
                     transitions={SayState.outcome_success: GettingCommandBranch.outcome_success})

            self.add(listing_name,listing_state,
                    transitions={SayState.outcome_success: askcommand_name})


if __name__ == '__main__':
    import smach_ros
    rospy.init_node('interactive_demo')

    kuka_interface = KukaDialogueInterface()
    kuka_interface.connect()

    machine=GettingCommandBranch(robot_interface=kuka_interface)
