#! /usr/bin/env python

import rospy
import smach
import smach_ros
import sys

from sound_play.libsoundplay import SoundClient

from std_msgs.msg import String

from demo_collection import DemoCollectionMachine


class SayState(smach.State):
    # A state in the state machine can have multiple outcomes. The outcomes must
    # be unique names.

    outcome_success = 'success'
    outcomes = [outcome_success]

    def __init__(self, message):
        # The state initialization can store information.

        # Make sure to specify the outcomes here.
        super(SayState, self).__init__(outcomes=SayState.outcomes)
        # Init speaker

        self.soundhandle = SoundClient()
        self._message = message  # Store the message to say later.

    def speaking(self, text):
        self.soundhandle.say(text)

    def execute(self, user_data):
        # The execute function gets called when the node is active. In this
        # case, it just prints a message, sleeps.
        self.speaking(self._message)
        rospy.loginfo(self._message)

        # The execute function *must* return one of its defined outcomes. Here,
        # we only have one outcome (SayState.outcome_success) so return it.
        return SayState.outcome_success


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
        rospy.Subscriber(topic, String, self.callback, queue_size=1)

        # Internal data.
        self.msg = ''

    def execute(self, userdata):
        # This is called when the node is active. Currently it waits for the
        # user to press enter.

        rospy.loginfo('Executing ReadyState')
        raw_input('Press enter to be ready...')

        # There are two outcomes possible from this state; always return one of
        # them.
        if (self.msg == 'quit' or self.msg == 'stop' or self.msg == 'done'):
            rospy.loginfo('State machine is finished.')
            return ReadyState.outcome_finished
        elif (self.msg == "finish"):
            rospy.loginfo('What a success')
            return ReadyState.outcome_success
        elif (self.msg == 'askingspeed'):
            return ReadyState.outcome_askingspeed
        elif (self.msg == 'command'):
            return ReadyState.outcome_askcommand
        else:
            rospy.loginfo('The show will go on')
            return ReadyState.outcome_ready

    def callback(self, data):
        # This is the callback for the subscribed topic.
        self.msg = data.data
        rospy.loginfo('Got message: {}'.format(self.msg))


class ChangeSpeed(smach.State):
    outcome_speedchanged = 'speedchanged'
    outcomes = [outcome_speedchanged]

    def __init__(self):
        # specify the outcomes
        smach.State.__init__(self, outcomes=ChangeSpeed.outcomes)
        # Subscribe to a Topic
        topic = '/nl_command_parsed'
        rospy.Subscriber(topic, String, self.callback, queue_size=1)
        # internal data
        self.msg = ''
        self.speed_integer = None

        # This method does not change the class members directly.

    def string_to_number(self, msg):
        a = dict(one=1, two=2, three=3, four=4, five=5, six=6, seven=7, eight=8,
                 nine=9, ten=10, eleven=11)  # because it has to go to eleven
        b = -1
        msg_split = msg.split()  # Separe la string par mot (separateur *espace*
        length_msg = len(msg_split)  # Retourne le nombre de mots dans la string
        for i in range(length_msg):  # Parcoure chaque mot
            if (msg_split[
                    i] in a.keys()):  # Si un des mots est dans la string msg
                b = i  # Alors il donne la place du mot dans la string

        if (b >= 0):
            new_speed = a.get(msg_split[
                                  b])  # new_speed va contenir la valeur du dictionnaire se trouvant a la position b
            return new_speed
        else:
            return None

    def execute(self, userdata):
        rospy.loginfo('Executing ChangeSpeed')
        # Will change the speed of the robot
        while (self.speed_integer == None):
            self.speed_integer = self.string_to_number(
                self.msg)  # HAVE TO PUBLISH TO ROBOT TO CHANGE SPEED
            rospy.sleep(0.1)
        rospy.loginfo('New Speed is %s', self.speed_integer)
        return ChangeSpeed.outcome_speedchanged

    def callback(self, data):
        self.msg = data.data  # data.data == self.msg de ReadyState


class GetCommand(smach.State):
    outcome_getcommand = 'getcommand'
    outcome_unknowncommand = 'unknowncommand'
    outcomes = [outcome_getcommand, outcome_unknowncommand]

    def __init__(self):
        # specify the outcomes
        smach.State.__init__(self, outcomes=GetCommand.outcomes)
        # Subscribe to a Topic
        topic = '/nl_command_parsed'
        rospy.Subscriber(topic, String, self.callback, queue_size=1)
        # internal data
        self.msg = ''
        self.cmd = ''
        self.a = dict(faster=1, right=2, left=3, slower=4)

    def command_in_dictionnary(self, msg):
        b = -1
        msg_split = msg.split()  # Separe la string par mot (separateur *espace*
        length_msg = len(msg_split)  # Retourne le nombre de mots dans la string
        for i in range(length_msg):  # Parcoure chaque mot
            if (msg_split[
                    i] in self.a.keys()):  # Si un des mots est dans la string msg
                b = i  # Alors il donne la place du mot dans la string

        if (
            b >= 0):  # empty string is checked here and if the number is in the string also
            self.cmd = msg_split[
                b]  # contient le mot qui est dans le dictionnaire
            return True
        else:
            return False

    def execute(self, userdata):
        while True:
            if (self.msg != ''):
                if (self.command_in_dictionnary):
                    return GetCommand.outcome_getcommand
                    break
                else:
                    rospy.loginfo(
                        'This command does not exist yet.')  ##May I call the SayState in the getCommand State, or should redefine it in this stat ?
                    return GetCommand.outcome_unknowncommand
                    break

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

        finished_state = SayState("I am finished")
        finished_name = 'SAY_FINISHED'

        askingspeed_state = SayState('At which speed do you want me to go ?')
        askingspeed_name = 'ASKING_SPEED'

        speedchanged_state = ChangeSpeed()
        speedchanged_name = 'CHANGED_SPEED'

        validatespeed_state = SayState('New Velocity implemented')
        validatespeed_name = 'VALIDATE_SPEED'

        askcommand_state = SayState('Which command would you like me to do ?')
        askcommand_name = 'ASK_COMMAND'

        getcommand_state = GetCommand()
        getcommand_name = 'GET_COMMAND'

        commanddone_state = SayState('Okay I have done your command')
        commanddone_name = 'COMMAND_DONE'

        unknowncommand_state = SayState('Unknown Command')
        unknowncommand_name = 'UNKNOWN_COMMAND'

        # For executing commands, assume there exists a list of all the
        # available commands the robot can execute: you can define this
        # for yourself in the constructor.

        # All states are now defined. Connect them.
        with self:
            # The first state added is the initial state.
            self.add(hw_name, hw_state,
                     transitions={SayState.outcome_success: ready_name})

            # For each state, all connections must be mapped to another
            # state. In this example, the ready outcome from ReadyState goes to
            # the collect node (identified by collect_name), and the finshed
            # outcome goes to the SAY_FINISHED node (again, identified by its
            # name). It's important to remember that all transitions are defined
            # by *strings*, not the underlying nodes.
            self.add(ready_name, ready_state,
                     transitions={ReadyState.outcome_ready: collect_name,
                                  ReadyState.outcome_finished: finished_name,
                                  ReadyState.outcome_success: hw_name,
                                  ReadyState.outcome_askingspeed: askingspeed_name,
                                  ReadyState.outcome_askcommand: askcommand_name})

            # Here the connected state is actually a whole other
            # StateMachine. This is valid as long as its outcomes are properly
            # connected.
            self.add(collect_name, collect_machine,
                     transitions={
                         DemoCollectionMachine.outcome_success: hw_name,
                         # Go back to hello world
                         DemoCollectionMachine.outcome_failure: UserInteraction.outcome_failure})

            self.add(finished_name, finished_state,
                     transitions={
                         SayState.outcome_success: UserInteraction.outcome_success})


            # NEW
            # Changing Speed States
            self.add(askingspeed_name, askingspeed_state,
                     transitions={SayState.outcome_success: speedchanged_name})

            self.add(speedchanged_name, speedchanged_state,
                     transitions={
                         ChangeSpeed.outcome_speedchanged: validatespeed_name})

            self.add(validatespeed_name, validatespeed_state,
                     transitions={SayState.outcome_success: hw_name})

            # Giving a command to do States
            self.add(askcommand_name, askcommand_state,
                     transitions={SayState.outcome_success: getcommand_name})

            self.add(getcommand_name, getcommand_state,
                     transitions={
                         GetCommand.outcome_getcommand: commanddone_name,
                         GetCommand.outcome_unknowncommand: unknowncommand_name})

            self.add(unknowncommand_name, unknowncommand_state,
                     transitions={SayState.outcome_success: hw_name})

            self.add(commanddone_name, commanddone_state,
                     transitions={SayState.outcome_success: hw_name})

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
