#! /usr/bin/env python

import rospy
import smach
import sys

from std_msgs.msg import String

class ReadyState(smach.State):
    outcome_ready = 'ready'
    outcomes = [outcome_ready]

    def __init__(self):

        smach.State.__init__(self, outcomes=ReadyState.outcomes)

        topic = '/nl_command_parsed'
        rospy.Subscriber(topic, String, self.callback, queue_size=1)

        pass

    def execute(self, userdata):
        rospy.loginfo('Executing ReadyState')
        raw_input('Press enter to be ready...')

        return ReadyState.outcome_ready

    def callback(self, data):
        rospy.loginfo('Got message: {}'.format(data.data))


class SayState(smach.State):

    outcome_success = 'success'
    outcomes = [outcome_success]
    def __init__(self, message):
        smach.State.__init__(self, outcomes=SayState.outcomes)
        self._message = message

    def execute(self, userdata):
        rospy.loginfo('Executing SayState')
        print('--- {} ---'.format(self._message))
        return SayState.outcome_success



class InteractiveDemoMachine(smach.StateMachine):

    outcome_success = 'success'
    outcome_failure = 'failure'
    outcomes = [outcome_success, outcome_failure]

    def __init__(self):

        smach.StateMachine.__init__(self,
                                    outcomes=InteractiveDemoMachine.outcomes)

        say_state = SayState('Hello, world')
        say_name = 'SAYING'

        ready_state = ReadyState()
        ready_name = 'READY'

        finished_state = SayState("I am finished")
        finished_name = 'FINISHED'

        say_transitions = {SayState.outcome_success: ready_name}
        ready_transitions = {ReadyState.outcome_ready: finished_name}
        finished_transitions = {SayState.outcome_success: InteractiveDemoMachine.outcome_success}

        with self:
            self.add(say_name, say_state, transitions=say_transitions)
            self.add(ready_name, ready_state,
                     transitions=ready_transitions)
            self.add(finished_name, finished_state, finished_transitions)

        pass


def run(arguments):
    print 'Hello'

    rospy.init_node('interactive_demo')

    machine = InteractiveDemoMachine()

    outcome = machine.execute()

    rospy.loginfo('Outcome: {}'.format(outcome))




if __name__ == '__main__':
    arguments = sys.argv[1:]  # argv[0] is the program name.
    run(arguments)
