#! /usr/bin/env python

import rospy
import smach
import smach_ros
import sys

from std_msgs.msg import String

from demo_collection import DemoCollectionMachine


class ReadyState(smach.State):
    outcome_ready = 'ready'
    outcome_finished = 'finished'
    outcomes = [outcome_ready, outcome_finished]

    def __init__(self):

        smach.State.__init__(self, outcomes=ReadyState.outcomes)

        topic = '/nl_command_parsed'
        rospy.Subscriber(topic, String, self.callback, queue_size=1)
        self._finished = False

    def execute(self, userdata):
        rospy.loginfo('Executing ReadyState')
        raw_input('Press enter to be ready...')

        if self._finished:
            rospy.loginfo('State machine is finished.')
            return ReadyState.outcome_finished
        else:
            rospy.loginfo('The show will go on')
            return ReadyState.outcome_ready

    def callback(self, data):
        msg = data.data
        rospy.loginfo('Got message: {}'.format(msg))

        if 'quit' in msg or 'stop' in msg or 'done' in msg:
            self._finished = True


class SayState(smach.State):

    outcome_success = 'success'
    outcomes = [outcome_success]

    def __init__(self, message):
        super(SayState, self).__init__(outcomes=SayState.outcomes)
        self._message = message

    def execute(self, user_data):
        rospy.loginfo('Executing SayState')
        print('--- {} ---'.format(self._message))
        return SayState.outcome_success


class UserInteraction(smach.StateMachine):

    outcome_success = 'success'
    outcome_failure = 'failure'
    outcomes = [outcome_success, outcome_failure]

    def __init__(self):

        super(UserInteraction, self).__init__(
            outcomes=UserInteraction.outcomes)

        say_state = SayState('Hello, world')
        say_name = 'SAYING'

        ready_state = ReadyState()
        ready_name = 'READY'

        collect_name = 'COLLECT'
        collect_machine = DemoCollectionMachine()

        finished_state = SayState("I am finished")
        finished_name = 'FINISHED'

        with self:
            self.add(say_name, say_state,
                     transitions={SayState.outcome_success: ready_name})
            self.add(ready_name, ready_state,
                     transitions={ReadyState.outcome_ready: collect_name,
                                  ReadyState.outcome_finished: finished_name})
            self.add(collect_name, collect_machine,
                     transitions={DemoCollectionMachine.outcome_success: say_name,  # Go back to say_state
                                  DemoCollectionMachine.outcome_failure: UserInteraction.outcome_failure})
            self.add(finished_name, finished_state,
                     transitions={SayState.outcome_success: UserInteraction.outcome_success})

        pass


def run(arguments):
    rospy.init_node('interactive_demo')

    machine = UserInteraction()

    # Visualize the machine.
    machine_viz = smach_ros.IntrospectionServer(
        'smash_server', machine, '/SM_ROOT')
    machine_viz.start()

    outcome = machine.execute()

    rospy.loginfo('Outcome: {}'.format(outcome))

    machine_viz.stop()

if __name__ == '__main__':
    arguments = sys.argv[1:]  # argv[0] is the program name.
    run(arguments)
