#! /usr/bin/env python

import rospy
import smach
import std_msgs

from say_state import SayState


class GetTeachCommand(smach.State):

    outcome_commandteached ='commandteached'
    outcomes = [outcome_commandteached]

    def __init__(self): #bidouillage, ajout d'un argument inutile to make it works
        smach.State.__init__(self, outcomes=GetTeachCommand.outcomes)
        topic_sub = '/nl_command_parsed'
        rospy.Subscriber(topic_sub, std_msgs.msg.String, self.callback, queue_size=1)

    def execute(self, userdata):
        self.cmd=''
        while(self.cmd==''):
            rospy.sleep(0.5)
            return GetTeachCommand.outcome_commandteached

    def current_command(self):
        currentcommand=self.cmd
        return currentcommand

    def callback(self, data):
        self.cmd = data.data


class RightOrWrong(smach.State):

    outcome_success ='success'
    outcome_failure ='failure'
    outcome_explanation='explanation'
    outcomes = [outcome_success, outcome_failure, outcome_explanation]

    def __init__(self):
        smach.State.__init__(self, outcomes=RightOrWrong.outcomes)
        topic_sub = '/nl_command_parsed'
        rospy.Subscriber(topic_sub, std_msgs.msg.String, self.callback, queue_size=1)

    def execute(self, userdata):
        if(self.cmd=='Yes'):
            return RightOrWrong.outcome_success
        elif(self.cmd=='No'):
            return RightOrWrong.outcome_failure
        else:
            return RightOrWrong.outcome_explanation

    def callback(self, data):
        self.cmd=data.data


class Demonstration(smach.State):

    outcome_demonstration ='Demonstration'
    outcomes = [outcome_demonstration]

    def __init__(self):
        smach.State.__init__(self, outcomes=Demonstration.outcomes)

    def execute(self, userdata):
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
    outcome_quitteaching = 'Quit Teaching'
    outcome_othertry = 'Other Try'

    outcomes = [outcome_success, outcome_failure, outcome_quitteaching, outcome_othertry]

    def __init__(self):
        smach.State.__init__(self, outcomes=GetConfirmation.outcomes)
        topic_sub = '/nl_command_parsed'
        rospy.Subscriber(topic_sub, std_msgs.msg.String, self.callback, queue_size=1)

    def execute(self, userdata):
        if(self.cmd=='Yes'):
            return RightOrWrong.outcome_success
        elif(self.cmd=='No'):
            return RightOrWrong.outcome_failure
        else:
            return RightOrWrong.outcome_othertry

    def callback(self, data):
        self.cmd=data.data


class TeachingCommandMachine(smach.StateMachine):
    outcome_success = 'success'
    outcome_failure = 'failure'
    outcomes = [outcome_success, outcome_failure]

    def __init__(self):

        super(TeachingCommandMachine, self).__init__(
            outcomes=TeachingCommandMachine.outcomes)

        askteachcommand_name='Which command ?'
        askteachcommand_state=SayState('Which command would you like to teach me ?')

        getteachcommand_name='Get Command'
        getteachcommand_state=GetTeachCommand()  #bidouillage

        confirmteachcommand_name='Right command ?'
        confirmteachcommand_state=SayState('Are you going to teach ') + getteachcommand_state.current_command())

        rw_name='Right or Wrong'
        rw_state=RightOrWrong()

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
                     transitions={GetTeachCommand.outcome_commandteached: confirmteachcommand_name})

            self.add(confirmteachcommand_name, confirmteachcommand_state,
                     transitions={SayState.outcome_success: rw_name})

            self.add(rw_name, rw_state,
                     transitions={RightOrWrong.outcome_success: startdemonstration_name,
                                  RightOrWrong.outcome_failure: askteachcommand_name,
                                  RightOrWrong.outcome_explanation: explanation_name})

            self.add(explanation_name, explanation_state,
                     transitions={SayState.outcome_success: rw_name})

            self.add(startdemonstration_name, startdemonstration_state,
                     transitions={SayState.outcome_success: demonstration_name})

            self.add(demonstration_name, demonstration_state,
                     transitions={Demonstration.outcome_demonstration: confirmation_name})

            self.add(confirmation_name, confirmation_state,
                     transitions={SayState.outcome_success: getconfirmation_name})

            self.add(getconfirmation_name, getconfirmation_state,
                     transitions={GetConfirmation.outcome_success: TeachingCommandMachine.outcome_success,
                                  GetConfirmation.outcome_failure: TeachingCommandMachine.outcome_failure,
                                  GetConfirmation.outcome_othertry: startdemonstration_name})



if __name__ == '__main__':

    import smach_ros
    rospy.init_node('interactive_demo')

    machine = TeachingCommandMachine()

    # Visualize the machine.
    machine_viz = smach_ros.IntrospectionServer(
        'smash_server', machine, '/SM_ROOT')
    machine_viz.start()

    outcome = machine.execute()

    rospy.loginfo('Outcome: {}'.format(outcome))

    machine_viz.stop()


