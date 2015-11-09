#! /usr/bin/env python

import rospy
import smach
import std_msgs

from say_state import SayState

class ChangeSpeed(smach.State):
    outcome_speedchanged = 'speedchanged'
    outcomes = [outcome_speedchanged]

    def __init__(self):
        # specify the outcomes
        smach.State.__init__(self, outcomes=ChangeSpeed.outcomes)
        # Subscribe to a Topic
        topic_sub = '/nl_command_parsed'
        rospy.Subscriber(topic_sub, std_msgs.msg.String, self.callback, queue_size=1)
        topic_pub = 'robot_control/desired_velocity'
        self.pub = rospy.Publisher(topic_pub, std_msgs.msg.Int8, queue_size=5)
        # internal data
        self.msg = ''


        # This method does not change the class members directly.

    def string_to_number(self, message):
        number_dict = dict(one=1, two=2, three=3, four=4, five=5, six=6, seven=7, eight=8,
                 nine=9, ten=10, eleven=11)
        b = -1
        msg_split = message.split() # Separe la string par mot (separateur *espace*)
        length_msg = len(msg_split)  # Retourne le nombre de mots dans la string
        for i in range(length_msg):  # Parcoure chaque mot
            if (msg_split[i] in number_dict.keys()):  # Si un des mots est dans la string msg
                b = i  # Alors il donne la place du mot dans la string
        if (b >= 0):
            new_speed = number_dict.get(msg_split[b])  # new_speed va contenir la valeur du dictionnaire se trouvant a la position b
            return new_speed
        else:
            return None

    def execute(self, userdata):
        rospy.loginfo('Executing ChangeSpeed')
        # Will change the speed of the robot
        speed_integer=None
        while (speed_integer == None):
            speed_integer = self.string_to_number(self.msg)
            rospy.sleep(0.1)
        self.pub.publish(speed_integer)
        rospy.loginfo('New Speed is %s', speed_integer)
        return ChangeSpeed.outcome_speedchanged

    def callback(self, data):
        self.msg = data.data  # data.data == self.msg de ReadyState




class ChangingSpeedBranch(smach.StateMachine):
    outcome_success = 'success'
    outcomes = [outcome_success]

    def __init__(self):

        super(ChangingSpeedBranch, self).__init__(
            outcomes=ChangingSpeedBranch.outcomes)

        askingspeed_state = SayState('At which speed do you want me to go ?')
        askingspeed_name = 'ASKING_SPEED'

        speedchanged_state = ChangeSpeed()
        speedchanged_name = 'CHANGED_SPEED'

        validatespeed_state = SayState('New Velocity implemented')
        validatespeed_name = 'VALIDATE_SPEED'

        with self:
            self.add(askingspeed_name, askingspeed_state,
                     transitions={SayState.outcome_success: speedchanged_name})

            self.add(speedchanged_name, speedchanged_state,
                     transitions={
                         ChangeSpeed.outcome_speedchanged: validatespeed_name})

            self.add(validatespeed_name, validatespeed_state,
                     transitions={SayState.outcome_success: hw_name})

if __name__ == '__main__':
    import smach_ros
    rospy.init_node('interactive_demo')
    machine=ChangingSpeedBranch()
