#!/usr/bin/env python

from robot_dialogue_interface import RobotDialogueInterface
from geometry_msgs.msg import Twist
import rospy


class TurtleDialogueInterface(RobotDialogueInterface):
    def __init__(self, default_speed=3):
        super(TurtleDialogueInterface, self).__init__()
        self._turtle_speed = default_speed
        self.pub = None

    def connect(self, topic='turtle1/cmd_vel'):
        self.pub = rospy.Publisher(topic, Twist, queue_size=5)

    def fill_default_command_mappings(self):

        up_twist = self.make_twist(['linear.x'], self._turtle_speed)
        self._known_commands['up'] = up_twist

        down_twist = self.make_twist(['linear.x'], -1 * self._turtle_speed)
        self._known_commands['down'] = down_twist

        right_twist = self.make_twist(['angular.z'], -1 * self._turtle_speed)
        self._known_commands['right'] = right_twist

        left_twist = self.make_twist(['angular.z'], self._turtle_speed)
        self._known_commands['left'] = left_twist

    @classmethod
    def make_twist(cls, fields_list, velocity):
        """
        Generate a Twist() where several fields are set to the given velocity.

        :param fields_list: List of fields that will all have the given
        velocity. Other fields will have a default (zero) value.
        :param velocity: Scalar velocity
        :return: A new twist object.
        """

        assert isinstance(fields_list, list), 'Must provide list of fields.'

        twist = Twist()
        for field in fields_list:
            cls.set_twist_field(twist, field, velocity)
        return twist

    @classmethod
    def set_twist_field(cls, twist, field, velocity):
        """
        Set a specific field of a Twist() to a given velocity.

        :param twist: The twist object, modified *in place*
        :param field: The field as a string, e.g., 'linear.x' or 'angular.y'
        :param velocity: The velocity to set the field.
        :return: nothing.
        """
        nested_fields = field.split('.')
        assert len(nested_fields) == 2, \
            ('Must have exactly two fields for '
             'generating the Twist, have {}'.format(field))

        if not twist or not isinstance(twist, Twist):
            assert False, 'Wrong type: {}'.format(twist)

        inner = nested_fields[0]
        outer = nested_fields[1]

        assert hasattr(twist, inner), 'Wrong fields'
        assert hasattr(getattr(twist, inner), outer), 'Wrong fields'

        setattr(getattr(twist, inner), outer, velocity)

    def _robot_do_command(self, command, **kwargs):

        if not self.pub:
            return False

        self.pub.publish(command)
        return True

    def _robot_set_speed(self, speed, **kwargs):
        self._turtle_speed = speed

    def _robot_record_command(self, *args, **kwargs):
        pass


def run():
    rospy.init_node('turtle_dialogue_interface', anonymous=False)
    rospy.loginfo('Robot dialogue interface')

    interface = TurtleDialogueInterface()
    interface.connect()
    interface.fill_default_command_mappings()

    rospy.loginfo('All systems running')
    rospy.spin()


if __name__ == '__main__':
    run()
