#!/usr/bin/env python

from robot_dialogue_interface import RobotDialogueInterface
from geometry_msgs.msg import Twist


class TurtleDialogueInterface(RobotDialogueInterface):
    def __init__(self, default_speed=3):
        super(TurtleDialogueInterface, self).__init__()
        self._turtle_speed = default_speed

    def fill_default_command_mappings(self):
        pass

    @classmethod
    def make_twist(cls, fields_list, velocity):
        twist = Twist()
        for field in fields_list:
            cls.set_twist_field(twist, field, velocity)
        return twist

    @classmethod
    def set_twist_field(cls, twist, field, velocity):
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

        pass

    def _robot_set_speed(self, speed, **kwargs):
        self._turtle_speed = speed

    def _robot_record_command(self, *args, **kwargs):
        pass


def run():
    print 'Robot dialogue interface'
    interface = TurtleDialogueInterface()

if __name__ == '__main__':
    run()