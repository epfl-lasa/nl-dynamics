#!/usr/bin/env python

from robot_dialogue_interface import RobotDialogueInterface
from geometry_msgs.msg import Twist
import rospy
import numpy as np


class TurtleDialogueInterface(RobotDialogueInterface):
    def __init__(self, default_speed=3):
        super(TurtleDialogueInterface, self).__init__()
        self._turtle_speed = default_speed
        self.pub = None

        self._recording = False
        self._recorded_data = []

    def connect(self, velocity_topic='turtle1/cmd_vel'):
        self.pub = rospy.Publisher(velocity_topic, Twist, queue_size=5)
        rospy.Subscriber(velocity_topic, Twist, self.twist_callback, queue_size=1)

    def twist_callback(self, data):
        """
        Records the received twist if the turtle is actively recording.
        Otherwise, returns.
        :param data: Received message.
        :return:
        """
        if not self._recording:
            return

        self._recorded_data.append(data)

    def fill_default_command_mappings(self):

        duration = rospy.Duration(0)
        # Each command is a tuple with the twist field to activate, a sign to
        # multiply the velocity, and a duration to sleep *after* the message is
        # published (the command goes out).
        self._known_commands['up'] = [('linear.x', 1, duration)]
        self._known_commands['down'] = [('linear.x', -1, duration)]
        self._known_commands['right'] = [('angular.z', -1, duration)]
        self._known_commands['left'] = [('angular.z', 1, duration)]

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

    @classmethod
    def twist_to_dict(cls, twist):
        """
        Converts a twist to a dictionary of field-value mappings.

        All fields will be contained in the dictionary.

        :param twist: Twist message.
        :return: Dictionary with {linear/angular}x{x/y/z} values.
        """
        d = {}
        fields = ['linear.x', 'linear.y', 'linear.z',
                  'angular.x', 'angular.y', 'angular.z']
        for f in fields:
            f_split = f.split('.')
            val = getattr(getattr(twist, f_split[0]), f_split[1])
            d[f] = val
        return d

    @classmethod
    def extract_twist_field(cls, twist):
        """
        Given a Twist with exactly one active element (i.e. non-zero), returns
        the string representation of the active field (e.g. 'linear.z') and the
        sign of the velocity (+1, -1).

        If the Twist does not have exactly one active (non-zero) field, returns
        None as the field and 0 as the sign.

        :param twist: ROS Twist message.
        :return: (field, sign) tuple.
        """
        field = None
        sign = 0

        field_vals = cls.twist_to_dict(twist)

        # Check for exactly one non-zero field: otherwise return (None, 0).
        num_nonzero = np.count_nonzero(field_vals.values())
        if num_nonzero != 1:
            return field, sign

        # Get the maximum absolute value element in the dict.
        field = max(field_vals.iterkeys(), key=lambda k: abs(field_vals[k]))
        sign = np.sign(field_vals[field])

        return field, sign

    def _robot_do_command(self, stuff_to_do, **kwargs):
        """
        Publish a list of time-Twist tuples out to the robot.

        Each tuple is a twist field accompanied by a Duration:
          - The twist field represents which field is active (e.g., 'linear.x')
          - The Duration is time to sleep *after* publishing the Twist.

        This creates a Twist message with the current velocity; each resulting
        Twist only has a single element 'active'.

        The final duration should be zero, but it is not ignored if it isn't.

        :param stuff_to_do: [(twist_field, duration), ...]
        :param kwargs:
        :return:
        """

        if not self.pub:
            return False

        for (field, sign, duration) in stuff_to_do:
            rospy.loginfo('Sending a twist for {} and pausing {}'.format(
                field, duration.to_sec()))

            assert isinstance(field, basestring), 'Must get a field name (str)'
            assert isinstance(duration, rospy.Duration), 'Must get a Duration'

            twist = self.make_twist([field], sign * self._turtle_speed)
            self.pub.publish(twist)

            rospy.sleep(duration)

        return True

    def _robot_set_speed(self, speed, **kwargs):
        self._turtle_speed = speed
        rospy.loginfo('Set turtle speed: {}'.format(self._turtle_speed))

    def _robot_record_command(self, *args, **kwargs):
        # Record a single Twist.
        rospy.loginfo('Recording command consisting of a single Twist')

        # Clear data and start recording
        self._recorded_data = []
        self._recording = True

        while len(self._recorded_data) < 1:
            rospy.sleep(0.1)

        self._recording = False
        twist = self._recorded_data[0]

        assert isinstance(twist, Twist)

        (field, sign) = self.extract_twist_field(twist)
        duration = rospy.Duration(0)

        rospy.loginfo('Recorded command: {} {} {}'.format(
            field, sign, duration.to_sec()))

        # Make sure to return a list of length 1.
        return [(field, sign, duration)]


def run():
    rospy.init_node('turtle_dialogue_interface', anonymous=False)
    rospy.loginfo('Turtlebot dialogue interface')

    interface = TurtleDialogueInterface()
    interface.connect()


    interface.fill_default_command_mappings()

    rospy.loginfo('All systems running')

    rospy.sleep(1)  # Wait for connections.
    #interface.execute_command('up')

    interface.record_command('dance')
    rospy.sleep(3)
    interface.execute_command('dance')

    #rospy.spin()


if __name__ == '__main__':
    run()
