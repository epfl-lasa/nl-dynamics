import unittest
from robot_dialogue_interface.turtle_dialogue_interface import \
    TurtleDialogueInterface
from geometry_msgs.msg import Twist


class TestMessageGeneration(unittest.TestCase):

    def setUp(self):
        pass

    def test_make_twist(self):
        fields = ['linear.x', 'angular.y']
        vel = 2.0

        twist = TurtleDialogueInterface.make_twist(fields, vel)
        ref = Twist()
        ref.linear.x = 2.0
        ref.angular.y = 2.0
        self.assertEqual(ref, twist)

    def test_make_twist_not_list(self):

        field = 'linear.x'
        with self.assertRaisesRegexp(AssertionError,
                                     'Must provide list of fields'):
            twist = TurtleDialogueInterface.make_twist(field, 2.0)

    def test_set_twist_field_1(self):
        field = 'linear.x'
        vel = 3.0
        twist = Twist()
        TurtleDialogueInterface.set_twist_field(twist, field, vel)

        ref = Twist()
        ref.linear.x = 3.0

        self.assertEqual(ref, twist)

    def test_set_twist_field_2(self):
        field = 'angular.z'
        vel = -4.0
        twist = Twist()
        TurtleDialogueInterface.set_twist_field(twist, field, vel)

        ref = Twist()
        ref.angular.z = -4.0

        self.assertEqual(ref, twist)

    def test_set_twist_wrong_number_of_fields(self):
        bad_field = 'linear'
        twist = Twist()

        with self.assertRaisesRegexp(AssertionError, 'Must have exactly two'):
            TurtleDialogueInterface.set_twist_field(twist, bad_field, 3.0)

    def test_set_twist_bad_fields(self):
        bad_field = 'linear.alpha'
        twist = Twist()

        with self.assertRaisesRegexp(AssertionError, 'Wrong fields'):
            TurtleDialogueInterface.set_twist_field(twist, bad_field, 3.0)

    def test_set_twist_null_object(self):
        with self.assertRaisesRegexp(AssertionError, 'Wrong type'):
            TurtleDialogueInterface.set_twist_field(None, 'linear.x', 3.0)

    def test_set_twist_bad_type(self):
        other = []
        with self.assertRaisesRegexp(AssertionError, 'Wrong type'):
            TurtleDialogueInterface.set_twist_field(other, 'linear.x', 3.0)

    def test_fill_fields(self):
        interface = TurtleDialogueInterface()
        interface.fill_default_command_mappings()

        known = interface.known_commands()
        self.assertTrue(known is not None)
        self.assertTrue('right' in known)
        self.assertTrue('left' in known)
        self.assertTrue('up' in known)
        self.assertTrue('down' in known)
