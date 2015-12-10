import unittest
import rospy

from controller_demonstrations.robot_dialogue_interface.turtle_dialogue_interface import \
    TurtleDialogueInterface
from geometry_msgs.msg import Twist


class TestCase(unittest.TestCase):

    # A fake publisher.
    class MockPub(object):
        def __init__(self):
            self.num_pub = 0
            self.last_published = None

        def publish(self, data):
            self.num_pub += 1
            self.last_published = data

    def setUp(self):
        self.interface = TurtleDialogueInterface(default_speed=4.0)
        self.interface.fill_default_command_mappings()

    def test_get_command(self):
        cmd_to_execute = self.interface._known_commands['right']
        self.assertIsNotNone(cmd_to_execute)
        self.assertIsInstance(cmd_to_execute, list)
        self.assertEqual(1, len(cmd_to_execute))

        (twist_field, sign, duration) = cmd_to_execute[0]
        self.assertIsInstance(twist_field, basestring)
        self.assertIsInstance(sign, int)
        self.assertIsInstance(duration, rospy.Duration)

    def test_do_command_no_publisher(self):
        cmd_to_execute = self.interface._known_commands['right']
        ret = self.interface._robot_do_command(cmd_to_execute)
        self.assertEqual(False, ret)

    def test_do_command_mock_publisher(self):
        mock_pub = self.MockPub()
        self.interface.pub = mock_pub
        cmd_to_execute = self.interface._known_commands['up']
        ret = self.interface._robot_do_command(cmd_to_execute)
        self.assertEqual(True, ret)
        self.assertEqual(1, mock_pub.num_pub)

        ref_twist = Twist()
        ref_twist.linear.x = 4.0
        self.assertEqual(ref_twist, mock_pub.last_published)

    def test_execute_known_mock_publisher(self):
        mock_pub = self.MockPub()
        self.interface.pub = mock_pub
        ret = self.interface.execute_command('right')

        self.assertEqual(True, ret)
        self.assertEqual(1, mock_pub.num_pub)
        ref_twist = Twist()
        ref_twist.angular.z = -4.0

        self.assertEqual(ref_twist, mock_pub.last_published)

    def test_execute_known_no_publisher(self):
        # Returns false because there is no publisher.
        ret = self.interface.execute_command('right')
        self.assertEqual(False, ret)

    def test_execute_unknown(self):
        # Returns false because the command is unknown.
        ret = self.interface.execute_command('jibberish')
        self.assertEqual(False, ret)


if __name__ == '__main__':
    unittest.main()
