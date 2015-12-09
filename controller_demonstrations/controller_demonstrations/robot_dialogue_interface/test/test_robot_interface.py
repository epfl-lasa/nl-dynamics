import unittest
from robot_dialogue_interface.robot_dialogue_interface import \
    RobotDialogueInterface


class TestAbstractClass(unittest.TestCase):
    """
    Unit tests for the abstract dialogue interface class: mostly ensure things
    raise NotImplementedAssertions.
    """

    def setUp(self):
        self.interface = RobotDialogueInterface()
        self.command1 = 'unit_test'
        self.command2 = 'command'

    def test_execute_unknown(self):
        ret = self.interface.execute_command(self.command1)
        self.assertFalse(ret)

    def test_execute_known(self):
        self.interface._known_commands[self.command1] = 1
        self.assertRaises(NotImplementedError,
                          self.interface.execute_command, self.command1)

    def test_known_command_empty(self):
        known = self.interface.known_commands()
        self.assertIsNotNone(known)
        self.assertEqual([], known)

    def test_known_command_add(self):
        self.interface._known_commands[self.command1] = 1
        known = self.interface.known_commands()
        self.assertEqual(1, len(known))
        self.assertIn(self.command1, known)

        self.interface._known_commands[self.command2] = 2
        known = self.interface.known_commands()
        self.assertIsNotNone(known)
        self.assertEqual(2, len(known))
        self.assertIn(self.command1, known)
        self.assertIn(self.command2, known)

    def test_known_command_add_same_command_twice(self):
        self.interface._known_commands[self.command1] = 1
        known = self.interface.known_commands()
        self.assertIsNotNone(known)
        self.assertEqual(1, len(known))
        self.assertIn(self.command1, known)

        # Update command.
        self.interface._known_commands[self.command1] = 2
        known = self.interface.known_commands()
        self.assertIsNotNone(known)
        self.assertEqual(1, len(known))
        self.assertIn(self.command1, known)

    def test_change_speed(self):
        self.assertRaises(NotImplementedError, self.interface.change_speed, 0)

    def test_raise_do(self):
        self.assertRaises(NotImplementedError,
                          self.interface._robot_do_command)

    def test_raise_record(self):
        self.assertRaises(NotImplementedError,
                          self.interface._robot_record_command)

    def test_raise_set_speed(self):
        self.assertRaises(NotImplementedError, self.interface._robot_set_speed)


class BasicInterface(RobotDialogueInterface):
    """
    A trivial implementation of the robot dialogue interface for testing.
    """

    def __init__(self):
        super(BasicInterface, self).__init__()
        self._speed = 0
        self._execution_count = 0

    def _robot_do_command(self, *args, **kwargs):
        self._execution_count += 1
        return True

    def _robot_record_command(self, command, **kwargs):
        return [1]

    def _robot_set_speed(self, speed, **kwargs):
        self._speed = speed
        return True


class TestSimpleInterface(unittest.TestCase):
    """
    Unit tests for the trivial dialogue interface defined above.
    """

    def setUp(self):
        self.interface = BasicInterface()
        self.command1 = 'unit_test'
        self.command2 = 'command'

    def test_setup(self):
        self.assertEqual(0, self.interface._speed)
        self.assertEqual(0, self.interface._execution_count)

    def test_train_adds_known_command(self):
        ret = self.interface.record_command(self.command1)
        self.assertTrue(ret)
        known = self.interface.known_commands()
        self.assertEqual(1, len(known))
        self.assertIn(self.command1, known)

    def test_train_adds_two_known_commands(self):
        ret = self.interface.record_command(self.command1)
        ret = self.interface.record_command(self.command2)

        known = self.interface.known_commands()
        self.assertEqual(2, len(known))
        self.assertIn(self.command1, known)
        self.assertIn(self.command2, known)

    def test_train(self):
        self.interface.record_command(self.command1)
        self.assertIn(self.command1, self.interface._known_commands)
        stuff = self.interface._known_commands.get(self.command1)
        self.assertIsNotNone(stuff)
        self.assertEqual([1], stuff)

    def test_known_command_add(self):
        self.interface.record_command(self.command1)
        known = self.interface.known_commands()
        self.assertIsNotNone(known)
        self.assertEqual(1, len(known))
        self.assertIn(self.command1, known)

        self.interface.record_command(self.command2)
        known = self.interface.known_commands()
        self.assertIsNotNone(known)
        self.assertEqual(2, len(known))
        self.assertIn(self.command1, known)
        self.assertIn(self.command2, known)

    def test_known_command_add_same_command_twice(self):
        self.interface.record_command(self.command1)
        known = self.interface.known_commands()
        self.assertIsNotNone(known)
        self.assertEqual([self.command1], known)
        self.assertEqual(1, len(known))
        self.assertIn(self.command1, known)

        self.interface.record_command(self.command1)
        known = self.interface.known_commands()
        self.assertIsNotNone(known)
        self.assertEqual(1, len(known))
        self.assertIn(self.command1, known)

    def test_record_command_returns_true(self):
        ret = self.interface.record_command('stuff')
        self.assertTrue(ret)

    def test_record_command_twice(self):
        ret = self.interface.record_command('stuff')
        self.assertTrue(ret)
        ret = self.interface.record_command('stuff')
        self.assertTrue(ret)

    def test_record_command_returns_false(self):
        def record_fails(command):
            return False

        self.interface._robot_record_command = record_fails
        ret = self.interface.record_command(self.command1)
        self.assertFalse(ret)

        known = self.interface.known_commands()
        self.assertNotIn(self.command1, known)

    def test_execute_have_command(self):
        self.interface.record_command(self.command1)
        ret = self.interface.execute_command(self.command1)
        self.assertTrue(ret)
        self.assertEqual(1, self.interface._execution_count)

    def test_execute_unknown_command(self):
        self.interface.record_command(self.command1)
        ret = self.interface.execute_command(self.command2)
        self.assertFalse(ret)
        self.assertEqual(0, self.interface._execution_count)

    def test_change_speed(self):
        ret = self.interface.change_speed(5)
        self.assertEqual(True, ret)
        self.assertEqual(5, self.interface._speed)


if __name__ == '__main__':
    unittest.main()
