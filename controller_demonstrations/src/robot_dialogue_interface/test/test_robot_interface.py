import unittest

from  robot_dialogue_interface.robot_dialogue_interface import RobotDialogueInterface


class TestAbstractClass(unittest.TestCase):
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

    def test_raise_do(self):
        self.assertRaises(NotImplementedError, self.interface._robot_do_command)

    def test_raise_record(self):
        self.assertRaises(NotImplementedError, self.interface._robot_record_command)

    def test_raise_set_speed(self):
        self.assertRaises(NotImplementedError, self.interface._robot_set_speed)



class BasicInterface(RobotDialogueInterface):
    def _robot_do_command(self, **kwargs):
        return True

    def _robot_record_command(self, command, **kwargs):
        self._known_commands[command] = [1]
        return True

    def _robot_set_speed(self, speed, **kwargs):
        return True


class TestSimpleInterface(unittest.TestCase):

    def setUp(self):
        self.interface = BasicInterface()
        self.command1 = 'unit_test'
        self.command2 = 'command'

    def train(self, cmd):
        self.interface.record_command(cmd)

    def test_train(self):

        self.fail()

    def test_known_command_add(self):
        self.train(self.command1)
        known = self.interface.known_commands()
        self.assertIsNotNone(known)
        self.assertEqual([self.command1], known)

        self.train(self.command2)
        known = self.interface.known_commands()
        self.assertIsNotNone(known)
        self.assertEqual([self.command2, self.command1], known)

    def test_known_command_add_same_command_twice(self):
        self.train(self.command1)
        known = self.interface.known_commands()
        self.assertIsNotNone(known)
        self.assertEqual([self.command1], known)

        self.train(self.command1)
        known = self.interface.known_commands()
        self.assertIsNotNone(known)
        self.assertEqual([self.command1], known)


    def test_execute(self):
        self.train(self.command1)
        self.fail()


if __name__ == '__main__':
    unittest.main()
