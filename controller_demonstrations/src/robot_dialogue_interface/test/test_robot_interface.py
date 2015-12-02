import unittest

from  robot_dialogue_interface.robot_dialogue_interface import RobotDialogueInterface

class TestAbstractClass(unittest.TestCase):
    def test_something(self):

        self.assertEqual(True, False)

class TestSimpleInterface(unittest.TestCase):
    def test_something(self):
        self.fail()

if __name__ == '__main__':
    unittest.main()
