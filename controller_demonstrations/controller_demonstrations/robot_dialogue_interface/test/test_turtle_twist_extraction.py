import unittest
from controller_demonstrations.robot_dialogue_interface.turtle_dialogue_interface import \
    TurtleDialogueInterface
from geometry_msgs.msg import Twist


class TestTwistExtraction(unittest.TestCase):

    def setUp(self):
        self.twistX = Twist()
        self.twistX.linear.x = 2

        self.twistY = Twist()
        self.twistY.linear.y = -4.2

        self.twistZ = Twist()
        self.twistZ.angular.z = 0.001

        self.twistZero = Twist()

        self.twistMultiple = Twist()
        self.twistMultiple.linear.x = 0.4
        self.twistMultiple.angular.y = -2.0

    def test_extract_field_positive(self):
        (field, sign) = TurtleDialogueInterface.extract_twist_field(self.twistX)
        self.assertEqual('linear.x', field)
        self.assertEqual(1, sign)

    def test_extract_field_negativen(self):
        (field, sign) = TurtleDialogueInterface.extract_twist_field(self.twistY)
        self.assertEqual('linear.y', field)
        self.assertEqual(-1, sign)

    def test_extract_field_angular(self):
        (field, sign) = TurtleDialogueInterface.extract_twist_field(self.twistZ)
        self.assertEqual('angular.z', field)
        self.assertEqual(1, sign)

    def test_extract_field_zero(self):
        (field, sign) = TurtleDialogueInterface.extract_twist_field(
            self.twistZero)
        self.assertEqual(None, field)
        self.assertEqual(0, sign)

    def test_extract_field_multiple(self):
        (field, sign) = TurtleDialogueInterface.extract_twist_field(
            self.twistMultiple)
        self.assertEqual(None, field)
        self.assertEqual(0, sign)


    def test_twist_to_dict(self):
        ret = TurtleDialogueInterface.twist_to_dict(self.twistX)
        self.assertIsNotNone(ret)

        self.assertEqual(6, len(ret))
        self.assertEqual(2.0, ret['linear.x'])
        other_fields = ['linear.y', 'linear.z',
                        'angular.x', 'angular.y', 'angular.z']
        for f in other_fields:
            self.assertEqual(0.0, ret[f])

    def test_twist_to_dict_multiple(self):
        ret = TurtleDialogueInterface.twist_to_dict(self.twistMultiple)
        self.assertIsNotNone(ret)

        self.assertEqual(6, len(ret))
        self.assertEqual(0.4, ret['linear.x'])
        self.assertEqual(-2.0, ret['angular.y'])

        other_fields = ['linear.y', 'linear.z', 'angular.x', 'angular.z']
        for f in other_fields:
            self.assertEqual(0.0, ret[f])
