__author__ = 'felixd'

import unittest

from interactive_demonstration.interactive_demo import ChangeSpeed

class TestStringToNum(unittest.TestCase):

    def setUp(self):
        self.change_speed = ChangeSpeed()

    def test_one(self):
        ret = self.change_speed.string_to_number('one')
        self.assertEqual(1, ret)


if __name__ == '__main__':
    unittest.main()
