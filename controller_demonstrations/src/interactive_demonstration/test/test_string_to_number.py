__author__ = 'felixd'

import unittest

from interactive_demonstration.interactive_demo import ChangeSpeed

class TestStringToNum(unittest.TestCase):

    def setUp(self):
        self.change_speed = ChangeSpeed()

    def test_one(self):
        ret = self.change_speed.string_to_number('one')
        self.assertEqual(1, ret)

    def test_two(self): #always begin with test (nosetests to run)
        ret = self.change_speed.string_to_number('two')
        self.assertEqual(2, ret)

    def test_longer_sentence(self):
        ret = self.change_speed.string_to_number('there is a number eight here')
        self.assertEqual(8, ret)

    def test_empty_string(self):
    	ret = self.change_speed.string_to_number('')
    	self.assertEqual(None, ret)

    def test_nonumber_string(self):
    	ret = self.change_speed.string_to_number('I am happy')
    	self.assertEqual(None, ret)

if __name__ == '__main__':
    unittest.main()


#always begin with def test 
#To run this code from the terminal use : nosetests (nostests will run
# all the file with the strin test in the name which are in the directory we are executing the command)