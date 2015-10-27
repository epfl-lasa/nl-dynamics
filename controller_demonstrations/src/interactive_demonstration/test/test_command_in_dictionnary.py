__author__ = 'cgschmit'

import unittest

from interactive_demonstration.interactive_demo import GetCommand

class TestCommandInDictionnary(unittest.TestCase):

    def setUp(self):
        self.in_dictionnary = GetCommand()

    def test_one(self):
        ret = self.in_dictionnary.command_in_dictionnary('one')
        self.assertEqual(True, ret)




if __name__ == '__main__':
    unittest.main()


#always begin with def test 

#To run this code from the terminal use : nosetests (nostests will run
# all the file with the strin test in the name which are in the directory 
#we are executing the command)