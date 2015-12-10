__author__ = 'cgschmit'

import unittest

from controller_demonstrations.interactive_demonstration.branch_gettingcommand import GetCommand
import rospy

class TestCommandInDictionnary(unittest.TestCase):

    def setUp(self):
        self.in_dictionnary = GetCommand(['faster', 'slower'])

    def test_one(self):
        ret = self.in_dictionnary.command_in_dictionnary('one')
        self.assertEqual(None, ret)

    def test_in_dictionnary(self): #revoir
        ret = self.in_dictionnary.command_in_dictionnary('faster')
        self.assertEqual('faster', ret)

    def test_in_dictionnary_uppercase(self): #revoir
        ret = self.in_dictionnary.command_in_dictionnary('FasteR')
        self.assertEqual(None, ret)

    def test_in_sentence(self): #revoir
        ret = self.in_dictionnary.command_in_dictionnary('Go faster please !')
        self.assertEqual('faster', ret)

    def test_empty(self):
        ret = self.in_dictionnary.command_in_dictionnary('')
        self.assertEqual(None, ret)

    def test_slower(self):
        ret = self.in_dictionnary.command_in_dictionnary('go slower please !')
        self.assertEqual('slower', ret)

    def test_both(self):
        # Note we will return the first.
        ret = self.in_dictionnary.command_in_dictionnary('go faster then slower')
        self.assertEqual('faster', ret)

    def test_several_word_commands(self):
        # Our algorithm should work for multi-word tokens.
        cmd = GetCommand(['go up', 'down'])
        ret = cmd.command_in_dictionnary('go up')
        self.assertEqual('go up', ret)

if __name__ == '__main__':
    unittest.main()


#always begin with def test

#To run this code from the terminal use : nosetests (nostests will run
# all the file with the strin test in the name which are in the directory
#we are executing the command)
