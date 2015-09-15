__author__ = 'felixd'

import unittest

import send_one_demonstration
import send_multiple_demonstrations


class TestSendDemos(unittest.TestCase):

    def setUp(self):
        self.args_demo = ['--demo_dir', './test/test_data']


    def ztest_send_multiple_demos(self):
        send_multiple_demonstrations.run_send_multiple_commands(self.args_demo)

        pass

    def test_send_one_demo(self):
        command = 'left'
        command_args = ['command', command]
        send_one_demonstration.run_send_one_command(self.args_demo + command_args)

        
if __name__ == '__main__':
    unittest.main()
