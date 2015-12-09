import unittest

from controller_demonstrations.robot_dialogue_interface.kuka_dialogue_interface import \
    KukaDialogueInterface


class TestKukaInterface(unittest.TestCase):

    def setUp(self):
        self.interface = KukaDialogueInterface()

    def test_demonstration_linkage(self):
        # Make sure adding a demonstration in the dialogue interface also adds
        # it to the correction publisher's internal store.

        # Add a demonstration to the dialogue interface

        # Make sure the correction publishder also has it.
        #  TODO self.fail()
        pass


if __name__ == '__main__':
    unittest.main()
