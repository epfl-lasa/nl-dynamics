import unittest

from nl_msgs.msg import AnchoredDemonstration
from nl_msgs.srv import DemonstrationResponse


class TestCase(unittest.TestCase):

    def setUp(self):
        pass

    def test_create_response_fail(self):
        response = DemonstrationResponse(success=False, demonstration=None)
        self.assertIsNotNone(response)

    def test_create_response_success(self):
        demo = AnchoredDemonstration()
        response = DemonstrationResponse(success=False, demonstration=demo)
        self.assertIsNotNone(response)


if __name__ == '__main__':
    unittest.main()
