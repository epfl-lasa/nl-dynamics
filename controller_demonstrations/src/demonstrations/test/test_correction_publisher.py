__author__ = 'felixd'

import unittest
from correction_publisher import PublishCorrections
import file_util

class TestDemoPublisher(unittest.TestCase):

    def setUp(self):
        demo_loc = file_util.get_fpath('test', 'test_data', relative=True)

        self.corr = PublishCorrections(demo_loc)

    def test_init(self):

        self.assertIsNotNone(self.corr)
        self.assertEqual(2, len(self.corr.words()))

    def test_words(self):
        words = self.corr.words()
        assert 'left' in words
        assert 'right' in words


if __name__ == '__main__':
    unittest.main()
