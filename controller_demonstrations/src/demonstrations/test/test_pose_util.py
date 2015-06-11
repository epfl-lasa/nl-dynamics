#!/usr/bin/env python
PKG='test_foo'

import unittest

class TestDemonstrationPoseUtils(unittest.TestCase):
    def test_foo(self):
        print 'testing test'
        self.fail()

    def test_success(self):
        print 'success'
        pass;


if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, 'test_demonstration_pose_utils', TestDemonstrationPoseUtils)
