__author__ = 'felixd'

import unittest
import numpy as np

from gpmds import GPMDS


class MyTestCase(unittest.TestCase):

    def setUp(self):
        self._pose = np.array((5, 2))
        self._velocity = np.array((1.0, 1.0))

        # GP hyper parameters
        (ell, sigmaF, sigmaN) = (30, 1.0, 0.4)
        self._gp = GPMDS.GPMDS(ell, sigmaF, sigmaN, velocity_cap=(-1e12, 1e12))

        dynamics = GPMDS.originalDynamicsLinear
        self._gp.setOriginalDynamics(dynamics)

    def test_dynamics(self):
        out = GPMDS.originalDynamicsLinear(self._pose)
        ref = (2,)
        self.assertEqual(ref, out.shape)

    def test_vcap(self):
        self.assertEqual(1e12, self._gp.v_capHigh)
        self.assertEqual(-1e12, self._gp.v_capLow)

    def test_no_vcap(self):
        self._gp = GPMDS.GPMDS(0, 0, 0)
        self.assertEqual(1.0, self._gp.v_capHigh)
        self.assertEqual(0.1, self._gp.v_capLow)

    def test_add_data(self):
        self._gp.addData(self._pose, self._velocity)
        self.assertEqual(1, self._gp.mGPR.nData)

    def test_reshaped_without_prepared(self):
        self._gp.addData(self._pose, self._velocity)

        with self.assertRaisesRegexp(AssertionError, 'prepareRegression'):
            self._gp.reshapedDynamics(self._pose)

    def test_reshaped_no_data(self):
        ref = self._gp.originalDynamics(self._pose)
        out = self._gp.reshapedDynamics(self._pose)

        self.assertEqual(0, self._gp.mGPR.nData)
        self.assertTrue(np.array_equal(ref, out))

    def test_reshaped_dynamics(self):
        self._gp.addData(self._pose, self._velocity)
        self._gp.addData(np.array((4., 1.)), np.array((1, 2)))
        self._gp.addData(np.array((4.5, 2)), np.array((1, -1)))
        self._gp.mGPR.prepareRegression()

        out = self._gp.reshapedDynamics(self._pose)
        self.assertAlmostEqual(2.94708499418, np.linalg.norm(out))


if __name__ == '__main__':
    unittest.main()
