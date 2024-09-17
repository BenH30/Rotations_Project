import unittest
import numpy as np
from scipy.spatial.transform import Rotation as R
from main import compute_attitude, compute_maneuver, plot_setup

class TestRotationFunctions(unittest.TestCase):

    def test_euler_to_matrix(self):
        # Test conversion from Euler angles to rotation matrix
        angles = [45, 30, 60]
        rotation = R.from_euler('ZYX', angles, degrees=True)
        matrix = rotation.as_matrix()

        expected_matrix = np.array([
            [0.6123725, -0.0473672,  0.7891491],
            [0.6123725,  0.6597396, -0.4355958],
            [-0.5000000,  0.7500000,  0.4330127]
        ])
        np.testing.assert_array_almost_equal(matrix, expected_matrix, decimal=5)

    def test_compute_attitude_single(self):
        # Test the compute_attitude function with a single attitude
        single_angle = {0: [0, 0, 0]}
        try:
            compute_attitude(single_angle, 'ZYX', True)
        except Exception as e:
            self.fail(f"compute_attitude raised an exception with a single input: {e}")

    def test_compute_maneuver_single(self):
        # Test the compute_maneuver function with a single attitude
        single_attitude = {0: [0, 0, 0]}
        try:
            compute_maneuver(single_attitude, 'ZYX', True)
        except Exception as e:
            self.fail(f"compute_maneuver raised an exception with a single input: {e}")

    def test_maneuver_angles(self):
        # Test that maneuver angles are calculated correctly
        pre_attitude = R.from_euler('ZYX', [0, 0, 0], degrees=True)
        post_attitude = R.from_euler('ZYX', [45, 0, 0], degrees=True)
        maneuver = post_attitude * pre_attitude.inv()
        maneuver_angles = maneuver.as_euler('ZYX', degrees=True)

        expected_maneuver_angles = [45, 0, 0]
        np.testing.assert_array_almost_equal(maneuver_angles, expected_maneuver_angles, decimal=5)

    def test_invalid_inputs(self):
        # Test that invalid inputs are handled (e.g., wrong dimension or non-numeric input)
        with self.assertRaises(ValueError):
            invalid_angle = {0: ['invalid', 0, 0]}  # Non-numeric input
            compute_attitude(invalid_angle, 'ZYX', True)

if __name__ == '__main__':
    unittest.main()
