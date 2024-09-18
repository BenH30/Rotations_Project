import unittest
import numpy as np
from scipy.spatial.transform import Rotation as R
from main import compute_single_rotation, combine_rotations, plot_setup, plot_attitudes


class TestRotationFunctions(unittest.TestCase):

    def setUp(self):
        self.initial_attitude = np.array([10., 0., 0.])
        self.angle_dict_short = {1: [30, 0, 0]}
        self.angle_dict_long = {1: [30, 0, 0], 2: [100, 0, 0], 3: [60, 0, 0], 4: [42, 18, 77]}
        self.euler_sequence = 'ZYX'
        self.radians = False

    def test_varying_dictionary_lengths(self):
        # Short dictionary
        maneuvers, attitudes = combine_rotations(initial_attitude=self.initial_attitude,
                                                 angle_dictionary=self.angle_dict_short,
                                                 euler_angle_type='commanded_attitude',
                                                 euler_sequence=self.euler_sequence, degrees=self.radians)
        self.assertEqual(len(maneuvers), 1)
        self.assertEqual(len(attitudes), 2)

        # Long dictionary
        maneuvers, attitudes = combine_rotations(initial_attitude=self.initial_attitude,
                                                 angle_dictionary=self.angle_dict_long,
                                                 euler_angle_type='commanded_attitude',
                                                 euler_sequence=self.euler_sequence, degrees=self.radians)
        self.assertEqual(len(maneuvers), 4)
        self.assertEqual(len(attitudes), 5)

    def test_degrees_vs_radians(self):
        # Compute with degrees
        maneuvers_deg, attitudes_deg = combine_rotations(initial_attitude=self.initial_attitude,
                                                         angle_dictionary=self.angle_dict_short,
                                                         euler_angle_type='commanded_attitude',
                                                         euler_sequence=self.euler_sequence, degrees=True)

        # Convert to radians and compute
        self.initial_attitude_rad = np.radians(self.initial_attitude)
        angle_dict_rad = {k: np.radians(v) for k, v in self.angle_dict_short.items()}
        maneuvers_rad, attitudes_rad = combine_rotations(initial_attitude=self.initial_attitude_rad,
                                                         angle_dictionary=angle_dict_rad,
                                                         euler_angle_type='commanded_attitude',
                                                         euler_sequence=self.euler_sequence, degrees=False)

        # Convert results back to degrees for comparison
        attitudes_rad_deg = {k: np.degrees(v) for k, v in attitudes_rad.items()}
        for index in attitudes_deg:
            np.testing.assert_almost_equal(attitudes_deg[index], attitudes_rad_deg[index], decimal=5)

    def test_commanded_attitude_vs_maneuver(self):
        # First calculate using commanded attitude
        predicted_maneuvers, commanded_attitudes = combine_rotations(initial_attitude=self.initial_attitude,
                                                                     angle_dictionary=self.angle_dict_long,
                                                                     euler_angle_type='commanded_attitude',
                                                                     euler_sequence=self.euler_sequence,
                                                                     degrees=self.radians)

        # Now use the resulting maneuvers and try to recover original attitudes
        recovered_maneuvers, recovered_attitudes = combine_rotations(initial_attitude=self.initial_attitude,
                                                                     angle_dictionary=predicted_maneuvers,
                                                                     euler_angle_type='commanded_maneuver',
                                                                     euler_sequence=self.euler_sequence,
                                                                     degrees=self.radians)

        # Assert that the original and recovered attitudes are approximately equal
        for index in commanded_attitudes:
            np.testing.assert_almost_equal(commanded_attitudes[index], recovered_attitudes[index], decimal=5)


if __name__ == '__main__':
    unittest.main()
