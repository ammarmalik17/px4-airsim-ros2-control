import math
import unittest

from airsim_offboard.velocity import decay_velocity, step_axis, wrap_yaw


class TestVelocityHelpers(unittest.TestCase):

    def test_decay_velocity_stops_at_zero_without_sign_flip(self):
        self.assertAlmostEqual(decay_velocity(0.3, 0.05), 0.25)
        self.assertEqual(decay_velocity(0.05, 0.05), 0.0)   # exact stop, no negative residue
        self.assertEqual(decay_velocity(-0.05, 0.05), 0.0)  # symmetric on the negative side
        self.assertEqual(decay_velocity(0.0, 0.05), 0.0)

    def test_step_axis_clamps_at_limits(self):
        self.assertEqual(step_axis(4.8, 1, 0.5, 5.0), 5.0)    # horizontal limit
        self.assertEqual(step_axis(-4.8, -1, 0.5, 5.0), -5.0)
        self.assertEqual(step_axis(0.0, 1, 0.5, 10.0), 0.5)   # vertical limit is wider
        self.assertEqual(step_axis(9.9, 1, 0.5, 10.0), 10.0)

    def test_wrap_yaw_stays_within_pi(self):
        self.assertAlmostEqual(wrap_yaw(math.pi + 0.1), -math.pi + 0.1)
        self.assertAlmostEqual(wrap_yaw(-math.pi - 0.1), math.pi - 0.1)
        self.assertEqual(wrap_yaw(0.0), 0.0)
        self.assertGreaterEqual(wrap_yaw(7 * math.pi), -math.pi)
        self.assertLessEqual(wrap_yaw(7 * math.pi), math.pi)


if __name__ == '__main__':
    unittest.main()
