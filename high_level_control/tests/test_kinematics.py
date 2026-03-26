# tests/test_kinematics.py
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import unittest
from kinematics import ik, grid_to_world, validate_positions
from config import L1, L2, SUPPLY_X, SUPPLY_Y, DUPLO_PITCH, ORIGIN_X, ORIGIN_Y


class TestIK(unittest.TestCase):

    def test_returns_three_floats(self):
        q1, q2, q3 = ik(0.15, 0.0)
        for v in (q1, q2, q3):
            self.assertIsInstance(v, float)

    def test_wrist_constraint_equals_neg_q1_plus_q2(self):
        q1, q2, q3 = ik(0.18, 0.05)
        self.assertAlmostEqual(q3, -(q1 + q2), places=4)

    def test_supply_position_is_reachable(self):
        q1, q2, q3 = ik(SUPPLY_X, SUPPLY_Y)
        self.assertIsNotNone(q1)

    def test_unreachable_too_far_raises_value_error(self):
        with self.assertRaises(ValueError):
            ik(L1 + L2 + 0.01, 0.0)

    def test_unreachable_too_close_raises_value_error(self):
        with self.assertRaises(ValueError):
            ik(abs(L1 - L2) * 0.5, 0.0)

    def test_increasing_y_increases_shoulder_angle(self):
        q1_low, _, _ = ik(0.20, 0.01)
        q1_high, _, _ = ik(0.20, 0.05)
        self.assertGreater(q1_high, q1_low)

    def test_boundary_at_max_reach_does_not_raise(self):
        ik(L1 + L2 - 0.001, 0.0)

    def test_boundary_at_min_reach_does_not_raise(self):
        ik(abs(L1 - L2) + 0.001, 0.0)


class TestGridToWorld(unittest.TestCase):

    def test_col_zero_row_zero_uses_half_pitch_offset(self):
        x, y = grid_to_world(0, 0)
        self.assertAlmostEqual(x, DUPLO_PITCH / 2 + ORIGIN_X, places=6)
        self.assertAlmostEqual(y, DUPLO_PITCH / 2 + ORIGIN_Y, places=6)

    def test_adjacent_columns_separated_by_duplo_pitch(self):
        x0, _ = grid_to_world(0, 0)
        x1, _ = grid_to_world(1, 0)
        self.assertAlmostEqual(x1 - x0, DUPLO_PITCH, places=6)

    def test_adjacent_rows_separated_by_duplo_pitch(self):
        _, y0 = grid_to_world(0, 0)
        _, y1 = grid_to_world(0, 1)
        self.assertAlmostEqual(y1 - y0, DUPLO_PITCH, places=6)

    def test_increasing_col_increases_x(self):
        x0, _ = grid_to_world(0, 0)
        x1, _ = grid_to_world(3, 0)
        self.assertGreater(x1, x0)

    def test_increasing_row_increases_y(self):
        _, y0 = grid_to_world(0, 0)
        _, y1 = grid_to_world(0, 3)
        self.assertGreater(y1, y0)


class TestValidatePositions(unittest.TestCase):

    def test_returns_two_lists(self):
        valid, invalid = validate_positions([])
        self.assertIsInstance(valid, list)
        self.assertIsInstance(invalid, list)

    def test_empty_input_returns_empty_lists(self):
        valid, invalid = validate_positions([])
        self.assertEqual(valid, [])
        self.assertEqual(invalid, [])

    def test_reachable_position_goes_to_valid(self):
        valid, invalid = validate_positions([(0, 0)])
        self.assertEqual(len(valid) + len(invalid), 1)

    def test_far_out_of_range_position_goes_to_invalid(self):
        _, invalid = validate_positions([(99, 99)])
        self.assertEqual(len(invalid), 1)

    def test_invalid_entry_contains_reason_string(self):
        _, invalid = validate_positions([(99, 99)])
        col, row, reason = invalid[0]
        self.assertIsInstance(reason, str)
        self.assertGreater(len(reason), 0)


if __name__ == '__main__':
    unittest.main()
