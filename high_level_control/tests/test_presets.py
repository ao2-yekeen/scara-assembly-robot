# tests/test_presets.py
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import unittest
from presets import PRESETS, apply_preset
from config import GRID_ROWS, GRID_COLS


class TestPresets(unittest.TestCase):

    def test_all_preset_keys_present(self):
        for key in ('1', '2', '3', '4', '5', '6', '7'):
            self.assertIn(key, PRESETS)

    def test_each_preset_has_name_and_callable_or_none(self):
        for key, (name, fn) in PRESETS.items():
            self.assertIsInstance(name, str)
            self.assertTrue(fn is None or callable(fn))

    def test_all_shape_presets_produce_non_empty_grid(self):
        for key, (name, fn) in PRESETS.items():
            if fn is None:
                continue
            grid = apply_preset(key)
            count = sum(v for row in grid for v in row)
            self.assertGreater(count, 0, f"Preset '{key}' ({name}) produced empty grid")

    def test_all_preset_grids_have_correct_dimensions(self):
        for key in PRESETS:
            grid = apply_preset(key)
            self.assertEqual(len(grid), GRID_ROWS,
                             f"Preset {key} wrong row count")
            for row in grid:
                self.assertEqual(len(row), GRID_COLS,
                                 f"Preset {key} wrong col count")

    def test_all_cells_are_zero_or_one(self):
        for key in PRESETS:
            grid = apply_preset(key)
            for row in grid:
                for v in row:
                    self.assertIn(v, (0, 1))

    def test_custom_preset_returns_blank_grid(self):
        grid = apply_preset('7')
        self.assertEqual(sum(v for row in grid for v in row), 0)


if __name__ == '__main__':
    unittest.main()
