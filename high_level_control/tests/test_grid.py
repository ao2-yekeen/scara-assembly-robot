# tests/test_grid.py
import sys, os, tempfile
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import unittest
from grid import (
    empty_grid, grid_to_positions, positions_to_grid,
    toggle_cell, load_floor_plan, save_floor_plan,
)
from config import GRID_ROWS, GRID_COLS


class TestEmptyGrid(unittest.TestCase):

    def test_has_correct_number_of_rows(self):
        self.assertEqual(len(empty_grid()), GRID_ROWS)

    def test_has_correct_number_of_cols(self):
        self.assertEqual(len(empty_grid()[0]), GRID_COLS)

    def test_all_cells_are_zero(self):
        grid = empty_grid()
        self.assertEqual(sum(v for row in grid for v in row), 0)

    def test_returns_independent_grid_each_call(self):
        g1, g2 = empty_grid(), empty_grid()
        g1[0][0] = 1
        self.assertEqual(g2[0][0], 0)


class TestGridToPositions(unittest.TestCase):

    def test_empty_grid_returns_empty_list(self):
        self.assertEqual(grid_to_positions(empty_grid()), [])

    def test_single_cell_returns_correct_col_row(self):
        grid = empty_grid()
        grid[2][3] = 1
        self.assertEqual(grid_to_positions(grid), [(3, 2)])

    def test_multiple_cells_returns_all_positions(self):
        grid = empty_grid()
        grid[0][0] = 1
        grid[1][2] = 1
        positions = grid_to_positions(grid)
        self.assertIn((0, 0), positions)
        self.assertIn((2, 1), positions)
        self.assertEqual(len(positions), 2)


class TestPositionsToGrid(unittest.TestCase):

    def test_roundtrip_preserves_positions(self):
        original = [(1, 2), (3, 4), (0, 0)]
        result = grid_to_positions(positions_to_grid(original))
        self.assertEqual(sorted(result), sorted(original))

    def test_out_of_bounds_positions_are_ignored(self):
        grid = positions_to_grid([(999, 999), (0, 0)])
        self.assertEqual(grid_to_positions(grid), [(0, 0)])

    def test_empty_list_returns_empty_grid(self):
        grid = positions_to_grid([])
        self.assertEqual(sum(v for row in grid for v in row), 0)

    def test_duplicate_positions_do_not_raise(self):
        grid = positions_to_grid([(0, 0), (0, 0)])
        self.assertEqual(grid[0][0], 1)


class TestToggleCell(unittest.TestCase):

    def test_toggle_zero_becomes_one(self):
        grid = empty_grid()
        toggle_cell(grid, 1, 1)
        self.assertEqual(grid[1][1], 1)

    def test_toggle_one_becomes_zero(self):
        grid = empty_grid()
        grid[1][1] = 1
        toggle_cell(grid, 1, 1)
        self.assertEqual(grid[1][1], 0)

    def test_double_toggle_returns_to_original(self):
        grid = empty_grid()
        toggle_cell(grid, 0, 0)
        toggle_cell(grid, 0, 0)
        self.assertEqual(grid[0][0], 0)


class TestFloorPlanIO(unittest.TestCase):

    def test_save_and_load_roundtrip(self):
        grid = empty_grid()
        grid[1][2] = 1
        grid[3][4] = 1
        with tempfile.NamedTemporaryFile(suffix='.txt', delete=False) as f:
            fname = f.name
        try:
            save_floor_plan(grid, fname)
            loaded = load_floor_plan(fname)
            self.assertIn((2, 1), loaded)
            self.assertIn((4, 3), loaded)
        finally:
            os.unlink(fname)

    def test_load_missing_file_calls_sys_exit(self):
        with self.assertRaises(SystemExit):
            load_floor_plan("this_file_does_not_exist_xyz.txt")

    def test_saved_file_has_correct_dimensions(self):
        grid = empty_grid()
        with tempfile.NamedTemporaryFile(suffix='.txt', delete=False, mode='w') as f:
            fname = f.name
        try:
            save_floor_plan(grid, fname)
            with open(fname) as f:
                lines = f.readlines()
            self.assertEqual(len(lines), GRID_ROWS)
            self.assertEqual(len(lines[0].strip().split()), GRID_COLS)
        finally:
            os.unlink(fname)


if __name__ == '__main__':
    unittest.main()
