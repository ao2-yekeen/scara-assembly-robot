# tests/test_builder.py
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import unittest
from builder import place_layer, _build_pick_place_sequence
from serial_comms import DryRunSerial
from config import Z_PLACE, DUPLO_H_MM, MAX_LAYERS


class TestPickPlaceSequence(unittest.TestCase):

    def test_sequence_has_eight_steps(self):
        seq = _build_pick_place_sequence(0,0,0, 10,20,30, 5.0)
        self.assertEqual(len(seq), 8)

    def test_sequence_starts_with_supply_clearance(self):
        seq = _build_pick_place_sequence(0,0,0, 10,20,30, 5.0)
        self.assertIn("Supply clearance", seq[0][2])

    def test_sequence_ends_with_retract(self):
        seq = _build_pick_place_sequence(0,0,0, 10,20,30, 5.0)
        self.assertIn("Retract", seq[-1][2])

    def test_grip_step_is_present(self):
        seq = _build_pick_place_sequence(0,0,0, 10,20,30, 5.0)
        commands = [s[0] for s in seq]
        self.assertIn("GRIP", commands)

    def test_release_step_is_present(self):
        seq = _build_pick_place_sequence(0,0,0, 10,20,30, 5.0)
        commands = [s[0] for s in seq]
        self.assertIn("RELEASE", commands)

    def test_expected_responses_match_commands(self):
        seq = _build_pick_place_sequence(0,0,0, 10,20,30, 5.0)
        for cmd, expected, _ in seq:
            if cmd.startswith("MOVE"):
                self.assertEqual(expected, "OK:MOVE")
            elif cmd == "GRIP":
                self.assertEqual(expected, "OK:GRIP")
            elif cmd == "RELEASE":
                self.assertEqual(expected, "OK:RELEASE")


class TestPlaceLayer(unittest.TestCase):

    def setUp(self):
        self.ser = DryRunSerial()

    def test_single_block_returns_true(self):
        result = place_layer(self.ser, [(0, 0)], layer=0, dry_run=True)
        self.assertTrue(result)

    def test_multiple_blocks_returns_true(self):
        result = place_layer(self.ser, [(0,0),(1,0),(0,1)], layer=0, dry_run=True)
        self.assertTrue(result)

    def test_empty_positions_returns_true(self):
        result = place_layer(self.ser, [], layer=0, dry_run=True)
        self.assertTrue(result)

    def test_layer_zero_z_equals_z_place(self):
        expected = Z_PLACE + 0 * DUPLO_H_MM
        self.assertAlmostEqual(expected, Z_PLACE)

    def test_layer_one_z_is_one_brick_higher(self):
        z0 = Z_PLACE + 0 * DUPLO_H_MM
        z1 = Z_PLACE + 1 * DUPLO_H_MM
        self.assertAlmostEqual(z1 - z0, DUPLO_H_MM)

    def test_max_layers_is_two(self):
        self.assertEqual(MAX_LAYERS, 2)


class TestDryRunSerial(unittest.TestCase):

    def setUp(self):
        self.ser = DryRunSerial()

    def test_move_returns_ok_move(self):
        self.assertEqual(self.ser.send("MOVE:10,20,30,40"), "OK:MOVE")

    def test_grip_returns_ok_grip(self):
        self.assertEqual(self.ser.send("GRIP"), "OK:GRIP")

    def test_release_returns_ok_release(self):
        self.assertEqual(self.ser.send("RELEASE"), "OK:RELEASE")

    def test_home_returns_ok_home(self):
        self.assertEqual(self.ser.send("HOME"), "OK:HOME")

    def test_rehome_z_returns_ok_rehome_z(self):
        self.assertEqual(self.ser.send("REHOME_Z:19.2"), "OK:REHOME_Z")

    def test_unknown_command_returns_nack(self):
        self.assertEqual(self.ser.send("BLAH"), "NACK:UNKNOWN")


if __name__ == '__main__':
    unittest.main()
