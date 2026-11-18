"""Protocol-frame regression tests without importing the Maix device runtime."""

import ast
from pathlib import Path
import unittest


SOURCE = Path(__file__).resolve().parents[1] / "vision" / "main.py"
EXPORTED = {"crc8_payload", "landmark_packet", "ball_packet"}


def load_packet_functions():
    module = ast.parse(SOURCE.read_text(encoding="utf-8"), filename=str(SOURCE))
    functions = [node for node in module.body
                 if isinstance(node, ast.FunctionDef) and node.name in EXPORTED]
    namespace = {}
    exec(compile(ast.Module(body=functions, type_ignores=[]), str(SOURCE), "exec"), namespace)
    return namespace


class PacketTests(unittest.TestCase):
    def test_landmark_frame(self):
        packets = load_packet_functions()
        self.assertEqual(packets["landmark_packet"](3, 2, 161, 320).hex(),
                         "aa0302a100400171")

    def test_ball_valid_frame(self):
        packets = load_packet_functions()
        self.assertEqual(packets["ball_packet"](1, 2, 123, 45, 320, 240, 7, 3).hex(),
                         "aa5501327b002d004001f0000700ae")

    def test_ball_lost_frame(self):
        packets = load_packet_functions()
        self.assertEqual(packets["ball_packet"](0, 0, 0, 0, 320, 240, 8).hex(),
                         "aa550000000000004001f0000800bf")


if __name__ == "__main__":
    unittest.main()
