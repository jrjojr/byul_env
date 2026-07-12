from pathlib import Path
import sys

wrapper_path = Path(__file__).resolve().parents[1] / "modules"

sys.path.insert(0, str(wrapper_path.resolve()))

import unittest

from byul_tick import c_tick

class TestTick(unittest.TestCase):
    def setUp(self):
        self.tick = c_tick()
        self.log = []

    def tearDown(self):
        self.tick.close()

    def test_single_callback(self):
        def tick_fn(ctx, dt):
            self.log.append((ctx, dt))

        self.tick.attach(tick_fn, context="A")
        self.tick.update(0.1)

        self.assertEqual(len(self.log), 1)
        self.assertEqual(self.log[0][0], "A")
        self.assertAlmostEqual(self.log[0][1], 0.1)

    def test_multiple_callbacks(self):
        def tick1(ctx, dt): self.log.append((ctx, dt))
        def tick2(ctx, dt): self.log.append((ctx, dt))

        self.tick.attach(tick1, context="Entity1")
        self.tick.attach(tick2, context="Entity2")

        self.tick.update(0.2)

        self.assertEqual(len(self.log), 2)

        self.assertEqual(self.log[0][0], "Entity1")
        self.assertAlmostEqual(self.log[0][1], 0.2, places=6)

        self.assertEqual(self.log[1][0], "Entity2")
        self.assertAlmostEqual(self.log[1][1], 0.2, places=6)

    def test_detach(self):
        def tick_fn(ctx, dt): self.log.append(f"{ctx}:{dt}")

        self.tick.attach(tick_fn, context="Target")
        self.tick.detach(context="Target")

        self.tick.update(0.1)
        self.assertEqual(len(self.log), 0)

    def test_list_attached(self):
        def dummy(ctx, dt): pass
        self.tick.attach(dummy, context="X")
        self.tick.attach(dummy, context="Y")

        attached = self.tick.list_attached()
        contexts = [entry["context"] for entry in attached]

        self.assertIn("X", contexts)
        self.assertIn("Y", contexts)

# 🔽 직접 실행 시 동작
if __name__ == '__main__':
    unittest.main()
