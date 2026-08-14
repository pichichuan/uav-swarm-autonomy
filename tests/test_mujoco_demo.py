import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from swarm_autonomy.mujoco_demo import run_mujoco_mission


class MujocoMissionTests(unittest.TestCase):
    def test_two_uavs_complete_without_collision(self):
        result = run_mujoco_mission()
        self.assertTrue(result.success, result)
        self.assertEqual(result.collisions, 0)
        self.assertGreater(result.dynamic_interventions, 0)


if __name__ == "__main__":
    unittest.main()
