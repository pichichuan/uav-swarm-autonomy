import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from swarm_autonomy.mapping import OccupancyMap
from swarm_autonomy.models import SemanticObservation
from swarm_autonomy.simulation import run_mission


class SwarmCoreTests(unittest.TestCase):
    def test_semantic_maps_merge(self):
        first, second = OccupancyMap(), OccupancyMap()
        second.update([SemanticObservation("person", "person", (2.0, 1.0), 0.9, True)])
        first.merge(second)
        self.assertEqual(len(first.cells), 1)
        self.assertTrue(next(iter(first.cells.values()))["dynamic"])

    def test_mission_completes_without_collision(self):
        report = run_mission()
        self.assertTrue(report.success, report)
        self.assertEqual(report.collisions, 0)
        self.assertGreater(report.dynamic_interventions, 0)
        self.assertEqual(report.completed_goals, 2)


if __name__ == "__main__":
    unittest.main()
