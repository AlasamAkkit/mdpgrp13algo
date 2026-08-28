"""Deterministic regression tests for the public Task 1 planner API."""

from __future__ import annotations

import math
from pathlib import Path
import sys
import unittest


# Permit discovery both from ``Algorithm`` (the documented command) and from
# the repository's parent directory.
ALGORITHM_DIR = Path(__file__).resolve().parents[2]
if str(ALGORITHM_DIR) not in sys.path:
    sys.path.insert(0, str(ALGORITHM_DIR))

from task1_simulator.planner import (  # noqa: E402
    GridPose,
    Heading,
    Obstacle,
    Task1Planner,
)


def open_arena_obstacles() -> tuple[Obstacle, ...]:
    """The deterministic four-obstacle public demo, without importing the UI."""

    return (
        Obstacle(1, 5, 8, Heading.S, 14),
        Obstacle(2, 9, 15, Heading.E, 22),
        Obstacle(3, 15, 12, Heading.W, 31),
        Obstacle(4, 13, 4, Heading.N, 38),
    )


class CompletePlanTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.obstacles = open_arena_obstacles()
        cls.result = Task1Planner(cls.obstacles).plan(
            GridPose(1, 1, Heading.N)
        )

    def test_complete_route_visits_each_image_once(self) -> None:
        self.assertTrue(self.result.complete)
        self.assertEqual((1, 4, 3, 2), self.result.visit_order)
        self.assertEqual((), self.result.unreachable_ids)

        scans = tuple(
            motion.obstacle_id
            for motion in self.result.motions
            if motion.code == "SCAN"
        )
        self.assertEqual(self.result.visit_order, scans)
        self.assertEqual(len(scans), len(set(scans)))
        self.assertLess(self.result.estimated_seconds, 6 * 60)
        self.assertAlmostEqual(
            self.result.estimated_seconds,
            sum(motion.duration_s for motion in self.result.motions),
        )
        self.assertAlmostEqual(
            self.result.total_distance_cm,
            sum(abs(motion.distance_cm) for motion in self.result.motions),
        )

    def test_recognition_and_collision_geometry(self) -> None:
        by_id = {obstacle.id: obstacle for obstacle in self.obstacles}
        expected_scan_pose = {
            1: (5.0, 4.0, 90.0),
            4: (13.0, 8.0, 270.0),
            3: (11.0, 12.0, 0.0),
            2: (13.0, 15.0, 180.0),
        }

        scan_motions = [m for m in self.result.motions if m.code == "SCAN"]
        for motion in scan_motions:
            self.assertIsNotNone(motion.obstacle_id)
            obstacle_id = motion.obstacle_id
            sample = motion.samples[-1]
            self.assertEqual(
                expected_scan_pose[obstacle_id],
                (sample.x, sample.y, sample.heading_deg),
            )

            obstacle = by_id[obstacle_id]
            center_standoff_cm = 10.0 * math.hypot(
                sample.x - obstacle.x, sample.y - obstacle.y
            )
            self.assertAlmostEqual(40.0, center_standoff_cm)
            # 40 cm center spacing - 15 cm robot half-envelope - 5 cm
            # obstacle half-width gives the briefing's preferred 20 cm gap.
            self.assertAlmostEqual(20.0, center_standoff_cm - 15.0 - 5.0)

        for motion in self.result.motions:
            if motion.code.endswith("090"):
                self.assertEqual(19, len(motion.samples))
                self.assertAlmostEqual(
                    math.pi * 30.0 / 2.0,
                    abs(motion.distance_cm),
                )
            for sample in motion.samples:
                # A 30 cm envelope keeps the center in cells 1..18.
                self.assertGreaterEqual(sample.x, 1.0 - 1e-9)
                self.assertLessEqual(sample.x, 18.0 + 1e-9)
                self.assertGreaterEqual(sample.y, 1.0 - 1e-9)
                self.assertLessEqual(sample.y, 18.0 + 1e-9)

                # A 10 cm obstacle inflated by the 30 cm robot envelope is a
                # 40 cm square. Boundary contact is allowed; its interior is not.
                for obstacle in self.obstacles:
                    inside_inflated_obstacle = (
                        abs(sample.x - obstacle.x) < 2.0 - 1e-9
                        and abs(sample.y - obstacle.y) < 2.0 - 1e-9
                    )
                    self.assertFalse(
                        inside_inflated_obstacle,
                        f"sample {sample} enters obstacle {obstacle.id} clearance",
                    )

    def test_command_conversion_matches_uppercase_protocol(self) -> None:
        self.assertEqual(
            (
                "SF030",
                "RB090",
                "SB040",
                "RF090",
                "P___1",
                "SB010",
                "LF090",
                "SB080",
                "RB090",
                "SF010",
                "P___4",
                "SB010",
                "RB090",
                "SF010",
                "P___3",
                "SB030",
                "RF090",
                "SB030",
                "LB090",
                "SF010",
                "P___2",
            ),
            self.result.command_messages,
        )
        self.assertEqual(
            ("P___1", "P___4", "P___3", "P___2"),
            tuple(
                command
                for command in self.result.command_messages
                if command.startswith("P___")
            ),
        )


class PartialAndInvalidInputTests(unittest.TestCase):
    def test_outward_boundary_image_is_reported_unreachable(self) -> None:
        obstacles = (
            Obstacle(1, 5, 8, Heading.S, 14),
            Obstacle(2, 9, 15, Heading.E, 22),
            Obstacle(3, 15, 12, Heading.W, 31),
            # Its recognition pose would be (23, 19), outside the arena.
            Obstacle(4, 19, 19, Heading.E, 38),
        )
        result = Task1Planner(obstacles).plan()

        self.assertFalse(result.complete)
        self.assertEqual((1, 3, 2), result.visit_order)
        self.assertEqual((4,), result.unreachable_ids)
        self.assertNotIn("P___4", result.command_messages)
        self.assertEqual(
            (1, 3, 2),
            tuple(
                motion.obstacle_id
                for motion in result.motions
                if motion.code == "SCAN"
            ),
        )

    def test_current_task_count_and_duplicate_ids_are_rejected(self) -> None:
        with self.assertRaisesRegex(ValueError, "requires 4 to 8 obstacles"):
            Task1Planner(open_arena_obstacles()[:3])

        duplicate_id = (
            Obstacle(1, 5, 8, Heading.S),
            Obstacle(1, 9, 15, Heading.E),
            Obstacle(3, 15, 12, Heading.W),
            Obstacle(4, 13, 4, Heading.N),
        )
        with self.assertRaisesRegex(ValueError, "duplicate obstacle id 1"):
            Task1Planner(duplicate_id)


if __name__ == "__main__":
    unittest.main()
