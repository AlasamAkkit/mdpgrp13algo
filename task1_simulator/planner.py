"""Dependency-free motion planner for the MDP Task 1 simulator.

The submitted Task 1 stack expresses arena positions in a 20 by 20 grid,
executes straight movements at 30 cm/s, uses a 30 cm turning radius, and
allows 0.25 s for every image scan.  This module keeps those assumptions in
one place and exposes immutable, UI-friendly plan records.

Grid coordinates are zero-based cell centres: ``(0, 0)`` is the south-west
cell and ``(19, 19)`` is the north-east cell.  Since the robot has a 3 by 3
cell envelope, its centre is constrained to the inclusive range 1..18.
"""

from __future__ import annotations

from dataclasses import dataclass, replace
from enum import Enum
import heapq
import itertools
import math
from typing import Iterable


GRID_CELLS = 20
CELL_CM = 10
ROBOT_ENVELOPE_CELLS = 3
ROBOT_HALF_EXTENT_CELLS = ROBOT_ENVELOPE_CELLS / 2
INFLATED_OBSTACLE_CELLS = 4
INFLATED_OBSTACLE_HALF_CELLS = INFLATED_OBSTACLE_CELLS / 2
RECOGNITION_DISTANCE_CELLS = 4

ROBOT_SPEED_CM_S = 30.0
SCAN_SECONDS = 0.25
ARC_SAMPLE_COUNT = 19  # Five-degree samples, including both endpoints.

_EPSILON = 1e-9


class Heading(str, Enum):
    """A cardinal arena heading.

    The string values deliberately match the Android/RPi obstacle protocol.
    Angles use the mathematical convention: east is 0 degrees and north is
    90 degrees.
    """

    N = "N"
    E = "E"
    S = "S"
    W = "W"

    @property
    def degrees(self) -> int:
        return {
            Heading.N: 90,
            Heading.E: 0,
            Heading.S: 270,
            Heading.W: 180,
        }[self]

    @property
    def vector(self) -> tuple[int, int]:
        return {
            Heading.N: (0, 1),
            Heading.E: (1, 0),
            Heading.S: (0, -1),
            Heading.W: (-1, 0),
        }[self]

    @property
    def dx(self) -> int:
        return self.vector[0]

    @property
    def dy(self) -> int:
        return self.vector[1]

    def left(self) -> Heading:
        return {
            Heading.N: Heading.W,
            Heading.W: Heading.S,
            Heading.S: Heading.E,
            Heading.E: Heading.N,
        }[self]

    def right(self) -> Heading:
        return {
            Heading.N: Heading.E,
            Heading.E: Heading.S,
            Heading.S: Heading.W,
            Heading.W: Heading.N,
        }[self]

    def opposite(self) -> Heading:
        return {
            Heading.N: Heading.S,
            Heading.S: Heading.N,
            Heading.E: Heading.W,
            Heading.W: Heading.E,
        }[self]

    # More descriptive aliases are convenient at call sites.
    turn_left = left
    turn_right = right

    @classmethod
    def from_degrees(cls, degrees: float) -> Heading:
        if isinstance(degrees, bool) or not isinstance(degrees, (int, float)):
            raise ValueError("heading degrees must be a finite number")
        if not math.isfinite(float(degrees)):
            raise ValueError("heading degrees must be a finite number")
        normalised = float(degrees) % 360.0
        cardinal = round(normalised / 90.0) * 90 % 360
        if not math.isclose(normalised, cardinal, abs_tol=_EPSILON):
            raise ValueError(f"{degrees!r} degrees is not a cardinal heading")
        return {
            0: Heading.E,
            90: Heading.N,
            180: Heading.W,
            270: Heading.S,
        }[cardinal]

    @classmethod
    def parse(cls, value: Heading | str | int | float) -> Heading:
        """Parse a protocol letter, cardinal word, or cardinal angle."""

        if isinstance(value, Heading):
            return value
        if isinstance(value, str):
            token = value.strip().upper()
            names = {
                "N": Heading.N,
                "NORTH": Heading.N,
                "E": Heading.E,
                "EAST": Heading.E,
                "S": Heading.S,
                "SOUTH": Heading.S,
                "W": Heading.W,
                "WEST": Heading.W,
            }
            if token in names:
                return names[token]
            raise ValueError(f"unknown heading {value!r}; use N, E, S, or W")
        return cls.from_degrees(value)


def _require_int(name: str, value: object) -> None:
    if isinstance(value, bool) or not isinstance(value, int):
        raise ValueError(f"{name} must be an integer")


def _require_finite(name: str, value: object) -> None:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} must be a finite number")
    if not math.isfinite(float(value)):
        raise ValueError(f"{name} must be a finite number")


@dataclass(frozen=True, slots=True)
class GridPose:
    x: int
    y: int
    heading: Heading

    def __post_init__(self) -> None:
        _require_int("pose x", self.x)
        _require_int("pose y", self.y)
        if not isinstance(self.heading, Heading):
            raise ValueError("pose heading must be a Heading")


@dataclass(frozen=True, slots=True)
class PoseSample:
    x: float
    y: float
    heading_deg: float

    def __post_init__(self) -> None:
        _require_finite("sample x", self.x)
        _require_finite("sample y", self.y)
        _require_finite("sample heading", self.heading_deg)


@dataclass(frozen=True, slots=True)
class Obstacle:
    id: int
    x: int
    y: int
    face: Heading
    target_id: int | None = None

    def __post_init__(self) -> None:
        _require_int("obstacle id", self.id)
        if self.id <= 0:
            raise ValueError("obstacle id must be a positive integer")
        _require_int(f"obstacle {self.id} x", self.x)
        _require_int(f"obstacle {self.id} y", self.y)
        if not isinstance(self.face, Heading):
            raise ValueError(f"obstacle {self.id} face must be a Heading")
        if self.target_id is not None:
            _require_int(f"obstacle {self.id} target_id", self.target_id)
            if self.target_id <= 0:
                raise ValueError(
                    f"obstacle {self.id} target_id must be positive when supplied"
                )


@dataclass(frozen=True, slots=True)
class Motion:
    code: str
    label: str
    duration_s: float
    distance_cm: float
    samples: tuple[PoseSample, ...]
    obstacle_id: int | None = None
    target_id: int | None = None

    def __post_init__(self) -> None:
        if not isinstance(self.code, str) or not self.code:
            raise ValueError("motion code must be a non-empty string")
        if not isinstance(self.label, str) or not self.label:
            raise ValueError("motion label must be a non-empty string")
        _require_finite("motion duration_s", self.duration_s)
        _require_finite("motion distance_cm", self.distance_cm)
        if self.duration_s < 0:
            raise ValueError("motion duration_s cannot be negative")
        object.__setattr__(self, "samples", tuple(self.samples))
        if not self.samples:
            raise ValueError("motion samples cannot be empty")
        if not all(isinstance(sample, PoseSample) for sample in self.samples):
            raise ValueError("motion samples must contain PoseSample values")


@dataclass(frozen=True, slots=True)
class PlanResult:
    motions: tuple[Motion, ...]
    visit_order: tuple[int, ...]
    unreachable_ids: tuple[int, ...]
    total_distance_cm: float
    estimated_seconds: float
    expanded_states: int
    command_messages: tuple[str, ...]
    complete: bool
    notes: tuple[str, ...]

    def __post_init__(self) -> None:
        object.__setattr__(self, "motions", tuple(self.motions))
        object.__setattr__(self, "visit_order", tuple(self.visit_order))
        object.__setattr__(self, "unreachable_ids", tuple(self.unreachable_ids))
        object.__setattr__(self, "command_messages", tuple(self.command_messages))
        object.__setattr__(self, "notes", tuple(self.notes))


@dataclass(frozen=True, slots=True)
class _Edge:
    end: GridPose
    motion: Motion


@dataclass(frozen=True, slots=True)
class _Leg:
    motions: tuple[Motion, ...]
    duration_s: float
    distance_cm: float


@dataclass(frozen=True, slots=True)
class _Route:
    duration_s: float
    distance_cm: float
    order: tuple[int, ...]


class Task1Planner:
    """Plan a minimum-estimated-time Task 1 route for four to eight targets."""

    def __init__(
        self,
        obstacles: Iterable[Obstacle],
        turn_radius_cells: int = 3,
    ) -> None:
        if isinstance(obstacles, (str, bytes)):
            raise ValueError("obstacles must be an iterable of Obstacle values")
        try:
            supplied = tuple(obstacles)
        except TypeError as exc:
            raise ValueError("obstacles must be an iterable of Obstacle values") from exc

        if not 4 <= len(supplied) <= 8:
            raise ValueError(
                f"Task 1 requires 4 to 8 obstacles; received {len(supplied)}"
            )
        if not all(isinstance(obstacle, Obstacle) for obstacle in supplied):
            raise ValueError("every obstacle must be an Obstacle value")

        _require_int("turn_radius_cells", turn_radius_cells)
        if turn_radius_cells <= 0:
            raise ValueError("turn_radius_cells must be a positive integer")

        seen_ids: set[int] = set()
        seen_cells: dict[tuple[int, int], int] = {}
        for obstacle in supplied:
            if obstacle.id in seen_ids:
                raise ValueError(f"duplicate obstacle id {obstacle.id}")
            seen_ids.add(obstacle.id)

            if not (0 <= obstacle.x < GRID_CELLS and 0 <= obstacle.y < GRID_CELLS):
                raise ValueError(
                    f"obstacle {obstacle.id} coordinate ({obstacle.x}, {obstacle.y}) "
                    f"is outside the 0..{GRID_CELLS - 1} arena"
                )
            cell = (obstacle.x, obstacle.y)
            if cell in seen_cells:
                other_id = seen_cells[cell]
                raise ValueError(
                    f"obstacles {other_id} and {obstacle.id} overlap at {cell}"
                )
            seen_cells[cell] = obstacle.id

        # Sorting once makes validation, A*, DP tie-breaking, and output stable even
        # when callers provide a set or a differently ordered list.
        self.obstacles = tuple(sorted(supplied, key=lambda obstacle: obstacle.id))
        self.turn_radius_cells = turn_radius_cells
        self._edge_cache: dict[GridPose, tuple[_Edge, ...]] = {}

    def plan(self, start: GridPose = GridPose(1, 1, Heading.N)) -> PlanResult:
        if not isinstance(start, GridPose):
            raise ValueError("start must be a GridPose")
        if not self._pose_is_valid(start):
            raise ValueError(
                "start pose is outside the robot-safe arena or overlaps an "
                "inflated obstacle"
            )

        targets = tuple(self._recognition_pose(obstacle) for obstacle in self.obstacles)
        legs: dict[tuple[int, int], _Leg | None] = {}
        expanded_states = 0

        # Source -1 is the robot start; non-negative sources are target indices.
        for target_index, target in enumerate(targets):
            leg, expanded = self._astar(start, target)
            legs[(-1, target_index)] = leg
            expanded_states += expanded

        for source_index, source in enumerate(targets):
            for target_index, target in enumerate(targets):
                if source_index == target_index:
                    continue
                leg, expanded = self._astar(source, target)
                legs[(source_index, target_index)] = leg
                expanded_states += expanded

        route = self._select_route(legs)
        motions = self._assemble_motions(route, legs, targets)
        command_messages = self._command_messages(motions)
        visit_order = tuple(self.obstacles[index].id for index in route.order)
        visited = set(visit_order)
        unreachable_ids = tuple(
            obstacle.id for obstacle in self.obstacles if obstacle.id not in visited
        )
        complete = len(route.order) == len(self.obstacles)

        total_distance_cm = sum(abs(motion.distance_cm) for motion in motions)
        estimated_seconds = sum(motion.duration_s for motion in motions)

        notes = [
            "Coordinates are zero-based cell centres; the 3x3 robot envelope "
            "keeps its centre within cells 1..18.",
            "Collision checks use the 40x40 cm configuration-space envelope "
            "around every obstacle, including every sampled turn pose.",
            "Timing mirrors the submitted stack: 30 cm/s movement, a radius-"
            f"{self.turn_radius_cells * CELL_CM}-cm 90-degree arc, and 0.25 s per scan.",
        ]
        if complete:
            notes.append("A collision-free route was found for every target.")
        elif route.order:
            notes.append(
                "No route could visit every target; this is the fastest route "
                "among those visiting the maximum reachable target count."
            )
        else:
            notes.append("No recognition pose is reachable from the start pose.")

        return PlanResult(
            motions=motions,
            visit_order=visit_order,
            unreachable_ids=unreachable_ids,
            total_distance_cm=total_distance_cm,
            estimated_seconds=estimated_seconds,
            expanded_states=expanded_states,
            command_messages=command_messages,
            complete=complete,
            notes=tuple(notes),
        )

    def _recognition_pose(self, obstacle: Obstacle) -> GridPose:
        dx, dy = obstacle.face.vector
        return GridPose(
            obstacle.x + RECOGNITION_DISTANCE_CELLS * dx,
            obstacle.y + RECOGNITION_DISTANCE_CELLS * dy,
            obstacle.face.opposite(),
        )

    def _sample_is_valid(self, x: float, y: float) -> bool:
        # GridPose coordinates denote cell centres, so x=1 is physically at
        # 15 cm.  A 15 cm half-envelope then just touches the arena boundary.
        minimum = ROBOT_HALF_EXTENT_CELLS - 0.5
        maximum = GRID_CELLS - ROBOT_HALF_EXTENT_CELLS - 0.5
        if x < minimum - _EPSILON or x > maximum + _EPSILON:
            return False
        if y < minimum - _EPSILON or y > maximum + _EPSILON:
            return False

        # A 10 cm obstacle inflated by a 30 cm robot is a 40 by 40 cm
        # configuration-space square.  Boundary contact is allowed, matching
        # the strict comparisons in the submitted uppercase stack.
        for obstacle in self.obstacles:
            if (
                abs(x - obstacle.x) < INFLATED_OBSTACLE_HALF_CELLS - _EPSILON
                and abs(y - obstacle.y)
                < INFLATED_OBSTACLE_HALF_CELLS - _EPSILON
            ):
                return False
        return True

    def _pose_is_valid(self, pose: GridPose) -> bool:
        return self._sample_is_valid(float(pose.x), float(pose.y))

    @staticmethod
    def _sample_for_pose(pose: GridPose) -> PoseSample:
        return PoseSample(float(pose.x), float(pose.y), float(pose.heading.degrees))

    def _edges(self, pose: GridPose) -> tuple[_Edge, ...]:
        cached = self._edge_cache.get(pose)
        if cached is not None:
            return cached

        edges: list[_Edge] = []
        # This ordering mirrors the active uppercase stack: forward, reverse,
        # forward-left, forward-right, reverse-right, reverse-left.
        action_specs = (
            ("F", "Forward", 1, 0),
            ("B", "Reverse", -1, 0),
            ("LF", "Forward left 90 degrees", 1, 1),
            ("RF", "Forward right 90 degrees", 1, -1),
            ("RB", "Reverse right 90 degrees", -1, 1),
            ("LB", "Reverse left 90 degrees", -1, -1),
        )
        for prefix, label, drive_sign, turn_sign in action_specs:
            edge = self._build_edge(pose, prefix, label, drive_sign, turn_sign)
            if edge is not None:
                edges.append(edge)

        result = tuple(edges)
        self._edge_cache[pose] = result
        return result

    def _build_edge(
        self,
        pose: GridPose,
        prefix: str,
        label: str,
        drive_sign: int,
        turn_sign: int,
    ) -> _Edge | None:
        if turn_sign == 0:
            dx, dy = pose.heading.vector
            end = GridPose(
                pose.x + drive_sign * dx,
                pose.y + drive_sign * dy,
                pose.heading,
            )
            samples = tuple(
                PoseSample(
                    pose.x + drive_sign * dx * fraction,
                    pose.y + drive_sign * dy * fraction,
                    float(pose.heading.degrees),
                )
                for fraction in (0.0, 0.25, 0.5, 0.75, 1.0)
            )
            if not all(self._sample_is_valid(sample.x, sample.y) for sample in samples):
                return None
            signed_distance_cm = drive_sign * CELL_CM
            motion = Motion(
                code=prefix,
                label=f"{label} {CELL_CM} cm",
                duration_s=CELL_CM / ROBOT_SPEED_CM_S,
                distance_cm=float(signed_distance_cm),
                samples=samples,
            )
            return _Edge(end=end, motion=motion)

        theta_0 = math.radians(pose.heading.degrees)
        delta_theta = turn_sign * math.pi / 2
        radius = float(self.turn_radius_cells)
        curvature = turn_sign / (drive_sign * radius)
        samples_list: list[PoseSample] = []
        for index in range(ARC_SAMPLE_COUNT):
            fraction = index / (ARC_SAMPLE_COUNT - 1)
            theta = theta_0 + delta_theta * fraction
            x = pose.x + (math.sin(theta) - math.sin(theta_0)) / curvature
            y = pose.y - (math.cos(theta) - math.cos(theta_0)) / curvature
            samples_list.append(
                PoseSample(x, y, math.degrees(theta) % 360.0)
            )

        final_heading = Heading.from_degrees(
            pose.heading.degrees + turn_sign * 90
        )
        end = GridPose(
            int(round(samples_list[-1].x)),
            int(round(samples_list[-1].y)),
            final_heading,
        )
        # Exact endpoint values avoid tiny floating-point seams in animation.
        samples_list[-1] = self._sample_for_pose(end)
        samples = tuple(samples_list)
        if not all(self._sample_is_valid(sample.x, sample.y) for sample in samples):
            return None

        signed_distance_cm = (
            drive_sign * radius * CELL_CM * math.pi / 2
        )
        motion = Motion(
            code=f"{prefix}090",
            label=label,
            duration_s=abs(signed_distance_cm) / ROBOT_SPEED_CM_S,
            distance_cm=signed_distance_cm,
            samples=samples,
        )
        return _Edge(end=end, motion=motion)

    @staticmethod
    def _heuristic(pose: GridPose, goal: GridPose) -> float:
        # No heading term is needed for admissibility.  Straight travel is the
        # fastest possible centre displacement at three cells per second.
        return math.hypot(goal.x - pose.x, goal.y - pose.y) / (
            ROBOT_SPEED_CM_S / CELL_CM
        )

    def _astar(self, start: GridPose, goal: GridPose) -> tuple[_Leg | None, int]:
        if not self._pose_is_valid(start) or not self._pose_is_valid(goal):
            return None, 0

        serial = itertools.count()
        frontier: list[tuple[float, float, int, int, int, int, GridPose]] = []
        heading_rank = {Heading.N: 0, Heading.E: 1, Heading.S: 2, Heading.W: 3}
        heapq.heappush(
            frontier,
            (
                self._heuristic(start, goal),
                0.0,
                start.x,
                start.y,
                heading_rank[start.heading],
                next(serial),
                start,
            ),
        )
        best_cost: dict[GridPose, float] = {start: 0.0}
        parent: dict[GridPose, tuple[GridPose, Motion]] = {}
        expanded = 0

        while frontier:
            _, queued_cost, _, _, _, _, current = heapq.heappop(frontier)
            current_cost = best_cost.get(current)
            if current_cost is None or queued_cost > current_cost + _EPSILON:
                continue

            expanded += 1
            if current == goal:
                path: list[Motion] = []
                cursor = current
                while cursor != start:
                    previous, motion = parent[cursor]
                    path.append(motion)
                    cursor = previous
                path.reverse()
                return (
                    _Leg(
                        motions=tuple(path),
                        duration_s=sum(motion.duration_s for motion in path),
                        distance_cm=sum(abs(motion.distance_cm) for motion in path),
                    ),
                    expanded,
                )

            for edge in self._edges(current):
                candidate_cost = current_cost + edge.motion.duration_s
                known_cost = best_cost.get(edge.end)
                if known_cost is not None and candidate_cost >= known_cost - _EPSILON:
                    continue
                best_cost[edge.end] = candidate_cost
                parent[edge.end] = (current, edge.motion)
                heapq.heappush(
                    frontier,
                    (
                        candidate_cost + self._heuristic(edge.end, goal),
                        candidate_cost,
                        edge.end.x,
                        edge.end.y,
                        heading_rank[edge.end.heading],
                        next(serial),
                        edge.end,
                    ),
                )

        return None, expanded

    def _route_is_better(self, candidate: _Route, incumbent: _Route) -> bool:
        if candidate.duration_s < incumbent.duration_s - _EPSILON:
            return True
        if candidate.duration_s > incumbent.duration_s + _EPSILON:
            return False
        if candidate.distance_cm < incumbent.distance_cm - _EPSILON:
            return True
        if candidate.distance_cm > incumbent.distance_cm + _EPSILON:
            return False
        candidate_ids = tuple(self.obstacles[index].id for index in candidate.order)
        incumbent_ids = tuple(self.obstacles[index].id for index in incumbent.order)
        return candidate_ids < incumbent_ids

    def _select_route(self, legs: dict[tuple[int, int], _Leg | None]) -> _Route:
        count = len(self.obstacles)
        routes: dict[tuple[int, int], _Route] = {}

        for target_index in range(count):
            leg = legs.get((-1, target_index))
            if leg is None:
                continue
            routes[(1 << target_index, target_index)] = _Route(
                duration_s=leg.duration_s + SCAN_SECONDS,
                distance_cm=leg.distance_cm,
                order=(target_index,),
            )

        for mask in range(1, 1 << count):
            for last in range(count):
                route = routes.get((mask, last))
                if route is None:
                    continue
                for target_index in range(count):
                    bit = 1 << target_index
                    if mask & bit:
                        continue
                    leg = legs.get((last, target_index))
                    if leg is None:
                        continue
                    candidate = _Route(
                        duration_s=route.duration_s + leg.duration_s + SCAN_SECONDS,
                        distance_cm=route.distance_cm + leg.distance_cm,
                        order=route.order + (target_index,),
                    )
                    key = (mask | bit, target_index)
                    incumbent = routes.get(key)
                    if incumbent is None or self._route_is_better(candidate, incumbent):
                        routes[key] = candidate

        if not routes:
            return _Route(0.0, 0.0, ())

        full_mask = (1 << count) - 1
        full_routes = [
            route
            for (mask, _), route in routes.items()
            if mask == full_mask
        ]
        candidates = full_routes
        if not candidates:
            maximum_count = max(mask.bit_count() for mask, _ in routes)
            candidates = [
                route
                for (mask, _), route in routes.items()
                if mask.bit_count() == maximum_count
            ]

        best = candidates[0]
        for candidate in candidates[1:]:
            if self._route_is_better(candidate, best):
                best = candidate
        return best

    def _assemble_motions(
        self,
        route: _Route,
        legs: dict[tuple[int, int], _Leg | None],
        targets: tuple[GridPose, ...],
    ) -> tuple[Motion, ...]:
        assembled: list[Motion] = []
        source_index = -1
        for target_index in route.order:
            obstacle = self.obstacles[target_index]
            leg = legs[(source_index, target_index)]
            if leg is None:  # The DP only records existing legs.
                raise RuntimeError("route reconstruction referenced a missing A* leg")
            assembled.extend(
                replace(
                    motion,
                    obstacle_id=obstacle.id,
                    target_id=obstacle.target_id,
                )
                for motion in leg.motions
            )
            assembled.append(
                Motion(
                    code="SCAN",
                    label=f"Scan obstacle {obstacle.id}",
                    duration_s=SCAN_SECONDS,
                    distance_cm=0.0,
                    samples=(self._sample_for_pose(targets[target_index]),),
                    obstacle_id=obstacle.id,
                    target_id=obstacle.target_id,
                )
            )
            source_index = target_index

        return self._compress_straights(tuple(assembled))

    @staticmethod
    def _compress_straights(motions: tuple[Motion, ...]) -> tuple[Motion, ...]:
        compressed: list[Motion] = []
        for motion in motions:
            prefix = motion.code
            if (
                prefix in {"F", "B"}
                and compressed
                and compressed[-1].code == prefix
                and compressed[-1].obstacle_id == motion.obstacle_id
                and compressed[-1].target_id == motion.target_id
            ):
                previous = compressed[-1]
                distance_cm = previous.distance_cm + motion.distance_cm
                magnitude_cm = int(round(abs(distance_cm)))
                label_prefix = "Forward" if prefix == "F" else "Reverse"
                compressed[-1] = Motion(
                    code=prefix,
                    label=f"{label_prefix} {magnitude_cm} cm",
                    duration_s=previous.duration_s + motion.duration_s,
                    distance_cm=distance_cm,
                    samples=previous.samples + motion.samples[1:],
                    obstacle_id=motion.obstacle_id,
                    target_id=motion.target_id,
                )
            else:
                compressed.append(motion)
        return tuple(compressed)

    @staticmethod
    def _command_messages(motions: tuple[Motion, ...]) -> tuple[str, ...]:
        """Convert simulator motions to the active uppercase-stack protocol."""

        messages: list[str] = []
        for motion in motions:
            if motion.code == "F":
                messages.append(f"SF{int(round(abs(motion.distance_cm))):03d}")
            elif motion.code == "B":
                messages.append(f"SB{int(round(abs(motion.distance_cm))):03d}")
            elif motion.code == "SCAN":
                if motion.obstacle_id is None:
                    raise RuntimeError("scan motion is missing its obstacle id")
                messages.append(f"P___{motion.obstacle_id}")
            else:
                messages.append(motion.code)
        return tuple(messages)


__all__ = [
    "GridPose",
    "Heading",
    "Motion",
    "Obstacle",
    "PlanResult",
    "PoseSample",
    "Task1Planner",
]
