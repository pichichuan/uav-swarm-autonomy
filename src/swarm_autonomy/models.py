from __future__ import annotations

from dataclasses import dataclass, field


Point = tuple[float, float]


@dataclass
class DroneState:
    drone_id: str
    position: Point
    velocity: Point = (0.0, 0.0)
    goal: Point | None = None
    completed: bool = False


@dataclass(frozen=True)
class Obstacle:
    obstacle_id: str
    position: Point
    radius: float
    dynamic: bool = False
    velocity: Point = (0.0, 0.0)


@dataclass(frozen=True)
class SemanticObservation:
    obstacle_id: str
    label: str
    position: Point
    confidence: float
    dynamic: bool


@dataclass
class MissionReport:
    success: bool
    collisions: int
    dynamic_interventions: int
    completed_goals: int
    map_cells: int
    history: list[dict[str, object]] = field(default_factory=list)
