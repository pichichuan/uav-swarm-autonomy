from __future__ import annotations

import math

from .models import DroneState, Obstacle, SemanticObservation


def observe(drone: DroneState, obstacles: list[Obstacle], range_m: float = 3.5) -> list[SemanticObservation]:
    """Simulation adapter standing in for YOLO + depth/LiDAR association."""
    result = []
    for obstacle in obstacles:
        distance = math.dist(drone.position, obstacle.position)
        if distance <= range_m:
            result.append(SemanticObservation(
                obstacle.obstacle_id, "person" if obstacle.dynamic else "structure",
                obstacle.position, max(0.55, 1.0 - distance / (2 * range_m)), obstacle.dynamic,
            ))
    return result
