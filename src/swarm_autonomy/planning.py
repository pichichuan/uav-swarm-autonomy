from __future__ import annotations

import math

from .coordination import peer_repulsion
from .models import DroneState, Obstacle


def safe_velocity(drone: DroneState, drones: list[DroneState], obstacles: list[Obstacle], dt: float) -> tuple[tuple[float, float], bool]:
    if drone.goal is None:
        return (0.0, 0.0), False
    dx, dy = drone.goal[0] - drone.position[0], drone.goal[1] - drone.position[1]
    distance = math.hypot(dx, dy)
    if distance < 0.28:
        drone.completed = True
        return (0.0, 0.0), False
    vx, vy = 1.35 * dx / distance, 1.35 * dy / distance
    intervention = False
    for obstacle in obstacles:
        predicted = (obstacle.position[0] + obstacle.velocity[0] * 1.2, obstacle.position[1] + obstacle.velocity[1] * 1.2)
        ox, oy = drone.position[0] - predicted[0], drone.position[1] - predicted[1]
        separation = math.hypot(ox, oy)
        buffer = 1.15 + obstacle.radius
        if separation < buffer:
            intervention = intervention or obstacle.dynamic
            strength = 1.8 * (buffer - separation) / buffer
            if separation > 1e-4:
                vx += strength * ox / separation
                vy += strength * oy / separation
    px, py = peer_repulsion(drone, drones)
    vx += 1.6 * px
    vy += 1.6 * py
    speed = math.hypot(vx, vy)
    if speed > 1.55:
        vx, vy = 1.55 * vx / speed, 1.55 * vy / speed
    return (vx, vy), intervention
