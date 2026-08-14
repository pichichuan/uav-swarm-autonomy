from __future__ import annotations

import math

from .models import DroneState, Point


def assign_goals(drones: list[DroneState], goals: list[Point]) -> None:
    """Deterministic nearest-first task allocation; replace with auction/CBBA as needed."""
    available = goals.copy()
    for drone in sorted(drones, key=lambda item: item.drone_id):
        goal = min(available, key=lambda point: math.dist(drone.position, point))
        drone.goal = goal
        available.remove(goal)


def peer_repulsion(drone: DroneState, peers: list[DroneState], safety_radius: float = 1.0) -> Point:
    rx = ry = 0.0
    for peer in peers:
        if peer.drone_id == drone.drone_id:
            continue
        dx, dy = drone.position[0] - peer.position[0], drone.position[1] - peer.position[1]
        distance = math.hypot(dx, dy)
        if 0.001 < distance < safety_radius:
            gain = (safety_radius - distance) / safety_radius
            rx += gain * dx / distance
            ry += gain * dy / distance
    return rx, ry
