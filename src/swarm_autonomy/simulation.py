from __future__ import annotations

import math

from .coordination import assign_goals
from .mapping import OccupancyMap
from .models import DroneState, MissionReport, Obstacle
from .perception import observe
from .planning import safe_velocity


def run_mission(steps: int = 240, dt: float = 0.1) -> MissionReport:
    drones = [DroneState("uav_1", (0.0, -1.5)), DroneState("uav_2", (0.0, 1.5))]
    assign_goals(drones, [(7.0, -1.45), (7.0, 1.45)])
    maps = {drone.drone_id: OccupancyMap() for drone in drones}
    static = [Obstacle("warehouse", (3.2, 0.0), 0.72), Obstacle("tower", (5.4, 2.55), 0.45)]
    moving = Obstacle("inspector", (4.6, -2.4), 0.30, True, (0.0, 0.72))
    interventions = collisions = 0
    history = []
    for index in range(steps):
        phase = (index * dt * moving.velocity[1]) % 7.2
        y = -2.4 + (phase if phase <= 3.6 else 7.2 - phase)
        moving = Obstacle("inspector", (4.6, y), 0.30, True, (0.0, 0.72 if phase <= 3.6 else -0.72))
        obstacles = static + [moving]
        for drone in drones:
            observations = observe(drone, obstacles)
            maps[drone.drone_id].update(observations)
        maps["uav_1"].merge(maps["uav_2"])
        maps["uav_2"].merge(maps["uav_1"])
        commands = []
        for drone in drones:
            velocity, active = safe_velocity(drone, drones, obstacles, dt)
            commands.append(velocity)
            interventions += int(active)
        for drone, velocity in zip(drones, commands):
            drone.velocity = velocity
            drone.position = (drone.position[0] + velocity[0] * dt, drone.position[1] + velocity[1] * dt)
        for drone in drones:
            for obstacle in obstacles:
                if math.dist(drone.position, obstacle.position) < obstacle.radius + 0.22:
                    collisions += 1
        if index % 10 == 0:
            history.append({"t": round(index * dt, 1), "uav_1": drones[0].position, "uav_2": drones[1].position, "dynamic_y": round(y, 2)})
        if all(drone.completed for drone in drones):
            break
    return MissionReport(
        success=all(drone.completed for drone in drones) and collisions == 0,
        collisions=collisions,
        dynamic_interventions=interventions,
        completed_goals=sum(drone.completed for drone in drones),
        map_cells=len(maps["uav_1"].cells), history=history,
    )
