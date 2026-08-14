from __future__ import annotations

from .models import SemanticObservation


class OccupancyMap:
    """Small semantic grid; replace with KISS-ICP/VINS + voxel mapping in ROS 2."""

    def __init__(self, resolution: float = 0.5) -> None:
        self.resolution = resolution
        self.cells: dict[tuple[int, int], dict[str, object]] = {}

    def update(self, observations: list[SemanticObservation]) -> None:
        for item in observations:
            cell = (round(item.position[0] / self.resolution), round(item.position[1] / self.resolution))
            self.cells[cell] = {"label": item.label, "dynamic": item.dynamic, "confidence": item.confidence}

    def merge(self, peer: "OccupancyMap") -> None:
        for cell, value in peer.cells.items():
            if cell not in self.cells or float(value["confidence"]) >= float(self.cells[cell]["confidence"]):
                self.cells[cell] = value.copy()
