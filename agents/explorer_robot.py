from abc import ABC, abstractmethod
import heapq

from mesa import Model
from mesa.discrete_space import Cell, CellAgent


class ExplorerRobot(ABC, CellAgent):
    def __init__(
        self,
        model: Model,
        cell: Cell,
        view_radius: int,
        view_angle: int,
        view_resolution: int,
        orientation: int,
    ):
        super().__init__(model)
        self.cell = cell
        self.view_radius = view_radius
        if view_angle == 360:
            self.view_angle = 359
        else:
            self.view_angle = view_angle
        self.view_resolution = view_resolution
        self.orientation = orientation

        self.cell_blocking = True # Blocking of cell for moving agents
        self.scan_blocking = False # Blocking of environment perception for other agents

        self.viewport: list[tuple[int, int]] = []

    @abstractmethod
    def move(self):
        pass

    @abstractmethod
    def step(self):
        pass