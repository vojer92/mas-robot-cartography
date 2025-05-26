import math

from mesa import Model
from mesa.discrete_space import Cell
from numpy import sqrt

from .explorer_robot import ExplorerRobot
from .obstacle import Obstacle


class RandomWalkRobot(ExplorerRobot):
    def __init__(
        self,
        model: Model,
        cell: Cell,
        view_radius: int = 1,
        view_angle: int = 180,
        view_resulution: int = 4,
    ):
        super().__init__(model, cell, view_radius, view_angle, view_resulution)

    def move(self):
        neighborhood = self.cell.get_neighborhood(radius=1)
        cells_without_Robot = neighborhood.select(
            lambda cell: not any(
                isinstance(agent, ExplorerRobot) or isinstance(agent, Obstacle)
                for agent in cell.agents
            )
        )

        self.cell = cells_without_Robot.select_random_cell()
