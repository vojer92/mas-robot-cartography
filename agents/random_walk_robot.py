import math

from mesa import Model
from mesa.discrete_space import Cell

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
        orientation: int = -90,
    ):
        super().__init__(
            model,
            cell,
            view_radius=view_radius,
            view_angle=view_angle,
            view_resolution=view_resulution,
            orientation=orientation,
        )

    def move(self):
        neighborhood = self.cell.get_neighborhood(radius=1)
        cells_without_Robot = neighborhood.select(
            lambda cell: not any(
                isinstance(agent, ExplorerRobot) or isinstance(agent, Obstacle)
                for agent in cell.agents
            )
        )

        cells_in_viewport = cells_without_Robot.select(
            lambda cell: cell.coordinate in self.viewport
        )

        current_position = self.cell.coordinate
        self.cell = cells_in_viewport.select_random_cell()
        target_position = self.cell.coordinate

        self.orientation = round(
            (
                math.degrees(
                    math.atan2(
                        target_position[0] - current_position[0],
                        target_position[1] - current_position[1],
                    )
                )
            )
        )
