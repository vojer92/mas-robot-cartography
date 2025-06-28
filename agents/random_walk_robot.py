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
        view_angle: int = 90,
        view_resolution: int = 5,
        orientation: int = -90,
    ):
        super().__init__(
            model,
            cell,
            view_radius=view_radius,
            view_angle=view_angle,
            view_resolution=view_resolution,
            orientation=orientation,
        )

    def step(self):

        # 1. Scan environment()
        self.viewport = self.scan_environment()
        if self.viewport is None:
            raise RuntimeError(f"RandomWalkRobot {self.unique_id} received no viewport from scanning the environment")

        # 2. Move random
        neighborhood = self.cell.get_neighborhood(radius=1) # To make sure to move only inside grid borders
        cells_in_viewport = neighborhood.select(
            lambda cell: cell.coordinate in self.viewport
        )
        current_position = self.cell.coordinate
        if cells_in_viewport:
            target_position = cells_in_viewport.select_random_cell().coordinate
            # Calculate new orientation
            self.orientation = self.normalize_round45_angle(
                (
                    math.degrees(
                        math.atan2(
                            target_position[1] - current_position[1],
                            target_position[0] - current_position[0],
                        )
                    )
                )
            )
            # Move
            self.model.grid.move_agent(self, target_position)
        else:
            self.orientation = self.normalize_round45_angle(self.random.randint(0, 360))

