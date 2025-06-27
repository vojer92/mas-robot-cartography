from mesa import Model
from mesa.discrete_space import Cell

from agents.explorer_robot import ExplorerRobot

from algorithms.pathfinding.pathfinder_enum import PathfinderEnum
from algorithms.pathfinding.pathfinder_factory import PathfinderFactory

from algorithms.movement_goal_finding.movement_goal_finder_enum import MovementGoalFinderEnum
from algorithms.movement_goal_finding.movement_goal_finder_factory import MovementGoalFinderFactory

from algorithms.movement_goal_selection.movement_goal_selector_enum import MovementGoalSelectorEnum
from algorithms.movement_goal_selection.movement_goal_selector_factory import MovementGoalSelectorFactory


class FBERobot(ExplorerRobot):
    """
    An explorer robot that uses the FBE method.
    """

    def __init__(
            self,
            model: Model,
            cell: Cell,
            view_radius: int = 1,
            view_angle: int = 90,
            view_resolution: int = 5,
            orientation: int = -90,
    ):
        super().__init__(model,
            cell,
            view_radius,
            view_angle,
            view_resolution,
            orientation
        )
        self.pathfinder = PathfinderFactory.give_pathfinder(self, PathfinderEnum.ASTAR)
        self.goal_finder = MovementGoalFinderFactory.give_movement_goal_finder(self, MovementGoalFinderEnum.ORIGINAL_FBE)
        self.goal_selector = MovementGoalSelectorFactory.give_movement_goal_selector(self, MovementGoalSelectorEnum.NEAREST_BIGGEST_FRONTIER)


     def step(self):
        # 1. Environment perception
        self.scan_environment()



        #TODO:
        # If there is no connection of already explored free cells from agent position to goal position, it is unable to
        # calculate a path towards it!
        # Kind of like alternative strategy is necessary.