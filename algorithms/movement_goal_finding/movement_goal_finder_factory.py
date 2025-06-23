from mesa.discrete_space import CellAgent
from typing import Callable
import numpy as np
from algorithms.movement_goal_finding.original_frontier_based_exploration import OriginalFBE
from algorithms.movement_goal_finding.movement_goal_finder import MovementGoalFinder
from algorithms.movement_goal_finding.movement_goal_finder_enum import MovementGoalFinderEnum

class MovementGoalFinderFactory:
    @staticmethod
    def give_movement_goal_finder(
            agent: CellAgent,
            name: MovementGoalFinderEnum,
            # TODO: Adjust Parameters
            heuristic: Callable[[tuple, tuple], float] = lambda a,b: np.linalg.norm(a - b), # Euclidean distance as default
            *args, **kwargs
    ) -> MovementGoalFinder:
        match name:
            case MovementGoalFinder.ORIGINAL_FBE:
                # TODO: Adjust Parameters
                return OriginalFBE(agent, *args, **kwargs)