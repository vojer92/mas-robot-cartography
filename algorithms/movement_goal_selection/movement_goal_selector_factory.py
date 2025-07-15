from typing import Callable

import numpy as np
from mesa.discrete_space import CellAgent

from algorithms.movement_goal_selection.movement_goal_selector import \
    MovementGoalSelector
from algorithms.movement_goal_selection.movement_goal_selector_enum import \
    MovementGoalSelectorEnum
from algorithms.movement_goal_selection.nearest_biggest_frontier import \
    NearestBiggestFrontier


class MovementGoalSelectorFactory:
    @staticmethod
    def give_movement_goal_selector(
            agent: CellAgent,
            name: MovementGoalSelectorEnum,
            factor_distance: float = 1.0,
            distance_heuristic: Callable[[tuple, tuple], float] = lambda a, b: np.linalg.norm((a[0] - b[0], a[1] - b[1])), # Euclidean distance as default
            factor_size: float = 0.1,
            *args, **kwargs
    ) -> MovementGoalSelector:
        match name:
            case MovementGoalSelectorEnum.NEAREST_BIGGEST_FRONTIER:
                return NearestBiggestFrontier(agent, factor_distance, distance_heuristic, factor_size, *args, **kwargs)
