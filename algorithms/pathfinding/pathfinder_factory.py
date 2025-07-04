from typing import Callable

import numpy as np
from mesa.discrete_space import CellAgent

from algorithms.pathfinding.astar import AStar
from algorithms.pathfinding.pathfinder import Pathfinder
from algorithms.pathfinding.pathfinder_enum import PathfinderEnum


class PathfinderFactory:
    @staticmethod
    def give_pathfinder(
            agent: CellAgent,
            name: PathfinderEnum,
            heuristic: Callable[[tuple, tuple], float] = lambda a,b: np.linalg.norm((a[0] - b[0], a[1] - b[1])), # Euclidean distance as default
            *args, **kwargs
    ) -> Pathfinder:
        match name:
            case PathfinderEnum.ASTAR:
                return AStar(agent, heuristic, *args, **kwargs)
