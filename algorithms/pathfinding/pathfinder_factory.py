from mesa.discrete_space import CellAgent
from typing import Callable
import numpy as np
from algorithms.pathfinding.astar import AStar
from algorithms.pathfinding.pathfinder import Pathfinder
from algorithms.pathfinding.pathfinder_enum import PathfinderEnum

class PathfinderFactory:
    @staticmethod
    def give_pathfinder(
            agent: CellAgent,
            name: PathfinderEnum,
            heuristic: Callable[[tuple, tuple], float] = lambda a,b: np.sum(np.abs(np.array(a) - np.array(b))), # Manhattan distance as default
            *args, **kwargs
    ) -> Pathfinder:
        match name:
            case PathfinderEnum.ASTAR:
                return AStar(agent, heuristic, *args, **kwargs)