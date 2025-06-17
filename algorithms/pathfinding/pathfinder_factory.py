from typing import Callable
from algorithms.manhattan_distance import manhattan_distance
from algorithms.pathfinding.astar import AStar
from algorithms.pathfinding.pathfinder import Pathfinder
from algorithms.pathfinding.pathfinder_enum import PathfinderEnum

class PathfinderFactory:
    @staticmethod
    def give_pathfinder(
            agent: CellAgent,
            name: PathfinderEnum,
            heuristic: Callable[[tuple, tuple], float] = manhattan_distance,
            *args, **kwargs
    ) -> Pathfinder:
        match name:
            case PathfinderEnum.ASTAR:
                return AStar(agent, heuristic)