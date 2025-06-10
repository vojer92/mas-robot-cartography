from abc import ABC, abstractmethod
from enum import Enum, auto


class PathfinderEnum(Enum):
    DIKSTRA = auto()
    ASTAR = auto()
    # weitere algorithmen mit namen

class Pathfinder(abc):

    @abstractmethod
    def find_path() -> list[tuple[int, int]]:
        pass


class Dikstra(Pathfinder):
    def find_path() -> list[tuple[int, int]]:
        # hier steht mein super duper code
        return [(0, 0)]

class AStar(Pathfinder):
    def find_path() -> list[tuple[int, int]]:
        # hier steht mein super duper code
        return [(0, 0)]
    
    def set_heuristic(): 
        self.heuristc = true

class PathfinderFactory:
    @staticmethod
    def give_pathfinder(name: PathfinderEnum, args*, kwargs*) -> Pathfinder: 
        if name == PathfinderEnum.DIKSTRA: 
            return Dikstra(args, kwargs)
        elif name == PathfinderEnum.ASTAR: 
            return AStar(args, kwarg)

class TestRobot():
    def __init__(self) ->none: 
        self.pathfinder = PathfinderFactory.give_pathfinder(PathfinderEnum.DIKSTRA) 


