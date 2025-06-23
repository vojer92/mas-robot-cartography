from abc import ABC, abstractmethod

class Pathfinder(ABC):
    """
    Interface for all path finding algorithms.
    """
    @abstractmethod
    def find_path(self,
        goal_pos: tuple[int, int],
    ) -> 'list[tuple[int, int]] | None':
        pass