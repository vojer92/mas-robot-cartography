from abc import ABC, abstractmethod
from environment.multi_grid_with_properties import MultiGridWithProperties

class Pathfinder(ABC):
    """
    Interface for all path finding algorithms.
    """
    @abstractmethod
    def find_path(self,
        goal_pos: tuple[int, int],
    ) -> 'list[tuple[int, int]] | None':
        pass