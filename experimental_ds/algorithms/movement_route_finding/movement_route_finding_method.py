from abc import ABC, abstractmethod
from environment.multi_grid_with_properties import MultiGridWithProperties

class MovementRouteFindingMethod(ABC):
    """
    Interface for all route finding algorithms.
    """
    @abstractmethod
    def find_path(self,
        local_grid: MultiGridWithProperties,
        start_pos: tuple[int, int],
        goal_pos: tuple[int, int],
        moore: bool,
        current_time: int = None
    ) -> 'list[tuple[int, int]] | None':
        pass