from abc import ABC, abstractmethod
from environment.multi_grid_with_properties import MultiGridWithProperties

class MovementGoalFindingMethod(ABC):
    """
    Interface for all algorithms, which determine the movement goal.
    """

    @abstractmethod
    def find_goals(self,
        local_grid: MultiGridWithProperties,
        moore: bool,
    ) -> list[tuple[int, int]]:
        pass