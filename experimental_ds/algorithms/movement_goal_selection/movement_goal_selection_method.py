from abc import ABC, abstractmethod
from environment.multi_grid_with_properties import MultiGridWithProperties


class MovementGoalSelectionMethod(ABC):
    """
    Interface for the Methods to select one movement goal from a list of movement goals
    """
    @abstractmethod
    def select_movement_goal(self,
        local_grid: MultiGridWithProperties,
        current_pos: tuple[int, int],
        goals: list[tuple[int, int]],
        agent,
        model,
        **kwargs
    ) -> tuple[int, int]:
        pass