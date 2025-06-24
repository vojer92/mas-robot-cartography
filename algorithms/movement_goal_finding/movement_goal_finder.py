from abc import ABC, abstractmethod

class MovementGoalFinder(ABC):
    """
    Interface for all algorithms, which determine the movement goal.
    """
    @abstractmethod
    def find_goals(self,
    ) -> 'list[tuple[int, int]] | None':
        pass