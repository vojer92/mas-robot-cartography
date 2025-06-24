from abc import ABC, abstractmethod

class MovementGoalSelector(ABC):
    """
    Interface for all algorithms, which select the movement goal.
    """

    @abstractmethod
    def select_goal(self,
    goals: list[tuple[int, int]],
    ) -> 'tuple[int, int] | None':
        pass