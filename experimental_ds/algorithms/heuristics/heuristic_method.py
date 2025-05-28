from abc import ABC, abstractmethod

class HeuristicMethod(ABC):
    """
    Interface for heuristic methods.
    """
    @abstractmethod
    def calculate(self,
        pos_a: tuple[int, int],
        pos_b: tuple[int, int]
    ) -> float:
        pass