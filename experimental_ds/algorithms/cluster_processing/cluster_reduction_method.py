from abc import ABC, abstractmethod

class ClusterReductionMethod(ABC):
    @abstractmethod
    def reduce(self,
        cluster: list[tuple[int, int]]
    ) -> tuple[int, int]:
        """
        Interface for cluster reduction methods.
        """
        pass