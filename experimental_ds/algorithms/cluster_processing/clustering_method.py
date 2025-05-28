# TODO:
#  Update auf mesa 3.2.0

from abc import ABC, abstractmethod
from environment.multi_grid_with_properties import MultiGridWithProperties

class ClusteringMethod(ABC):
    @abstractmethod
    def cluster(self,
        cells: list[tuple[int, int]],
        grid: MultiGridWithProperties,
        moore: bool,
        property_name: str = None,
        property_value=None,
        **kwargs
    ) -> list[list[tuple[int, int]]]:
        """
        Interface for methods to cluster given cells.
        """
        pass