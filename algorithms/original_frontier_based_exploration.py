# TODO: Change to mesa 3.2.0!
#  MultiGridWithProperties -> DiscreteSpace + CellObjects + PropertyLayer

# TODO:
#  If charging stations (or other objects which dont block cells) are introduced
#  is_cell_empty cannot be used anymore!

# TODO:
#  Clustering of frontiers komplett auslagern, da je nach Kooperation / Kommunikation zuerst andere Schritte stattfinden,
#  bevor ggf. Frontiers geclustert und auf Zentroiden zusammengefasst werden!
#  Das sollte mglst. allgemein sein und auch auf andere FBE-Methoden angewendet werden können!

# ------------------------------------------------------------------------------------------
# repository.py
from algorithms.original_frontier_based_exploration import OriginalFrontierBasedExploration
from algorithms.raycasting_bresenham import MultiGridWithProperties

...
    "original_frontier_based_exploration": OriginalFrontierBasedExploration
...

# ------------------------------------------------------------------------------------------
# frontier_clustering_algorithm.py
from abc import ABC, abstractmethod

class FrontierClusteringAlgorithm(ABC):
    @abstractmethod
    def cluster(self,
        frontier_cells: list[tuple[int, int]],
        local_grid: MultiGridWithProperties,
        moore: bool
    ) -> list[list[tuple[int, int]]]:
        """
        Interface for clustering algorithm to cluster the given frontier cells.
        """
        pass

# ------------------------------------------------------------------------------------------
# flood_fill.py
from algorithms.frontier_clustering_algorithm import FrontierClusteringAlgorithm

class FloodFillFrontierClustering(FrontierClusteringAlgorithm):
    def cluster(self,
        frontier_cells: list[tuple[int, int]],
        local_grid: MultiGridWithProperties,
        moore: bool
    ) -> list[list[tuple[int, int]]]:
        clusters = []
        unvisited = set(frontier_cells)
        while unvisited:
            cluster = []
            stack = [unvisited.pop()]
            while stack:
                cell = stack.pop()
                cluster.append(cell)
                for neighbor_pos in local_grid.iter_neighborhood(cell, moore, include_center=False, radius=1):
                    if neighbor_pos in unvisited:
                        unvisited.remove(neighbor_pos)
                        stack.append(neighbor_pos)
            clusters.append(cluster)
        return clusters

# ------------------------------------------------------------------------------------------
# goal_selection_strategy.py
from abc import ABC, abstractmethod

class GoalSelectionStrategy(ABC):
    @abstractmethod
    def select_goals(self,
        clusters: list[list[tuple[int, int]]]
    ) -> list[tuple[int, int]]:
        pass

# ------------------------------------------------------------------------------------------
# goal_selection_strategy.py
from algorithms.goal_selection_strategy import GoalSelectionStrategy

class Centroids(GoalSelectionStrategy):
    def select_goals(self,
        clusters: list[list[tuple[int, int]]]
    ) -> list[tuple[int, int]]:
        return [self._centroid(cluster) for cluster in clusters]

    @staticmethod
    def _centroid(points: list[tuple[int, int]]) -> tuple[int, int]:
        xs = [p[0] for p in points]
        ys = [p[1] for p in points]
        return (int(sum(xs) / len(xs)), int(sum(ys) / len(ys)))

# ------------------------------------------------------------------------------------------
# goal_finding_algorithm.py
from abc import ABC, abstractmethod

class GoalFindingAlgorithm(ABC):
    """
    Interface for all algorithms, which determine the movement goal.
    """

    @abstractmethod
    def find_goals(self,
        local_grid: MultiGridWithProperties,
        moore: bool
    ) -> list[tuple[int, int]]:
        pass

# ------------------------------------------------------------------------------------------
# original_frontier_based_exploration.py
from algorithms.goal_finding_algorithm import GoalFindingAlgorithm
from algorithms.frontier_clustering_algorithm import FrontierClusteringAlgorithm
from algorithms.goal_selection_strategy import GoalSelectionStrategy

class OriginalFrontierBasedExploration(GoalFindingAlgorithm):
    def __init__(self,
        group_frontiers: bool,
        frontier_clustering_algorithm: FrontierClusteringAlgorithm,
        goal_selection_strategy: GoalSelectionStrategy
    ):
        self.group_frontiers = group_frontiers
        self.frontier_clustering_algorithm = frontier_clustering_algorithm
        self.goal_selection_strategy = goal_selection_strategy

    def find_goals(self,
        local_grid: MultiGridWithProperties,
        moore: bool,
    ) -> list[tuple[int, int]]:
        all_frontier_cells = self._find_frontier_cells(local_grid, moore)
        if not all_frontier_cells:
            return []
        if self.group_frontiers:
            # Cluster frontiers and return centroids of the cluster
            clusters = self.frontier_clustering_algorithm.cluster(all_frontier_cells, local_grid, moore)
            return self.goal_selection_strategy.select_goals(clusters)
        else:
            # Return all single frontier-cells
            return all_frontier_cells


    def _find_frontier_cells(self,
        local_grid: MultiGridWithProperties,
        moore: bool
    ) -> list[tuple[int, int]]:
        frontier_cells = []
        for x in range(local_grid.width):
            for y in range(local_grid.height):
                pos = (x, y)
                if self._is_frontier(local_grid, pos, moore):
                    frontier_cells.append(pos)
        return frontier_cells


    def _is_frontier(self,
        local_grid: MultiGridWithProperties,
        pos: tuple[int, int],
        moore: bool,
    ) -> bool:
        """
        Uses the following frontier definition:
        Frontier = A cell which is known and free with at least one unknown neighbor cell.
        """
        # Check grid borders
        if local_grid.out_of_bounds(pos):
            return False
        # Check if known and free
        if local_grid.get_property(pos, "unknown") or not local_grid.is_cell_empty(pos):
            return False
        # Check for unknown neighbor cells
        for neighbor_pos in local_grid.iter_neighborhood(pos, moore, include_center=False, radius=1):
            if local_grid.get_property(neighbor_pos, "unknown"):
                return True
        return False