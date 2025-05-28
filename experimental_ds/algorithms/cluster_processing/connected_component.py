from algorithms.cluster_processing.clustering_method import ClusteringMethod
from environment.multi_grid_with_properties import MultiGridWithProperties

class ConnectedComponent(ClusteringMethod):
    def cluster(self,
        cells: list[tuple[int, int]],
        grid: MultiGridWithProperties,
        moore: bool,
        property_name: str = None,
        property_value=None,
        **kwargs
    ) -> list[list[tuple[int, int]]]:
        """
        Groups spatially contiguous cells into clusters via neighbor traversal.
        property_name, property_value and **kwargs are not required.
        """
        clusters = []
        unvisited = set(cells)
        while unvisited:
            cluster = []
            stack = [unvisited.pop()]
            while stack:
                cell = stack.pop()
                cluster.append(cell)
                for neighbor_pos in grid.iter_neighborhood(cell, moore, include_center=False, radius=1):
                    if neighbor_pos in unvisited:
                        unvisited.remove(neighbor_pos)
                        stack.append(neighbor_pos)
            clusters.append(cluster)
        return clusters