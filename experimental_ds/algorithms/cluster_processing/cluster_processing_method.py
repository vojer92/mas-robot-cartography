from algorithms.cluster_processing.clustering_method import ClusteringMethod
from algorithms.cluster_processing.cluster_reduction_method import ClusterReductionMethod
from environment.multi_grid_with_properties import MultiGridWithProperties

class ClusterProcessingMethod():
    """
    Clustering and cluster reduction of cells.
    """
    def __init__(self,
        clustering_method: ClusteringMethod,
        cluster_reduction_method: ClusterReductionMethod
    ):
        self.clustering_method = clustering_method
        self.cluster_reduction_method = cluster_reduction_method

    def process(self,
        cells: list[tuple[int, int]],
        grid: MultiGridWithProperties,
        moore: bool,
        property_name: str = None,
        property_value = None,
        **kwargs
    ) -> list[list[tuple[int, int]]]:
        clusters = self.clustering_method.cluster(cells, grid, moore, property_name, property_value, **kwargs)
        reduced_clusters = [self.cluster_reduction_method.reduce(cluster) for cluster in clusters]
        return reduced_clusters