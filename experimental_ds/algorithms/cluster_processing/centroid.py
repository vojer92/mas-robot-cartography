from algorithms.cluster_processing.cluster_reduction_method import ClusterReductionMethod

class Centroid(ClusterReductionMethod):
    def reduce(self,
        cluster: list[tuple[int, int]]
    ) -> tuple[int, int]:
        """
        Computes the centroid (mean position) of a cluster of cells.
        """
        if not cluster:
            raise ValueError("Cluster is empty")
        xs = [cell[0] for cell in cluster]
        ys = [cell[1] for cell in cluster]
        centroid_x = int(round(sum(xs) / len(xs)))
        centroid_y = int(round(sum(ys) / len(ys)))
        return (centroid_x, centroid_y)