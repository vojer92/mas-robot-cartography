from algorithms.heuristics.heuristic_method import HeuristicMethod

class EuclideanDistance(HeuristicMethod):
    def calculate(self,
        pos_a: tuple[int, int],
        pos_b: tuple[int, int]
    ) -> float:
        """
        Calculate the Euclidean distance between two positions.
        """
        return sum((a - b) ** 2 for a, b in zip(pos_a, pos_b)) ** 0.5