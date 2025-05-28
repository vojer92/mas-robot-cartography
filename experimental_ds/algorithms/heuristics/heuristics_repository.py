from algorithms.heuristics.euclidean import EuclideanDistance
from algorithms.heuristics.manhattan import ManhattanDistance

ALGORITHM_MAP = {
    "euclidean": EuclideanDistance,
    "manhattan": ManhattanDistance,
}

def get_algorithm(name: str, *args, **kwargs) -> callable:
    algorithm_class = ALGORITHM_MAP.get(name.lower())
    if algorithm_class is None:
        raise ValueError(f"Algorithm {name} not found.")
    return algorithm_class(*args, **kwargs)