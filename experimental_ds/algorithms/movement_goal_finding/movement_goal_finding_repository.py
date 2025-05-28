from algorithms.movement_goal_finding.original_frontier_based_exploration import OriginalFrontierBasedExploration

ALGORITHM_MAP = {
    "original_frontier_based_exploration": OriginalFrontierBasedExploration
}

def get_algorithm(name: str, *args, **kwargs) -> callable:
    algorithm_class = ALGORITHM_MAP.get(name.lower())
    if algorithm_class is None:
        raise ValueError(f"Algorithm {name} not found.")
    return algorithm_class(*args, **kwargs)