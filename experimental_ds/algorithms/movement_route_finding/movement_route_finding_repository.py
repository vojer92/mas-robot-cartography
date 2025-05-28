from algorithms.movement_route_finding.astar import AStar
from algorithms.movement_route_finding.dynamic_potential_field import DynamicPotentialField

ALGORITHM_MAP = {
    "astar": AStar,
    "dynamic_potential_field": DynamicPotentialField,
}

def get_algorithm(name: str, *args, **kwargs) -> callable:
    algorithm_class = ALGORITHM_MAP.get(name.lower())
    if algorithm_class is None:
        raise ValueError(f"Algorithm {name} not found.")
    return algorithm_class(*args, **kwargs)