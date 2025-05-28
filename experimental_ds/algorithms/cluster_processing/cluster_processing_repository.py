from algorithms.cluster_processing.connected_component import ConnectedComponent

ALGORITHM_MAP = {
    "connected_component": ConnectedComponent
}

def get_algorithm(name: str, *args, **kwargs) -> callable:
    algorithm_class = ALGORITHM_MAP.get(name.lower())
    if algorithm_class is None:
        raise ValueError(f"Algorithm {name} not found.")
    return algorithm_class(*args, **kwargs)