from algorithms.environment_perception.raycasting_bresenham import RaycastingBresenham

ALGORITHM_MAP = {
    "raycasting_bresenham": RaycastingBresenham
}

def get_algorithm(name: str, *args, **kwargs) -> callable:
    algorithm_class = ALGORITHM_MAP.get(name.lower())
    if algorithm_class is None:
        raise ValueError(f"Algorithm {name} not found.")
    return algorithm_class(*args, **kwargs)