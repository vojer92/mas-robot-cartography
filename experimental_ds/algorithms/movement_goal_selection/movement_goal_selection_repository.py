from algorithms.movement_goal_selection.random_goal import RandomGoal
from algorithms.movement_goal_selection.min_heuristic_goal import MinHeuristicGoal
from algorithms.movement_goal_selection.max_heuristic_goal import MaxHeuristicGoal

ALGORITHM_MAP = {
    "random_goal": RandomGoal,
    "min_heuristic_goal": MinHeuristicGoal,
    "max_heuristic_goal": MaxHeuristicGoal,
}

def get_algorithm(name: str, *args, **kwargs) -> callable:
    algorithm_class = ALGORITHM_MAP.get(name.lower())
    if algorithm_class is None:
        raise ValueError(f"Algorithm {name} not found.")
    return algorithm_class(*args, **kwargs)