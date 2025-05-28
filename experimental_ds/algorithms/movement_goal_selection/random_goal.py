from algorithms.movement_goal_selection.movement_goal_selection_method import MovementGoalSelectionMethod
from environment.multi_grid_with_properties import MultiGridWithProperties

class RandomGoal(MovementGoalSelectionMethod):
    """
    Selects a random movement goal from a list of movement goals.
    """
    def select_movement_goal(self,
        local_grid: MultiGridWithProperties,
        current_pos: tuple[int, int],
        goals: list[tuple[int, int]],
        agent,
        model,
        **kwargs
    ) -> tuple[int, int]:
        return model.random.choice(goals)