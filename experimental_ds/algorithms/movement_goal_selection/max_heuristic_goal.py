from algorithms.movement_goal_selection.movement_goal_selection_method import MovementGoalSelectionMethod
from algorithms.heuristics.heuristic_method import HeuristicMethod
from environment.multi_grid_with_properties import MultiGridWithProperties


class MaxHeuristicGoal(MovementGoalSelectionMethod):
    """
    Selects the movement goal from a list of movement goals with the maximal heuristic value.
    Depending on the used heuristic it is the farthest, maximal benefit, ...
    """

    def __init__(self, heuristic: HeuristicMethod):
        self.heuristic = heuristic

    def select_movement_goal(self,
        local_grid: MultiGridWithProperties,
        current_pos: tuple[int, int],
        goals: list[tuple[int, int]],
        agent,
        model,
        **kwargs
    ) -> tuple[int, int]:
        if not goals:
            raise Exception('No goals available')
        return max(
            goals,
            key=lambda goal: self.heuristic.calculate(current_pos, goal)
        )