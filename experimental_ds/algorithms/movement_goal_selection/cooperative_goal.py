# TODO:
#  Change from blackboard["assigned_goals"] to CommunicationMethod Interfaces get-Function

from algorithms.movement_goal_selection.movement_goal_selection_method import MovementGoalSelectionMethod
from algorithms.heuristics.heuristic_method import HeuristicMethod
from algorithms.communication.communication_method import CommunicationMethod
from environment.multi_grid_with_properties import MultiGridWithProperties

class  CooperativeGoal(MovementGoalSelectionMethod):
    """
    Gets the individual goal from a global list of coordinated movement goals.

    Fallback movement goal selection method has to be without coordination to be a real fallback method!

    Needs a global blackboard with "assigned_goals"
    """
    def __init__(self,
        heuristic: HeuristicMethod,
        fallback_movement_goal_selection_method: MovementGoalSelectionMethod,
        communication_method: CommunicationMethod,
    ):
        self.heuristic = heuristic
        self.fallback_movement_goal_selection_method = fallback_movement_goal_selection_method
        self.communication_method = communication_method



    def select_movement_goal(self,
        local_grid: MultiGridWithProperties,
        current_pos: tuple[int, int],
        goals: list[tuple[int, int]],
        agent,
        model,
        **kwargs
    ) -> tuple[int, int]:
        assigned_goal = self.communication_method.receive("assigned_goals",agent.unique_id)
        if assigned_goal is None:
            print(f"Agent {agent.unique_id}: No assigned goal found, using fallback.")
            assigned_goal = self.fallback_movement_goal_selection_method.select_movement_goal(
                local_grid=local_grid,
                current_pos=current_pos,
                goals=goals,
                agent=agent,
                model=model,
                **kwargs
            )
        return assigned_goal