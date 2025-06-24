from typing import Callable
from mesa.discrete_space import Cell, CellAgent
from algorithms.movement_goal_selection.movement_goal_selector import MovementGoalSelector

class NearestBiggestFrontier(MovementGoalSelector):
    """
    Selects the nearest and biggest (regarding number of connected goals) goal from given goals.
    With factor_distance and factor_size is fine adjustment between those two aspects possible.
    """
    def __init__(self,
        agent: CellAgent,
        factor_distance: float,
        distance_heuristic: Callable[[tuple, tuple], float],
        factor_size: float,
    ):
        self.agent = agent
        self.factor_distance = factor_distance # Multiplication with (-1) is fix implemented, because higher distance reduces attractivity
        self.distance_heuristic = distance_heuristic
        self.factor_size = factor_size

    def select_goal(self,
    goals: list[tuple[int, int]],
    ) -> 'tuple[int, int] | None':
        goal_scores = {}

        # Calculate number of connected frontier cells
        for cluster in self._connected_component_clustering(goals):
            size_score = len(cluster)
            for position in cluster:
                goal_scores[position] = {
                    "size_score": size_score,
                    "distance_score": None,
                    "attractivity": None
                }
        # Calculate distance from agent to goal and
        #  overall attractivity of a specific goal for the current agent
        for position in goal_scores:
            goal_scores[position]["distance_score"] = self.distance_heuristic(self.agent.cell.coordinate, position)
            goal_scores[position]["attractivity"] = (-1 * self.factor_distance * goal_scores[position]["distance_score"] +
                    self.factor_size * goal_scores[position]["size_score"])

        # Sort dict by attractivity (and distance as second sort order, to select the faster reachable)
        sorted_goals = sorted(
            goal_scores.items(),
            key=lambda item: (-item[1]["attractivity"], item[1]["distance_score"]),
        )

        # Select best goal and return its position
        return sorted_goals[0][0] if sorted_goals else None


    def _connected_component_clustering(self,
        positions: list[tuple[int, int]],
    ) -> list[list[tuple[int, int]]]:
        """
        Groups spatially contiguous cells into clusters via neighbor traversal.
        """
        clusters = []
        unvisited = set(positions)
        while unvisited:
            cluster = []
            stack = [unvisited.pop()]
            while stack:
                position = stack.pop()
                cluster.append(position)
                for cell in self.agent.model.grid[position].get_neighborhood(radius=1, include_center=False):
                    neighbor_position = cell.coordinate
                    if neighbor_position in unvisited:
                        unvisited.remove(neighbor_position)
                        stack.append(neighbor_position)
            clusters.append(cluster)
        return clusters