from typing import Callable
import heapq
from mesa.discrete_space import CellAgent
from algorithms.pathfinding.pathfinder import Pathfinder

class Node:
    """
    Represents a node in the A* search with position, cost from start, estimated cost to goal and parent reference.
    """
    def __init__(
            self,
            pos: tuple[int, int],
            real_costs_from_start: float=0,
            estimated_costs_till_goal: float=0,
            parent: 'Cell | None'=None
        ):
        self.pos = pos
        self.real_costs_from_start = real_costs_from_start
        self.estimated_costs_till_goal = estimated_costs_till_goal
        self.parent = parent

    def __lt__(self, other): # Comparison method, necessary for heap
        return self.cost < other.cost

    @property
    def cost(self):
        return self.real_costs_from_start + self.estimated_costs_till_goal

class AStar(Pathfinder):
    def __init__(self,
            agent: CellAgent,
            heuristic: Callable[[tuple, tuple], float],
        ):
        self.heuristic = heuristic

        def find_path(self,
            goal_pos: tuple[int, int],
        ) -> 'list[tuple[int, int]] | None':
            """
            Path finding using AStar algorithm.
            :return: List of positions representing the optimal path from agents
                current position to goal position.
            """
            # Initialize helper structures and start node
            open_list = []  # Nodes to be evaluated
            heapq.heapify(open_list)  # Converts open_list to heap (priority queue)
            start_node = Node(
                pos=agent.cell.coordinate,
                real_costs_from_start=0,
                estimated_costs_till_goal=heuristic(self.pos, goal_pos),
                parent=None,
            )
            heapq.heappush(open_list, start_node) # Add first node

            closed_set = set() # Already evaluated nodes. Set for faster in-operation

            node_map = {agent.cell.coordinate: start_node} # Link Node-object to position for better accessibility

            # Forward procession
            while open_list:
                current_node = heapq.heappop(open_list) # Get first / cheapes entry from priority queue
                if current_node.pos = goal_pos:
                    # Termination! Continue with backward procession.
                    return self._reconstruct_path(current_node)
                closed_set.add(current_node.pos)  # Add to already evaluated positions
