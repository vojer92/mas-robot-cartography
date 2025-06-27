from typing import Callable
import heapq
import math

from agents.explorer_robot import ExplorerRobot
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
            parent: 'Node | None'=None
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
            agent: ExplorerRobot,
            heuristic: Callable[[tuple, tuple], float],
        ):
        self.agent = agent
        self.heuristic = heuristic

    def find_path(self,
        goal_pos: tuple[int, int],
    ) -> 'list[tuple[int, int]] | None':
        """
        Path finding in Moore-grid using AStar algorithm.
        :return: List of positions representing the optimal path from agents
            current position to goal position.
        """
        # Initialize helper structures and start node
        open_list = []  # Nodes to be evaluated
        heapq.heapify(open_list)  # Converts open_list to heap (priority queue)
        start_node = Node(
            pos=self.agent.cell.coordinate,
            real_costs_from_start=0,
            estimated_costs_till_goal=self.heuristic(self.agent.cell.coordinate, goal_pos),
            parent=None,
        )
        heapq.heappush(open_list, start_node) # Add first node

        closed_set = set() # Already evaluated nodes. Set for faster in-operation

        node_map = {self.agent.cell.coordinate: start_node} # Link Node-object to position for better accessibility

        # Forward procession
        while open_list:
            # Get first entry from priority queue
            current_node = heapq.heappop(open_list)

            # Check for termination
            if current_node.pos == goal_pos:
                # Termination! Continue with backward procession.
                return self._reconstruct_path(current_node)

            # Add position to already evaluated positions
            closed_set.add(current_node.pos)

            # Check all already explored neighbor cells
            for neighbor_pos in self.agent.local_memory.get_known_neighbor_positions(current_node.pos):

                # Check if neighbor cell was already evaluated
                if neighbor_pos in closed_set:
                    continue

                cell_info = self.agent.local_memory.grid_info[neighbor_pos]

                # Check if cell is blocked by other agent
                if any(agent_info.cell_blocking for agent_info in cell_info.agents):
                    continue

                # If neighbor is not in node_map / unknown - create new node, add to open_list
                # If the new path to this neighbor is cheaper than the  already known path - update the node add to open_list again
                step_cost = 1
                #NOTE:
                # To calculate higher costs for diagonal movement change to
                # step_cost = 1 if dx == 0 or dy == 0 else math.sqrt(2)
                new_real_costs_from_start = current_node.real_costs_from_start + step_cost
                if (neighbor_pos not in node_map or
                        new_real_costs_from_start < node_map[neighbor_pos].real_costs_from_start):
                    neighbor_cell = Node(
                        pos=neighbor_pos,
                        real_costs_from_start=new_real_costs_from_start,
                        estimated_costs_till_goal=self.heuristic(neighbor_pos, goal_pos),
                        parent=current_node
                    )
                    node_map[neighbor_pos] = neighbor_cell
                    heapq.heappush(open_list, neighbor_cell)

        # No path found
        return None

    def _reconstruct_path(self, end_node: Node) -> list[tuple[int, int]]:
        """
        Reconstruct the path in reverse in context of the A* algorithm.
        """
        path = []
        current = end_node
        while current:
            path.append(current.pos)
            current = current.parent
        return path[::-1]