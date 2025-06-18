from typing import Callable
import heapq
from mesa.discrete_space import CellAgent #For all cell agents, not only explorer robots
from algorithms.pathfinding.pathfinder import Pathfinder

from agents.obstacle import Obstacle
from agents.ground import Ground

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
        self.agent = agent
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

            # Get all neighbor cells
            neighbor_cells = self.agent.model.grid[current_node.pos].get_neighborhood(
                    radius=1, include_center=False)

            # Check neighbor cells
            for cell in neighbor_cells:
                # Check if cell is blocked by other agent
                # getattr, because e.g. Ground-agents dont have blocking properties!
                if any(getattr(agent, "cell_blocking", False) is True for agent in cell.agents):
                    continue

                # Check if cell is already explored
                ground_agents = [agent for agent in cell.agents if isinstance(agent, Ground)]
                if not ground_agents:
                    raise RuntimeError(f"Environment Error: Cell {cell.coordinate} has no Obstacle and no Ground agent.")
                if not ground_agents[0].explored:
                    continue

                # Check if neighbor was already evaluated
                neighbor_pos = cell.coordinate
                if neighbor_pos in closed_set:
                    continue

                # If neighbor is not in node_map / unknown - create new node, add to open_list
                # If the new path to this neighbor is cheaper than the  already known path - update the node add to open_list again
                new_real_costs_from_start = current_node.real_costs_from_start + 1
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