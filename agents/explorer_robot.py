from abc import ABC, abstractmethod
import heapq

from mesa import Model
from mesa.discrete_space import Cell, CellAgent

from algorithms.manhattan_distance import calculate_manhattan_distance


class ExplorerRobot(ABC, CellAgent):
    def __init__(
        self,
        model: Model,
        cell: Cell,
        orientation: int,

    ):
        super().__init__(model)
        self.cell = cell
        self.view_radius = view_radius
        if view_angle == 360:
            self.view_angle = 359
        else:
            self.view_angle = view_angle
        self.view_resolution = view_resolution
        self.orientation = orientation
        self.viewport: list[tuple[int, int]] = []

    @abstractmethod
    def move(self):
        pass

    @abstractmethod
    def step(self):
        pass





















--------------------------------------------------------------------------------------------------------

    # Route finding
    class Node:
        """
        Represents a node in the A* search with position, cost from start, estimated cost to goal and parent reference.
        """
        def __init__(self,
            pos: tuple[int, int],
            real_costs_from_start: float = 0,
            estimated_costs_till_goal: float = 0,
            parent: 'Cell | None' = None
        ):
            self.pos = pos
            self.real_costs_from_start = real_costs_from_start
            self.estimated_costs_till_goal = estimated_costs_till_goal
            self.parent = parent

        def __lt__(self, other): #Comparisation method, necessary for heap
            return self.cost < other.cost

        @property
        def cost(self) -> float:
            return self.real_costs_from_start + self.estimated_costs_till_goal

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

    def find_path(self,
        start_pos: tuple[int, int],
        goal_pos: tuple[int, int],
    ) -> list[tuple[int, int]] | None:
        """
        Route search using A* algorithm.
        :return: List of positions representing the optimal path from start position to goal position.
        """
        # Initialize structures and start cell
        open_list = []  # Nodes to be evaluated
        heapq.heapify(open_list)  # Converts open_list to heap (priority queue)
        start_node = Node(
            pos=start_pos,
            real_costs_from_start=0,
            estimated_costs_till_goal=calculate_manhattan_distance(start_pos, goal_pos),
            parent=None
        )
        heapq.heappush(open_list, start_node)

        closed_set = set()  # Cells already evaluated. Set for faster in-operation

        node_map = {start_pos: start_node}  # Link Cell-object to position for better accessibility

        # Forward procession
        while open_list:
            current_node = heapq.heappop(open_list)  # Get first / cheapest entry from priority queue
            if current_node.pos == goal_pos:
                # Termination, continue with backward procession
                return self._reconstruct_path(current_node)
            closed_set.add(current_node.pos)  # Add to already evaluated nodes





            valid_neighbors = [] # Only known and not occupied neighbor cells
            for cell in self.cell.get_neighborhood(radius=1, include_center=False):
                agents_in_cell = cell.agents
                # Ceck if cell is known and
                # >= 1 agent is blocking
                #
                if
                    not any(getattr(agent,  "blocking", False) for agent in agents_in_cell)








            for neighbor_pos in valid_neighbors:
                if neighbor_pos in closed_set:
                    continue
                # If neighbor is not in node_map / unknown - create new node, add to open_list
                # If the new path to this neighbor is cheaper than the  already known path - update the node, add to open_list again
                new_real_costs_from_start = current_node.real_costs_from_start + 1
                if (neighbor_pos not in node_map or
                        new_real_costs_from_start < node_map[neighbor_pos].real_costs_from_start):
                    neighbor_cell = Node(
                        pos=neighbor_pos,
                        real_costs_from_start=new_real_costs_from_start,
                        estimated_costs_till_goal=calculate_manhattan_distance(neighbor_pos, goal_pos),
                        parent=current_node
                    )
                    node_map[neighbor_pos] = neighbor_cell
                    heapq.heappush(open_list, neighbor_cell)
                # No path found
            return None