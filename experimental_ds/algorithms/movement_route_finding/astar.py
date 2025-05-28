# TODO: Change to mesa 3.2.0!
#  MultiGridWithProperties -> DiscreteSpace + CellObjects + PropertyLayer

# TODO:
#  If charging stations (or other objects which dont block cells) are introduced
#  is_cell_empty cannot be used anymore!

import heapq
from algorithms.movement_route_finding.movement_route_finding_method import MovementRouteFindingMethod
from algorithms.heuristics.heuristic_method import HeuristicMethod
from environment.multi_grid_with_properties import MultiGridWithProperties

class Cell:
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

class AStar(MovementRouteFindingMethod):
    def __init__(self, heuristic: HeuristicMethod):
        self.heuristic = heuristic

    def find_path(self,
        grid: 'MultiGridWithProperties',
        start_pos: tuple[int, int],
        goal_pos: tuple[int, int],
        moore: bool,
        current_time: int = None  # Not used in astar-algorithm
    ) -> 'list[tuple[int, int]] | None':
        """
        Route search using A* algorithm.
        :return: List of positions representing the optimal path from start position to goal position.
        """

        # Initialize structures and start cell
        open_list = []  # Cells to be evaluated
        heapq.heapify(open_list)  # Converts open_list to heap (priority queue)
        start_cell = Cell(
            pos=start_pos,
            real_costs_from_start=0,
            estimated_costs_till_goal=self.heuristic.calculate(start_pos, goal_pos),
            parent=None
        )
        heapq.heappush(open_list, start_cell)

        closed_set = set()  # Cells already evaluated. Set for faster in-operation

        cell_map = {start_pos: start_cell}  # Link Cell-object to position for better accessibility

        # Forward procession
        while open_list:
            current_cell = heapq.heappop(open_list)  # Get first / cheapest entry from priority queue
            if current_cell.pos == goal_pos:
                # Termination, continue with backward procession
                return self._reconstruct_path(current_cell)
            closed_set.add(current_cell.pos)  # Add to already evaluated cells
            for neighbor_pos in self._get_neighbors(grid, current_cell.pos, moore):
                if neighbor_pos in closed_set:
                    continue
                # If neighbor is not in cell_map / unknown - create new node, add to open_list
                # If the new path to this neighbor is cheaper than the  already known path - update the node, add to open_list again
                new_real_costs_from_start = current_cell.real_costs_from_start + 1
                if (neighbor_pos not in cell_map or
                        new_real_costs_from_start < cell_map[neighbor_pos].real_costs_from_start):
                    neighbor_cell = Cell(
                        pos=neighbor_pos,
                        real_costs_from_start=new_real_costs_from_start,
                        estimated_costs_till_goal=self.heuristic.calculate_distance(neighbor_pos, goal_pos),
                        parent=current_cell
                    )
                    cell_map[neighbor_pos] = neighbor_cell
                    heapq.heappush(open_list, neighbor_cell)
        # No path found
        return None

    def _get_neighbors(self,
            grid: MultiGridWithProperties,
            pos: tuple[int, int],
            moore: bool = False
    ) -> list[tuple[int, int]]:
        """
        Get the neighbor positions of the cell at the given position in a given grid.
        """
        neighbors_iter = grid.iter_neighborhood(pos, moore, include_center=False, radius=1)
        valid_neighbors = []
        for neighbor_pos in neighbors_iter:
            if not grid.get_property(neighbor_pos,"unknown") and grid.is_cell_empty(neighbor_pos):
                valid_neighbors.append(neighbor_pos)
        return valid_neighbors

    def _reconstruct_path(self, end_node: Cell) -> list[tuple[int, int]]:
        """
        Reconstruct the path in reverse in context of the A* algorithm.
        """
        path = []
        current = end_node
        while current:
            path.append(current.pos)
            current = current.parent
        return path[::-1]