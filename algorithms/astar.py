import numpy as np
import heapq

class Cell:
    """
    Represents a node in the A* search with position, cost from start, estimated cost to goal and parent reference.
    """
    def __init__(
            self,
            position: tuple[int, int],
            real_costs_from_start: float=0,
            estimated_costs_till_goal: float=0,
            parent: 'Cell | None'=None
        ):
        self.position = position
        self.real_costs_from_start = real_costs_from_start
        self.estimated_costs_till_goal = estimated_costs_till_goal
        self.parent = parent

    def __lt__(self, other): # Comparison method, necessary for heap
        return self.cost < other.cost

    @property
    def cost(self):
        return self.real_costs_from_start + self.estimated_costs_till_goal

def manhattan_distance(
        point_a: tuple[int, int],
        point_b: tuple[int, int]
) -> float:
    """
    Calculate the Manhattan distance between two points.
    """
    return sum(abs(a - b) for a, b in zip(point_a, point_b))

def euclidean_distance(
        point_a: tuple[int, int],
        point_b: tuple[int, int]
) -> float:
    """
    Calculate the Euclidean distance between two points.
    """
    return sum((a - b) ** 2 for a, b in zip(point_a, point_b)) ** 0.5

def get_neighbors(
        grid: np.ndarray,
        position: tuple[int, int],
        diagonal: bool = False
) -> list[tuple[int, int]]:
    """
    Get the neighbor positions of the cell at the given position in a given grid.
    """
    directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]
    if diagonal:
        directions += [(-1, -1), (-1, 1), (1, -1), (1, 1)]
    neighbors = []
    x, y = position
    for dx, dy in directions:
        nx, ny = x + dx, y + dy
        if (
            0 <= nx < grid.shape[0] and
            0 <= ny < grid.shape[1] and
            grid[nx, ny] == 0 #0 = free, 1 = occupied, -1 = unknown to the agent
        ):
            neighbors.append((nx, ny))
    return neighbors

def reconstruct_path(end_node: Cell) -> list[tuple[int, int]]:
    """
    Reconstruct the path in reverse in context of the A* algorithm.
    """
    path = []
    current = end_node
    while current:
        path.append(current.position)
        current = current.parent
    return path[::-1]

def a_star_search(
        grid: np.ndarray,
        start_position: tuple[int,int],
        goal_position: tuple[int,int],
        costs_per_moved_cell: float,
        heuristic: callable[tuple[int,int],tuple[int,int]],
        diagonal_movement_allowed: bool
)->'list[tuple[int, int]] | None':
    """
    Route search using A* algorithm.
    :return: List of positions representing the optimal path from start_position to goal_position.
    """

    # Initialize structures and start cell
    open_list = [] # Cells to be evaluated
    heapq.heapify(open_list) # Converts open_list to heap (priority queue)
    start_cell = Cell(
        position=start_position,
        real_costs_from_start=0,
        estimated_costs_till_goal=heuristic(start_position, goal_position),
        parent = None
    )
    heapq.heappush(open_list, start_cell)

    closed_set = set() # Cells already evaluated. Set for faster in-operation

    cell_map = {start_position: start_cell} # Link Cell-object to position for better accessibility

    # Forward procession
    while open_list:
        current_cell = heapq.heappop(open_list) # Get first / cheapest entry from priority queue
        if current_cell.position == goal_position:
            # Termination, continue with backward procession
            return reconstruct_path(current_cell)
        closed_set.add(current_cell.position) # Add to already evaluated cells
        for neighbor_position in get_neighbors(grid, current_cell.position, diagonal_movement_allowed):
            if neighbor_position in closed_set:
                continue
            # If neighbor is not in cell_map / unknown - create new node, add to open_list
            # If the new path to this neighbor is cheaper than the  already known path - update the node, add to open_list again
            new_real_costs_from_start = current_cell.real_costs_from_start + costs_per_moved_cell
            if (neighbor_position not in cell_map or
                new_real_costs_from_start < cell_map[neighbor_position].real_costs_from_start):
                neighbor_cell = Cell(
                    position=neighbor_position,
                    real_costs_from_start = new_real_costs_from_start,
                    estimated_costs_till_goal = heuristic(neighbor_position, goal_position),
                    parent = current_cell
                )
                cell_map[neighbor_position] = neighbor_cell
                heapq.heappush(open_list, neighbor_cell)
    # No path found
    return None