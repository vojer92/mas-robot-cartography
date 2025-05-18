# TODO:
#  Agenten-Typbezeichnungen abstimmen

# TODO:
#  Funktionen für Anziehung und Abstoßung als Parameter beim DynamicPotentialField.__init__

# TODO:
#  Schönere Methode (z.B. Ringförmig um aktuelle Position per iter_neighborhood(sight_range)) implementieren
#  bei _get_current_perception_cells!
#  Bis dahin einfach komplettes Grid durchgehen.

# TODO: Change to mesa 3.2.0!
#  MultiGridWithProperties -> DiscreteSpace + CellObjects + PropertyLayer

# TODO:
#  What happens if an agent is surrounded by repulsive objects?
#  Is a absolute check (for example with is_cell_empty (! non repulsive objects)) necessary?

#------------------------------------------------------------------------------------------
# ???

class MultiGridWithProperties(MultiGrid):
    ...

#------------------------------------------------------------------------------------------
# agent.py
    # agent.py

    ...
    # Initialisierung
    ...
    self.local_environment_memory = MultiGridWithProperties(width, height, torus)
    # Latest perception time for aging of the memory and knowing the newest scan
    self.local_environment_memory.add_property("perception_time")
    ...


# ------------------------------------------------------------------------------------------
# route_finding_algorithm.py
from abc import ABC, abstractmethod
from ??? import MultiGridWithProperties

# Main interface for all route-finding-algorithms
class RouteFindingAlgorithm(ABC):
    """
    Interface for all route finding algorithms.
    """
    @abstractmethod
    def find_path(self,
        local_grid: MultiGridWithProperties,
        start_pos: tuple[int, int],
        goal_pos: tuple[int, int],
        moore: bool,
        current_time: int
    ) -> 'list[tuple[int, int]] | None':
        pass


# ------------------------------------------------------------------------------------------
# repository.py
from algorithms.dynamic_potential_field import DynamicPotentialField

...
    "dynamic_potential_field": DynamicPotentialField
...

#------------------------------------------------------------------------------------------
# dynamic_potential_field.py
from algorithms.route_finding_algorithm import RouteFindingAlgorithm
from algorithms.heuristics.heuristic_strategy import HeuristicStrategy, EuclideanDistance
from ??? import MultiGridWithProperties
from mesa import random

class DynamicPotentialField(RouteFindingAlgorithm):
    """
    Route finding algorithm in current perception area for collision avoidance via dynamic potential field.

    ! One charging station can only be used by one agent at a time, because one repulsive object on a cell
    makes the whole cell repulsive!
    """

    def __init__(self,
        distance_heuristic: HeuristicStrategy = EuclideanDistance,
        goal_attraction_strength: float = 1,
        repulsive_objects: list[str] = ["Obstacle"], # Object types of repulsive objects: all except charging station
        object_repulsion_strength: float = 10,
        random_generator: random.Random = None, # In agent: ..., random_generator=self.random)
     ):
        self.distance_heuristic = distance_heuristic
        self.goal_attraction_strength = goal_attraction_strength
        self.repulsive_objects = repulsive_objects
        self.object_repulsion_strength = object_repulsion_strength
        self.random_generator = random_generator if random_generator is not None else random.Random()

    def find_path(self,
        local_grid: MultiGridWithProperties,
        start_pos: tuple[int, int],
        goal_pos: tuple[int, int], # Position of goal or furthest away cell of a already calculated path in current perception.
        moore: bool,
        current_time: int
    ) -> 'list[tuple[int, int]] | None':
        return self._gradient_descent(
            local_grid=local_grid,
            moore=moore,
            potential_field = self._calculate_potential_field(
                local_grid=local_grid,
                relevant_cells=self._get_current_perception_cells(
                    local_grid=local_grid,
                    current_time=current_time
                ),
                goal_pos=goal_pos
            ),
            start_pos=start_pos,
            goal_pos=goal_pos
        )

    def gradient_descent(self,
        local_grid: MultiGridWithProperties,
        moore: bool,
        potential_field: dict[tuple[int, int], float],
        start_pos: tuple[int, int],
        goal_pos: tuple[int, int],
        max_iterations: int = 100
    ) -> 'list[tuple[int, int]] | None':
        if start_pos not in potential_field:
            return None
        path = [start_pos]
        current_pos = start_pos
        for _ in range(max_iterations):
            if current_pos == goal_pos:
                return path
            neighbors = [n for n in local_grid.get_neighborhood(current_pos, moore, include_center=False)
                         if n in potential_field]
            if not neighbors:
                break
            min_pot = min(potential_field[n] for n in neighbors)
            min_neighbors = [n for n in neighbors if potential_field[n] == min_pot]
            current_pos = self.random_generator.choice(min_neighbors)
            path.append(current_pos)
        return None # No path found

    def _calculate_potential_field(self,
        local_grid: MultiGridWithProperties,
        relevant_cells: list[tuple[int, int]],
        goal_pos: tuple[int, int]
    ) -> dict[tuple[int, int], float]:
        """
        Calculate the potential field of the relevant cells.
        """
        # Get all objects in relevant_cells
        object_positions = []
        for pos in relevant_cells:
            if not local_grid.is_cell_empty(pos):
                objects = local_grid.get_cell_list_contents([pos])
                for obj in objects:
                    if type(obj).__name__ in self.repulsive_objects:
                        object_positions.append(pos)
                        break  # One repulsive object is enough
        # Calculate potential field
        potential_field = {}
        for pos in relevant_cells:
            # Attraction to goal
            attraction = 0
            dist_to_goal = self.distance_heuristic.calculate_distance(pos, goal_pos)
            attraction = self.goal_attraction_strength * dist_to_goal
            # Repulsion from objects
            repulsion = 0
            for obj_pos in object_positions:
                dist_to_object = self.distance_heuristic.calculate_distance(obj_pos, pos)
                repulsion += self.object_repulsion_strength / (1e-6 + dist_to_object)
            # Potential
            potential_field[pos] = attraction + repulsion
        return potential_field

    def _get_current_perception_cells(self,
        local_grid: MultiGridWithProperties,
        current_time: int
    ) -> list[tuple[int, int]]:
        """
        Get all cells, which were percept in the current time step.
        """
        return[
            (x, y)
            for x in range(local_grid.width)
            for y in range(local_grid.height)
            if local_grid.get_property((x, y), "perception_time") == current_time
        ]